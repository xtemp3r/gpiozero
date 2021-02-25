from __future__ import (
    unicode_literals,
    absolute_import,
    print_function,
    division,
    )
nstr = str
str = type('')

import os
import struct
from time import monotonic

import rgpio

from . import SPI
from .pi import PiFactory, PiPin, spi_port_device
from ..mixins import SharedMixin
from ..exc import (
    PinInvalidFunction,
    PinSetInput,
    PinFixedPull,
    PinInvalidPull,
    PinInvalidBounce,
    PinInvalidState,
    PinSPIUnsupported,
    SPIBadArgs,
    SPIInvalidClockMode,
    PinPWMFixedValue,
    DeviceClosed
)


class RGPIOFactory(PiFactory):
    """
    Extends :class:`~gpiozero.pins.pi.PiFactory`. Uses the `rgpio`_
    library to interface to a remote computer's GPIO pins. The rgpio library
    talks over a socket interface to the `rgpiod daemon`_. The daemon is not
    specific to the Raspberry Pi, although this class is currently constructed
    to assume that rgpiod is indeed running on a Raspberry Pi.

    You can construct rgpio pins manually like so::

        from gpiozero.pins.rgpio import RGPIOFactory
        from gpiozero import LED

        factory = RGPIOFactory()
        led = LED(12, pin_factory=factory)

    By default, the factory will attempt to connect to rgpiod on localhost
    at the default port number (8889). This can be changed via the environment
    with the RGPIO_ADDR and RGPIO_PORT values. Alternatively, you can specify
    this in the constructor::

        from gpiozero.pins.rgpio import RGPIOFactory
        from gpiozero import LED

        factory = RGPIOFactory(host='mypi', port=2000, chip=0)
        led = LED(12, pin_factory=factory)

    The *chip* parameter to the constructor specifies which gpiochip device to
    attempt to open. It defaults to 0 and thus doesn't normally need to be
    specified (the example above only includes it for completeness).

    .. _rgpio: http://abyz.me.uk/lg/py_rgpio.html
    .. _rgpiod daemon: http://abyz.me.uk/lg/rgpiod.html
    """
    def __init__(self, host=None, port=None, chip=0):
        super(RGPIOFactory, self).__init__()
        if host is None:
            host = os.environ.get('RGPIO_ADDR', 'localhost')
        if port is None:
            # XXX Use getservbyname
            port = int(os.environ.get('RGPIO_PORT', 8889))
        self.pin_class = RGPIOPin
        self._connection = rgpio.sbc(host, port)
        if not self._connection.connected:
            raise IOError('failed to connect to %s:%s' % (host, port))
        self._handle = self._connection.gpiochip_open(chip)
        self._host = host
        self._port = port
        self._chip = chip
        self._spis = []

    def close(self):
        super(RGPIOFactory, self).close()
        if self._connection is not None:
            self._connection.stop()
            self._connection = None

    @property
    def host(self):
        return self._host

    @property
    def port(self):
        return self._port

    @property
    def chip(self):
        return self._chip

    def _get_revision(self):
        h = self._connection.file_open(
            '/proc/device-tree/system/linux,revision', rgpio.FILE_READ)
        try:
            revision = None
            if h >= 0:
                count, buf = self._connection.file_read(h, 4)
                revision = hex(struct.unpack(nstr('>L'), buf)[0])[2:]
            else:
                h = self._connection.file_open('/proc/cpuinfo', rgpio.FILE_READ)
                if h >= 0:
                    data = b''
                    while True:
                        count, buf = self._connection.file_read(h, 1024)
                        if not buf:
                            break
                        data += buf
                    for line in data.decode('ascii').splitlines():
                        if line.startswith('Revision'):
                            revision = line.split(':')[1].strip()
            if revision is not None:
                overvolted = revision.startswith('100')
                if overvolted:
                    revision = revision[-4:]
                return int(revision, base=16)
        finally:
            if h >= 0:
                self._connection.file_close(h)
        raise PinUnknownPi(
            'unable to locate Pi revision in /proc/device-tree or /proc/cpuinfo')

    def _get_spi_class(self, shared, hardware):
        if hardware:
            return [RGPIOHardwareSPI, RGPIOHardwareSPIShared][shared]
        raise PinSPIUnsupported('Software SPI not supported by this pin factory')

    @staticmethod
    def ticks():
        return monotonic()

    @staticmethod
    def ticks_diff(later, earlier):
        return max(0, later - earlier)


class RGPIOPin(PiPin):
    """
    Extends :class:`~gpiozero.pins.pi.PiPin`. Pin implementation for the
    `rgpio`_ library. See :class:`RGPIOFactory` for more information.

    .. _rgpio: http://abyz.me.uk/lg/py_rgpio.html
    """
    GPIO_IS_KERNEL         = 1 << 0
    GPIO_IS_OUT            = 1 << 1
    GPIO_IS_ACTIVE_LOW     = 1 << 2
    GPIO_IS_OPEN_DRAIN     = 1 << 3
    GPIO_IS_OPEN_SOURCE    = 1 << 4
    GPIO_IS_BIAS_PULL_UP   = 1 << 5
    GPIO_IS_BIAS_PULL_DOWN = 1 << 6
    GPIO_IS_BIAS_DISABLE   = 1 << 7
    GPIO_IS_LG_INPUT       = 1 << 8
    GPIO_IS_LG_OUTPUT      = 1 << 9
    GPIO_IS_LG_ALERT       = 1 << 10
    GPIO_IS_LG_GROUP       = 1 << 11

    GPIO_LINE_FLAGS_MASK = (
        GPIO_IS_ACTIVE_LOW | GPIO_IS_OPEN_DRAIN | GPIO_IS_OPEN_SOURCE |
        GPIO_IS_BIAS_PULL_UP | GPIO_IS_BIAS_PULL_DOWN | GPIO_IS_BIAS_DISABLE)

    GPIO_EDGES = {
        'both':    rgpio.BOTH_EDGES,
        'rising':  rgpio.RISING_EDGE,
        'falling': rgpio.FALLING_EDGE,
        }

    GPIO_EDGES_NAMES = {v: k for (k, v) in GPIO_EDGES.items()}

    def __init__(self, factory, number):
        super(RGPIOPin, self).__init__(factory, number)
        self._pwm = None
        self._bounce = None
        self._callback = None
        self._edges = rgpio.BOTH_EDGES
        self.factory._connection.gpio_claim_input(
            self.factory._handle, self.number, rgpio.SET_BIAS_DISABLE)

    def _get_function(self):
        mode = self.factory._connection.gpio_get_mode(
            self.factory._handle, self.number)
        return ['input', 'output'][bool(mode & self.GPIO_IS_OUT)]

    def _set_function(self, value):
        # XXX Disable existing callbacks first?
        try:
            {
                'input': self.factory._connection.gpio_claim_input,
                'output': self.factory._connection.gpio_claim_output,
            }[value](self.factory._handle, self.number)
        except KeyError:
            raise PinInvalidFunction('invalid function "%s" for pin %r' % (value, self))

    def _get_state(self):
        if self._pwm:
            return self._pwm[1] / 100
        else:
            return bool(self.factory._connection.gpio_read(
                self.factory._handle, self.number))

    def _set_state(self, value):
        if self._pwm:
            freq, duty = self._pwm
            self._pwm = (freq, min(100, max(0, int(value * 100))))
            self.factory._connection.tx_pwm(
                self.factory._handle, self.number, *self._pwm)

    def _get_pull(self):
        mode = self.factory._connection.gpio_get_mode(
            self.factory._handle, self.number)
        if mode & self.GPIO_IS_BIAS_PULL_UP:
            return 'up'
        elif mode & self.GPIO_IS_BIAS_PULL_DOWN:
            return 'down'
        else:
            return 'floating'

    def _set_pull(self, value):
        if self.function != 'input':
            raise PinFixedPull('cannot set pull on non-input pin %r' % self)
        if value != 'up' and self.factory.pi_info.pulled_up(repr(self)):
            raise PinFixedPull('%r has a physical pull-up resistor' % self)
        try:
            flags = {
                'up':       rgpio.SET_BIAS_PULL_UP,
                'down':     rgpio.SET_BIAS_PULL_DOWN,
                'floating': rgpio.SET_BIAS_DISABLE,
            }[value]
        except KeyError:
            raise PinInvalidPull('invalid pull "%s" for pin %r' % (value, self))
        else:
            # Simply calling gpio_claim_input is insufficient to change the
            # line flags on a pin; it needs to be freed and re-claimed
            self.factory._connection.gpio_free(
                self.factory._handle, self.number)
            self.factory._connection.gpio_claim_input(
                self.factory._handle, self.number, flags)

    def _get_frequency(self):
        if self._pwm:
            freq, duty = self._pwm
            return freq
        else:
            return None

    def _set_frequency(self, value):
        if not self._pwm and value is not None and value > 0:
            if self.function != 'output':
                raise PinPWMFixedValue('cannot start PWM on pin %r' % self)
            self.factory._connection.tx_pwm(
                self.factory._handle, self.number, value, 0)
            self._pwm = (value, 0)
        elif self._pwm and value is not None and value > 0:
            freq, duty = self._pwm
            self.factory._connection.tx_pwm(
                self.factory._handle, self.number, value, duty)
            self._pwm = (value, duty)
        elif self._pwm and (value is None or value == 0):
            self.factory._connection.tx_pwm(
                self.factory._handle, self.number, 0, 0)
            self._pwm = None

    def _get_bounce(self):
        return None if not self._bounce else self._bounce / 1000000

    def _set_bounce(self, value):
        if value is None:
            value = 0
        elif value < 0:
            raise PinInvalidBounce('bounce must be 0 or greater')
        value = int(value * 1000000)
        self.factory._connection.gpio_set_debounce_micros(
            self.factory._handle, self.number, value)
        self._bounce = value

    def _get_edges(self):
        return self.GPIO_EDGES_NAMES[self._edges]

    def _set_edges(self, value):
        f = self.when_changed
        self.when_changed = None
        try:
            self._edges = self.GPIO_EDGES[value]
        finally:
            self.when_changed = f

    def _call_when_changed(self, chip, gpio, level, ticks):
        super(RGPIOPin, self)._call_when_changed(ticks / 1000000000, level)

    def _enable_event_detect(self):
        self.factory._connection.gpio_claim_alert(
            self.factory._handle, self.number, self._edges,
            self.factory._connection.gpio_get_mode(
                self.factory._handle, self.number) & self.GPIO_LINE_FLAGS_MASK)
        self._callback = self.factory._connection.callback(
            self.factory._handle, self.number, self._edges,
            self._call_when_changed)

    def _disable_event_detect(self):
        if self._callback is not None:
            self._callback.cancel()
            self._callback = None
        self.factory._connection.gpio_claim_input(
            self.factory._handle, self.number,
            self.factory._connection.gpio_get_mode(
                self.factory._handle, self.number) & self.GPIO_LINE_FLAGS_MASK)


class RGPIOHardwareSPI(SPI):
    """
    Hardware SPI implementation for the `rgpio`_ library. Uses the ``spi_*``
    functions from the rgpio API.

    .. _lgpio: http://abyz.me.uk/lg/py_rgpio.html
    """
    def __init__(self, clock_pin, mosi_pin, miso_pin, select_pin, pin_factory):
        port, device = spi_port_device(
            clock_pin, mosi_pin, miso_pin, select_pin)
        self._port = port
        self._device = device
        self._baud = 500000
        self._spi_flags = 0
        self._handle = None
        super(RGPIOHardwareSPI, self).__init__(pin_factory=pin_factory)
        to_reserve = {clock_pin, select_pin}
        if mosi_pin is not None:
            to_reserve.add(mosi_pin)
        if miso_pin is not None:
            to_reserve.add(miso_pin)
        self.pin_factory.reserve_pins(self, *to_reserve)
        self._handle = self.pin_factory._connection.spi_open(
            self._port, self._device, self._baud, self._spi_flags)

    def _conflicts_with(self, other):
        return not (
            isinstance(other, RGPIOHardwareSPI) and
            (self.pin_factory.host, self._port, self._device) !=
            (other.pin_factory.host, other._port, other._device)
            )

    def close(self):
        if not self.closed:
            self.pin_factory._connection.spi_close(self._handle)
        self._handle = None
        self.pin_factory.release_all(self)
        super(RGPIOHardwareSPI, self).close()

    @property
    def closed(self):
        return self._handle is None

    def __repr__(self):
        try:
            self._check_open()
            return 'SPI(port=%d, device=%d)' % (self._port, self._device)
        except DeviceClosed:
            return 'SPI(closed)'

    def _get_clock_mode(self):
        return self._spi_flags

    def _set_clock_mode(self, value):
        self._check_open()
        if not 0 <= value < 4:
            raise SPIInvalidClockMode("%d is not a valid SPI clock mode" % value)
        self.pin_factory._connection.spi_close(self._handle)
        self._spi_flags = value
        self._handle = self.pin_factory._connection.spi_open(
            self._port, self._device, self._baud, self._spi_flags)

    def _get_rate(self):
        return self._baud

    def _set_rate(self, value):
        self._check_open()
        value = int(value)
        self.pin_factory._connection.spi_close(self._handle)
        self._baud = value
        self._handle = self.pin_factory._connection.spi_open(
            self._port, self._device, self._baud, self._spi_flags)

    def read(self, n):
        self._check_open()
        count, data = self.pin_factory._connection.spi_read(self._handle, n)
        if count < 0:
            raise IOError('SPI transfer error %d' % count)
        return [int(b) for b in data]

    def write(self, data):
        self._check_open()
        count = self.pin_factory._connection.spi_write(self._handle, data)
        if count < 0:
            raise IOError('SPI transfer error %d' % count)
        return len(data)

    def transfer(self, data):
        self._check_open()
        count, data = self.pin_factory._connection.spi_xfer(self._handle, data)
        if count < 0:
            raise IOError('SPI transfer error %d' % count)
        return [int(b) for b in data]


class RGPIOHardwareSPIShared(SharedMixin, RGPIOHardwareSPI):
    @classmethod
    def _shared_key(cls, clock_pin, mosi_pin, miso_pin, select_pin, pin_factory):
        return (pin_factory.host, clock_pin, select_pin)
