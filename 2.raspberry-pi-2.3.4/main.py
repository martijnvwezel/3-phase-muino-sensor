"""
Python library for the VEML6030 sensor.

"""

from enum import IntEnum, IntFlag

ADDRESS = [0x48, 0x10]

SETTING_REG = 0x00
H_THRESH_REG = 0x01
L_THRESH_REG = 0x02
POWER_SAVE_REG = 0x03
AMBIENT_LIGHT_DATA_REG = 0x04
WHITE_LIGHT_DATA_REG = 0x05
INTERRUPT_REG = 0x06


class Gain(IntEnum):
    x1 = 0b00    # 1x Gain
    x2 = 0b01    # 2x Gain
    x1_8 = 0b10  # 1/8 Gain
    x1_4 = 0b11  # 1/4 Gain


class IntegrationTime(IntEnum):
    ms25 = 0b1100
    ms50 = 0b1000
    ms100 = 0b0000
    ms200 = 0b0001
    ms400 = 0b0010
    ms800 = 0b0011


class ProtectNum(IntEnum):
    n1 = 0b00
    n2 = 0b01
    n4 = 0b10
    n8 = 0b11


class PowerSaveMode(IntEnum):
    m1 = 0b00  # Fastest, most current
    m2 = 0b01
    m3 = 0b10
    m4 = 0b11  # Slowest, least current


class Interrupt(IntFlag):
    High = 0b01
    Low = 0b10


_lux_coeff = {
    IntegrationTime.ms800: {
        Gain.x2: .0036,
        Gain.x1: .0072,
        Gain.x1_4: .0288,
        Gain.x1_8: .0576
    },
    IntegrationTime.ms400: {
        Gain.x2: .0072,
        Gain.x1: .0144,
        Gain.x1_4: .0576,
        Gain.x1_8: .1152
    },
    IntegrationTime.ms200: {
        Gain.x2: .0144,
        Gain.x1: .0288,
        Gain.x1_4: .1152,
        Gain.x1_8: .2304
    },
    IntegrationTime.ms100: {
        Gain.x2: .0288,
        Gain.x1: .0576,
        Gain.x1_4: .2304,
        Gain.x1_8: .4608
    },
    IntegrationTime.ms50: {
        Gain.x2: .0576,
        Gain.x1: .1152,
        Gain.x1_4: .4608,
        Gain.x1_8: .9216
    },
    IntegrationTime.ms25: {
        Gain.x2: .1152,
        Gain.x1: .2304,
        Gain.x1_4: .9216,
        Gain.x1_8: 1.8432
    }
}

def set_bits(register: int, value, index, length=1):
    """
    Set selected bits in register and return new value

    :param register: Input register value
    :type register: int
    :param value: Bits to write to register
    :type value: int
    :param index: Start index (from right)
    :type index: int
    :param length: Number of bits (default 1)
    :type length: int
    :return: Output new register value
    :rtype: int
    """
    mask = (1 << length)-1
    register = register & ~(mask << index)
    register = register | (mask & value) << index
    return register


def get_bits(register, index, length=1):
    """
    Get selected bit(s) from register while masking out the rest.
    Returns as boolean if length==1

    :param register: Register value
    :type register: int
    :param index: Start index (from right)
    :type index: int
    :param length: Number of bits (default 1)
    :type length: int
    :return: Selected bit(s)
    :rtype: Union[int, bool]
    """
    result = (register >> index) & ((1 << length) - 1)
    if length == 1:
        return result == 1
    return result


MAX_LUX = 120000
MIN_LUX = 0


class VEML6030(object):

    def __init__(self, bus, address=ADDRESS[0]):
        """
        Object for connecting to VEML6030 sensor via i2c.

        :param bus: i2c bus to connect to VEML6030 sensor. Must be compatible with smbus.SMBus object
        :type bus: smbus.SMBus
        :param address: i2c address of VEML6030 sensor
        :type address: int
        """

        # Must be one of two valid addresses for this board
        if address not in ADDRESS:
            raise ValueError("Invalid Address: {0:#x}".format(address))
        self.address = address

        # Make sure the bus input is valid
        if bus is None:
            raise ValueError("Invalid bus, must pass in SMBus object")
        self.bus = bus

        # Initialize sensor with default values
        self.power_on()
        self.set_gain(Gain.x2)
        self.set_integration_time(IntegrationTime.ms800)

    def set_gain(self, gain):
        """
        Set gain for VEML6030

        :param gain:
        :type gain: Gain
        :return:
        """
        self._write_bits_to_register(SETTING_REG, gain, 11, 2)

    def get_gain(self):
        """
        Get gain for VEML6030

        :return: Set gain value
        :rtype: Gain
        """
        return Gain(self._read_bits_from_register(SETTING_REG, 11, 2))

    def set_integration_time(self, time):
        """
        Set integration time for VEML6030

        :param time: Integration Time setting
        :type time: IntegrationTime
        """
        self._write_bits_to_register(SETTING_REG, time, 6, 4)

    def get_integration_time(self):
        """
        Get set integration time for VEML6030

        :return: Integration Time setting
        :rtype: IntegrationTime
        """
        return IntegrationTime(self._read_bits_from_register(SETTING_REG, 6, 4))

    def set_protect(self, protect):
        """
        Set persistence protection for VEML6030.

        :param protect: Input protection for VEML6030
        :type protect: ProtectNum
        """
        self._write_bits_to_register(SETTING_REG, protect, 4, 2)

    def get_protect(self):
        """
        Get persistence protection for VEML6030.

        :return: Input protection for VEML6030
        :rtype: ProtectNum
        """
        return ProtectNum(self._read_bits_from_register(SETTING_REG, 4, 2))

    def enable_interrupt(self):
        """
        Enable interrupts for VEML6030
        """
        self.set_interrupt_setting(True)

    def disable_interrupt(self):
        """
        Disable interrupts for VEML6030
        """
        self.set_interrupt_setting(False)

    def set_interrupt_setting(self, enable):
        """
        Enable or disable interrupts for VEML6030

        :param enable: Enable Interrupts setting
        :type enable: bool
        """
        self._write_bits_to_register(SETTING_REG, enable, 1)

    def get_interrupt_setting(self):
        """
        Get Interrupt setting for VEML6030

        :return: Interrupts setting
        :rtype: bool
        """
        return self._read_bits_from_register(SETTING_REG, 1)

    def shutdown(self):
        """
        Powers off VEML6030.
        The current value will be saved for reading.
        No further readings will be taken until "power_on" is called.
        """
        self._write_bits_to_register(SETTING_REG, True, 0)

    def power_on(self):
        """
        Powers on VEML6030 from off
        """
        self._write_bits_to_register(SETTING_REG, False, 0)

    def enable_power_save(self):
        """
        Enables power save mode
        """
        self._write_bits_to_register(POWER_SAVE_REG, True, 0)

    def disable_power_save(self):
        """
        Disables power save mode
        """
        self._write_bits_to_register(POWER_SAVE_REG, False, 0)

    def set_power_save_mode(self, mode):
        """
        Sets the mode for power saving, see Datasheet pg. 9 for details

        :param mode: Power saving mode
        :type mode: PowerSaveMode
        """
        self._write_bits_to_register(POWER_SAVE_REG, mode, 1, 2)

    def get_power_save_mode(self):
        """
        Sets the mode for power saving

        :return: Set power saving mode
        :rtype: PowerSaveMode
        """
        return PowerSaveMode(self._read_bits_from_register(POWER_SAVE_REG, 1, 2))

    def get_interrupt(self):
        """
        Gets status of active interrupts, returns Interrupt.0 if empty

        :return: Interrupts triggered (if any)
        :rtype: Interrupt
        """
        return Interrupt(self._read_bits_from_register(INTERRUPT_REG, 14, 2))

    def set_low_interrupt_thresh(self, lux):
        """
        Sets the threshold for the low interrupt for the light.
        Note: Values out of the range [0, 120000] will raise a ValueError

        :param lux: Lux value for threshold (uncompensated)
        :type lux: float
        """
        if lux < MIN_LUX or lux > MAX_LUX:
            raise ValueError(f"Lux value {lux} out of range")

        count = self._calculate_dc(lux)
        self._write_register(L_THRESH_REG, count)

    def get_low_interrupt_thresh(self):
        """
        Get the low lux threshold for ambient light readings

        :return: Lux value for threshold (uncompensated)
        :rtype: float
        """
        count = self._read_register(L_THRESH_REG)
        return self._calculate_lux(count)

    def set_high_interrupt_thresh(self, lux):
        """
        Sets the threshold for the high interrupt for the light.
        Note: Values out of the range [0, 120000] will raise a ValueError

        :param lux: Lux value for threshold (uncompensated)
        :type lux: float
        """
        if lux < MIN_LUX or lux > MAX_LUX:
            raise ValueError(f"Lux value {lux} out of range")

        count = self._calculate_dc(lux)
        self._write_register(H_THRESH_REG, count)

    def get_high_interrupt_thresh(self):
        """
        Get the high lux threshold for ambient light readings

        :return: Lux value for threshold (uncompensated)
        :rtype: float
        """
        count = self._read_register(H_THRESH_REG)
        return self._calculate_lux(count)

    def read_light(self, compensate=True):
        """
        Reads ambient light reading from VEML6030 in lux

        :param compensate: Whether to compensate lux values over 1000.
        Only set to false if gathering values for thresholds
        :type compensate: bool
        :return: Ambient light read by sensor
        :rtype: float
        """
        count = self._read_register(AMBIENT_LIGHT_DATA_REG)
        lux = self._calculate_lux(count)

        # Compensate lux if high enough
        if lux > 1000 and compensate:
            lux = self._lux_compensation(lux)

        return lux

    def read_white_light(self, compensate=True):
        """
        Reads white light reading from VEML6030 in lux.

        :param compensate: Whether to compensate lux values over 1000.
        Only set to false if gathering values for thresholds
        :type compensate: bool
        :return: White light read by sensor
        :rtype: float
        """
        count = self._read_register(WHITE_LIGHT_DATA_REG)
        lux = self._calculate_lux(count)

        # Compensate lux if high enough
        if lux > 1000 and compensate:
            lux = self._lux_compensation(lux)

        return lux

    def _lux_compensation(self, lux):
        """
        Compensates lux when raw lux reading is >1000. Only used when gathering readings.
        There is no reverse compensation available.

        :param lux: Raw lux value (only use if lux > 1000)
        :type lux: float
        :return: Compensated lux
        :rtype: float
        """
        # TODO: Create reverse lux compensation

        return (0.00000000000060135 * lux**4
                - 0.0000000093924 * lux**3
                + 0.000081488 * lux**2
                + 1.0023 * lux)

    def _calculate_lux(self, count):
        """
        Calculate lux from digital count bits (based on current gain and integration time)

        :param count:
        :type count: int
        :return:
        :rtype: float
        """
        g = self.get_gain()
        it = self.get_integration_time()

        return count*_lux_coeff[it][g]

    def _calculate_dc(self, lux):
        """
        Calculate digital count that corresponds to lux value (based on current gain and integration time)

        :param lux:
        :type lux: float
        :return:
        :rtype: int
        """
        g = self.get_gain()
        it = self.get_integration_time()

        return int(round(lux/_lux_coeff[it][g]))

    def _read_bits_from_register(self, register, index, length=1):
        """
        Read value from register on VEML6030

        :param register: Register address to acquire
        :type register: int
        :param index: position to pull bits from
        :type index: int
        :param length: number of bits to grab
        :type length: int
        :return: Bits from register
        :rtype: Union[int, bool]
        """
        return get_bits(self._read_register(register), index, length)

    def _write_bits_to_register(self, register, value, index, length=1):
        """
        Write value to register on VEML6030

        :param register: Address of register you are writing to
        :type register: int
        :param value: Bits to write to register
        :type value: Union[int, bool]
        :param index: position to write bits to
        :type index: int
        :param length: number of bits to grab
        :type length: int
        """
        self._write_register(register, set_bits(self._read_register(register), value, index, length))

    def _read_register(self, register):
        """
        Read value from register on VEML6030

        :param register: Register address to acquire
        :type register: int
        :return: Word in register
        :rtype: int
        """
        return self.bus.read_word_data(self.address, register)

    def _write_register(self, register, value):
        """
        Write value to register on VEML6030

        :param register: Address of register you are writing to
        :type register: int
        :param value: Word to write to register
        :type value: int
        """
        self.bus.write_word_data(self.address, register, value)



class IOEXTENDER():
    PI4IO_CHIP_ID = 0x1
    PI4IO_IO_DIRECTION = 0x3
    PI4IO_OUTPUT = 0x5
    PI4IO_OUTPUT_HI_IMPEDANCE = 0x7
    PI4IO_INPUT_STATUS = 0xF # This register is not writable
    PI4IO_INTERRUPT_STATUS = 0x13
    PI4IO_CHIP_ID_VAL = 0xA0
    PI4IO_CHIP_ID_MASK = 0xFC # This register is not writable
    PI4IO_I2C_ADDR = 0x43
    LED = 1<<6

    def __init__(self, bus):
        if bus is None:
            raise ValueError("Invalid bus, must pass in SMBus object")
        self.bus = bus
        
        self.init()
        
    def init(self):
        self._write_register(0x01, 1)
        sleep(0.001)
        print(f"Device ID {self.bus.read_byte_data(self.PI4IO_I2C_ADDR, self.PI4IO_I2C_ADDR)}")

        # * Setup pins # TODO fix the following code
        self.bus.write_byte_data(self.PI4IO_I2C_ADDR, self.PI4IO_IO_DIRECTION, 120)     # (1 << 3) | (1 << 4) | (1 << 5) | self.LED
        self.bus.write_byte_data(self.PI4IO_I2C_ADDR, self.PI4IO_OUTPUT,  self.LED)    
        self.bus.write_byte_data(self.PI4IO_I2C_ADDR, self.PI4IO_OUTPUT_HI_IMPEDANCE,  ~120)    

        print(f"IODIR: {self._read_register(0x03)}")
        print(f"Output: {self._read_register(self.PI4IO_OUTPUT)}")
        print(f"HighZ: {self._read_register(self.PI4IO_OUTPUT)}")



    def _write_bits_to_register(self, register, value, index, length=1):
        """
        Write value to register on VEML6030

        :param register: Address of register you are writing to
        :type register: int
        :param value: Bits to write to register
        :type value: Union[int, bool]
        :param index: position to write bits to
        :type index: int
        :param length: number of bits to grab
        :type length: int
        """
        self._write_register(register, set_bits(self._read_register(register), value, index, length))

    def _read_register(self, register):
        """
        Read value from register on VEML6030

        :param register: Register address to acquire
        :type register: int
        :return: Word in register
        :rtype: int
        """
        return self.bus.read_word_data(self.PI4IO_I2C_ADDR, register)

    def _write_register(self, register, value):
        """
        Write value to register on VEML6030

        :param register: Address of register you are writing to
        :type register: int
        :param value: Word to write to register
        :type value: int
        """
        self.bus.write_byte_data(self.PI4IO_I2C_ADDR, register, value)


    def select_read_sensor(self, sensor_id):
        if sensor_id == 0:
            self._write_register(self.PI4IO_OUTPUT, (1 << 3) | self.LED)
        elif sensor_id == 1:
            self._write_register(self.PI4IO_OUTPUT, (1 << 4) | self.LED)
        elif sensor_id == 2:
            self._write_register(self.PI4IO_OUTPUT, (1 << 5) | self.LED)
        else:
            return -1

        return self._read_register(1)


 
def read_light_sensor(VEML6030, IOEXTENDER, sensor_id):
    IOEXTENDER.select_read_sensor(sensor_id)
    return VEML6030._read_register(AMBIENT_LIGHT_DATA_REG)

import smbus
from time import sleep

bus = smbus.SMBus(1)  # For Raspberry Pi 
sensor = VEML6030(bus, ADDRESS[1])
ioextender = IOEXTENDER(bus)

sensor.set_gain(Gain.x2)
sensor.set_integration_time(IntegrationTime.ms100)
while True:
    print(f"{read_light_sensor(sensor, ioextender, 0)}\t{read_light_sensor(sensor, ioextender, 1)}\t{read_light_sensor(sensor, ioextender, 2)}")
    sleep(0.2)