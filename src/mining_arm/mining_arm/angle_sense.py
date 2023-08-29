#!/usr/bin/env python
import smbus
import time

# Register and other configuration values:
ADS1x15_DEFAULT_ADDRESS = 0x48
ADS1x15_POINTER_CONVERSION = 0x00
ADS1x15_POINTER_CONFIG = 0x01
ADS1x15_POINTER_LOW_THRESHOLD = 0x02
ADS1x15_POINTER_HIGH_THRESHOLD = 0x03
ADS1x15_CONFIG_OS_SINGLE = 0x8000
ADS1x15_CONFIG_MUX_OFFSET = 12
# Mapping of gain values to config register values.
ADS1x15_CONFIG_GAIN = {
    2/3: 0x0000,
    1:   0x0200,
    2:   0x0400,
    4:   0x0600,
    8:   0x0800,
    16:  0x0A00
}
ADS1x15_CONFIG_MODE_CONTINUOUS = 0x0000
ADS1x15_CONFIG_MODE_SINGLE = 0x0100
# Mapping of data/sample rate to config register values for ADS1015 (faster).
ADS1015_CONFIG_DR = {
    128:   0x0000,
    250:   0x0020,
    490:   0x0040,
    920:   0x0060,
    1600:  0x0080,
    2400:  0x00A0,
    3300:  0x00C0
}
# Mapping of data/sample rate to config register values for ADS1115 (slower).
ADS1115_CONFIG_DR = {
    8:    0x0000,
    16:   0x0020,
    32:   0x0040,
    64:   0x0060,
    128:  0x0080,
    250:  0x00A0,
    475:  0x00C0,
    860:  0x00E0
}
ADS1x15_CONFIG_COMP_WINDOW = 0x0010
ADS1x15_CONFIG_COMP_ACTIVE_HIGH = 0x0008
ADS1x15_CONFIG_COMP_LATCHING = 0x0004
ADS1x15_CONFIG_COMP_QUE = {
    1: 0x0000,
    2: 0x0001,
    4: 0x0002
}
ADS1x15_CONFIG_COMP_QUE_DISABLE = 0x0003

arm_length = 0.569
drum_radius = 0.195


class AngleSensor:
    def __init__(self, gain=2/3, bus=1):
        self.bus = smbus.SMBus(bus)
        self.gain = gain

        self.start_adc(0)

    def get_angle(self) -> float:
        return self.computeDegrees(
            5,
            self.computeVolts(self.get_last_result())
        )

    def _read(self, mux, data_rate, mode):
        """
        Perform an ADC read with the provided mux, gain, data_rate, and mode.

        Returns the signed integer result of the read.
        """
        # Go out of power-down mode for conversion.
        config = ADS1x15_CONFIG_OS_SINGLE
        # Specify mux value.
        config |= (mux & 0x07) << ADS1x15_CONFIG_MUX_OFFSET
        # Validate the passed in gain and then set it in the config.
        if self.gain not in ADS1x15_CONFIG_GAIN:
            raise ValueError('Gain must be one of: 2/3, 1, 2, 4, 8, 16')
        config |= ADS1x15_CONFIG_GAIN[self.gain]
        # Set the mode (continuous or single shot).
        config |= mode
        # Get the default data rate if none is specified (default differs
        # between ADS1015 and ADS1115).
        if data_rate is None:
            data_rate = 128
        # Set the data rate (this is controlled by the subclass as it differs
        # between ADS1015 and ADS1115).
        config |= self._data_rate_config(data_rate)
        config |= ADS1x15_CONFIG_COMP_QUE_DISABLE  # Disble comparator mode.
        # Send the config value to start the ADC conversion.
        # Explicitly break the 16-bit value down to a big endian pair of bytes.
        self.bus.write_i2c_block_data(
            ADS1x15_DEFAULT_ADDRESS,
            ADS1x15_POINTER_CONFIG,
            [(config >> 8) & 0xFF, config & 0xFF]
        )
        # Wait for the ADC sample to finish based on the sample rate plus a
        # small offset to be sure (0.1 millisecond).
        time.sleep(1.0/data_rate+0.0001)
        # Retrieve the result.
        result = self.bus.read_i2c_block_data(
            ADS1x15_DEFAULT_ADDRESS, ADS1x15_POINTER_CONVERSION, 2)
        return self._conversion_value(result[1], result[0])

    def start_adc(self, channel, data_rate=None):
        """
        Start continuous ADC conversions on the specified channel (0-3).

        Will return an initial conversion result, then call the
        get_last_result() function to read the most recent conversion
        result. Call stop_adc() to stop conversions.
        """
        assert 0 <= channel <= 3, 'Channel must be a value within 0-3!'
        # Start continuous reads and set the mux value to the channel plus
        # the highest bit (bit 3) set.
        return self._read(
            channel + 0x04,
            data_rate,
            ADS1x15_CONFIG_MODE_CONTINUOUS
        )

    def get_last_result(self):
        """
        Read the last conversion result when in continuous conversion mode.

        Will return a signed integer value.
        """
        # Retrieve the conversion register value, convert to a signed int, and
        # return it.
        result = self.bus.read_i2c_block_data(
            ADS1x15_DEFAULT_ADDRESS, ADS1x15_POINTER_CONVERSION, 2)
        return self._conversion_value(result[1], result[0])

    def _data_rate_config(self, data_rate):
        if data_rate not in ADS1115_CONFIG_DR:
            raise ValueError(
                'Data rate must be one of: 8, 16, 32, 64, 128, 250, 475, 860')
        return ADS1115_CONFIG_DR[data_rate]

    def _conversion_value(self, low, high):
        # Convert to 16-bit signed value.
        value = ((high & 0xFF) << 8) | (low & 0xFF)
        # Check for sign bit and turn into a negative value if set.
        if value & 0x8000 != 0:
            value -= 1 << 16
        return value

    def computeVolts(self, counts):
        fsRange = 0

        if self.gain == 2/3:
            fsRange = 6.144
        elif self.gain == 1:
            fsRange = 4.096
        elif self.gain == 2:
            fsRange = 2.048
        elif self.gain == 4:
            fsRange = 1.024
        elif self.gain == 8:
            fsRange = 0.512
        elif self.gain == 16:
            fsRange = 0.256
        return counts * (fsRange / (32768))

    def computeDegrees(self, vdd, output):
        upper_bound = vdd*.9
        lower_bound = vdd*.1
        slope = (270-90)/(upper_bound-lower_bound)
        angle = (output-lower_bound)*slope+57.8
        return angle
