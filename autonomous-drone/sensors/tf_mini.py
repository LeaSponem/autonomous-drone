import smbus2
from range_sensors import RangeSensor

DEFAULT_CRITICAL_DISTANCE = 100


class TFMiniPlus(RangeSensor):
    """Class for a specific range sensor : the TFMini Plus"""
    def __init__(self, address, angle=0, critical_distance=DEFAULT_CRITICAL_DISTANCE):
        """Constructor : can take as input the critical distance of the sensor under which we detect an obstacle"""
        RangeSensor.__init__(self, critical_distance)  # Calls the constructor of the parent class
        self._name = "Lidar"
        self.angle = angle  # Angle between the drone front axis and the lidar axis
        self._address = address
        self._time_between_readings = 0.02  # Ask a reading every 20 ms

    def read_distance(self):
        """Read the distance return by the TFMiniPlus"""
        write = smbus2.i2c_msg.write(self._address, [0x5a, 0x05, 0x00, 0x01, 0x60])
        read = smbus2.i2c_msg.read(self._address, 8)
        with smbus2.SMBus(1) as bus:
            bus.i2c_rdwr(write, read)
            data = list(read)
        if data[1] == 89 and data[2] == 89:
            strength = data[5] + data[6] * 256
            if strength < 100:  # signal not strong enough : return 0
                self.set_distance(0)
                return True
            if 100 <= strength <= 65536:
                self.set_distance(data[3] + data[4] * 256)
                return True
        return False
        
    def lidar_reading(self):
        """
        Check if the time since the last reading is superior to the fixed time between two readings
        """
        return self.time_since_last_reading() > self._time_between_readings
