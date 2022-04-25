from tf_mini import TFMiniPlus


class DroneSensors(object):
    def __init__(self, lidar_address, lidar_angle, critical_distance_lidar=50):
        self._sensors_number = len(lidar_address)
        self._critical_distance_lidar = critical_distance_lidar
        self.lidar_sensors = self.init_lidar_sensors(lidar_address, lidar_angle)

    def init_lidar_sensors(self, lidar_address, lidar_angle):
        lidar_sensors = []
        if lidar_address is not None:
            index = dict(zip(lidar_address, lidar_angle))
            for address, angle in index.items():
                self.lidar_sensors.append(TFMiniPlus(address, angle, self._critical_distance_lidar))
        return lidar_sensors
