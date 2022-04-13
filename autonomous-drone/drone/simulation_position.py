import numpy as np
import time


class FrameConverter:
    def __init__(self, ref_latitude):
        self._ref_lat = ref_latitude

    def convert_global_to_metric(self, delta_lat, delta_long):
        """
        Converts a distance measured in latitude and longitude difference metric distance along the North and East direction.
        This conversion depends on which latitude the drone is located
        Params :
        - currentLat : the current latitude at which the drone is operating (in degrees)
        - deltaLat : the latitude distance that needs to be converted (in degrees)
        - deltaLon : the longitude distance that needs to be converted (in degrees)

        Outputs: returns the converted distances in mm along the North and East directions
        """
        latitude_conversion_coef, longitude_conversion_coef = self.get_geo_to_mm_coef()
        return delta_lat * latitude_conversion_coef, delta_long * longitude_conversion_coef

    def convert_metric_to_global(self, delta_north, delta_east):
        """
        Converts a distance measured in millimeters along the North and East directions into latitude and longitude distances.
        This conversion depends on which latitude the drone is located
        Params :
        - currentLat : the current latitude at which the drone is operating (in degrees)
        - deltaNorth : the distance along the North directions that needs to be converted to latitude (in mm)
        - deltaEast : the distance along the East directions that needs to be converted to longitude (in mm)

        Outputs: returns the converted distances in latitude and longitude in degrees
        """
        latitude_to_mm_coef, longitude_to_mm_coef = self.get_geo_to_mm_coef()
        return delta_north / latitude_to_mm_coef, delta_east / longitude_to_mm_coef

    def get_geo_to_mm_coef(self):
        # reference latitude converted in radians
        radian_latitude = self.deg_to_rad(self._ref_lat)

        # factor converting 1E-7 degrees of latitude to the equivalent in mm at a given latitude
        latitude_to_mm_coef = (111132.92 - 559.82 * np.cos(2 * radian_latitude) + 1.175 * np.cos(4 * radian_latitude) - 0.0023 * np.cos(6 * radian_latitude)) * 1000

        # factor converting 1E-7 degrees of latitude to the equivalent in mm at a given latitude
        longitude_to_mm_coef = (111412.84 * np.cos(radian_latitude) - 93.5 * np.cos(3 * radian_latitude) - 0.118 * np.cos(5 * radian_latitude)) * 1000

        return latitude_to_mm_coef, longitude_to_mm_coef

    def deg_to_rad(self, degrees):
        return degrees * np.pi / 180

    def rad_to_deg(self, radians):
        return int(radians * 180 / np.pi)


class SimulationPosition:
    def __init__(self, ref_lat, ref_long, angle_north_x_axes):
        self._time_last_update_position = time.time()
        self._ref_lat = ref_lat
        self._ref_long = ref_long
        self._angle_north_x_axes = angle_north_x_axes
        self._newPosition = False
        self._frame_converter = FrameConverter(self._ref_lat)

    def get_position(self, drone_location):
        """
        For testing purposes only : recreates a X,Y position relative to a point on the map to simulate a local
        positioning system
        """

        delta_lat = drone_location.lat - self._ref_lat
        delta_long = drone_location.lon - self._ref_long
        x_north, y_east = self._frame_converter.convert_global_to_metric(delta_lat, delta_long)
        x = x_north * np.cos(self._angle_north_x_axes * np.pi / 180) + y_east * np.sin(self._angle_north_x_axes * np.pi / 180)
        y = x_north * np.sin(self._angle_north_x_axes * np.pi / 180) - y_east * np.cos(self._angle_north_x_axes * np.pi / 180)

        return int(x), int(y)

    def update_position(self):
        if time.time() - self._time_last_update_position > 100:
            self._time_last_update_position = time.time()
            return True
        else:
            return False
