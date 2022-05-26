import time
import sys
from inspection_drone import InspectionDrone
from simulation_position import SimulationPosition
sys.path.insert(0, '../sensors')
from virtual_tf_mini import VirtualTFMiniPlus
from virtual_drone_sensors import VirtualThreeLidarSensorsDetection


class VirtualDrone(InspectionDrone):
    """
    Specific class for a virtual drone used on a simulator
    Deprecated class from InspectionDrone with parameters and methods for virtual positions
    """
    def __init__(self, connection_string, baudrate, two_way_switches, three_way_switches,
                 lidar_angle, critical_distance_lidar=100):
        InspectionDrone.__init__(self, connection_string, baudrate, two_way_switches, three_way_switches,
                                 lidar_angle=lidar_angle, critical_distance_lidar=critical_distance_lidar)
        # Drone virtual coordinates
        self._drone_x = 0
        self._drone_y = 0
        # Drone GPS coordinates
        self._location = self.vehicle.location.global_relative_frame
        # Initial angle between X axis and the North
        self._update_yaw()
        self._initial_angle = 90 + self._yaw
        # Init local frame for simulated positions
        self._local_frame = self._init_local_frame()
        self.lidar = VirtualThreeLidarSensorsDetection(lidar_angle, critical_distance_lidar)

    def _init_local_frame(self):
        """
        Initialize the local frame used to create virtual drone positions
        The positions are given relatively to the initial latitude and longitude of the drone
        """
        local_frame = SimulationPosition(self._location.lat, self._location.lon, self._initial_angle)
        return local_frame

    def _update_location(self):
        """
        Update the drone GPS coordinates
        Return (latitude, longitude, altitude)
        """
        self._location = self.vehicle.location.global_relative_frame

    def _update_virtual_position(self):
        """
        Convert the GPS coordinates to x and y coordinates relatively to the drone home position
        Convert the coordinates in centimeters
        """
        self._update_location()
        self._drone_x, self._drone_y = self._local_frame.get_position(self._location)
        self._drone_x *= 100
        self._drone_y *= 100

    def get_angle(self):
        """Return the angle between the drone direction and the X axis"""
        self._update_yaw()
        return self._initial_angle - self._yaw

    def get_sensor_angle(self, sensor_angle):
        """Return the angle between the sensor axis and the X axis"""
        angle = self.get_angle() - sensor_angle
        if angle > 360:
            angle -= 360
        return angle

    def get_virtual_position(self):
        """
        Update and return the drone virtual position
        """
        self._update_virtual_position()
        return self._drone_x, self._drone_y

    def update_detection(self, use_lidar=True, debug=False, walls=None):
        """
        Read the distance returned by the sensor and return if an obstacle is detected
        The distance is read relatively to the input list of obstacles
        """
        self._update_virtual_position()
        if self.lidar.read_distance(self._drone_x, self._drone_y, self.get_angle(), walls) and debug:
            print("Lidar range:" + str(self.lidar.get_distance()))
        if use_lidar and self.lidar.critical_distance_reached():
            if self.obstacle_detected():
                self._time_last_obstacle_detected = time.time()
            self._obstacle_detected = True
        else:
            self._obstacle_detected = False

    def update_side_detection(self, debug=False, walls=None):
        """
        Read the distance returned by right and left sensors and check if an obstacle is detected
        The distance is read relatively to the input list of obstacles
        """
        self._update_virtual_position()
        if self.lidar.get_left_lidar() is not None:
            if self.lidar.read_left_distance(self._drone_x, self._drone_y, self.get_sensor_angle(-90), walls) and debug:
                print("Left lidar range:" + str(self.lidar.get_left_lidar().get_distance()))
            if self.lidar.get_left_lidar().critical_distance_reached():
                self.lidar._obstacle_detected_left = True
            else:
                self.lidar._obstacle_detected_left = False

        if self.lidar.get_right_lidar() is not None:
            if self.lidar.read_right_distance(self._drone_x, self._drone_y, self.get_sensor_angle(90), walls) and debug:
                print("Right lidar range:" + str(self.lidar.get_right_lidar().get_distance()))
            if self.lidar.get_right_lidar().critical_distance_reached():
                self.lidar._obstacle_detected_right = True
            else:
                self.lidar._obstacle_detected_right = False

    def arm_and_takeoff(self, tgt_altitude):
        """
        Function from the dronekit documentation used to arm and takeoff the drone on the simulator
        Wait for the drone to reach the target altitude
        """
        print("Arming motors")
        while not self.vehicle.is_armable:
            time.sleep(1)
        self.set_guided_mode()
        self.vehicle.armed = True
        while not self.vehicle.armed:
            print("Waiting for arming")
            time.sleep(1)
        print("Takeoff")
        self.vehicle.simple_takeoff(tgt_altitude)
        while True:
            altitude = self.vehicle.location.global_relative_frame.alt
            if altitude >= tgt_altitude - 1:
                print("Altitude reached")
                break
            time.sleep(1)
