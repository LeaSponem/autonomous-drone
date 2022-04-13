import time
import sys
import numpy as np
from dronekit import connect, VehicleMode
from pymavlink import mavutil
from rc_switch import Switch
sys.path.insert(0, '../sensors')
from tf_mini import TFMiniPlus


class InspectionDrone(object):
    def __init__(self, connection_string, baudrate, two_way_switches, three_way_switches,
                 lidar_address=None, critical_distance_lidar=30):
        """
        :rtype: object
        """
        try:
            print("Trying to connect to the drone")
            self.vehicle = connect(connection_string, baudrate, wait_ready=True)
        except Exception as e:
            print(str(e))
            raise ValueError("Unable to connect to the drone")
        print("Success")
        # Initialise switch states, Relies on T12K mapping
        self.switches = {}
        for key in two_way_switches:
            self.switches[key] = Switch(2)
        for key in three_way_switches:
            self.switches[key] = Switch(3)

        def set_rc(vehicle, chnum, value):
            vehicle._channels._update_channel(str(chnum), value)

        @self.vehicle.on_message('RC_CHANNELS')
        def RC_CHANNEL_listener(vehicle, name, message):
            # Write the correct values to the vehicle._channels
            # (without this, vehicle._channels is not correctly updated (only 8 channels)
            set_rc(vehicle, 1, message.chan1_raw)
            set_rc(vehicle, 2, message.chan2_raw)
            set_rc(vehicle, 3, message.chan3_raw)
            set_rc(vehicle, 4, message.chan4_raw)
            set_rc(vehicle, 5, message.chan5_raw)
            set_rc(vehicle, 6, message.chan6_raw)
            set_rc(vehicle, 7, message.chan7_raw)
            set_rc(vehicle, 8, message.chan8_raw)
            set_rc(vehicle, 9, message.chan9_raw)
            set_rc(vehicle, 10, message.chan10_raw)
            set_rc(vehicle, 11, message.chan11_raw)
            set_rc(vehicle, 12, message.chan12_raw)
            set_rc(vehicle, 13, message.chan13_raw)
            set_rc(vehicle, 14, message.chan14_raw)
            set_rc(vehicle, 15, message.chan15_raw)
            set_rc(vehicle, 16, message.chan16_raw)
            vehicle.notify_attribute_listeners('channels', vehicle.channels)

        self._two_way_switches = two_way_switches
        self._three_way_switches = three_way_switches
        self._start_time = time.time()
        self._mission_start_time = 0
        self._obstacle_detected = False
        self._time_last_obstacle_detected = None
        self._elapsed_time_connexion = time.time() - self._start_time
        self._elapsed_time_mission = 0
        self._mission_running = False
        self._last_flight_mode = self.vehicle.mode
        self._rotation_angle = 0
        self._absolute_yaw = np.pi/2
        self._lidar = TFMiniPlus(lidar_address, critical_distance_lidar)

    # Will update the switch. We enumerate every value in the vehicle.channels dictionary, and set switch mode
    # according to the mapping
    def update_switch_states(self):
        for index, (key, value) in enumerate(self.vehicle.channels.items()):

            if int(key) in self._two_way_switches:
                if value < 1500:
                    self.switches[int(key)].set_state("down")
                if value > 1500:
                    self.switches[int(key)].set_state("up")

            if int(key) in self._three_way_switches:
                if value < 1200:
                    self.switches[int(key)].set_state("down")
                if 1200 < value < 1800:
                    self.switches[int(key)].set_state("middle")
                if 1800 < value:
                    self.switches[int(key)].set_state("up")

    def update_detection(self, use_lidar=True, debug=False):
        if self._lidar.read_distance() and debug:
            print("Lidar range:" + str(self._lidar.get_distance()))
        if use_lidar and self._lidar.critical_distance_reached():
            if self.obstacle_detected():
                self._time_last_obstacle_detected = time.time()
            self._obstacle_detected = True
        else:
            self._obstacle_detected = False

    def time_since_last_obstacle_detected(self):
        if self._time_last_obstacle_detected is None or self.obstacle_detected():
            return -1
        else:
            return time.time() - self._time_last_obstacle_detected

    def do_lidar_reading(self):
        return self._lidar.lidar_reading()

    def obstacle_detected(self):
        return self._obstacle_detected

    def update_time(self):
        self._elapsed_time_connexion = time.time() - self._start_time
        if self._mission_running:
            self._elapsed_time_mission = time.time() - self._mission_start_time

    def print_switches_states(self):
        print(' ; '.join("{}: {}".format(k, v) for k, v in self.switches.items()))

    def _send_ned_velocity(self, velocity_x, velocity_y, velocity_z):
        """
        Move vehicle in direction based on specified velocity vectors.
        Modified version of the dronekit one, only sends 1 mavlink message
        """
        msg = self.vehicle.message_factory.set_position_target_local_ned_encode(
            0,  # time_boot_ms (not used)
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # frame
            0b0000111111000111,  # type_mask (only speeds enabled)
            0, 0, 0,  # x, y, z positions (not used)
            velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
            0, 0, 0,  # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
            0, 0)  # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
        self.vehicle.send_mavlink(msg)

    def _send_condition_yaw_command(self, angle, direction):
        """
        Set the vehicle yaw to a specified value in degrees
        The yaw is set relatively to the actual yaw value
        """
        msg = self.vehicle.message_factory.command_long_encode(
            0, 0,  # target system, target component
            mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
            0,  # confirmation
            angle,  # yaw in degrees
            0,  # yaw speed deg/s
            direction,  # direction -1 ccw, 1 cw
            1,  # 1 for relative yaw value
            0, 0, 0)
        self.vehicle.send_mavlink(msg)

    def send_mavlink_go_forward(self, velocity):
        print("Going forward")
        self._send_ned_velocity(velocity, 0, 0)

    def send_mavlink_go_left(self, velocity):
        print("Going left")
        self._send_ned_velocity(0, -velocity, 0)

    def send_mavlink_go_right(self, velocity):
        print("Going right")
        self._send_ned_velocity(0, velocity, 0)

    def send_mavlink_go_backward(self, velocity):
        print("Going backward")
        self._send_ned_velocity(-velocity, 0, 0)

    def send_mavlink_go_in_plane(self, velocity_x, velocity_y):
        self._send_ned_velocity(velocity_x, velocity_y, 0)

    def send_mavlink_stay_stationary(self):
        print("Stopping")
        self._send_ned_velocity(0, 0, 0)

    def send_mavlink_right_rotate(self, angle):
        if self._rotation_angle > 0:
            print("Drone already rotating")
        else:
            print("Right rotation")
            self._rotation_angle = angle
            self._send_condition_yaw_command(angle, 1)

    def send_mavlink_left_rotate(self, angle):
        if self._rotation_angle > 0:
            print("Drone already rotating")
        else:
            print("Left rotation")
            self._rotation_angle = angle
            self._send_condition_yaw_command(angle, -1)

    def check_rotation(self, threshold):
        if self._rotation_angle == 0:
            return True
        elif ((180 / np.pi) * self.vehicle.attitude.yaw+np.pi/2 - self._absolute_yaw - self._rotation_angle) < threshold:
            self._absolute_yaw = self._absolute_yaw + self._rotation_angle + self.vehicle.attitude.yaw + np.pi/2
            self._rotation_angle = 0
            return True
        return False

    def is_armable(self):
        return self.vehicle.is_armable

    def arm(self):
        self.vehicle.armed

    def is_in_auto_mode(self):
        return self.vehicle.mode == VehicleMode("AUTO")

    def is_in_guided_mode(self):
        return self.vehicle.mode == VehicleMode("GUIDED")

    def set_auto_mode(self):
        self.vehicle.mode = VehicleMode("AUTO")

    def set_guided_mode(self):
        self.vehicle.mode = VehicleMode("GUIDED")

    def get_last_flight_mode(self):
        return self._last_flight_mode

    def update_last_flight_mode(self):
        self._last_flight_mode = self.vehicle.mode

    def mission_running(self):
        return self._mission_running

    def launch_mission(self):
        self._mission_start_time = time.time()
        self._mission_running = True

    def abort_mission(self):
        self._mission_running = False

    def time_since_mission_launch(self):
        return self._elapsed_time_mission

    def set_flight_mode(self, flightmode):
        self.vehicle.mode = flightmode
