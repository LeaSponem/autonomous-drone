# dronekit-sitl copter-3.3 --home=48.8411292,2.5879308,584,353
# mavproxy.exe --master tcp:127.0.0.1:5760 --out udp:127.0.0.1:14550 --out udp:127.0.0.1:14551
# python test_simu.py --connect udp:127.0.0.1:14551
import sys
import time
import numpy as np
import argparse
from dronekit import VehicleMode, LocationGlobalRelative
sys.path.insert(0, '../drone')
sys.path.insert(0, '../obstacles')
from inspection_drone import InspectionDrone
from wall import WallObstacle
from simulation_position import SimulationPosition

parser = argparse.ArgumentParser(description='commands')
parser.add_argument('--connect')
args = parser.parse_args()

connection_string = args.connect
use_simulator = True
if connection_string is None:
    connection_string = '/dev/serial0'
    use_simulator = False

drone = InspectionDrone(connection_string,baudrate=115200,
                        two_way_switches=[7, 8], three_way_switches=[5, 6, 8, 9, 10, 11, 12],
                        lidar_address=0x10, critical_distance_lidar=1)

wall = WallObstacle(-10, 10, 20, 0)
walls = [wall]

def arm_and_takeoff(vehicle, tgt_altitude):
    print("Arming motors")
    while not vehicle.is_armable:
        time.sleep(1)
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True
    while not vehicle.armed:
        print("Waiting for arming")
        time.sleep(1)
    print("Takeoff")
    vehicle.simple_takeoff(tgt_altitude)
    while True:
        altitude = vehicle.location.global_relative_frame.alt
        if altitude >= tgt_altitude -1:
            print("Altitude reached")
            break
        time.sleep(1)


loc = drone.vehicle.location.global_relative_frame
angle = (180/np.pi)*(np.pi/2 + drone.vehicle.attitude.yaw)
local_frame = SimulationPosition(loc.lat, loc.lon, (180/np.pi)*(np.pi/2 + drone.vehicle.attitude.yaw))
arm_and_takeoff(drone.vehicle, 2)
first_detection = True
drone.launch_mission()

while drone.mission_running():
    drone.update_time()  # update time since connexion and mission's start
    loc = drone.vehicle.location.global_relative_frame
    x, y = local_frame.get_position(loc)
    if drone.do_lidar_reading():  # ask a reading every 20 ms
        drone.update_detection(use_lidar=True, debug=True,
                               x_drone=0.001*x, y_drone=0.001*y,
                               drone_angle=angle - (180 / np.pi) * drone.vehicle.attitude.yaw,
                               walls=walls)  # distance measure
    if drone.obstacle_detected() and first_detection:
        print("Obstacle detected")
        drone.send_mavlink_stay_stationary()
        drone.send_mavlink_left_rotate(90)
    if not drone.obstacle_detected() and not first_detection:
        if drone.check_rotation(2):
            if drone.time_since_last_obstacle_detected() > 4:
                drone.send_mavlink_stay_stationary()
                drone.send_mavlink_right_rotate(90)
            else:
                drone.send_mavlink_go_forward(0.25)
    drone.send_mavlink_go_forward(1)
    print(0.001*x, 0.001*y)
    time.sleep(0.1)
