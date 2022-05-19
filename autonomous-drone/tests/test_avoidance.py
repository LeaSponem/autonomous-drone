# dronekit-sitl copter-3.3 --home=48.8411292,2.5879308,584,353
# mavproxy.exe --master tcp:127.0.0.1:5760 --out udp:127.0.0.1:14550 --out udp:127.0.0.1:14551
# python test_avoidance.py --connect udp:127.0.0.1:14551
import sys
import time
import argparse
sys.path.insert(0, '../drone')
sys.path.insert(0, '../obstacles')
from virtual_drone import VirtualDrone
from wall import WallObstacle
from inspection_drone import InspectionDrone


simulation = False

parser = argparse.ArgumentParser(description='commands')
parser.add_argument('--connect')
args = parser.parse_args()

connection_string = args.connect

if connection_string is None:
    connection_string = '/dev/serial0'

if simulation:
    drone = VirtualDrone(connection_string=connection_string, baudrate=115200,
                         two_way_switches=[7, 8], three_way_switches=[5, 6, 8, 9, 10, 11, 12],
                         lidar_angle=[0, 90, -90], critical_distance_lidar=100)
    first_detection = True

else:
    drone = InspectionDrone(connection_string, baudrate=115200,
                            two_way_switches=[7, 8], three_way_switches=[5, 6, 8, 9, 10, 11, 12],
                            lidar_angle=[0, 90, -90], lidar_address=[0x10, 0x12, 0x11],
                            critical_distance_lidar=200)


wall1 = WallObstacle(-1000, 1000, 2000, 0)
wall2 = WallObstacle(-300, 500, 5000, 90)
wall3 = WallObstacle(-1000, 1200, 2000, 90)
wall4 = WallObstacle(-1000, -1000, 2000, 0)
walls = [wall1, wall2]

drone.launch_mission()
if simulation:
    drone.arm_and_takeoff(2)

while drone.mission_running():
    drone.update_time()  # update time since connexion and mission's start
    drone.update_switch_states()
    if drone.do_lidar_reading():  # ask a reading every 20 ms
        if simulation:
            drone.update_detection(use_lidar=True, debug=True, walls=walls)  # distance measure
            drone.update_side_detection(debug=True, walls=walls)
        else:
            drone.update_detection(use_lidar=True, debug=True)  # distance measure
            drone.update_side_detection(use_lidar=True, debug=True)
    if drone.obstacle_detected() and drone.is_in_auto_mode():
        drone.set_guided_mode()
        drone.send_mavlink_stay_stationary()
    if drone.obstacle_detected() and first_detection and simulation:
        print("Obstacle detected")
        drone.set_guided_mode()
        drone.send_mavlink_stay_stationary()
        first_detection = False
    if drone.obstacle_detected() and drone.is_in_guided_mode():
        drone.lidar.update_path(drone.obstacle_detected())
        if drone.lidar.go_left:
            drone.send_mavlink_go_left(0.5)
        elif drone.lidar.go_right:
            drone.send_mavlink_go_right(0.5)
    if not drone.obstacle_detected() and drone.is_in_guided_mode()\
            and drone.time_since_last_obstacle_detected() > 3 and not simulation:
        drone.set_auto_mode()
        drone.lidar.update_path(drone.obstacle_detected())
    if not drone.obstacle_detected() and drone.is_in_guided_mode() \
            and drone.time_since_last_obstacle_detected() > 3 and simulation:
        first_detection = True
        drone.lidar.update_path(drone.obstacle_detected())
    if not drone.obstacle_detected() and simulation and first_detection:
        drone.send_mavlink_go_forward(0.5)
    if drone.time_since_last_obstacle_detected() > 60:
        drone.abort_mission()
    time.sleep(0.1)
