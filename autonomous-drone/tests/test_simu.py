# dronekit-sitl copter-3.3 --home=48.8411292,2.5879308,584,353
# mavproxy.exe --master tcp:127.0.0.1:5760 --out udp:127.0.0.1:14550 --out udp:127.0.0.1:14551
# python test_simu.py --connect udp:127.0.0.1:14551
import sys
import time
import argparse
sys.path.insert(0, '../drone')
sys.path.insert(0, '../obstacles')
from virtual_drone import VirtualDrone
from wall import WallObstacle

parser = argparse.ArgumentParser(description='commands')
parser.add_argument('--connect')
args = parser.parse_args()

connection_string = args.connect

drone = VirtualDrone(connection_string, baudrate=115200,
                     two_way_switches=[7, 8], three_way_switches=[5, 6, 8, 9, 10, 11, 12],
                     lidar_angle=[0], critical_distance_lidar=100)

wall1 = WallObstacle(-1000, 1000, 2000, 0)
wall2 = WallObstacle(-1000, 1100, 2000, 0)
wall3 = WallObstacle(-1000, 1200, 2000, 90)
wall4 = WallObstacle(-1000, -1000, 2000, 0)

walls = [wall1, wall2, wall3, wall4]

drone.arm_and_takeoff(2)
drone.launch_mission()

while drone.mission_running():
    drone.update_time()  # update time since connexion and mission's start
    if drone.do_lidar_reading():  # ask a reading every 20 ms
        drone.update_detection(use_lidar=True, debug=True, walls=walls)  # distance measure
    if drone.obstacle_detected():
        print("Obstacle detected")
    if drone.time_since_last_obstacle_detected() > 60:
        drone.abort_mission()
    drone.send_mavlink_go_forward(0.5)
    time.sleep(0.1)
