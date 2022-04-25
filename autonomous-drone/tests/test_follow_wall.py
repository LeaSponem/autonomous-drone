"""
@author: Clara
@subject: Suivi de mur

"""
import sys
import time
import numpy as np
import argparse
sys.path.insert(0, '../drone')
sys.path.insert(0, '../sensors')
from virtual_drone import VirtualDrone
from inspection_drone import InspectionDrone
from dronekit import VehicleMode, LocationGlobalRelative

parser = argparse.ArgumentParser(description='commands')
parser.add_argument('--connect')
args = parser.parse_args()

connection_string = args.connect
target_distance = 300     #Distance that must be kept between the drone and the wall

drone = VirtualDrone(connection_string, baudrate=115200,
                     two_way_switches=[7, 8],
                     three_way_switches=[5, 6, 8, 9, 10, 11, 12],
                     critical_distance_lidar=3)             #Make sure the distance is the same (but in meters) as the target_distance

"""
drone = InspectionDrone('/dev/serial0', baudrate=115200,
                        two_way_switches=[7, 8],
                        three_way_switches=[5, 6, 8, 9, 10, 11, 12],
                        lidar_address=0x10, critical_distance_lidar=target_distance)
"""


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

arm_and_takeoff(drone.vehicle, 2)
first_detection = False
drone.launch_mission()

total_test_time = 30
switch_obstacle = 9
drone.launch_mission()
obstacle_detected = False

K = 0.001                   #Coefficient entre l'erreur de distance et la vitesse selon x

x = 0   #Increment for Simulator use only

while drone.mission_running():
    #Updating everything
    drone.update_time()
    drone.update_switch_states()
    #drone.update_detection(use_lidar=True, debug=True)

    """
    # Following a wall mode IRL

    #Testing if there is an obstacle
    if drone.obstacle_detected() and not obstacle_detected:
        drone.set_guided_mode()
        print("Obstacle detected")
        obstacle_detected = True
        first_detection = True
    if not drone.obstacle_detected() and obstacle_detected:
        obstacle_detected = False
        print("No obstacle")

    measured_distance = drone.get_distance()
    """
    # Following a wall mode in Simulator
    x+=1
    if x == 50 :                    #Begin simulation at 5 s
        first_detection = True
        print("following")
    if x == 500 :                    #End simulation at 50 s
        first_detection = False
    measured_distance = target_distance + 200 * np.sin(x)

    #Following a wall
    if first_detection is True:
        Vx = K*(measured_distance - target_distance)      #Forward speed proportionnal to the distance with the wall
        Vx = np.min([np.abs(Vx), 0.5])*np.sign(Vx)                    #Verify it doesn't exceed Vmax = 0.5 m/s
        Vy = 0.5                                                    #Lateral speed is 0.5 m/s
        print("Vx=", Vx," and Vy=", Vy)
        drone._send_ned_velocity(Vx, Vy, 0)
    time.sleep(0.1)
