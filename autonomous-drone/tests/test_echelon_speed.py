"""
@author: Clara
@subject: Suivi de mur

"""
import sys
import time
import numpy as np
import matplotlib.pyplot as plt
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
time_length_simu = 30     #Length of the simulation in second

""" -------- Initialization ------- """
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

" -------- Parameters for Automatic Speed Control -------- "
K = 0.1                 #Coefficient for the PID
V_targeted = 0.2           # m/s

" -------- Variables for the log --------"
mission_time = 0           #Increment for the plot log
V_ordered = 0             #Definition of Vx
V_measured = 0
yaw = 0

" -------- Definition of the log -------- "
list_V_targeted = [V_targeted]
list_V_ordered = [V_ordered]
list_V_measured = [V_measured]
list_yaw = []
list_time = [0]

" ------- Mission running -------- "
time_0 = time.time()
drone._send_ned_velocity(0, 0, 0)
while drone.mission_running() and mission_time < time_length_simu:
    mission_time = time.time() - time_0

    #Updating everything
    drone.update_time()
    drone.update_switch_states()
    #drone.update_detection(use_lidar=True, debug=True)

    """------ Automatic Speed Control ------
    - V_targeted is the speed we want
    - V_ordered is the speed we enter the drone so it goes at V_targeted
    - V_measured is the real speed of the drone 
    """
    V_measured = drone.get_velocity()[0]
    V_ordered = K * (V_targeted - V_measured)

    drone._send_ned_velocity(V_ordered, 0, 0)

    #Updating the log
    yaw = drone.get_yaw()

    list_time.append(mission_time)
    list_V_targeted.append(V_targeted)
    list_V_ordered.append(V_ordered)
    list_V_measured.append(V_measured)
    list_yaw.append(yaw)

    time.sleep(0.1)
    #End of the while simulation

drone.vehicle.mode = VehicleMode("RTL")         #REMOVE THIS LINE IF IRL, FOR SIMULATOR ONLY

""" -------- Plot of the logs -------- """
plt.plot(list_time,list_V_targeted, label="Targeted")
plt.plot(list_time,list_V_ordered, label="Ordered")
plt.plot(list_time,list_V_measured, label="Measured")
plt.xlabel("Time")
plt.ylabel("Speed on x")
plt.legend()
title = "K=" + str(K)
plt.title(title)
plt.show()
#plt.savefig("Vx_log.png")

"""
plt.plot(list_time,list_yaw)
plt.xlabel("Time")
plt.ylabel("Yaw")
plt.show()
#plt.savefig("Yaw_log.png")
"""