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

drone.arm_and_takeoff(2)
first_detection = False
drone.launch_mission()

total_test_time = 30
switch_obstacle = 9
drone.launch_mission()
obstacle_detected = False

K = 0.001                  #Coefficient for the PID
mission_time = 0           #Increment for the plot log
Vx_ordered = 0             #Definition of Vx
Vx_measured = 0
yaw = 0
x = 0                       #Increment for Simulator use only

" -------- Definition of a log -------- "
list_Vx_ordered = []
list_Vx_measured = []
list_measured_distance = []
list_yaw = []
list_time = []

" ------- Mission running -------- "
time_0 = time.time()
while drone.mission_running() and mission_time < time_length_simu:
    mission_time = time.time() - time_0

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

    #Echelon
    measured_distance = 500
    #Rampe
    #measured_distance = mission_time * 2
    #Sinus
    #measured_distance = target_distance + 200 * np.sin(mission_time * 0.1)

    #Following a wall
    Vx_ordered = K*(measured_distance - target_distance)      #Forward speed proportionnal to the distance with the wall
    Vx_ordered = np.min([np.abs(Vx_ordered), 0.5])*np.sign(Vx_ordered)                    #Verify it doesn't exceed Vmax = 0.5 m/s
    Vy = 0.5                                                    #Lateral speed is 0.5 m/s
    drone._send_ned_velocity(Vx_ordered, Vy, 0)

    #Updating the log
    Vx_measured = drone.get_velocity()[0]
    yaw = drone.get_yaw()
    list_time.append(mission_time)
    list_Vx_ordered.append(Vx_ordered)
    list_Vx_measured.append(Vx_measured)
    list_measured_distance.append(measured_distance)
    list_yaw.append(yaw)

    time.sleep(0.1)
    #End of the while simulation

drone.vehicle.mode = VehicleMode("RTL")         #REMOVE THIS LINE IF IRL, FOR SIMULATOR ONLY

""" -------- Plot of the logs -------- """
plt.plot(list_time,list_Vx_ordered, label="Ordered")
plt.plot(list_time,list_Vx_measured, label="Measured")
plt.xlabel("Time")
plt.ylabel("Speed on x")
plt.legend()
title = "K=" + str(K)
plt.title(title)
plt.show()
#plt.savefig("Vx_log.png")

plt.plot(list_time,list_measured_distance)
plt.xlabel("Time")
plt.ylabel("Measured Distance")
plt.show()
#plt.savefig("Measured_distance_log.png")

"""
plt.plot(list_time,list_yaw)
plt.xlabel("Time")
plt.ylabel("Yaw")
plt.show()
#plt.savefig("Yaw_log.png")
"""
