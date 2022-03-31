"""
@author: Clara
@subject: Suivi de mur V1

"""
import sys
import time
sys.path.insert(0, '../drone')
sys.path.insert(0, '../sensors')
from inspection_drone import InspectionDrone


drone = InspectionDrone('/dev/serial0',
                        baudrate=115200,
                        two_way_switches=[7, 8],
                        three_way_switches=[5, 6, 8, 9, 10, 11, 12],
                        buzzer_pin=23, lidar_address=0x10, critical_distance_lidar=300)

total_test_time = 30
switch_obstacle = 9
drone.launch_mission()
obstacle_detected = False

target_distance = 300     #Distance that must be kept between the drone and the wall
K = 1                   #Coefficient entre l'erreur de distance et la vitesse selon x

while drone.mission_running():
    #Updating everything
    drone.update_time()
    drone.update_switch_states()
    drone.update_detection(use_lidar=True, debug=False)

    #Testing if there is an obstacle
    if drone.obstacle_detected() and drone._lidar.get_distance()>0 and not obstacle_detected:
        drone.set_guided_mode()
        print("Obstacle detected")
        obstacle_detected = True
    if not drone.obstacle_detected() and drone._lidar.get_distance()>0 and obstacle_detected:
        obstacle_detected = False
        print("No obstacle")

    #Mode suivi de mur
    if obstacle_detected is True:
        Vx = K*(target_distance - drone._lidar.get_distance())
        Vy = 0.5    #Le drone se decale sur sa droite à 0.5 m/s
        drone._send_ned_velocity(Vx, Vy, 0)
    time.sleep(0.1)