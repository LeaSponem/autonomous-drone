import sys
import time
sys.path.insert(0, '../drone')
sys.path.insert(0, '../sensors')
from inspection_drone import InspectionDrone
from tf_mini import tfmini_distance

drone = InspectionDrone('/dev/serial0',
                        baudrate=115200,
                        two_way_switches=[7, 8],
                        three_way_switches=[5, 6, 8, 9, 10, 11, 12],
                        buzzer_pin=23, critical_distance_lidar=300)

total_test_time = 30
switch_obstacle = 9
critical_distance = 300
drone.launch_mission()
obstacle_detected = False

while drone.mission_running():
    drone.update_time()
    drone.update_switch_states()
    if critical_distance > tfmini_distance(0x10) > 0 and not obstacle_detected:
        drone.set_guided_mode()
        print("Obstacle detected")
        obstacle_detected = True
    if tfmini_distance(0x10) > critical_distance and tfmini_distance(0x10) > 0 and obstacle_detected:
        obstacle_detected = False
    time.sleep(0.1)
