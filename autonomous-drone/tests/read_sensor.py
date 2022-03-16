import sys
import time
sys.path.insert(0, '../drone')
from inspection_drone import InspectionDrone


drone = InspectionDrone('/dev/serial0',
                        baudrate=115200,
                        two_way_switches=[7, 8],
                        three_way_switches=[5, 6, 8, 9, 10, 11, 12],
                        buzzer_pin=23, critical_distance_lidar=300)

total_test_time = 30
drone.launch_mission()

while drone.mission_running():
    drone.update_time()
    if drone.time_since_mission_launch() > total_test_time:
        drone.abort_mission()
    drone.update_detection(use_lidar=True, debug=False)
    if drone.obstacle_detected():
        print("Obstacle")
    time.sleep(0.01)
