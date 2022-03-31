import sys
import time
sys.path.insert(0, '../drone')
sys.path.insert(0, '../sensors')
from inspection_drone import InspectionDrone


drone = InspectionDrone('/dev/serial0',
                        baudrate=115200,
                        two_way_switches=[7, 8],
                        three_way_switches=[5, 6, 8, 9, 10, 11, 12],
                        buzzer_pin=23, lidar_address=0x10, critical_distance_lidar=400)


drone.launch_mission()
drone.update_last_flight_mode()

while drone.mission_running():
    drone.update_time()
    drone.update_switch_states()
    if drone.do_lidar_reading():  # ask a reading every 20 ms
        drone.update_detection(use_lidar=True, debug=True)  # update obstacle detection and print read distance
    if drone.obstacle_detected():
        if not drone.is_in_guided_mode():  # first time the obstacle is detected
            print("Obstacle detected")
            drone.update_last_flight_mode()  # store value of the last flight mode
            drone.set_guided_mode()
    if not drone.obstacle_detected():
        if drone.is_in_guided_mode():
            if drone.time_since_last_obstacle_detected() > 5:
                print("Resume mission")
                drone.set_flight_mode(drone.get_last_flight_mode())
    time.sleep(0.1)
