"""
Test auto and guided modes
The drone follows a mission in auto mode, then we switch its mode to Guided
Then we send a few MAVLink commands and resume the mission
"""
import sys
import time
sys.path.insert(0, '../drone')
from inspection_drone import InspectionDrone

drone = InspectionDrone('/dev/serial0', baudrate=115200,
                        two_way_switches=[7, 8], three_way_switches=[5, 6, 9, 10, 11, 12])


while True:
    drone.update_switch_states()
    drone.update_time()
    if drone.is_in_guided_mode():  # set GUIDED mode
        print("Petite pause")
        for _ in range(2):  # drone stays stationary
            drone.send_mavlink_stay_stationary()
            time.sleep(1)
        for _ in range(6):  # drone goes forward for 6 s
            drone.send_mavlink_go_forward(0.5)
            time.sleep(1)
        for _ in range(2):  # drone stays stationary
            drone.send_mavlink_stay_stationary()
            time.sleep(1)
        drone.set_flight_mode("AUTO")  # drone resumes its mission
    time.sleep(0.1)
