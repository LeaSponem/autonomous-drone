import sys
import time
sys.path.insert(0, '../drone')
from inspection_drone import InspectionDrone

drone = InspectionDrone('/dev/serial0', baudrate=115200,
                        two_way_switches=[7, 8], three_way_switches=[5, 6, 9, 10, 11, 12])

while True:
    if drone.is_in_guided_mode():
        print("Petite pause")
        for _ in range(5):
            drone.send_mavlink_stay_stationary()
            time.sleep(1)
        drone.set_flight_mode("AUTO")