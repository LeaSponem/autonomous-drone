"""
Test sending MAVLink messages to the drone
When the mode is set to GUIDED, the drone moves forward for 10 s then backward for 10 s
"""
import sys
import time
sys.path.insert(0, '../drone')
from inspection_drone import InspectionDrone

connect_string = '/dev/serial0'

drone = InspectionDrone(connection_string=connect_string, baudrate=115200,
                        two_way_switches=[7, 8], three_way_switches=[5, 6, 9, 10, 11, 12])


last_flight_mode = drone.vehicle.mode
drone.launch_mission()

while drone.mission_running():
    drone.update_time()
    drone.update_switch_states()
    if drone.is_in_guided_mode():  # set guided mode
        print("Going forward for 10 s at 0.1 m/s")
        for _ in range(10):  # drone goes forward for 10 s
            drone.send_mavlink_go_forward(0.1)
            time.sleep(1)
        drone.set_flight_mode("GUIDED")
        print("Going backward for 10 s at 0.1 m/s")
        for _ in range(10):  # drone goes backward for 10 s
            drone.send_mavlink_go_backward(0.1)
            time.sleep(1)
        drone.set_flight_mode(last_flight_mode)
        drone.abort_mission()
    time.sleep(0.1)
drone.vehicle.close()
