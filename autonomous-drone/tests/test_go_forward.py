from inspection_drone import InspectionDrone
import time

start_mission_switch = 8

drone = InspectionDrone('/dev/serial0', baudrate=115200,
                        two_way_switches=[7, 8], three_way_switches=[5, 6, 9, 10, 11, 12])

last_flight_mode = drone.vehicle.mode
test_time=4
total_test_time = 4 * 2 + test_time

while not drone.mission_running():
    drone.update_switch_states()
    if drone.switches[start_mission_switch].is_up():
        start_time = time.time()
        time_elapsed = time.time() - start_time
        drone.launch_mission()
        print("Starting mission")
while time_elapsed < total_test_time:
    time_elapsed = time.time() - start_time
    if time_elapsed < 4+test_time:
        drone.send_mavlink_go_forward(0.5)
    if 4+test_time < time_elapsed < total_test_time:
        drone.send_mavlink_go_backward(0.5)
