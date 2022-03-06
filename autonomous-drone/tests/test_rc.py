from drone.inspection_drone import InspectionDrone
import time

guided_mode_switch = 6
start_mission_switch = 8

drone = InspectionDrone('/dev/serial0', baudrate=115200,
                        two_way_switches=[7, 8], three_way_switches=[5, 6, 9, 10, 11, 12])

last_flight_mode = drone.vehicle.mode

while True:
    time.sleep(0.5)
    drone.update_switch_states()
    if drone.switches[start_mission_switch].is_up() and not drone.mission_running():
        drone.launch_mission()
        print("Starting mission")
    if drone.mission_running():
        if drone.switches[guided_mode_switch].is_up() and not drone.is_in_guided_mode():
            last_flight_mode = drone.vehicle.mode
            drone.set_guided_mode()
            print("Guided mode")
        if drone.switches[guided_mode_switch].is_down() and drone.is_in_guided_mode():
            drone.set_flight_mode(last_flight_mode)
            print("Taking back control")
