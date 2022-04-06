from drone.inspection_drone import InspectionDrone
import time

drone = InspectionDrone('/dev/serial0',
                        baudrate=115200,
                        two_way_switches=[7, 8],
                        three_way_switches=[5, 6, 8, 9, 10, 11, 12],
                        buzzer_pin=23, lidar_address=0x10, critical_distance_lidar=400)

drone.launch_mission()
drone.update_last_flight_mode()

while drone.mission_running():
    drone.update_switch_states()
    drone.update_time()  # update time since connexion and mission's start
    if drone.do_lidar_reading():  # ask a reading every 20 ms
        drone.update_detection(use_lidar=True, debug=False)  # distance measure
    if drone.obstacle_detected():
        if not drone.is_in_guided_mode():
            print("Obstacle detected")
            drone.update_last_flight_mode()
            drone.set_guided_mode()
            drone.send_mavlink_left_rotate(90)
    if not drone.obstacle_detected() and drone.is_in_guided_mode():
        if drone.check_rotation(0.05):
            if drone.time_since_last_obstacle_detected() > 4:
                drone.send_mavlink_stay_stationary()
                drone.send_mavlink_right_rotate(90)
            else:
                drone.send_mavlink_go_forward(0.25)
    drone.send_mavlink_go_forward(0.5)
    time.sleep(0.1)
