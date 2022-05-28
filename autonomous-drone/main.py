"""
Script for a demo with obstacle detection and avoidance with three lidar sensors
The drone follows a mission in auto mode until it detects an obstacle in front of him
It stops and goes right or left depending on the situation
"""
from drone.inspection_drone import InspectionDrone
import time

drone = InspectionDrone('/dev/serial0',
                        baudrate=115200,
                        two_way_switches=[7, 8],
                        three_way_switches=[5, 6, 8, 9, 10, 11, 12],
                        lidar_angle=[0, 90, -90], lidar_address=[0x10, 0x12, 0x11],
                        critical_distance_lidar=200)

drone.launch_mission()

while drone.mission_running():
    drone.update_time()  # update time since connexion and mission's start
    drone.update_switch_states()  # update RC transmitter switch state
    if drone.do_lidar_reading():  # ask a reading every 20 ms
        drone.update_detection(use_lidar=True, debug=True)  # front distance measure
        drone.update_side_detection(use_lidar=True, debug=True)  # right and left distance measure
    if drone.obstacle_detected() and drone.is_in_auto_mode():  # obstacle detected in front of the drone
        drone.set_guided_mode()
        drone.send_mavlink_stay_stationary()
    if drone.obstacle_detected() and drone.is_in_guided_mode():  # check path
        drone.lidar.update_path(drone.obstacle_detected())  # update right and left path
        if drone.lidar.go_left:  # no obstacle on the left
            drone.send_mavlink_go_left(0.5)
        elif drone.lidar.go_right:  # no obstacle on the right
            drone.send_mavlink_go_right(0.5)
    if not drone.obstacle_detected() and drone.is_in_guided_mode()\
            and drone.time_since_last_obstacle_detected() > 3:  # no obstacle in front of the drone
        drone.set_auto_mode()  # drone resumes its mission
        drone.lidar.update_path(drone.obstacle_detected())
    if drone.time_since_last_obstacle_detected() > 300:
        drone.abort_mission()
    time.sleep(0.1)
