import sys
import time
sys.path.insert(0, '../drone')
from inspection_drone import InspectionDrone

connect_string = '/dev/serial0'
switch_spin = 9

drone = InspectionDrone('/dev/serial0',
                        baudrate=115200,
                        two_way_switches=[7, 8],
                        three_way_switches=[5, 6, 8, 9, 10, 11, 12],
                        lidar_address=0x10, critical_distance_lidar=400)

drone.launch_mission()
spining = True

while drone.mission_running():
    drone.update_time()
    drone.update_switch_states()
    if drone.switches[switch_spin].is_down() and spining:
        print("Guided")
        drone.set_guided_mode()
        time.sleep(1)
        print("Spin")
        drone.send_mavlink_right_rotate(15)
        spining = False
    if drone.switches[switch_spin].is_up() and not spining:
        spining = True
    time.sleep(0.1)
