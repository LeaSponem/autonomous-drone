# cd C:\Users\leasp\Documents\ENPC\drone_autonome\autonomous-drone\autonomous-drone\tests
# python test_go_forward.py --connect udp:127.0.0.1:14551
import sys
import time
sys.path.insert(0, '../drone')
from inspection_drone import InspectionDrone

connect_string = '/dev/serial0'


drone = InspectionDrone(connection_string=connect_string, baudrate=115200,
                        two_way_switches=[7, 8], three_way_switches=[5, 6, 9, 10, 11, 12])


# if drone.switches[start_mission_switch].is_up():
#     start_time = time.time()
#     time_elapsed = time.time() - start_time
#     drone.launch_mission()
#     print("Starting mission")


last_flight_mode = drone.vehicle.mode
not_over = True

while not_over:
    print(drone.is_in_guided_mode())
    drone.update_switch_states()
    if drone.is_in_guided_mode():
        print("Going North for 10 s at 0.1 m/s")
        for _ in range(10):
            drone.send_mavlink_go_forward(0.1)
            time.sleep(1)
        drone.set_flight_mode("GUIDED")
        print("Going South for 10 s at 0.1 m/s")
        for _ in range(10):
            drone.send_mavlink_go_backward(0.1)
            time.sleep(1)
        drone.set_flight_mode(last_flight_mode)
        not_over = False

drone.vehicle.close()
