import time
import sys
sys.path.insert(0, '../drone')
from inspection_drone import InspectionDrone


# Connection between the Raspberry and the Pixhawk
drone = InspectionDrone('/dev/serial0', baudrate=115200,
                        two_way_switches=[7, 8], three_way_switches=[5, 6, 9, 10, 11, 12],
                        lidar_address=[0x10], lidar_angle=[0],
                        critical_distance_lidar=200)

# Access to parameters
for _ in range(10):
    print("Drone velocity :", drone.vehicle.velocity)
    time.sleep(1)

# Change mode
print("Actual mode:", drone.vehicle.mode)
drone.set_guided_mode()
print("New mode:", drone.vehicle.mode)
