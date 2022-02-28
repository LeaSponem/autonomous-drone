from dronekit import VehicleMode, connect
import time


def connection_drone(connection_string, baudrate):
    print("Trying to connect to the drone")
    vehicle = connect(connection_string, baudrate, wait_ready=True)
    print("Success")
    return vehicle


# Connection between the Raspberry Pi and the Pixhawk
drone = connection_drone('/dev/serial0', baudrate=115200)

# Access to parameters
for _ in range(10):
    print("Drone velocity :", drone.velocity)
    time.sleep(1)

# Change mode
print("Actual mode:", drone.mode)
drone.mode = VehicleMode("GUIDED")
print("New mode:", drone.mode)
