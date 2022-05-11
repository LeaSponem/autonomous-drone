import sys
sys.path.insert(0, '../sensors')
from tf_mini import TFMiniPlus
import time

mini=TFMiniPlus(0x11,50)
mini.read_distance()
time.sleep(0.005)
distance = -1

for i in range(200):
    if mini.lidar_reading():
        mini.read_distance()
        distance=mini.get_distance()
    print(distance)
    time.sleep(0.1)
print(mini.time_log[-1] )
print(mini.log)
# print(mini.time_log)
