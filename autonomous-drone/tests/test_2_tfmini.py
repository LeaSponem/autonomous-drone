import sys
sys.path.insert(0, '../sensors')
from tf_mini import TFMiniPlus
import time
distance = -1
distance2 = -1
mini=TFMiniPlus(0x11,50)
mini2=TFMiniPlus(0x12,50)
mini.read_distance()
time.sleep(0.005)
mini2.read_distance()
for i in range(200):
    if mini.lidar_reading():
        mini.read_distance()
        distance=mini.get_distance()
    if mini2.lidar_reading():
        mini2.read_distance()
        distance2=mini2.get_distance()
    print(distance,distance2)
    time.sleep(0.1)
print(mini.time_log[-1] )
print(mini.log)
# print(mini.time_log)
