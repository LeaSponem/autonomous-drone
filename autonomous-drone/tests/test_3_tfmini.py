import sys
sys.path.insert(0, '../sensors')
from tf_mini import TFMiniPlus
import time

mini1=TFMiniPlus(0x10,50)
mini2=TFMiniPlus(0x11,50)
mini3=TFMiniPlus(0x12,50)

mini1.read_distance()
time.sleep(0.05)
mini2.read_distance()
time.sleep(0.05)
mini3.read_distance()
time.sleep(0.05)

for i in range(20000):
    if mini1.lidar_reading():
        mini1.read_distance()
        distance1 = mini1.get_distance()
    if mini2.lidar_reading():
        mini2.read_distance()
        distance2=mini2.get_distance()
    if mini3.lidar_reading():
        mini3.read_distance()
        distance3=mini3.get_distance()
    print(distance1, distance2, distance3)
    time.sleep(0.1)
#print(mini1.time_log[-1] )
#print(mini1.log)
# print(mini.time_log)