import sys
import numpy as np
sys.path.insert(0, '../sensors')
from tf_mini import TFMiniPlus
import time
def compute_angle(L,x1,x2):
    angle=np.actan((x1-x2)/L)
    return angle*180/3.14

mini=TFMiniPlus(0x11,50)
mini2=TFMiniPlus(0x12,50)
mini.read_distance()
time.sleep(0.05)
mini2.read_distance()
time.sleep(0.05)
for i in range(20000):
    if mini.lidar_reading():
        mini.read_distance()
        distance=mini.get_distance()
    if mini2.lidar_reading():
        mini2.read_distance()
        distance2=mini2.get_distance()
    print(distance,distance2,compute_angle(15,distance,distance2))
# print(mini.time_log[-1] )
# print(mini.log)
# print(mini.time_log)
