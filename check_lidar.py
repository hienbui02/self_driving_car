import math
import numpy as np
from pop import LiDAR
import time

lidar = LiDAR.Rplidar()
lidar.connect()
lidar.startMotor()

def create_point(distance, step, angle, max_x):
    point = []
    distance=distance//2
    for i in range( 0, distance//2, step):
        rad = math.radians(angle)
        point.append([max_x//2 + int(i*math.sin(rad)), int(i*math.cos(rad))])
    return point

# giam chieu /2

x_axis = 3000
y_axis = 3000
ObstacleDistance = 500

padding = 50
step = 30

center = create_point(ObstacleDistance,step,0,x_axis)
print(center)
right = create_point(ObstacleDistance,step,30,x_axis)
print(right)
left = create_point(ObstacleDistance,step,-30,x_axis)
print(left)

while True:
    x_axis = x_axis
    y_axis = y_axis//2

    costmap = np.zeros((x_axis, y_axis))
    vectors = lidar.getVectors()
    for v in vectors:
        if(v[0] >= 0 and v[0] <= 90):
            rad = math.radians(v[0])
            x = int(v[1]*math.sin(rad) + x_axis//2)//2
            y = int(v[1]*math.cos(rad))//2
            
            if x < x_axis and y < y_axis:
                for m in range(padding):
                    if x + m < x_axis:
                        for n in range(padding):
                            if y + n < y_axis:
                                costmap[int(x) + m,int(y) + n] = 1

        if(v[0] >= 270 and v[0] <= 360):
            rad = math.radians(v[0]- 270)
            x = int(-v[1]*math.cos(rad) + x_axis//2)//2
            y = int(v[1]*math.sin(rad))//2
            if x < x_axis and y < y_axis:
                for m in range(padding):
                    if x + m < x_axis:
                        for n in range(padding):
                            if y + n < y_axis:
                                costmap[int(x) + m,int(y) + n] = 1

    center_object = 0
    right_object = 0
    left_object = 0
                            
    for i in range(len(center)):
        if costmap[center[i][0],center[i][1]] == 1:
            print("co vat can o giua cach: " + str(center[i][1]/math.cos(math.radians(0))))
            center_object = 1
            break
    
    for i in range(len(right)):
        if costmap[right[i][0],right[i][1]] == 1:
            print("co vat can ben phai cach:" + str(right[i][1]/math.cos(math.radians(45))))
            right_object = 1
            break
   
    for i in range(len(left)):
        if costmap[left[i][0],left[i][1]] == 1:
            print("co vat can ben trai cach:" + str(left[i][1]/math.cos(math.radians(45))))
            left_object = 1
            break
        
    if center_object == 0:
        print("k vat can o giua")
    if right_object == 0:
        print("k vat can ben phai")
    if left_object == 0:
        print("k vat can ben trai")    
    time.sleep(5)
