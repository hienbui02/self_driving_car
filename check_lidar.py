from pop import LiDAR, Pilot
import time
lidar = LiDAR.Rplidar()
lidar.connect()
lidar.startMotor()
while True:
    vectors = lidar.getVectors()
    front_r =[]
    front_l =[]
    right = []
    left = []

    for v in vectors:
        if(v[0] >= 0 and v[0] <= 20):
            front_r.append(v[1])
        if(v[0] >= 80 and v[0] <= 100):
            right.append(v[1])
        if(v[0] >= 260 and v[0] <= 280):
            left.append(v[1])
        if(v[0] >= 340 and v[0] <= 360):
            front_l.append(v[1])

    min_front_l = min(front_l, default= 1.73)/1000
    min_front_r = min(front_r, default= 1.73)/1000
    min_right = min(right, default= 0.3)/1000
    min_left = min(left, default= 0.3)/1000

    print(min_front_l, min_front_r, min_right, min_left)

    if min_right <= 0.3:
        obstacle_right = True
    else:
        obstacle_right = False
    print('right: ',obstacle_right)   




    if min_left <= 0.3:
        obstacle_left = True
    else:
        obstacle_left = False    

    print('left: ', obstacle_left)




    if min_front_l <= 1.73:
        obstacle_f_left = True
    else:
        obstacle_f_left = False 

    print('f_left: ', obstacle_f_left)


    if min_front_r <= 1.73:
        obstacle_f_right = True
    else:
        obstacle_f_right = False 

    print('f_right: ', obstacle_f_right)
    print('\n')
    time.sleep(2)