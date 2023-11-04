import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
from pop import Pilot, LiDAR
import threading
import time

gps_data = [0.0,0.0]
automatic = False
max_speed = 60
speed = 0.0
steering = 0.0
n_bins = int(12) # 4, 8, 12, 16
distance = 1500
safe_distance = 1000
width_of_bin_0 = 500
max_distance = 5.0

class DriveController(Node):
    def __init__(self):
        super().__init__('drive_controller')
        self.get_logger().info("Node Started")
        self.car = Pilot.AutoCar()
        self.car.setObstacleDistance(0)

        # subscriber
        self.automatic_sub = self.create_subscription(Bool, "/automatic", self.automatic_callback, 10)
        self.gps_sub = self.create_subscription(Float32MultiArray, "/gps", self.gps_callback, 10)
        self.cmd_vel_sub = self.create_subscription(Float32MultiArray, "/cmd_vel", self.cmd_vel_callback, 10)
        
    def gps_callback(sefl, data_msg = Float32MultiArray):
        global gps_data, gps_status
        gps_data = data_msg.data[0:2]
        
    def automatic_callback(self, data_msg: Bool):
        global automatic
        if data_msg.data:
            automatic = True
        else:
            automatic = False
        print(automatic)
            
    def cmd_vel_callback(self, cmd_vel_msg: Float32MultiArray):
        global speed, steering, automatic
        if not automatic:
            steering = cmd_vel_msg.data[0]
            speed = max_speed*cmd_vel_msg.data[1]
        
# Scan bins        
def get_bins(lidar, n_bins, angle_of_b):
    global distance, safe_distance, width_of_bin_0
    bins = [] 
    safe_bins = []
    points_in_bin = [] 
    points_in_safe_bin = []
    for bin in range(int(n_bins)):
        bins.append(0)
        safe_bins.append(0)
        points_in_bin.append(0)
        points_in_safe_bin.append(0)
    vectors = lidar.getVectors()
    for vector in vectors:
        if vector[0] <= 90 or vector[0] >=270:
            if vector[0] >= 270:
                angle_ = 360 - vector[0]
            else:
                angle_ = vector[0]
            rad = math.radians(angle_)
            if vector[1]*math.sin(rad) <= width_of_bin_0/2 and vector[1]*math.cos(rad) <= distance:
                points_in_bin[0] += 1
            if vector[1]*math.sin(rad) <= width_of_bin_0/2 and vector[1]*math.cos(rad) <= safe_distance:    
                points_in_safe_bin[0] += 1
                
        if vector[0] <= 360 - angle_of_b/2 and vector[0] >= angle_of_b/2:
            bin = int((angle_of_b/2+vector[0])/angle_of_b)
            if vector[1] <= distance: 
                points_in_bin[bin] += 1
            if vector[1] <= safe_distance:
                points_in_safe_bin[bin] += 1
                
    for bin in range(int(n_bins)): 
        if points_in_bin[bin] >= 2:
            bins[bin] = 1
        if points_in_safe_bin[bin] >= 2:
            safe_bins[bin] = 1
            
    return bins, safe_bins
    
def compute_desired_bins( beta, n_bins, angle_of_b, bins, safe_bins):
    bin_id = 0
    count = 0
    if beta < 0: #. beta in range (0,360) 
        beta += 360
    index = int((beta+angle_of_b/2) / angle_of_b)
    if index > n_bins-1:
        index = 0
    for i in range(int(n_bins/2)):
        if(bins[(index+i)%n_bins] == 0):
            bin_id = (index+i)%n_bins
            break
        if(bins[index-i] == 0):
            bin_id = index-i
            if bin_id < 0:
                bin_id = bin_id + n_bins 
            break
    if bins == [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]:
        return bin_id, False
    
    if bin_id <= n_bins//2 :
        for beta in range(0, bin_id - 1):
            if safe_bins[beta] == 1:
                return bin_id, False
        return bin_id, True
        
    elif bin_id > n_bins//2 :
        count += safe_bins[0]
        for beta in range(bin_id + 1, n_bins):
            if safe_bins[beta] == 1:
                return bin_id, False
        return bin_id, True

def check_distance(lat_end, lon_end, c, lon_start):
    global max_distance
    d_lat = lat_end - lat_end
    d_lon = lon_end - lon_start
    angle = math.sin(d_lat / 2) ** 2 + math.cos(lat_end) * math.cos(lat_end) * math.sin(d_lon / 2) ** 2
    c = 2 * math.atan2(math.sqrt(angle), math.sqrt(1 - angle))
    R = 6371000  # Approximate radius of the Earth in meters
    distance = R * c 
    print(distance)
    if distance < max_distance:
        return True
    return False 
 
def go_place(Car, lidar, lat_end, lon_end, lat_start, lon_start):
    global max_speed, n_bins, gps_status
    angle_of_b = 360/n_bins
    
    d_lon = lon_end - lon_start
    # Calculate the bearing using the haversine formula
    y = math.sin(d_lon) * math.cos(lat_end)
    x = math.cos(lat_start) * math.sin(lat_end) - math.sin(lat_start) * math.cos(lat_end) * math.cos(d_lon)
    initial_bearing = math.atan2(y, x)

    # Convert the bearing from radians to degrees
    initial_bearing = math.degrees(initial_bearing)

    destination_angle = (initial_bearing + 360) % 360 #angle a
    #destination_angle = 0
    current_angle = Car.getEuler('yaw') 
    
    # obstacle avoidance 
    bins, safe_bins = get_bins(lidar, n_bins, angle_of_b)
    beta = destination_angle - current_angle
    bin_id, success = compute_desired_bins( beta, n_bins, angle_of_b, bins, safe_bins)
    angle = bin_id * angle_of_b
    safety = 3
    if angle > 180:
        angle = angle - 360
    if success:
        if abs(angle) <= 15:
            steering = 0
        elif 15 < abs(angle) and abs(angle) <= 40:
            steering = 0.35
        elif 40 < abs(angle) and abs(angle) <= 65:
            steering = 0.7
        elif 65 < abs(angle) and abs(angle) <= 90:
            steering = 1
        else:
            steering = 1
                        
        if angle <= 0:
            steering = -steering
        
        for bin in range(-n_bins//4 + 1, n_bins//4):
            if safe_bins[bin] == 1:
                safety = 1
                break
        if safety > 1:
            for bin in range(-n_bins//4 + 1, n_bins//4):
                if bins[bin] == 1:
                    safety = 2
                    break
    else:
        steering = 0.0
        safety = 0.0  

    if safety == 3:
        speed = max_speed
    elif safety == 2:   
        speed = max_speed*4/5
    elif safety == 1:   
        speed = max_speed*3/5    
    else:
        speed = 0.0  
    print(f"bin_id: {bin_id}, can quay {angle}, da quay {Car.getEuler('yaw')-current_angle}")
    print(f"steering: {steering}, safety: {safety}")    
    return steering, speed
    
def set_lights( Car, l_start, l_end, color):
    for i in range(8):
        Car.setPixelDisplay(2**i,0,0,0)   

    for i in range(l_start,l_end):
        if color == "red":
            Car.setPixelDisplay(2**i,255,0,0)   
        if color == "green":
            Car.setPixelDisplay(2**i,0,255,0)  
        if color == "blue":
            Car.setPixelDisplay(2**i,0,0,255)        
                
            
def controller_thread():
    print("start controller")
    global automatic, speed, steering, gps_data
    Car = Pilot.AutoCar()
    place_id = 0
    Car.setObstacleDistance(distance=0)
    Car.setSensorStatus(euler=1)
    lidar = LiDAR.Rplidar()
    lidar.connect()
    lidar.startMotor()
    places = [[21.04834105425579, 105.8016542764174],[21.048343381786204, 105.80093602424105],[21.048178127036262, 105.80093353030988]]
    place = places[place_id]
    while True:
        if automatic:
            print("go to place")
            lat_end = math.radians(place[0])
            lon_end = math.radians(place[1])
            lat_start = math.radians(gps_data[0])
            lon_start = math.radians(gps_data[1])
            if not check_distance(lat_end, lon_end, lat_start, lon_start):
                steering, speed = go_place(Car, lidar, lat_end, lon_end, lat_start, lon_start)
                print("chua toi dich")
            else: 
                if place_id < len(places):
                    place_id += 1
                    place = places[place_id]
                else:
                    print("stop")
    
        Car.steering = steering                       
        if speed != 0:
            Car.forward(speed)      
            if steering > 0:
                set_lights( Car, 4, 8, 'red')
            elif steering < 0:
                set_lights( Car, 0, 4, 'red')
            else:
                set_lights( Car, 0, 8, 'white')
        else:
            set_lights( Car, 0, 8, 'blue')        
            Car.stop() 
        time.sleep(0.1)            
    
def main(args=None):
    controller = threading.Thread(target=controller_thread)
    controller.start()
    rclpy.init(args=args)
    node = DriveController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
