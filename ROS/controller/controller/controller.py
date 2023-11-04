import rclpy
import math
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Bool
from pop import Pilot, LiDAR
import threading
import time
from planning import local_planning


gps_data = [0.0,0.0]
automatic = True
speed = 0.0
steering = 0.0
distance = 1500
safe_distance = 1000
width_of_bin_0 = 500
max_speed = 60
n_bins = int(12) # 4, 8, 12, 16
gps_accurate = 5.0

class DriveController(Node):
    def __init__(self):
        super().__init__('drive_controller')
        self.get_logger().info("Node!")
        self.automatic_sub = self.create_subscription(Bool, "/automatic", self.automatic_callback, 10)
        self.gps_sub = self.create_subscription(Float32MultiArray, "/gps", self.gps_callback, 10)
        self.cmd_vel_sub = self.create_subscription(Float32MultiArray, "/cmd_vel", self.cmd_vel_callback, 10)
        
    def gps_callback(sefl, data_msg = Float32MultiArray):
        global gps_data
        gps_data = data_msg.data[0:2]
        print(gps_data)
          
    def automatic_callback(self, data_msg: Bool):
        global automatic
        if data_msg.data:
            automatic = True
        else:
            automatic = False
            
    def cmd_vel_callback(self, cmd_vel_msg: Float32MultiArray):
        global speed, steering, automatic
        if not automatic:
            steering = cmd_vel_msg.data[0]
            speed = max_speed*cmd_vel_msg.data[1]
            
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
    places = [[21.047939828195936, 105.80094216574687],[21.0483257655548, 105.80093777817802],[21.048348287067167, 105.80070414013677]]
    place_id = 0
    Car = Pilot.AutoCar()
    Car.setObstacleDistance(distance=0)
    Car.setSensorStatus(euler=1)
    lidar = LiDAR.Rplidar()
    lidar.connect()
    lidar.startMotor()
    planning = local_planning(distance, safe_distance, width_of_bin_0, max_speed, n_bins, gps_accurate)
    place = places[place_id]
    while True:
        if automatic:
            if not planning.check_distance(place[0], place[1], gps_data[0], gps_data[1]):
                steering, speed = planning.go_place(Car, lidar, place[0], place[1], gps_data[0], gps_data[1])
            else:
                print("next place!")
                place_id += 1
                place = places[place_id]
    
        Car.steering = steering                       
        # if speed != 0:
        #     Car.forward(speed)      
        #     if steering > 0:
        #         set_lights( Car, 4, 8, 'red')
        #     elif steering < 0:
        #         set_lights( Car, 0, 4, 'red')
        #     else:
        #         set_lights( Car, 0, 8, 'white')
        # else:
        #     set_lights( Car, 0, 8, 'blue')        
        #     Car.stop()  
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
