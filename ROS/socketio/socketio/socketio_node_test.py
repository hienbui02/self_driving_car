import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import socketio
from std_msgs.msg import Float32MultiArray

gps_data = [0.0,0.0]
class SocketIOListener(Node):
    def __init__(self):
        super().__init__('socketio_listener')
        self.SERVER_SOCKETIO = "http://10.10.11.93:5001"
        self.ID = "robot1"
        self.NAME = "123"
        self.auto_publisher = self.create_publisher(Bool, '/automatic', 10)
        self.cmd_vel_sub = self.create_subscription(Float32MultiArray, "/gps", self.gps_callback, 10)
        self.sio = socketio.Client()

        @self.sio.event
        def connect():
            self.get_logger().info('Socket.IO connected')

        @self.sio.event
        def disconnect():
            self.get_logger().info('Socket.IO disconnected')
            
        
        # Define event handlers
        @self.sio.on('connect')
        def on_connect():
            print("Connected to server ...")
            self.sio.emit("register_controller", {"robot_id" : self.ID, "robot_name" : self.NAME})

        @self.sio.on('register_controller')
        def on_message(data):
            print("Message received:", data)
            
        @self.sio.on("robot_location") 
        def thread_location():
            global gps_data
            while True:    
                try:
                    self.sio.emit("robot_location",{"robot_id" : self.ID, "location": gps_data})
                    self.time.sleep(0.1)
                    print("send to server")
                except:
                    pass
           
        @self.sio.on("automatic")
        def on_run_automatic(data):
            print(data['type'])
            msg = Bool()
            if data['type'] == 'Go':
                msg.data = True
            else:
                msg.data = False
            self.auto_publisher.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)
                 
        @self.sio.on('disconnect')
        def on_disconnect():
            print("Disconnected from server")
    
    def gps_callback(self, data_msg: Float32MultiArray):
        global gps_data
        gps_data = data_msg.data[0:2]
        
        
    def start(self):
        self.sio.connect(self.SERVER_SOCKETIO) # Replace with your Socket.IO server URL
        rclpy.spin(self)

    def stop(self):
        self.sio.disconnect()

def main(args=None):
    rclpy.init(args=args)
    socketio_listener = SocketIOListener()
    try:
        socketio_listener.start()
    except KeyboardInterrupt:
        pass
    socketio_listener.stop()
    socketio_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
