from pop import LiDAR, Pilot
import time
lidar = LiDAR.Rplidar()
Car = Pilot.AutoCar()
lidar.connect()
lidar.stopMotor()