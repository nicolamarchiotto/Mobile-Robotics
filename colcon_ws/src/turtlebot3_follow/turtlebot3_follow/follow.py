import rclpy
from rclpy.node import Node
import rclpy.qos

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Imu
import numpy as np

import time
import threading

class Turtlebot3Follow(Node):

    def __init__(self):
        super().__init__('turtlebot3_follow_node')

        # definition of publisher and subscriber object to /cmd_vel and /scan 
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 1)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.laser_callback, rclpy.qos.qos_profile_sensor_data)

        self.angle_min = 0
        self.angle_increment = 0
        self.ranges = []

        self.linear_vel = 0.0
        self.angular_vel = 0.0

        # list of distance in angles range
        self.range_view = []
        # list of rays angles in angles range
        self.angle_view = []
        self.stop = False
        timer_period = 0.1  # seconds

        self.timer = self.create_timer(timer_period, self.control_loop)


    # loop each 0.1 seconds
    def control_loop(self):
        msg_pub = Twist()
        msg_pub.linear.x  = self.linear_vel
        msg_pub.angular.z = self.angular_vel
        self.publisher_.publish(msg_pub)

    def stop_robot(self):
        self.stop = True
        
    # called each time new message is published
    def laser_callback(self, msg):


        # stop robot when shutdown node
        if self.stop:
            self.linear_vel = 0.0
            self.angular_vel = 0.0

            msg_pub = Twist()
            msg_pub.linear.x  = 0.0
            msg_pub.angular.z = 0.0
            self.publisher_.publish(msg_pub)
            return
        
        self.range_view.clear()
        self.angle_view.clear()
        


        # Write your code here
        self.ranges=msg.ranges
        self.angle_min=msg.angle_min
        self.angle_increment=msg.angle_increment



        # check sanity of ranges array
        if len(self.ranges) < 100:
            self.linear_vel = 0.0
            self.angular_vel = 0.0
            return


        for index in range(len(self.ranges)-45,len(self.ranges)-1):
            val=np.nan_to_num(self.ranges[index])
            self.range_view.append(val if val!=0 else 500)
            self.angle_view.append(self.angle_min+(index*self.angle_increment)-6.28)

        for index in range(45):
            val=np.nan_to_num(self.ranges[index])
            self.range_view.append(val if val!=0 else 500)
            self.angle_view.append(self.angle_min+(index*self.angle_increment))



        max_distance=0.6
        min_distance=0.2
        max_lin_vel=0.22
        max_ang_vel=0.8 #2.8 burger, 1.82 waffle
    
        val_dist=min(self.range_view)
        val_dist_index=self.range_view.index(val_dist)
        val_angle=self.angle_view[val_dist_index]


        if(val_dist>min_distance and val_dist<max_distance):
            print("Distance "+ str(val_dist))
            self.linear_vel=val_dist**2 if max_lin_vel > val_dist**2 else max_lin_vel 
            self.angular_vel=val_angle**2*np.sign(val_angle) if max_ang_vel > val_angle**2*np.sign(val_angle) else max_ang_vel
        else:
            print("Too close")
            self.linear_vel=0.0
            self.angular_vel=0.0

 

def main(args=None):
    print("Start")

    rclpy.init(args=args)

    turtlebot3_follow_node = Turtlebot3Follow()

    t = threading.Thread(target=rclpy.spin, args=[turtlebot3_follow_node])
    t.start()

    try:
        while rclpy.ok():
            time.sleep(5)
    except KeyboardInterrupt:
        turtlebot3_follow_node.stop_robot()
        time.sleep(0.5)

    # Destroy the node explicitly

    turtlebot3_follow_node.destroy_node()
    rclpy.shutdown()

    t.join()


if __name__ == '__main__':
    main()
