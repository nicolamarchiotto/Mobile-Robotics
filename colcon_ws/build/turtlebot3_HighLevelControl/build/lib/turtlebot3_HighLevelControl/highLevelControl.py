import rclpy
from rclpy.node import Node
import rclpy.qos

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan, Imu
import numpy as np
import math
import time
import threading

class Turtlebot3HighLevelControl(Node):

    def __init__(self):
        super().__init__('turtlebot3_HighLevelControl_node')
        # definition of publisher and subscriber object to /cmd_vel and /scan 
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 1)
        self.subscription = self.create_subscription(LaserScan, '/scan', self.laser_callback, rclpy.qos.qos_profile_sensor_data)
        
        # initial state of FSM
        self.state_ = 0

        #initialization dict of lidar regions
        self.regions = {
            'front': 0,
            'right': 0,
            'left': 0,
        }
        # definition of dict with state of FSM
        self.state_dict_ = {
            0: 'find the wall',
            1: 'align left',
            2: 'follow the wall',
        }
        # velocity command
        self.msg = Twist()

        # distance threshold to the wall
        self.th = 0.15

        timer_period = 0.1  # seconds

        self.timer = self.create_timer(timer_period, self.control_loop)


    # loop each 0.1 seconds
    def control_loop(self):
    
        # actions for states 
        if self.state_ == 0:
            self.find_wall()
        elif self.state_ == 1:
            self.align_left()
        elif self.state_ == 2:
            self.follow_the_wall()
            pass
        else:
            print("Unknown state")

        # print("\nregions ", self.regions)
        self.publisher_.publish(self.msg)

            
    # laser scanner callback
    def laser_callback(self, msg):
        # populate the lidar reagions with minimum distance 
        # where you find <ranges> you have to read from msg the desired interval
        # I suggesto to do parametric the ranges in this way you don't have to change the value for the real robot 

        
        ranges=msg.ranges
        dim=math.floor(len(ranges)/3) #120
        step=math.floor(dim/2) #60

        rangesTopRight=ranges[0:step]
        rangesTopLeft=ranges[(len(ranges)-step): len(ranges)]

        rangesRight=ranges[step: (3*step)]
        rangesLeft=ranges[(3*step): (5*step)]

        # print("\nranges len ", len(ranges))
        # print("\ndim ", dim)
        # print("\nstep ", step)
        # print("\n rangesTopLeft len ", len(rangesTopLeft))
        # print("\n rangesTopRight len ", len(rangesTopRight))
        # print("\n rangesRight len ", len(rangesRight))
        # print("\n rangesLeft len ", len(rangesLeft))

        # print("\ranges ", ranges)
        # print("\r\n\n\angesTopLeft ", rangesTopLeft)

    

        self.regions = {
        'front':  min(min(min(rangesTopLeft), 10), min(min(rangesTopRight), 10)),
        'left':  min(min(rangesLeft), 10),
        'right':  min(min(rangesRight), 10),
        }

        # function where are definied the rules for the change state
        self.take_action()


    def take_action(self):
        # you have to implement the if condition usign the lidar regions and threshold
        # you can add or remove statement if you prefere
        # call change_state function with the state index to enable the change state

        # self.state_dict_ = {
        #     0: 'find the wall',
        #     1: 'align left',
        #     2: 'follow the wall',
        # }

        if self.regions['right']>self.th:
            self.change_state(0)
        elif self.regions['front']<self.th:
            self.change_state(1)
        elif self.regions['front']>self.th and self.regions['left']>self.th and self.regions['right']<self.th:
            self.change_state(2)
        else:
            print(self.regions)

    # function to update state
    # don't modify the function
    def change_state(self, state):
        
        if state is not self.state_:
            print('Wall follower - [%s] - %s' % (state, self.state_dict_[state]))
            self.state_ = state


    # action to find the wall, move forward and wall side to find the wall
    def find_wall(self):
        # write velocity commands using the class variable self.msg
        print("find")
        self.msg.linear.x  = 0.06
        self.msg.angular.z = 0.05

    # action to torn left, move forward and left side
    def align_left(self):
        # write velocity commands using the class variable self.msg
        print("align")
        self.msg.linear.x  = 0.0
        self.msg.angular.z = 0.5

    # action to follow the wall, move forward 
    def follow_the_wall(self):
        # write velocity commands using the class variable self.msg
        print("follow")
        self.msg.linear.x  = 0.1
        self.msg.angular.z = 0.0



def main(args=None):
    print("main")
    rclpy.init(args=args)

    turtlebot3_HighLevelControl_node = Turtlebot3HighLevelControl()

    rclpy.spin(turtlebot3_HighLevelControl_node)


    turtlebot3_HighLevelControl_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
