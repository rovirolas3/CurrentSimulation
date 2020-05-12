#!/usr/bin/env python
import rospy
from mavros_msgs.msg import GlobalPositionTarget, State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, String
from pyquaternion import Quaternion
import time
import math
import yaml
import sys

class ObstacleAvoider:

    def __init__(self):

        self.yamlpath = "/home/miguel/catkin_ws/src/Firmware/data.yaml"   # ----------------Change the path to your corresponding one--------------------
        with open(self.yamlpath) as f:
    
           data = yaml.load(f, Loader=yaml.FullLoader)
           for key, value in data.items():
              if key == "rate_value":
                 rate_value = value

        rospy.init_node("obstacleavoider_left_node") # We initialize the node with the name: obstacleavoider_left_node
        rate = rospy.Rate(rate_value) # Rate of 20 Hz

        self.avoidobstacle = rospy.Publisher('gi/avoidobstacle/left', String, queue_size=10) # Custom publisher of avoidobstacle

    # Publish the topic with the correspond message
    def movement(self, message): 

        self.avoidobstacle.publish(message) # It publishes the string message


if __name__ == "__main__": # From here to the end we call all the functions in our order desired order

    #print(distance_obst_avoid)    

    avo = ObstacleAvoider()


    # Calls the function to send the corresponding message

    time.sleep(0.1)

    avo.movement(sys.argv[1])

    time.sleep(0.1)


