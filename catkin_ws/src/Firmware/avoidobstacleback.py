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

global yamlpath

class ObstacleBackAvoider:

    def __init__(self):
        global yamlpath
        with open(yamlpath) as f:
    
           data = yaml.load(f, Loader=yaml.FullLoader)
           for key, value in data.items():
              if key == "rate_value":
                 rate_value = value

        rospy.init_node("obstacleavoider_back_node") # We initialize the node with the name: obstacleavoider_back_node
        rate = rospy.Rate(rate_value) # Rate of 20 Hz

        self.avoidbackobstacle = rospy.Publisher('gi/avoidobstacle/back', Float32, queue_size=10) # Custom publisher of avoidobstacle
        self.avoidbackobstacle_return = rospy.Publisher('gi/avoidobstacle/back_return', Float32, queue_size=10) # Custom publisher of avoidobstacle
        self.blockmovementback = rospy.Publisher('gi/avoidobstacle/back_block', String, queue_size=10) # Custom publisher of avoidobstacle

    
    # Moves X meters to the left to evit back obstacle
    def avoid_back_obstacle(self, distance_obst_avoid): 

        self.avoidbackobstacle.publish(distance_obst_avoid) # It publishes the float distance to move


    # Moves X meters to the back to remove the distance moved previously
    def avoid_back_obstacle_return(self, distance_obst_avoid): 

        self.avoidbackobstacle_return.publish(distance_obst_avoid) # It publishes the float distance to move


    # Moves X meters to the back to remove the distance moved previously
    def block_back_movement(self, message): 

        self.blockmovementback.publish(message) # It publishes the string message with block or unblock





if __name__ == "__main__": # From here to the end we call all the functions in our order desired

    global yamlpath
    yamlpath = "/home/miguel/catkin_ws/src/Firmware/data.yaml"

    with open(yamlpath) as f:
   
        data = yaml.load(f, Loader=yaml.FullLoader)
        for key, value in data.items():
            if key == "distance_obst_avoid":
                distance_obst_avoid = value

    #print(distance_obst_avoid)    

    avo = ObstacleBackAvoider()


    # Depending on the message we call one function or another

    time.sleep(0.1)

    if sys.argv[1] == "BACK":
        avo.avoid_back_obstacle_return(distance_obst_avoid)

    elif sys.argv[1] == "BLOCK":

        avo.block_back_movement("True")

    elif sys.argv[1] == "UNBLOCK":
        avo.block_back_movement("False")

    else:
        avo.avoid_back_obstacle(distance_obst_avoid)

    time.sleep(0.1)


