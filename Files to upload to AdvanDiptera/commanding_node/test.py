#!/usr/bin/env python

import rospy
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget
from sensor_msgs.msg import Imu, NavSatFix
from quaternion import Quaternion
from geometry_msgs.msg import PoseStamped, Twist
import time
import yaml
from sensor_msgs.msg import Range
from std_msgs.msg import Float32, Float64, String

#from Diptera_move_drone import Move_Drone as Basic_movement
#from Diptera_move_drone_velocity import Move_Drone as Basic_movement
from Diptera_move_drone_version2 import Move_Drone as Basic_movement
class Testing:

    def __init__(self):

        self.threshold = 0.3                      # Can be changed in params.yaml
        self.down_sensor_distance = 0
        self.current_heading = int
        self.flying_status = str

        rospy.init_node("Test_node")
        self.imu_sub = rospy.Subscriber("/mavros/imu/data", Imu, self.cb_imu)
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.cb_local_pose)

        self.flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.custom_activity_pub = rospy.Publisher("/hover_status", String, queue_size=10)


    def cb_imu(self, msg):
        global global_imu, current_heading
        self.imu = msg
        self.current_heading = self.q2yaw(self.imu.orientation) # Transforms q into degrees of yaw
        self.received_imu = True

    def q2yaw(self, q):
        if isinstance(q, Quaternion): # Checks if the variable is of the type Quaternion
            rotate_z_rad = q.yaw_pitch_roll[0]
        else:
            q_ = Quaternion(q.w, q.x, q.y, q.z) # Converts into Quaternion
            rotate_z_rad = q_.yaw_pitch_roll[0]
        return rotate_z_rad 

    def cb_local_pose(self, msg):
        self.local_pose = msg
        self.local_enu_position = msg

    def start(self):
        for i in range(10): # Waits 5 seconds for initialization
            if self.current_heading is not None:
                break
            else:
                print("Waiting for initialization.")
                time.sleep(0.5)

        Basic_movement().test_x()      # Version 2
#        Basic_movement().test()        # Velocity


        
       


if __name__ == '__main__':
    takeoff = Testing()
    takeoff.start()
    time.sleep(5) 




