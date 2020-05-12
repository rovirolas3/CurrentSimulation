#!/usr/bin/env python
import rospy
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, Float64, String
import time
from pyquaternion import Quaternion
import math
import threading
import yaml


from random import seed
from random import randint



# Path of the yaml file

class Px4Controller:

    def __init__(self):
       
        self.yamlpath = '/home/miguel/catkin_ws/src/Firmware/data.yaml'
        with open(self.yamlpath) as f:
    
           data = yaml.load(f, Loader=yaml.FullLoader)
           for key, value in data.items():
              if key == "takeoff_height":
                 self.takeoff_height = value
              if key == "threshold_ground":
                 self.threshold_ground = value
              if key == "threshold_ground_minor":
                 self.threshold_ground_minor = value
              if key == "imu":
                 self.imu = value
              if key == "gps":
                 self.gps = value
              if key == "local_pose":
                 self.local_pose = value
              if key == "current_state":
                 self.current_state = value
              if key == "current_heading":
                 self.current_heading = value
              if key == "local_enu_position":
                 self.local_enu_position = value
              if key == "local_enu_position":
                 self.local_enu_position = value
              if key == "cur_target_pose":
                 self.cur_target_pose = value
              if key == "global_target":
                 self.global_target = value
              if key == "received_new_task":
                 self.received_new_task = value
              if key == "arm_state":
                 self.arm_state = value
              if key == "offboard_state":
                 self.offboard_state = value
              if key == "received_imu":
                 self.received_imu = value
              if key == "frame":
                 self.frame = value
              if key == "state":
                 self.state = value
              if key == "moving_random_distance":
                 self.moving_random_distance = value
              if key == "distance_obst_avoid":
                 self.distance_obst_avoid = value 
        
        # Counter where we know how much times we have moved from the desired position
        self.timesMovedFromPositionDesiredRight = 0  
        self.timesMovedFromPositionDesiredLeft = 0  
        self.timesMovedFromPositionDesiredUp = 0  
        self.timesMovedFromPositionDesiredDown = 0  
        self.timesMovedFromPositionDesiredBack = 0  
        self.timesMovedFromPositionDesiredFront = 0

        # Initialize the variables to False

        self.blockingMovementRight = False   
        self.blockingMovementLeft = False 
        self.blockingMovementUp = False 
        self.blockingMovementDown = False
        self.blockingMovementBack = False
        self.blockingMovementFront = False 

        # Initialize the variable to not previous movement
        # Previous movement position 
        # 0 right
        # 1 left
        # 2 back
        # 3 front
        # -1 None previous movement
        self.previousMovement = -1   
    

        '''
        ros subscribers
        '''
        # All the mavros topics can be found here: http://wiki.ros.org/mavros
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.local_pose_callback) # Subscriber of the local position by mavros 
                                                                                                                     # http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html
        self.mavros_sub = rospy.Subscriber("/mavros/state", State, self.mavros_state_callback) # Subscriber of the State by mavros http://docs.ros.org/api/mavros_msgs/html/msg/State.html
        self.gps_sub = rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.gps_callback) # Subscriber of the global position by mavros 
                                                                                                        # http://docs.ros.org/api/sensor_msgs/html/msg/NavSatFix.html
        self.imu_sub = rospy.Subscriber("/mavros/imu/data", Imu, self.imu_callback) # Subscriber of imu data by mavros http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html


        # Subscribe to the 4 custom topics published by the commander.py 
        self.set_target_position_sub = rospy.Subscriber("gi/set_pose/position", PoseStamped, self.set_target_position_callback) 
        self.set_target_yaw_sub = rospy.Subscriber("gi/set_pose/orientation", Float32, self.set_target_yaw_callback)
        self.custom_activity_sub = rospy.Subscriber("gi/set_activity/type", String, self.custom_activity_callback)
        self.custom_takeoff = rospy.Subscriber("gi/set_activity/takeoff", Float32, self.custom_takeoff_callback)

        self.avoid_right_obstacle = rospy.Subscriber("gi/avoidobstacle/right", String, self.avoid_right_obstacle_callback)
        self.avoid_left_obstacle = rospy.Subscriber("gi/avoidobstacle/left", String, self.avoid_left_obstacle_callback)
        self.avoid_up_obstacle = rospy.Subscriber("gi/avoidobstacle/up", String, self.avoid_up_obstacle_callback)
        self.avoid_down_obstacle = rospy.Subscriber("gi/avoidobstacle/down", String, self.avoid_down_obstacle_callback)
        self.avoid_front_obstacle = rospy.Subscriber("gi/avoidobstacle/front", String, self.avoid_front_obstacle_callback)
        self.avoid_back_obstacle = rospy.Subscriber("gi/avoidobstacle/back", String, self.avoid_back_obstacle_callback)

        '''
        ros publishers
        '''
        # It publishes to the position we want to move http://docs.ros.org/api/mavros_msgs/html/msg/PositionTarget.html
        self.local_target_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        
        '''
        ros services
        '''
        
        self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool) # Service of Arming status http://docs.ros.org/api/mavros_msgs/html/srv/CommandBool.html
        self.flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode) # Service mode provided by mavros http://docs.ros.org/api/mavros_msgs/html/srv/SetMode.html
                                                                                 # Custom modes can be seing here: http://wiki.ros.org/mavros/CustomModes


        print("Px4 Controller Initialized!")


    def start(self):
        rospy.init_node("offboard_node") # Initialize the node 
        for i in range(10): # Waits 5 seconds for initialization 
            if self.current_heading is not None:
                break
            else:
                print("Waiting for initialization.")
                time.sleep(0.5)

        self.cur_target_pose = self.construct_target(0, 0, 0, self.current_heading) # Initialize the drone to this current position

        #print ("self.cur_target_pose:", self.cur_target_pose, type(self.cur_target_pose))

        for i in range(10):
            self.local_target_pub.publish(self.cur_target_pose) # Publish the drone position we initialite during the first 2 seconds
            #self.arm_state = self.arm() # Arms the drone (not necessary here)
            self.offboard_state = self.offboard() # Calls the function offboard the will select the mode Offboard
            time.sleep(0.2)

        '''
        main ROS thread
        '''
        #while self.arm_state and self.offboard_state and (rospy.is_shutdown() is False): # The code implemented the arm state, not necesary in our case

        while self.offboard_state and (rospy.is_shutdown() is False): # While offboard state is true and we don't shutdown it, do the loop

            self.local_target_pub.publish(self.cur_target_pose) # Publish to the mavros local_targe_pub our desired new position

            if (self.state is "LAND") and (self.local_pose.pose.position.z < self.threshold_ground_minor): # If we land and we are under 0.15 in the z position...

                if(self.disarm()): # ... we disarm the drone

                    self.state = "DISARMED"


            time.sleep(0.1) # Rate to publish

    # Function to create the message PositionTarget
    def construct_target(self, x, y, z, yaw, yaw_rate = 1): 
        target_raw_pose = PositionTarget() # We will fill the following message with our values: http://docs.ros.org/api/mavros_msgs/html/msg/PositionTarget.html
        target_raw_pose.header.stamp = rospy.Time.now()

        target_raw_pose.coordinate_frame = 9 

        target_raw_pose.position.x = x
        target_raw_pose.position.y = y
        target_raw_pose.position.z = z

        target_raw_pose.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ \
                                    + PositionTarget.IGNORE_AFX + PositionTarget.IGNORE_AFY + PositionTarget.IGNORE_AFZ \
                                    + PositionTarget.FORCE

        target_raw_pose.yaw = yaw
        target_raw_pose.yaw_rate = yaw_rate

        return target_raw_pose



    '''
    cur_p : poseStamped
    target_p: positionTarget
    '''

    # Function that sees if our position and the target one is between our threshold 
    def position_distance(self, cur_p, target_p, threshold=0.1): 
        delta_x = math.fabs(cur_p.pose.position.x - target_p.position.x) # Calculates the absolute error between our position and the target
        delta_y = math.fabs(cur_p.pose.position.y - target_p.position.y)
        delta_z = math.fabs(cur_p.pose.position.z - target_p.position.z)

        if (delta_x + delta_y + delta_z < threshold): # the threshold is between the sum of our three values
            return True
        else:
            return False

    # Callbacks called in the initialization of the subscription to the mavros topics

    def local_pose_callback(self, msg): 
        self.local_pose = msg
        self.local_enu_position = msg


    def mavros_state_callback(self, msg):
        self.mavros_state = msg.mode


    def imu_callback(self, msg):
        global global_imu, current_heading
        self.imu = msg

        self.current_heading = self.q2yaw(self.imu.orientation) # Transforms q into degrees of yaw

        self.received_imu = True


    def gps_callback(self, msg):
        self.gps = msg


    def FLU2ENU(self, msg):

        # Converts the position of the map into the perspective of the drone using the following equations
        FLU_x = msg.pose.position.x * math.cos(self.current_heading) - msg.pose.position.y * math.sin(self.current_heading) # x*cos(alpha) - y*sin(alpha)
        FLU_y = msg.pose.position.x * math.sin(self.current_heading) + msg.pose.position.y * math.cos(self.current_heading) # x*sin(alpha) + y*cos(alpha)
        FLU_z = msg.pose.position.z

        return FLU_x, FLU_y, FLU_z

    # Callback of the target position
    def set_target_position_callback(self, msg):

        print("Received New Position Task!")

        # Depending of what are we looking we will look for a position respect the drone or respect the map
        if msg.header.frame_id == 'base_link': 
            '''
            BODY_FLU
            '''
            # For Body frame, we will use FLU (Forward, Left and Up)
            #           +Z     +X
            #            ^    ^
            #            |  /
            #            |/
            #  +Y <------body

            self.frame = "BODY"

            print("body FLU frame")

            ENU_X, ENU_Y, ENU_Z = self.FLU2ENU(msg) # Calls this function to get the ENU values


            ENU_X = ENU_X + self.local_pose.pose.position.x
            ENU_Y = ENU_Y + self.local_pose.pose.position.y
            ENU_Z = ENU_Z + self.local_pose.pose.position.z

            self.cur_target_pose = self.construct_target(ENU_X,
                                                         ENU_Y,
                                                         ENU_Z,
                                                         self.current_heading)


        else:
            '''
            LOCAL_ENU
            '''
            # For world frame, we will use ENU (EAST, NORTH and UP)
            #     +Z     +Y
            #      ^    ^
            #      |  /
            #      |/
            #    world------> +X

            self.frame = "LOCAL_ENU"
            print("local ENU frame")

            self.cur_target_pose = self.construct_target(msg.pose.position.x,
                                                         msg.pose.position.y,
                                                         msg.pose.position.z,
                                                         self.current_heading)

    '''
     Receive A Custom Activity
     '''
    # Custom activities depending on the String we are introducing

    def custom_activity_callback(self, msg):

        print("Received Custom Activity:", msg.data)

        if msg.data == "LAND": # Lands the drone ( z=0.05 so it is minor than the z we introduced 0.15 as minimum to disarm the drone)
            print("LANDING!")
            rospy.loginfo("Landing!")
            self.state = "LAND"
            self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x,
                                                         self.local_pose.pose.position.y,
                                                         0.05,
                                                         self.current_heading)
        elif msg.data == "TAKEOFF": # takeoff the drone into 0.1 position 
            print("TAKING OFF!")
            rospy.loginfo("Taking Off!")
            self.state = "TAKEOFF"
            print("Taking of X:")
            print(self.local_pose.pose.position.x)
            print("Taking of Y:")
            print(self.local_pose.pose.position.y)
            self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x,
                                                         self.local_pose.pose.position.y,
                                                         self.takeoff_height,
                                                         self.current_heading)
            self.arm_state = self.arm() # Arms the drone
 

            #if self.takeoff_detection(): # If it has been initialized correctly it will print one thing or the other
            #    print("Vehicle Took Off!")

            #else:
            #    print("Vehicle Took Off Failed!")
            #    return
 

        elif msg.data == "HOVER": # Hover the drone
            print("HOVERING!")
            rospy.loginfo("Hovering!")
            self.state = "HOVER"
            self.hover()


        elif msg.data == "RANDOMMOVE": # Creates a random movement to a location with no obstacles

            print("RANDOM MOVE!")
            
            print('IS DIRECTION BLOCKED?')
            print('Blocking Right: ' + str(self.blockingMovementRight))
            print('Blocking Left: ' + str(self.blockingMovementLeft))
            print('Blocking Back: ' + str(self.blockingMovementBack))
            print('Blocking Front: ' + str(self.blockingMovementFront))
            print('DIRECTIONS NOT TAKEN IN ACCOUNT:')
            print('Blocking Up: ' + str(self.blockingMovementUp))
            print('Blocking Down: ' + str(self.blockingMovementDown))
            print('DIRECTION CHOSEN:')

            # Previous movement position 
            # 0 right
            # 1 left
            # 2 back
            # 3 front

            exitloop = False

            if self.previousMovement == -1:
                exitloop = False

            elif self.previousMovement == 0:
                if self.blockingMovementRight == False: 
                    self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x,
                                                                 self.local_pose.pose.position.y - self.moving_random_distance,
                                                                 self.local_pose.pose.position.z,
                                                                 self.current_heading)
                    print("Random Movement ---> Still Right")
                    rospy.loginfo("Random Movement: Still Right!")
                    exitloop = True

            elif self.previousMovement == 1:
                if self.blockingMovementLeft == False: 
                    self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x,
                                                                 self.local_pose.pose.position.y + self.moving_random_distance,
                                                                 self.local_pose.pose.position.z,
                                                                 self.current_heading)
                    print("Random Movement ---> Still Left")
                    rospy.loginfo("Random Movement: Still Left!")
                    exitloop = True

            elif self.previousMovement == 2:
                if self.blockingMovementBack == False: 
                    self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x - self.moving_random_distance,
                                                                 self.local_pose.pose.position.y,
                                                                 self.local_pose.pose.position.z,
                                                                 self.current_heading)
                    print("Random Movement ---> Still Back")
                    rospy.loginfo("Random Movement: Still Back!")
                    exitloop = True

            elif self.previousMovement == 3:
                if self.blockingMovementFront == False: 
                    self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x + self.moving_random_distance,
                                                                 self.local_pose.pose.position.y,
                                                                 self.local_pose.pose.position.z,
                                                                 self.current_heading)
                    print("Random Movement ---> Still Front")
                    rospy.loginfo("Random Movement: Still Front!")
                    exitloop = True


            # If it is blocked or first random movement, generate random movement
            while exitloop == False: 

                  # generate random integer values
                  value = randint(0, 3) # Random int value between 0 and 3

                  if value == 0:
                      if self.blockingMovementRight == False:
                          exitloop = True
                          self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x,
                                                                       self.local_pose.pose.position.y - self.moving_random_distance,
                                                                       self.local_pose.pose.position.z,
                                                                       self.current_heading)
                          print("Random Movement ---> Right")
                          rospy.loginfo("Random Movement: Right!")
                          self.previousMovement = 0

                  elif value == 1:
                      if self.blockingMovementLeft == False:
                          exitloop = True
                          self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x,
                                                                       self.local_pose.pose.position.y + self.moving_random_distance,
                                                                       self.local_pose.pose.position.z,
                                                                       self.current_heading)
                          print("Random Movement ---> Left")
                          rospy.loginfo("Random Movement: Left!")
                          self.previousMovement = 1

                  elif value == 2:
                      if self.blockingMovementBack == False:
                          exitloop = True
                          self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x - self.moving_random_distance,
                                                                       self.local_pose.pose.position.y,
                                                                       self.local_pose.pose.position.z,
                                                                       self.current_heading)
                          print("Random Movement ---> Back")
                          rospy.loginfo("Random Movement: Back!")
                          self.previousMovement = 2

                  elif value == 3:
                      if self.blockingMovementFront == False:
                          exitloop = True
                          self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x + self.moving_random_distance,
                                                                       self.local_pose.pose.position.y,
                                                                       self.local_pose.pose.position.z,
                                                                       self.current_heading)
                          print("Random Movement ---> Front")
                          rospy.loginfo("Random Movement: Front!")
                          self.previousMovement = 3

            print ("--------------------------------")
  

        else:
            print("Received Custom Activity:", msg.data, "not supported yet!")


    # Takes off the drone in the desired height we have introduced
    def custom_takeoff_callback(self, msg): 
        print("Received Custom Take Off!")
        rospy.loginfo("Custom Take Off!")

        self.state = "TAKEOFF"
        self.cur_target_pose = self.construct_target(0, 0, msg.data, self.current_heading) # Sets the desired position
        self.arm_state = self.arm() # Arms the drone
        
        #if self.takeoff_detection(): # Detect if the drone has takeoff correctly or not
        #    print("Vehicle Took Off!")

        #else:
        #    print("Vehicle Took Off Failed!")
        #    return
        


# -------------------------------------------SONAR SENSORS-----------------------------------------------------------


# -------------------------------------------RIGHT-------------------------------------------------------------------

    def avoid_right_obstacle_callback(self, msg): # Makes the drone moves X meters in the left direction to avoid an obstacle

        if msg.data == "BLOCK":

            self.blockingMovementRight = True

            #print("Blocking Movement Right!")

        elif msg.data == "UNBLOCK": 

            self.blockingMovementRight = False

            #print("Unblocking Movement Right!")

        elif msg.data == "BACK": 

            #print("Possibility to return Right!")

            if self.state is "HOVER": # If the drone is hovering (not taking off or landing)
                self.blockingMovementRight = False
                if self.timesMovedFromPositionDesiredRight > 0: # If the drone has moved from the desired position previously return X meters to this position
                    # Sets the desired position
                    self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x, self.local_pose.pose.position.y - self.distance_obst_avoid, self.local_pose.pose.position.z, self.current_heading)  
                    self.timesMovedFromPositionDesiredRight = self.timesMovedFromPositionDesiredRight - 1

        elif msg.data == "EVIT":

            #print("Received Avoiding Right Obstacle!")

            if self.state is "HOVER": # If the drone is hovering (not taking off or landing)
                # Sets the desired position
                self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x, self.local_pose.pose.position.y + self.distance_obst_avoid, self.local_pose.pose.position.z, self.current_heading)
                self.timesMovedFromPositionDesiredRight = self.timesMovedFromPositionDesiredRight + 1
                self.blockingMovementRight = True
                rospy.logwarn("Avoiding Right Obstacle") 
   
             
# -------------------------------------------LEFT-------------------------------------------------------------------

    def avoid_left_obstacle_callback(self, msg): # Makes the drone moves X meters in the left direction to avoid an obstacle

        if msg.data == "BLOCK":

            self.blockingMovementLeft = True

            #print("Blocking Movement Left!")

        elif msg.data == "UNBLOCK": 

            self.blockingMovementLeft = False

            #print("Unblocking Movement Left!")

        elif msg.data == "BACK": 

            #print("Possibility to return Left!")

            if self.state is "HOVER": # If the drone is hovering (not taking off or landing)
                self.blockingMovementLeft = False
                if self.timesMovedFromPositionDesiredLeft > 0: # If the drone has moved from the desired position previously return X meters to this position
                    # Sets the desired position
                    self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x, self.local_pose.pose.position.y + self.distance_obst_avoid, self.local_pose.pose.position.z, self.current_heading)  
                    self.timesMovedFromPositionDesiredLeft = self.timesMovedFromPositionDesiredLeft - 1

        elif msg.data == "EVIT":

            #print("Received Avoiding Left Obstacle!")

            if self.state is "HOVER": # If the drone is hovering (not taking off or landing)
                # Sets the desired position
                self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x, self.local_pose.pose.position.y - self.distance_obst_avoid, self.local_pose.pose.position.z, self.current_heading)
                self.timesMovedFromPositionDesiredLeft = self.timesMovedFromPositionDesiredLeft + 1
                self.blockingMovementLeft = True
                rospy.logwarn("Avoiding Left Obstacle")  


# -------------------------------------------UP-------------------------------------------------------------------

    def avoid_up_obstacle_callback(self, msg): # Makes the drone moves X meters in the down direction to avoid an obstacle

        if msg.data == "BLOCK":

            self.blockingMovementUp = True

            #print("Blocking Movement Up!")

        if msg.data == "UNBLOCK":

            self.blockingMovementUp = False

            #print("Unblocking Movement Up!")

        elif msg.data == "BACK":

            #print("Possibility to return Up!")

            if self.state is not "LAND": # If the drone is not landing
                self.blockingMovementUp = False
                if self.timesMovedFromPositionDesiredUp > 0: # If the drone has moved from the desired position previously return X meters to this position
                    # Sets the desired position
                    self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x, self.local_pose.pose.position.y, self.local_pose.pose.position.z + self.distance_obst_avoid, self.current_heading)  
                    self.timesMovedFromPositionDesiredUp = self.timesMovedFromPositionDesiredUp - 1 

        elif msg.data == "EVIT":

            #print("Received Avoiding Up Obstacle!")

            self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x, self.local_pose.pose.position.y, self.local_pose.pose.position.z - self.distance_obst_avoid, self.current_heading)
            self.timesMovedFromPositionDesiredUp = self.timesMovedFromPositionDesiredUp + 1 
            self.blockingMovementUp = True
            rospy.logwarn("Avoiding Up Obstacle")


# -------------------------------------------DOWN-------------------------------------------------------------------

    def avoid_down_obstacle_callback(self, msg): # Makes the drone moves X meters in the left direction to avoid an obstacle

        if msg.data == "BLOCK":

            self.blockingMovementDown = True

            #print("Blocking Movement Down!")

        if msg.data == "UNBLOCK":

            self.blockingMovementDown = False

            #print("Unblocking Movement Down!")

        elif msg.data == "BACK":

            #print("Possibility to return Down!")

            if self.state is "HOVER": # If the drone is hovering (not taking off or landing)
                self.blockingMovementDown = False
                if self.timesMovedFromPositionDesiredDown > 0: # If the drone has moved from the desired position previously return X meters to this position
                    # Sets the desired position
                    self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x, self.local_pose.pose.position.y, self.local_pose.pose.position.z - self.distance_obst_avoid, self.current_heading)  
                    self.timesMovedFromPositionDesiredDown = self.timesMovedFromPositionDesiredDown - 1 


        elif msg.data == "EVIT":

            #print("Received Avoiding Down Obstacle!")

            if self.state is "HOVER": # If the drone is hovering (not taking off or landing)
                # Sets the desired position
                self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x, self.local_pose.pose.position.y, self.local_pose.pose.position.z + self.distance_obst_avoid, self.current_heading)
                self.timesMovedFromPositionDesiredDown = self.timesMovedFromPositionDesiredDown + 1 
                self.blockingMovementDown = True
                rospy.logwarn("Avoiding Down Obstacle")


# -------------------------------------------FRONT-------------------------------------------------------------------

    def avoid_front_obstacle_callback(self, msg): # Makes the drone moves X meters in the left direction to avoid an obstacle

        if msg.data == "BLOCK":

            self.blockingMovementFront = True

            #print("Blocking Movement Front!")

        if msg.data == "UNBLOCK":

            self.blockingMovementFront = False

            #print("Unblocking Movement Front!")

        elif msg.data == "BACK":

            #print("Possibility to return Front!")

            if self.state is "HOVER": # If the drone is hovering (not taking off or landing)
                self.blockingMovementFront = False
                if self.timesMovedFromPositionDesiredFront > 0: # If the drone has moved from the desired position previously return X meters to this position
                    # Sets the desired position
                    self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x + self.distance_obst_avoid, self.local_pose.pose.position.y, self.local_pose.pose.position.z, self.current_heading)  
                    self.timesMovedFromPositionDesiredFront = self.timesMovedFromPositionDesiredFront - 1 

        elif msg.data == "EVIT":

            #print("Received Avoiding Front Obstacle!")

            if self.state is "HOVER": # If the drone is hovering (not taking off or landing)
                # Sets the desired position
                self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x - self.distance_obst_avoid, self.local_pose.pose.position.y , self.local_pose.pose.position.z, self.current_heading)
                self.timesMovedFromPositionDesiredFront = self.timesMovedFromPositionDesiredFront + 1
                self.blockingMovementFront = True
                rospy.logwarn("Avoiding Front Obstacle") 

 
# -------------------------------------------BACK-------------------------------------------------------------------

    def avoid_back_obstacle_callback(self, msg): # Makes the drone moves X meters in the left direction to avoid an obstacle

        if msg.data == "BLOCK":

            self.blockingMovementBack = True

            #print("Blocking Movement Back!")

        if msg.data == "UNBLOCK":

            self.blockingMovementBack = False

            #print("Unblocking Movement Back!")

        elif msg.data == "BACK":

            #print("Possibility to return Back!")

            if self.state is "HOVER": # If the drone is hovering (not taking off or landing)
                self.blockingMovementBack = False
                if self.timesMovedFromPositionDesiredBack > 0: # If the drone has moved from the desired position previously return X meters to this position
                    # Sets the desired position
                    self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x - self.distance_obst_avoid, self.local_pose.pose.position.y, self.local_pose.pose.position.z, self.current_heading)  
                    self.timesMovedFromPositionDesiredBack = self.timesMovedFromPositionDesiredBack - 1


        elif msg.data == "EVIT":

            #print("Received Avoiding Back Obstacle!")

            if self.state is "HOVER": # If the drone is hovering (not taking off or landing)
                # Sets the desired position
                self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x + self.distance_obst_avoid, self.local_pose.pose.position.y, self.local_pose.pose.position.z, self.current_heading)
                self.timesMovedFromPositionDesiredBack = self.timesMovedFromPositionDesiredBack + 1
                self.blockingMovementBack = True
                rospy.logwarn("Avoiding Back Obstacle") 


# --------------------------------------------------------------------------------------------------------------


    def set_target_yaw_callback(self, msg): 
        print("Received New Yaw Task!")

        yaw_deg = msg.data * math.pi / 180.0 # Converts the data into degrees from radians (pi = 180 degrees)
        self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x,
                                                     self.local_pose.pose.position.y,
                                                     self.local_pose.pose.position.z,
                                                     yaw_deg)

    '''
    return yaw from current IMU
    '''

    # It returns the degrees we need to move, our desired rotation
    def q2yaw(self, q):
        if isinstance(q, Quaternion): # Checks if the variable is of the type Quaternion
            rotate_z_rad = q.yaw_pitch_roll[0]
        else:
            q_ = Quaternion(q.w, q.x, q.y, q.z) # Converts into Quaternion
            rotate_z_rad = q_.yaw_pitch_roll[0]

        return rotate_z_rad 

    #Arms the drone
    def arm(self): 
        if self.armService(True):
            rospy.loginfo("Drone armed!")
            return True
        else:
            print("Vehicle arming failed!")
            rospy.logerr("Drone arming failed!")
            return False

    #Disarms the drone
    def disarm(self): 

        self.timesMovedFromPositionDesiredRight = 0  
        self.timesMovedFromPositionDesiredLeft = 0   
        self.timesMovedFromPositionDesiredUp = 0  
        self.timesMovedFromPositionDesiredDown = 0  
        self.timesMovedFromPositionDesiredBack = 0 
        self.timesMovedFromPositionDesiredFront = 0

        if self.armService(False):
            rospy.loginfo("Drone disarmed!")
            return True
        else:
            print("Vehicle disarming failed!")
            rospy.logerr("Drone disarming failed!")
            return False

    # Initialize the drone into offboard service
    def offboard(self): 
        if self.flightModeService(custom_mode='OFFBOARD'):
            return True
        else:
            print("Vehicle Offboard failed")
            rospy.loginfo("Vehicle Offboard failed!")
            return False

    # Hover the drone in the actual position
    def hover(self): 

        self.cur_target_pose = self.construct_target(self.local_pose.pose.position.x,
                                                     self.local_pose.pose.position.y,
                                                     self.local_pose.pose.position.z,
                                                     self.current_heading)
    # Detects if the drone has takeoff correctly
    def takeoff_detection(self): 
        if self.local_pose.pose.position.z > self.threshold_ground_minor and self.offboard_state and self.arm_state:
            return True
        else:
            return False


#########################################################################################################################

if __name__ == '__main__': # From here to the end calls the functions desired
    con = Px4Controller()
    con.start()

