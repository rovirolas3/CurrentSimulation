#!/usr/bin/env python
import rospy, tf
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from geometry_msgs.msg import *
# from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, Float64, String
import time
from pyquaternion import Quaternion
import math
import threading
import yaml

from gazebo_msgs.srv import DeleteModel, SpawnModel


from random import seed
from random import randint
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Global variables to see if the RFID boxes are spawned or not
global spawned_vehicle_1
global spawned_vehicle_2
global spawned_vehicle_3
global spawned_vehicle_4

class RFIDpublisher:

    def __init__(self):

        global yamlpath        
        yamlpath = '/home/miguel/catkin_ws/src/Firmware/data.yaml'
        with open(yamlpath) as f:
    
           data = yaml.load(f, Loader=yaml.FullLoader)
           for key, value in data.items():
              if key == "RFID_1_x":
                 self.RFID_1_x = value
              if key == "RFID_1_y":
                 self.RFID_1_y = value
              if key == "RFID_2_x":
                 self.RFID_2_x = value
              if key == "RFID_2_y":
                 self.RFID_2_y = value
              if key == "RFID_3_x":
                 self.RFID_3_x = value
              if key == "RFID_3_y":
                 self.RFID_3_y = value
              if key == "RFID_4_x":
                 self.RFID_4_x = value
              if key == "RFID_4_y":
                 self.RFID_4_y = value
              if key == "rate_value":
                 self.rate_value = value
              if key == "RFID_sens_distance":
                 self.RFID_sens_distance = value
              if key == "sdfpath":
                 self.sdfpath = value

        # Pose of the drone (Initialize at 0)
        self.drone_pose_x = 0
        self.drone_pose_y = 0
        self.drone_pose_z = 0

        self.drone_roll = 0
        self.drone_pitch = 0
        self.drone_yaw = 0

        # Variable to know if we had spawn an RFID antenna or not
        global spawned_vehicle_1
        spawned_vehicle_1 = False
        global spawned_vehicle_2
        spawned_vehicle_2 = False
        global spawned_vehicle_3
        spawned_vehicle_3 = False
        global spawned_vehicle_4
        spawned_vehicle_4 = False

        # We will use gazebo services to spawn and delete RFID antennas in the simulation
        print("Waiting for gazebo services...")
        rospy.wait_for_service("gazebo/delete_model")
        rospy.wait_for_service("gazebo/spawn_sdf_model")
        print("Got it.")
        self.delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
        self.spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)

        rospy.init_node("rfid_node") # We initialize the node with the name: rfid_node

        '''
        ros subscribers
        '''
        # All the mavros topics can be found here: http://wiki.ros.org/mavros
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.local_pose_callback) # Subscriber of the local position by mavros 
                                                                                                                     # http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html


        '''
        ros publishers
        '''
        self.rfid_first_pub = rospy.Publisher('gi/rfid/first', PoseStamped, queue_size=10) # Position of the first RFID antenna
        self.rfid_second_pub = rospy.Publisher('gi/rfid/second', PoseStamped, queue_size=10) # Position of the second RFID antenna
        self.rfid_third_pub = rospy.Publisher('gi/rfid/third', PoseStamped, queue_size=10) # Position of the third RFID antenna
        self.rfid_fourth_pub = rospy.Publisher('gi/rfid/fourth', PoseStamped, queue_size=10) # Position of the fourth RFID antenna
        '''
        self.rfid_first_pub = rospy.Publisher('gi/rfid/first', PoseStamped, self.rfid_first_callback) # 
        self.rfid_second_pub = rospy.Publisher('gi/rfid/second', PoseStamped, self.rfid_second_callback) # 
        self.rfid_third_pub = rospy.Publisher('gi/rfid/third', PoseStamped, self.rfid_third_callback) #
        self.rfid_third_pub = rospy.Publisher('gi/rfid/fourth', PoseStamped, self.rfid_fourth_callback) #  
        '''

        print("RFID Antennas Initialized!")



    def start(self):

        '''
        main ROS thread
        '''

        while rospy.is_shutdown() is False: # While we don't shutdown it, do the loop
            
            self.print_RFID_in_range()           

            time.sleep(0.05) # Rate to publish



    # Callback to catch the position of the drone and the orientation
    def local_pose_callback(self, msg): 
        self.local_pose = msg
        self.local_enu_position = msg

        self.drone_pose_x = msg.pose.position.x
        self.drone_pose_y = msg.pose.position.y
        self.drone_pose_z = msg.pose.position.z

        orientation_list = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        (self.drone_roll, self.drone_pitch, self.drone_yaw) = euler_from_quaternion (orientation_list)



    # Function to create the message PoseStamped
    def construct_target(self, x, y, z, yaw, yaw_rate = 1): 
        target_raw_pose = PoseStamped() # We will fill the following message with our values: PoseStamped
        target_raw_pose.header.stamp = rospy.Time.now()
        target_raw_pose.header.frame_id = "RFID"        

 
        target_raw_pose.pose.position.x = x
        target_raw_pose.pose.position.y = y
        target_raw_pose.pose.position.z = z

        target_raw_pose.pose.orientation.x = 0
        target_raw_pose.pose.orientation.y = 0
        target_raw_pose.pose.orientation.z = 0
        target_raw_pose.pose.orientation.w = 0

        return target_raw_pose


    def print_RFID_in_range(self):

        #-------------------------------First RFID Antenna------------------------------------ 
        distance = self.distance_RFID_and_drone(self.RFID_1_x,self.RFID_1_y)

        # If it is in distance
        if distance < self.RFID_sens_distance:
            print('RFID antenna 1 in range ' + str(distance))

            # Calculates the angle of the RFID antenna and the front perspective of the drone - It takes in account orientation!
            angle = self.calculate_yaw_degree_RFID(self.RFID_1_x,self.RFID_1_y)
            angle_respect_orientation = angle - (self.drone_yaw * 180 / math.pi) # We substract the value of the orientation

            # Depending on the angle it represents one antenna or another
            self.print_detected_antenna(angle_respect_orientation)

            # If the RFID antenna it is not in the gazebo simulation, spawn it
            global spawned_vehicle_1
            if spawned_vehicle_1 == False:
             
                self.spawn_rfid_model(self.RFID_1_x, self.RFID_1_y, 1)

                spawned_vehicle_1 = True


            # Publish the location of the RFID antenna into a ROS topic
            self.cur_target_pose_rfid_first = self.construct_target(self.RFID_1_x, self.RFID_1_y, 0, 0) # Construct the message to put in the topic (PoseStamped)
            self.rfid_first_pub.publish(self.cur_target_pose_rfid_first) # Publish the topic


        # If it is NOT in distance
        else: 
            print('RFID antenna 1 NOT in range ' + str(distance))

            # Calculates the angle of the RFID antenna and the front perspective of the drone - It takes in account orientation!
            angle = self.calculate_yaw_degree_RFID(self.RFID_1_x,self.RFID_1_y)
            angle_respect_orientation = angle - (self.drone_yaw * 180 / math.pi)

            '''
            self.print_detected_antenna(angle_respect_orientation)
            '''
            # If the RFID antenna it is in the gazebo simulation, delete it
            global spawned_vehicle_1
            if spawned_vehicle_1 == True:

                self.delete_rfid_model(1)
 
                spawned_vehicle_1 = False


        #-------------------------------Second RFID Antenna------------------------------------ 

        distance = self.distance_RFID_and_drone(self.RFID_2_x,self.RFID_2_y)

        if distance < self.RFID_sens_distance:
            print('RFID antenna 2 in range ' + str(distance))

            angle = self.calculate_yaw_degree_RFID(self.RFID_2_x,self.RFID_2_y)
            angle_respect_orientation = angle - (self.drone_yaw * 180 / math.pi)

            self.print_detected_antenna(angle_respect_orientation)

            global spawned_vehicle_2
            if spawned_vehicle_2 == False:

                self.spawn_rfid_model(self.RFID_2_x, self.RFID_2_y, 2)

                spawned_vehicle_2 = True

            self.cur_target_pose_rfid_second = self.construct_target(self.RFID_2_x, self.RFID_2_y, 0, 0) 
            self.rfid_second_pub.publish(self.cur_target_pose_rfid_second) 

        else: 
            print('RFID antenna 2 NOT in range ' + str(distance))

            angle = self.calculate_yaw_degree_RFID(self.RFID_2_x,self.RFID_2_y)
            angle_respect_orientation = angle - (self.drone_yaw * 180 / math.pi)

            '''
            self.print_detected_antenna(angle_respect_orientation)
            '''
           
            global spawned_vehicle_2
            if spawned_vehicle_2 == True:

                self.delete_rfid_model(2)
 
                spawned_vehicle_2 = False

        #-------------------------------Third RFID Antenna------------------------------------ 

        distance = self.distance_RFID_and_drone(self.RFID_3_x,self.RFID_3_y)

        if distance < self.RFID_sens_distance:
            print('RFID antenna 3 in range ' + str(distance))

            angle = self.calculate_yaw_degree_RFID(self.RFID_3_x,self.RFID_3_y)
            angle_respect_orientation = angle - (self.drone_yaw * 180 / math.pi)

            self.print_detected_antenna(angle_respect_orientation)

            global spawned_vehicle_3
            if spawned_vehicle_3 == False:

                self.spawn_rfid_model(self.RFID_3_x, self.RFID_3_y, 3)

                spawned_vehicle_3 = True

            self.cur_target_pose_rfid_third = self.construct_target(self.RFID_3_x, self.RFID_3_y, 0, 0) 
            self.rfid_third_pub.publish(self.cur_target_pose_rfid_third) 

        else: 
            print('RFID antenna 3 NOT in range ' + str(distance))

            angle = self.calculate_yaw_degree_RFID(self.RFID_3_x,self.RFID_3_y)
            angle_respect_orientation = angle - (self.drone_yaw * 180 / math.pi)

            '''
            self.print_detected_antenna(angle_respect_orientation)
            '''
           
            global spawned_vehicle_3
            if spawned_vehicle_3 == True:

                self.delete_rfid_model(3)
 
                spawned_vehicle_3 = False


        #-------------------------------Fourth RFID Antenna------------------------------------ 

        distance = self.distance_RFID_and_drone(self.RFID_4_x,self.RFID_4_y)

        if distance < self.RFID_sens_distance:
            print('RFID antenna 4 in range ' + str(distance))

            angle = self.calculate_yaw_degree_RFID(self.RFID_4_x,self.RFID_4_y)
            angle_respect_orientation = angle - (self.drone_yaw * 180 / math.pi)

            self.print_detected_antenna(angle_respect_orientation)

            global spawned_vehicle_4
            if spawned_vehicle_4 == False:

                self.spawn_rfid_model(self.RFID_4_x, self.RFID_4_y, 4)

                spawned_vehicle_4 = True

            self.cur_target_pose_rfid_fourth = self.construct_target(self.RFID_4_x, self.RFID_4_y, 0, 0) 
            self.rfid_fourth_pub.publish(self.cur_target_pose_rfid_fourth)  

        else: 
            print('RFID antenna 4 NOT in range ' + str(distance))

            angle = self.calculate_yaw_degree_RFID(self.RFID_4_x,self.RFID_4_y)
            angle_respect_orientation = angle - (self.drone_yaw * 180 / math.pi)

            '''
            self.print_detected_antenna(angle_respect_orientation)
            '''
           
            global spawned_vehicle_4
            if spawned_vehicle_4 == True:

                item_name = "rfid_4"
                print("Deleting model: ", item_name)
                self.delete_model(item_name)
 
                spawned_vehicle_4 = False
     
    # -------------------------------------------------------------------------------------------------------------    


    # Function that calculates the angle between the drone and the RFID antenna 
    def calculate_yaw_degree_RFID(self, x, y):
        difference_x = x - self.drone_pose_x
        difference_y = y - self.drone_pose_y
      
        difference_hipotenuse = math.sqrt((difference_x**2)+(difference_y**2))
        
        angle = math.atan(difference_x/difference_y)
        return(angle * 180 / math.pi)


    # Function that calculates the distance between the drone and the RFID antenna
    def distance_RFID_and_drone(self, x, y):
        raw_distance = math.sqrt(((self.drone_pose_x - x)**2)+((self.drone_pose_y - y)**2))
        return raw_distance
        

    # Function that prints which antenna has detected the RFID
    def print_detected_antenna(self, angle_respect_orientation):
        print (angle_respect_orientation)

        if (angle_respect_orientation > -45) and (angle_respect_orientation < 45):
            print ("Detected by the front antenna")

        elif (angle_respect_orientation > 45) and (angle_respect_orientation < 135):
            print ("Detected by the left antenna") 

        elif (angle_respect_orientation < -45) and (angle_respect_orientation > -135):
            print ("Detected by the right antenna")
        
        else:
            print ("Detected by the back antenna")

    # Function that spawn an RFID antenna
    def spawn_rfid_model(self, RFID_x, RFID_y, number):
        with open(self.sdfpath, "r") as f:
            product_xml = f.read()

        orient = Quaternion(0,0,0,0)
                  
        item_name   =   "rfid_" + str(number)
        print("Spawning model: ", item_name)
        item_pose   =   Pose(Point(x=RFID_x, y=RFID_y,    z=0.2),   orient)
        self.spawn_model(item_name, product_xml, "", item_pose, "world")
     
    # Function that delete an RFID antenna
    def delete_rfid_model(self, number):
        item_name = "rfid_"+str(number)
        print("Deleting model: ", item_name)
        self.delete_model(item_name)




#########################################################################################################################

if __name__ == '__main__': # From here to the end calls the functions desired
    con = RFIDpublisher()
    con.start()

