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

from random import random

global front_number
global back_number
global left_number
global right_number


class RFIDpublisher:

    def __init__(self):

        global yamlpath        
        yamlpath = '/home/miguel/catkin_ws/src/Firmware/data.yaml'
        with open(yamlpath) as f:
    
           data = yaml.load(f, Loader=yaml.FullLoader)
           for key, value in data.items():
              if key == "rate_value":
                 self.rate_value = value
              if key == "RFID_sens_distance":
                 self.RFID_sens_distance = value
              if key == "sdfpath":
                 self.sdfpath = value
              if key == "sdfredpath":
                 self.sdfredpath = value

        # Pose of the drone (Initialize at 0)
        self.drone_pose_x = 0
        self.drone_pose_y = 0
        self.drone_pose_z = 0

        self.drone_roll = 0
        self.drone_pitch = 0
        self.drone_yaw = 0

        # Variable to save all the RFID antennas
        self.RFID_List = []


        global front_number
        front_number = 0
        global back_number
        back_number = 0
        global left_number
        left_number = 0
        global right_number
        right_number = 0


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
        
        self.rfid_front_pub = rospy.Publisher('gi/rfid/front', Float32, queue_size=10) # Position of the front RFID antenna
        self.rfid_back_pub = rospy.Publisher('gi/rfid/back', Float32, queue_size=10) # Position of the back RFID antenna
        self.rfid_left_pub = rospy.Publisher('gi/rfid/left', Float32, queue_size=10) # Position of the left RFID antenna
        self.rfid_right_pub = rospy.Publisher('gi/rfid/right', Float32, queue_size=10) # Position of the right RFID antenna
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
        
        self.build_RFID_antennas_list()

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
        #print (self.drone_roll* 180 / math.pi, self.drone_pitch* 180 / math.pi, self.drone_yaw* 180 / math.pi)



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


    def build_RFID_antennas_list(self):

        for i in range(50):
            item_name   =   "rfid_" + str(i)

            # -------------Equidistant---------------
            #bin_y   =   2 *   (i    /   10)  -   9.5 
            #bin_x   =   2 *   (i    %   10)  -   9.5
            
            # ---------------Random------------------
            bin_y   =   2 *   (10*random())  -   10 
            bin_x   =   2 *   (10*random())  -   10

            bin_z   =   0.2

            print (bin_x, bin_y)

            self.RFID_List.append(RFID_antenna("item_name", i, bin_x, bin_y, bin_z))
            self.spawn_rfid_model(self.RFID_List[i].x, self.RFID_List[i].y, self.RFID_List[i].z, i)



    def print_RFID_in_range(self):
        #print("Looping")
        for RFID_antenna in self.RFID_List:

            distance_x_y = self.distance_RFID_and_drone_x_y(RFID_antenna.x, RFID_antenna.y)
            distance = self.distance_RFID_and_drone(RFID_antenna.x, RFID_antenna.y, RFID_antenna.z)

            if distance < self.RFID_sens_distance:

                # Calculates the angle of the RFID antenna and the front perspective of the drone - It takes in account orientation!         - yaw
                angle = self.calculate_yaw_degree_RFID(RFID_antenna.x,RFID_antenna.y)
                angle_respect_yaw = angle - (self.drone_yaw * 180 / math.pi) # We substract the value of the orientation

                # If we add or we substract more than the +/-180 degrees, we change the yaw degree that corresponds with the analog one   
                if angle_respect_yaw > 180:
                    value = angle_respect_yaw - 180
                    angle_respect_yaw = -180+value
                elif  angle_respect_yaw < -180:
                    value = angle_respect_yaw + 180
                    angle_respect_yaw = 180+value

                # Calculates the angle of the RFID antenna and the front perspective of the drone - It takes in account orientation!         - pitch
                angle = self.calculate_pitch_degree_RFID(angle_respect_yaw, distance,RFID_antenna.z)
                angle_respect_pitch = angle - (self.drone_pitch * 180 / math.pi) # We substract the value of the orientation

                # Calculates the angle of the RFID antenna and the front perspective of the drone - It takes in account orientation!         - roll
                angle = self.calculate_roll_degree_RFID(angle_respect_yaw, distance,RFID_antenna.z)
                angle_respect_roll = angle - (self.drone_roll * 180 / math.pi) # We substract the value of the orientation
     
                # Depending on the angle it represents one antenna or another or no-one
                detection = self.print_detected_antenna(angle_respect_yaw, angle_respect_pitch, angle_respect_roll)

                # If the RFID antenna it is not in red in the gazebo simulation, spawn it and delete the white one
                if RFID_antenna.spawned_red_mode == False and (detection != "NONE"):
                    
                    global front_number
                    global back_number
                    global left_number
                    global right_number
                    
                    # Depending on the antenna we publish a topic with the number of RFID antennas detected by each antenna
                    if detection == "FRONT":
                        front_number = front_number + 1
                        self.rfid_front_pub.publish(front_number)
                    elif detection == "BACK":
                        back_number = back_number + 1
                        self.rfid_back_pub.publish(back_number)
                    elif detection == "RIGHT":
                        right_number = right_number + 1
                        self.rfid_right_pub.publish(right_number)
                    elif detection == "LEFT":
                        left_number = left_number + 1
                        self.rfid_left_pub.publish(left_number)


                    print("CHANGING")
                    print("Detected by the antenna - " + detection)
                    print("yaw: " + str(angle_respect_yaw) + " pitch: " + str(angle_respect_pitch) + " roll: " + str(angle_respect_roll))
                    print("Drone: ")
                    print(self.drone_pose_x,self.drone_pose_y, self.drone_pose_z)
                    print(RFID_antenna.x, RFID_antenna.y, RFID_antenna.z)

                    # Here it does the transform from a white RFID antenna, to a red antenna
                    self.delete_rfid_model(RFID_antenna.rfid_id)

                    self.spawn_rfid_red_model(RFID_antenna.x, RFID_antenna.y, RFID_antenna.z, RFID_antenna.rfid_id)

                    self.RFID_List[RFID_antenna.rfid_id].spawned_red_mode = True
 
    # -------------------------------------------------------------------------------------------------------------    


    # Function that calculates the angle between the drone and the RFID antenna - yaw
    def calculate_yaw_degree_RFID(self, x, y):
        difference_x = x - self.drone_pose_x
        difference_y = y - self.drone_pose_y
      
        difference_hipotenuse = math.sqrt((difference_x**2)+(difference_y**2))
        
        angle = math.atan(difference_y/difference_x)
        
        # It calculates a position in front of the drone
        advanced_positive_x = 3*math.cos(self.drone_yaw)+self.drone_pose_x   
        advanced_positive_y = 3*math.sin(self.drone_yaw)+self.drone_pose_y
        # It calculates a position back of the drone
        advanced_negative_x = -3*math.cos(self.drone_yaw)+self.drone_pose_x
        advanced_negative_y = -3*math.sin(self.drone_yaw)+self.drone_pose_y

        # It calculates the distance of the RFID antenna with this two positions we have calculated 
        distance_advanced = math.sqrt(((advanced_positive_x - x)**2)+((advanced_positive_y - y)**2)) 
        distance_demored = math.sqrt(((advanced_negative_x - x)**2)+((advanced_negative_y - y)**2))

        # Depending on the distance we now know if the yaw angle we have calculated it is in the triangle in front of the drone or in the back
        # Depending which triangle is, the yaw angle is calculated in the following form
        if distance_demored < distance_advanced: 
            if angle < 0:
                return (180-angle * 180 / math.pi)
            else: 
                return (-180-angle * 180 / math.pi)
        else:
            return(angle * 180 / math.pi)



    # Function that calculates the angle between the drone and the RFID antenna - pitch
    def calculate_pitch_degree_RFID(self, angle_respect_yaw, distance, z):
        difference_x = distance*math.cos(angle_respect_yaw)
        difference_z = z - self.drone_pose_z
      
        difference_hipotenuse = math.sqrt((difference_x**2)+(difference_z**2))
        
        angle = math.atan(abs(difference_z/difference_x))
        if z < self.drone_pose_z:
            return(-angle * 180 / math.pi)
        else:
            return(angle * 180 / math.pi)


    # Function that calculates the angle between the drone and the RFID antenna - roll
    def calculate_roll_degree_RFID(self, angle_respect_yaw, distance, z):
        difference_y = distance*math.sin(angle_respect_yaw)
        difference_z = z - self.drone_pose_z
      
        difference_hipotenuse = math.sqrt((difference_y**2)+(difference_z**2))
        
        angle = math.atan(abs(difference_z/difference_y))
        if z < self.drone_pose_z:
            return(-angle * 180 / math.pi)
        else:
            return(angle * 180 / math.pi)



    # Function that calculates the distance between the drone and the RFID antenna in the x/y plane
    def distance_RFID_and_drone_x_y(self, x, y):
        raw_distance = math.sqrt(((self.drone_pose_x - x)**2)+((self.drone_pose_y - y)**2))
        return raw_distance
        

    # Function that calculates the distance between the drone and the RFID antenna
    def distance_RFID_and_drone(self, x, y, z):
        raw_distance = math.sqrt(((self.drone_pose_x - x)**2)+((self.drone_pose_y - y)**2)+((self.drone_pose_z - z)**2))
        return raw_distance


    # Function that prints which antenna has detected the RFID
    def print_detected_antenna(self, angle_respect_yaw, angle_respect_pitch, angle_respect_roll):

        if (angle_respect_yaw > -45) and (angle_respect_yaw < 45) and (angle_respect_pitch > -45) and (angle_respect_pitch < 45):

            # print ("Detected by the front antenna")

            return ("FRONT")

        elif (angle_respect_yaw > 45) and (angle_respect_yaw < 135) and (angle_respect_roll > -45) and (angle_respect_roll < 45):

            # print ("Detected by the right antenna") 

            return ("RIGHT")

        elif (angle_respect_yaw < -45) and (angle_respect_yaw > -135) and (angle_respect_roll > -45) and (angle_respect_roll < 45):

            # print ("Detected by the left antenna")

            return ("LEFT")
        
        elif ((angle_respect_yaw > 135) or (angle_respect_yaw < -135)) and (angle_respect_pitch > -45) and (angle_respect_pitch < 45):

            # print ("Detected by the back antenna")

            return ("BACK")

        else:

            return ("NONE")


    # Function that spawn a white RFID antenna
    def spawn_rfid_model(self, RFID_x, RFID_y, RFID_z, number):
        with open(self.sdfpath, "r") as f:
            product_xml = f.read()

        orient = Quaternion(0,0,0,0)
                  
        item_name   =   "rfid_" + str(number)
        print("Spawning model: ", item_name)
        item_pose   =   Pose(Point(x=RFID_x, y=RFID_y, z=RFID_z),   orient)
        self.spawn_model(item_name, product_xml, "", item_pose, "world")
     
    # Function that delete a white RFID antenna
    def delete_rfid_model(self, number):
        item_name = "rfid_"+str(number)
        print("Deleting model: ", item_name)
        self.delete_model(item_name)


    # Function that spawn a white RFID antenna
    def spawn_rfid_red_model(self, RFID_x, RFID_y, RFID_z, number):
        with open(self.sdfredpath, "r") as f:
            product_xml = f.read()

        orient = Quaternion(0,0,0,0)
                  
        item_name   =   "rfid_red_" + str(number)
        print("Spawning model: ", item_name)
        item_pose   =   Pose(Point(x=RFID_x, y=RFID_y, z=RFID_z),   orient)
        self.spawn_model(item_name, product_xml, "", item_pose, "world")
     
    # Function that delete a white RFID antenna
    def delete_rfid_red_model(self, number):
        item_name = "rfid_red_"+str(number)
        print("Deleting model: ", item_name)
        self.delete_model(item_name)



# Class of each RFID antenna 
class RFID_antenna:

    def __init__(self, name=None, rfid_id=0, x=0, y=0, z=0.2, spawned_red_mode=False):
        self.name = name
        self.rfid_id = rfid_id
        self.x = x
        self.y = y        
        self.z = z
        self.spawned_red_mode = spawned_red_mode




#########################################################################################################################

if __name__ == '__main__': # From here to the end calls the functions desired
    con = RFIDpublisher()
    con.start()

