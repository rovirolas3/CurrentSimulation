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


        global spawned_vehicle_1
        spawned_vehicle_1 = False
        global spawned_vehicle_2
        spawned_vehicle_2 = False
        global spawned_vehicle_3
        spawned_vehicle_3 = False
        global spawned_vehicle_4
        spawned_vehicle_4 = False

        print("Waiting for gazebo services...")
        rospy.wait_for_service("gazebo/delete_model")
        rospy.wait_for_service("gazebo/spawn_sdf_model")
        print("Got it.")
        self.delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
        self.spawn_model = rospy.ServiceProxy("gazebo/spawn_sdf_model", SpawnModel)




        rospy.init_node("rfid_node") # We initialize the node with the name: commander_node
            

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
       
        self.cur_target_pose_rfid_first = self.construct_target(self.RFID_1_x, self.RFID_1_y, 0, 0) #  
        self.cur_target_pose_rfid_second = self.construct_target(self.RFID_2_x, self.RFID_2_y, 0, 0) # 
        self.cur_target_pose_rfid_third = self.construct_target(self.RFID_3_x, self.RFID_3_y, 0, 0) #
        self.cur_target_pose_rfid_fourth = self.construct_target(self.RFID_4_x, self.RFID_4_y, 0, 0) #        

        while rospy.is_shutdown() is False: # While we don't shutdown it, do the loop

            # Must be changed!
            self.rfid_first_pub.publish(self.cur_target_pose_rfid_first) # Has to be put in the if (Still not implemented in the right position of the code)
            self.rfid_second_pub.publish(self.cur_target_pose_rfid_second) # 
            self.rfid_third_pub.publish(self.cur_target_pose_rfid_third) #
            self.rfid_fourth_pub.publish(self.cur_target_pose_rfid_fourth) # 

            self.cur_target_pose_rfid_first = self.construct_target(self.RFID_1_x, self.RFID_1_y, 0, 0) #  
            self.cur_target_pose_rfid_second = self.construct_target(self.RFID_2_x, self.RFID_2_y, 0, 0) # 
            self.cur_target_pose_rfid_third = self.construct_target(self.RFID_3_x, self.RFID_3_y, 0, 0) #
            self.cur_target_pose_rfid_fourth = self.construct_target(self.RFID_4_x, self.RFID_4_y, 0, 0) #
  
            
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






    # Function to create the message PositionTarget
    def construct_target(self, x, y, z, yaw, yaw_rate = 1): 
        target_raw_pose = PoseStamped() # We will fill the following message with our values: 
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
            angle_respect_orientation = angle - (self.drone_yaw * 180 / math.pi)

            print (angle_respect_orientation)

            # Depending on the angle it represents one antenna or another
            if (angle_respect_orientation > -45) and (angle_respect_orientation < 45):
                print ("Detected by the front antenna")

            elif (angle_respect_orientation > 45) and (angle_respect_orientation < 135):
                print ("Detected by the left antenna") 

            elif (angle_respect_orientation < -45) and (angle_respect_orientation > -135):
                print ("Detected by the right antenna")
          
            else:
                print ("Detected by the back antenna")

            # If the RFID antenna it is not in the gazebo simulation, spawn it
            global spawned_vehicle_1
            if spawned_vehicle_1 == False:
             
                with open(self.sdfpath, "r") as f:
                    product_xml = f.read()

                orient = Quaternion(0,0,0,0)
                  
                item_name   =   "rfid_1"
                print("Spawning model: ", item_name)
                item_pose   =   Pose(Point(x=self.RFID_1_x, y=self.RFID_1_y,    z=0.2),   orient)
                self.spawn_model(item_name, product_xml, "", item_pose, "world")

                spawned_vehicle_1 = True

        # If it is NOT in distance
        else: 
            print('RFID antenna 1 NOT in range ' + str(distance))

            angle = self.calculate_yaw_degree_RFID(self.RFID_1_x,self.RFID_1_y)
            angle_respect_orientation = angle - (self.drone_yaw * 180 / math.pi)

            '''
            print (angle_respect_orientation)
            
            if (angle_respect_orientation > -45) and (angle_respect_orientation < 45):
                print ("Detected by the front antenna")

            elif (angle_respect_orientation > 45) and (angle_respect_orientation < 135):
                print ("Detected by the left antenna") 

            elif (angle_respect_orientation < -45) and (angle_respect_orientation > -135):
                print ("Detected by the right antenna")
          
            else:
                print ("Detected by the back antenna")
            '''
           
            global spawned_vehicle_1
            if spawned_vehicle_1 == True:

                item_name = "rfid_1"
                print("Deleting model: ", item_name)
                self.delete_model(item_name)
 
                spawned_vehicle_1 = False


        #-------------------------------Second RFID Antenna------------------------------------ 

        distance = self.distance_RFID_and_drone(self.RFID_2_x,self.RFID_2_y)

        if distance < self.RFID_sens_distance:
            print('RFID antenna 2 in range ' + str(distance))

            angle = self.calculate_yaw_degree_RFID(self.RFID_2_x,self.RFID_2_y)
            angle_respect_orientation = angle - (self.drone_yaw * 180 / math.pi)

            print (angle_respect_orientation)


            if (angle_respect_orientation > -45) and (angle_respect_orientation < 45):
                print ("Detected by the front antenna")

            elif (angle_respect_orientation > 45) and (angle_respect_orientation < 135):
                print ("Detected by the left antenna") 

            elif (angle_respect_orientation < -45) and (angle_respect_orientation > -135):
                print ("Detected by the right antenna")
          
            else:
                print ("Detected by the back antenna")

            global spawned_vehicle_2
            if spawned_vehicle_2 == False:

                with open(self.sdfpath, "r") as f:
                    product_xml = f.read()

                orient = Quaternion(0,0,0,0)
                  
                item_name   =   "rfid_2"
                print("Spawning model: ", item_name)
                item_pose   =   Pose(Point(x=self.RFID_2_x, y=self.RFID_2_y,    z=0.2),   orient)
                self.spawn_model(item_name, product_xml, "", item_pose, "world")

                spawned_vehicle_2 = True

        else: 
            print('RFID antenna 2 NOT in range ' + str(distance))

            angle = self.calculate_yaw_degree_RFID(self.RFID_2_x,self.RFID_2_y)
            angle_respect_orientation = angle - (self.drone_yaw * 180 / math.pi)

            '''
            print (angle_respect_orientation)
            
            if (angle_respect_orientation > -45) and (angle_respect_orientation < 45):
                print ("Detected by the front antenna")

            elif (angle_respect_orientation > 45) and (angle_respect_orientation < 135):
                print ("Detected by the left antenna") 

            elif (angle_respect_orientation < -45) and (angle_respect_orientation > -135):
                print ("Detected by the right antenna")
          
            else:
                print ("Detected by the back antenna")
            '''
           
            global spawned_vehicle_2
            if spawned_vehicle_2 == True:

                item_name = "rfid_2"
                print("Deleting model: ", item_name)
                self.delete_model(item_name)
 
                spawned_vehicle_2 = False

        #-------------------------------Third RFID Antenna------------------------------------ 

        distance = self.distance_RFID_and_drone(self.RFID_3_x,self.RFID_3_y)

        if distance < self.RFID_sens_distance:
            print('RFID antenna 3 in range ' + str(distance))

            angle = self.calculate_yaw_degree_RFID(self.RFID_3_x,self.RFID_3_y)
            angle_respect_orientation = angle - (self.drone_yaw * 180 / math.pi)

            print (angle_respect_orientation)


            if (angle_respect_orientation > -45) and (angle_respect_orientation < 45):
                print ("Detected by the front antenna")

            elif (angle_respect_orientation > 45) and (angle_respect_orientation < 135):
                print ("Detected by the left antenna") 

            elif (angle_respect_orientation < -45) and (angle_respect_orientation > -135):
                print ("Detected by the right antenna")
          
            else:
                print ("Detected by the back antenna")

            global spawned_vehicle_3
            if spawned_vehicle_3 == False:
             
                with open(self.sdfpath, "r") as f:
                    product_xml = f.read()

                orient = Quaternion(0,0,0,0)
                  
                item_name   =   "rfid_3"
                print("Spawning model: ", item_name)
                item_pose   =   Pose(Point(x=self.RFID_3_x, y=self.RFID_3_y,    z=0.2),   orient)
                self.spawn_model(item_name, product_xml, "", item_pose, "world")

                spawned_vehicle_3 = True

        else: 
            print('RFID antenna 3 NOT in range ' + str(distance))

            angle = self.calculate_yaw_degree_RFID(self.RFID_3_x,self.RFID_3_y)
            angle_respect_orientation = angle - (self.drone_yaw * 180 / math.pi)

            '''
            print (angle_respect_orientation)
            
            if (angle_respect_orientation > -45) and (angle_respect_orientation < 45):
                print ("Detected by the front antenna")

            elif (angle_respect_orientation > 45) and (angle_respect_orientation < 135):
                print ("Detected by the left antenna") 

            elif (angle_respect_orientation < -45) and (angle_respect_orientation > -135):
                print ("Detected by the right antenna")
          
            else:
                print ("Detected by the back antenna")
            '''
           
            global spawned_vehicle_3
            if spawned_vehicle_3 == True:

                item_name = "rfid_3"
                print("Deleting model: ", item_name)
                self.delete_model(item_name)
 
                spawned_vehicle_3 = False


        #-------------------------------Fourth RFID Antenna------------------------------------ 

        distance = self.distance_RFID_and_drone(self.RFID_4_x,self.RFID_4_y)

        if distance < self.RFID_sens_distance:
            print('RFID antenna 4 in range ' + str(distance))

            angle = self.calculate_yaw_degree_RFID(self.RFID_4_x,self.RFID_4_y)
            angle_respect_orientation = angle - (self.drone_yaw * 180 / math.pi)

            print (angle_respect_orientation)


            if (angle_respect_orientation > -45) and (angle_respect_orientation < 45):
                print ("Detected by the front antenna")

            elif (angle_respect_orientation > 45) and (angle_respect_orientation < 135):
                print ("Detected by the left antenna") 

            elif (angle_respect_orientation < -45) and (angle_respect_orientation > -135):
                print ("Detected by the right antenna")
          
            else:
                print ("Detected by the back antenna")

            global spawned_vehicle_4
            if spawned_vehicle_4 == False:
             
                with open(self.sdfpath, "r") as f:
                    product_xml = f.read()

                orient = Quaternion(0,0,0,0)
                  
                item_name   =   "rfid_4"
                print("Spawning model: ", item_name)
                item_pose   =   Pose(Point(x=self.RFID_4_x, y=self.RFID_4_y,    z=0.2),   orient)
                self.spawn_model(item_name, product_xml, "", item_pose, "world")

                spawned_vehicle_4 = True

        else: 
            print('RFID antenna 4 NOT in range ' + str(distance))

            angle = self.calculate_yaw_degree_RFID(self.RFID_4_x,self.RFID_4_y)
            angle_respect_orientation = angle - (self.drone_yaw * 180 / math.pi)

            '''
            print (angle_respect_orientation)
            
            if (angle_respect_orientation > -45) and (angle_respect_orientation < 45):
                print ("Detected by the front antenna")

            elif (angle_respect_orientation > 45) and (angle_respect_orientation < 135):
                print ("Detected by the left antenna") 

            elif (angle_respect_orientation < -45) and (angle_respect_orientation > -135):
                print ("Detected by the right antenna")
          
            else:
                print ("Detected by the back antenna")
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
        






#########################################################################################################################

if __name__ == '__main__': # From here to the end calls the functions desired
    con = RFIDpublisher()
    con.start()

