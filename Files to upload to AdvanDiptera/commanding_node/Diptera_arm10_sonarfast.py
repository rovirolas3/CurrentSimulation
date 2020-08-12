#!/usr/bin/env python

import rospy
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget, AttitudeTarget, BatteryStatus
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import String
from quaternion import Quaternion
import time, math
import yaml
from gpio_diptera import Rpi_gpio_comm as Gpio_start
from gpio_clean import Rpi_gpio_comm_off as Gpio_stop
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped, Twist

from px_comm.msg import OpticalFlow

class Arming_Modechng():
    # It nitializes all the variables, it starts the ros node and initizalizes the topics and services
    def __init__(self):
        self.yamlpath = '/home/ubuntu/AdvanDiptera/src/commanding_node/params/arm_params.yaml'
        with open(self.yamlpath) as file:
            data = yaml.load(file)
            for key, value in data.items():
                # Initial values
                if key == "initial_x_pos":
                    self.init_x = value
                if key == "initial_y_pos":
                    self.init_y = value
                if key == "initial_z_pos":
                    self.init_z = value
                if key == "initial_thrust":
                    self.init_thrust = value

                # Sensor distances
                if key == "Hover_sensor_altitude":
                    self.hover_sensor_altitude = value
                if key == "Hover_sensor_altitude_max":
                    self.hover_sensor_altitude_max = value
                if key == "Hover_sensor_altitude_min":
                    self.hover_sensor_altitude_min = value
                if key == "Landing_sensor_altitude_min":
                    self.landing_sensor_altitude_min = value
                if key == "Avoiding_obstacle_distance_min":
                    self.Avoiding_obstacle_distance_min = value
                if key == "Blocking_movement_distance_min":
                    self.Blocking_movement_distance_min = value
                if key == "Unblocking_movement_distance_min":
                    self.Unblocking_movement_distance_min = value

                # Thrust
                if key == "Liftoff_thrust":
                    self.Liftoff_thrust = value
                    self.Liftoff_thrust_first = self.Liftoff_thrust
                if key == "Hover_thrust":
                    self.hover_thrust = value
		if key == "Landing_thrust":
                    self.Landing_thrust = value
                if key == "Moving_max_thrust":
                    self.Moving_max_thrust = value
		if key == "Moving_min_thrust":
                    self.Moving_min_thrust = value
                if key == "accumulating_thrust_soft":
                    self.accumulating_thrust_soft = value
                if key == "accumulating_thrust":
                    self.accumulating_thrust = value
                if key == "Deaccumulating_thrust":
                    self.Deaccumulating_thrust = value
                if key == "Liftoff_thrust_maximum":
                    self.Liftoff_thrust_maximum = value	

                # Time between messages
		if key == "Time_between_messages":
                    self.Time_between_messages = value

                # Timers
                if key == "Liftoff_time":
                    self.Liftoff_time = value
                if key == "Hover_time":
                    self.hover_time = value
                if key == "Moving_time":
                    self.Moving_time = value
		if key == "Secure_time_landing":
                    self.Secure_time_landing = value	
		if key == "Max_time_landing":
                    self.Max_time_landing = value						
		if key == "Avoiding_obstacle_time":
                    self.Avoiding_obstacle_time = value	
		if key == "Keyboard_control_time":
                    self.Keyboard_control_time = value
		if key == "move_desired_direction_time":
                    self.move_desired_direction_time = value	
		if key == "move_opposite_direction_time":
                    self.move_opposite_direction_time = value 
		if key == "move_hover_time":
                    self.move_hover_time = value
	
                # Angles
                if key == "angle_roll_left":
                    self.angle_roll_left = value
                if key == "angle_roll_right":
                    self.angle_roll_right = value
		if key == "angle_pitch_back":
                    self.angle_pitch_back = value	
		if key == "angle_pitch_forward":
                    self.angle_pitch_forward = value

                if key == "angle_roll_left_hard":
                    self.angle_roll_left_hard = value
                if key == "angle_roll_right_hard":
                    self.angle_roll_right_hard = value
		if key == "angle_pitch_back_hard":
                    self.angle_pitch_back_hard = value	
		if key == "angle_pitch_forward_hard":
                    self.angle_pitch_forward_hard = value

                if key == "angle_roll_left_right_trim":
                    self.angle_roll_left_right_trim = value
                if key == "angle_pitch_forward_back_trim":
                    self.angle_pitch_forward_back_trim = value

		if key == "angle_pitch_flow_max":
                    self.angle_pitch_flow_max = value
		if key == "angle_roll_flow_max":
                    self.angle_roll_flow_max = value

		if key == "flow_rate_threshold":
                    self.flow_rate_threshold = value

        # FLOW
        self.local_flow_x = 0
        self.local_flow_y = 0
        self.angle_pitch_flow = 0 # positive front/ negative back 
        self.angle_roll_flow = 0 # positive left/ negative right

        self.skip_to_land = False   # In case of an error it will skip al the movements and will go automatically to landing 
        self.stop_the_drone = False # In case of giving the order with the keyboard it will stop the drone
	self.Liftoff_thrust_old = None
        self.Landing_thrust_old = None	
        # Initialize the variables to False
        self.blockingMovementRight = False   
        self.blockingMovementLeft = False 
        self.blockingMovementBack = False
        self.blockingMovementFront = False

        # States - UNBLOCK/BLOCK/AVOID
        self.stateRight = "UNBLOCK" 
        self.stateLeft = "UNBLOCK"
        self.stateForward = "UNBLOCK"
        self.stateBack = "UNBLOCK"

        # State Avoiding obstacle
        self.stateRightAvoiding = False 
        self.stateLeftAvoiding = False
        self.stateForwardAvoiding = False
        self.stateBackAvoiding = False
        self.stateAvoiding = False

        # Initialize the counters to 0
        self.counterRightAvoidObstacle = 0
        self.counterRightBlockMovement = 0
        self.counterRightUnblockMovement = 0
   
        self.counterLeftAvoidObstacle = 0
        self.counterLeftBlockMovement = 0
        self.counterLeftUnblockMovement = 0

        self.counterForwardAvoidObstacle = 0
        self.counterForwardBlockMovement = 0
        self.counterForwardUnblockMovement = 0

        self.counterBackAvoidObstacle = 0
        self.counterBackBlockMovement = 0
        self.counterBackUnblockMovement = 0

        self.front_sensor_distance = None
        self.back_sensor_distance = None
        self.right_sensor_distance = None
        self.left_sensor_distance = None  
        self.down_sensor_distance = None
        self.down_sensor_distance_old = None 
        self.down_sensor_changed = False
        self.down_sensor_distance_higher = False

	self.beh_type = None
       
        # Battery
        self.battery_voltage = None
        self.battery_current = None 
        self.battery_voltage_remaining = None


        # variable to know if the drone is being controlled by the keyboard
        self.keyboardcontrolstate = None
        self.lidarcontrolstate = "No Obstacle"

        self.printing_value = 0 
        self.current_heading = None

        rospy.init_node("Arming_safety_node")

        self.imu_sub = rospy.Subscriber("/mavros/imu/data", Imu,self.imu_callback)
        self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        self.landService = rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)

        self.flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.local_target_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        #self.takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        self.attitude_target_pub = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
        #subscribed_topic_d = "/sonarTP_D"
        subscribed_topic_d = "/range_d"
        self.down_sensor = rospy.Subscriber(subscribed_topic_d, Range, self.cb_down_sensor)
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.cb_local_pose)
        self.local_battery_sub = rospy.Subscriber("/mavros/battery", BatteryStatus, self.cb_local_battery)

        self.local_flow_sub = rospy.Subscriber("/px4flow/opt_flow", OpticalFlow, self.local_flow)

        # Custom subscribers
        self.lidar_obstacle_detection_sub = rospy.Subscriber("/custom/lidaravoidance", String, self.lidar_obstacle_detection_callback)
        self.keyboardcontrol_target_sub = rospy.Subscriber("/custom/keyboardcontrol", String, self.keyboardcontrol_callback)

    # Transforms quaternion into yaw degrees
    def q2yaw(self, q):
        if isinstance(q, Quaternion): # Checks if the variable is of the type Quaternion
            rotate_z_rad = q.yaw_pitch_roll[0]
        else:
            q_ = Quaternion(q.w, q.x, q.y, q.z) # Converts into Quaternion
            rotate_z_rad = q_.yaw_pitch_roll[0]
        return rotate_z_rad

    # Callback function for the location of the drone
    def cb_local_pose(self, msg):
        self.local_pose = msg
        self.local_enu_position = msg

    # Callback function for the lidar lite
    def cb_down_sensor(self, msg):
        self.down_sensor_distance_old = self.down_sensor_distance
        self.down_sensor_distance = msg.range
        self.down_sensor_changed = True

        if self.down_sensor_distance_old < self.down_sensor_distance:
            self.down_sensor_distance_higher = True
        else:
            self.down_sensor_distance_higher = False

        if self.down_sensor_distance > 2.5:
            self.skip_to_land = True


    # Callback function for the battery
    def cb_local_battery(self, msg):
        self.battery_voltage = msg.voltage
        self.battery_current = msg.current 
        self.battery_voltage_remaining = msg.remaining

    # Prints the status of the battery
    def print_battery_status(self):
        stringtorospy_1 = ("Current voltage: " + str(self.battery_voltage))
        stringtorospy_2 = ("Current current: " + str(self.battery_current))
        stringtorospy_3 = ("Remaining: " + str(self.battery_voltage_remaining))

        if self.battery_voltage < 13:
            rospy.logfatal(stringtorospy_1)
            rospy.logfatal(stringtorospy_2)
            rospy.logfatal(stringtorospy_3)

        elif self.battery_voltage < 14:
            rospy.logwarn(stringtorospy_1)
            rospy.logwarn(stringtorospy_2)
            rospy.logwarn(stringtorospy_3)

        else:
            rospy.loginfo(stringtorospy_1)
            rospy.loginfo(stringtorospy_2)
            rospy.loginfo(stringtorospy_3)

    # Callback function for the orientation
    def imu_callback(self, msg):
        global global_imu
        self.imu = msg
        self.current_heading = self.q2yaw(self.imu.orientation) # Transforms q into degrees of yaw
        self.received_imu = True
        
    # Callback function for the orders of the keyboard
    def keyboardcontrol_callback(self, msg):
        self.keyboardcontrolstate = msg.data 
        if self.keyboardcontrolstate == "Land":
            self.skip_to_land = True
        if self.keyboardcontrolstate == "Emergency land":
            self.skip_to_land = True
            self.change_thrusts(0.54, 0.54, 0.54)
        if self.keyboardcontrolstate == "Stop":
            self.stop_the_drone = True

    # Callback function for the px4flow
    def local_flow(self,msg):
        self.local_flow = msg
        self.local_flow_x = msg.flow_x
        self.local_flow_y = msg.flow_y
 
    # It transforms the euler degrees into quaternion - not used
    def euler2quaternion(self, roll = 0, pitch = 0, yaw = 0):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]

    # It transforms the euler degrees into quaternion  
    def to_quaternion(self, roll=0.0, pitch=0.0, yaw=0.0):
        t0 = math.cos(math.radians(yaw * 0.5))
        t1 = math.sin(math.radians(yaw * 0.5))
        t2 = math.cos(math.radians(roll * 0.5))
        t3 = math.sin(math.radians(roll * 0.5))
        t4 = math.cos(math.radians(pitch * 0.5))
        t5 = math.sin(math.radians(pitch * 0.5))
        w = t0 * t2 * t4 + t1 * t3 * t5
        x = t0 * t3 * t4 - t1 * t2 * t5
        y = t0 * t2 * t5 + t1 * t3 * t4
        z = t1 * t2 * t4 - t0 * t3 * t5
        return [w, x, y, z]

   
    # Calculate the iterations needed in each loop
    def calculate_recursions(self, total_time):
	recursions = total_time/self.Time_between_messages
        return int(recursions)

    # It changes the variables with the new thrusts
    def change_thrusts(self, newLiftoff_thrust, newhover_thrust, newLanding_thrust):
        self.Liftoff_thrust = newLiftoff_thrust
        self.hover_thrust = newhover_thrust
        self.Landing_thrust = newLanding_thrust
        self.Moving_max_thrust = self.Liftoff_thrust
        self.Moving_min_thrust = self.Landing_thrust

    # It calculates the thrust for the drone depending on the height of the drone
    def calculate_thrust(self):

        difference_distance = self.hover_sensor_altitude_max - self.hover_sensor_altitude_min
        difference_thrust = self.Liftoff_thrust - self.Landing_thrust
        proportional_thrust = difference_thrust / difference_distance
        thrust = float(self.Liftoff_thrust)

        if self.hover_sensor_altitude_max >= self.down_sensor_distance and self.down_sensor_distance >= self.hover_sensor_altitude_min:
            distance_more = self.down_sensor_distance - self.hover_sensor_altitude_min 
            thrust = self.Liftoff_thrust - distance_more * proportional_thrust
            #rospublishstring = 'Thrust: %f    The drone is in the range' % (thrust)
            #rospy.loginfo(rospublishstring) 
            print('Thrust: %f    The drone is in the range' % (thrust))           
		
        elif self.hover_sensor_altitude_max + 0.3 <= self.down_sensor_distance: # We have to go down + change thrusts variables
            thrust = self.Landing_thrust
            thrust = thrust - 0.0002
            self.change_thrusts(thrust + 0.01, thrust + 0.007, thrust)
            thrust = self.Landing_thrust
            #rospublishstring = 'Thrust: %f    The drone is going down  + new thrusts' % (thrust)
            #rospy.logwarn(rospublishstring) 
            print('Thrust: %f    The drone is going down' % (thrust))
				
        elif self.down_sensor_distance <= self.hover_sensor_altitude_min - 0.1 and thrust < self.Liftoff_thrust_maximum: # We have to go up + change thrusts variables
            thrust = self.Liftoff_thrust
            thrust = thrust + 0.0002
            self.change_thrusts(thrust, thrust - 0.007, thrust - 0.01)
            thrust = self.Liftoff_thrust
            #rospublishstring = 'Thrust: %f    The drone is going up + new thrusts' % (thrust)
            #rospy.logwarn(rospublishstring) 
            print self.Liftoff_thrust_maximum
            print('Thrust: %f    The drone is going up' % (thrust))
		
        elif self.hover_sensor_altitude_max <= self.down_sensor_distance: # We have to go down
            thrust = self.Landing_thrust
            #rospublishstring = 'Thrust: %f    The drone is going down' % (thrust)
            #rospy.loginfo(rospublishstring)
            print('Thrust: %f    The drone is going down' % (thrust))
				
        elif self.down_sensor_distance <= self.hover_sensor_altitude_min: # We have to go up
            thrust = self.Liftoff_thrust
            #rospublishstring = 'Thrust: %f    The drone is going up' % (thrust)
            #rospy.loginfo(rospublishstring)
            print('Thrust: %f    The drone is going up' % (thrust))

        else:
            thrust = self.Liftoff_thrust
            #rospublishstring = 'Thrust: %f    The drone is in the maximum thrust obtained' % (thrust)
            #rospy.loginfo(rospublishstring)
            print('Thrust: %f    The drone is in the maximum thrust obtained' % (thrust))

        return thrust

#----------------------change modes----------------------------

    def modechnge(self):
        if self.flightModeService(custom_mode='OFFBOARD'):
            rospy.loginfo("succesfully changed mode to OFFBOARD")
            return True
        else:
            rospy.loginfo("failed to change mode")
            return False        

    def modechnge_2(self): # Not used - For further investigation
        if self.flightModeService(custom_mode='STABILIZED'):
            rospy.loginfo("succesfully changed mode to STABILIZED")
            return True
        else:
            rospy.loginfo("failed to change mode")
            return False  

    def modechnge_3(self): # Not used - For further investigation
        if self.flightModeService(custom_mode='POSCTL'):
            rospy.loginfo("succesfully changed mode to POSCTL")
            return True
        else:
            rospy.loginfo("failed to change mode")
            return False  

#----------------------extra services----------------------------
    
    def service_1(self): # Not used - For further investigation
        if self.takeoffService():
            rospy.loginfo("succesfully takeoff")
            return True
        else:
            rospy.loginfo("failed to take off")
            return False 

    def service_2(self): # Not used - For further investigation
        if self.landService():
            rospy.loginfo("succesfully landing")
            return True
        else:
            rospy.loginfo("failed to land")
            return False 


#----------------------arming services----------------------------
    # Service to arm the drone
    def arm(self):
        if self.armService(True):
            rospy.loginfo("AdvanDiptera is Armed")
            return True
        else:
            rospy.loginfo("Failed to arm AdvanDiptera")
            return False

    # Service to disarm the drone
    def disarm(self):
        if self.armService(False):
            rospy.loginfo("Advandiptera succesfully disarmed")
            return True
        else:
            rospy.loginfo("Failed to disarm AdvanDiptera")
            return False
        
#----------------------messages constructor----------------------------
    # Construct the message for the location of the drone - not used. For further investigations is the convinient next step
    def construct_target(self, x, y, z, yaw, yaw_rate=1):
        target_raw_pose = PositionTarget()  # We will fill the following message with our values: http://docs.ros.org/api/mavros_msgs/html/msg/PositionTarget.html
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
    
    # Construct the message for the attitude of the drone 
    def construct_target_attitude(self, body_x = 0, body_y = 0, body_z = 0, thrust=0, roll_angle = 0, pitch_angle = 0, yaw_angle = 0):

        target_raw_attitude = AttitudeTarget()  # We will fill the following message with our values: http://docs.ros.org/api/mavros_msgs/html/msg/PositionTarget.html
        target_raw_attitude.header.stamp = rospy.Time.now()
        #target_raw_attitude.orientation = self.imu.orientation
        q = self.to_quaternion(roll_angle + self.angle_roll_left_right_trim, pitch_angle + self.angle_pitch_forward_back_trim, yaw_angle)
        target_raw_attitude.orientation.w = q[0]
        target_raw_attitude.orientation.x = q[1]
        target_raw_attitude.orientation.y = q[2]
        target_raw_attitude.orientation.z = q[3]
        target_raw_attitude.body_rate.x = body_x # ROLL_RATE
        target_raw_attitude.body_rate.y = body_y # PITCH_RATE
        target_raw_attitude.body_rate.z = body_z # YAW_RATE
        target_raw_attitude.thrust = thrust
        self.attitude_target_pub.publish(target_raw_attitude)
        time.sleep(self.Time_between_messages)



################################# OBSTACLE AVOIDANCE ########################################
   
    # Callback funtion to detect the obstacles coming from the lidar
    def lidar_obstacle_detection_callback(self, msg):
        self.lidarcontrolstate = msg.data 

    # Function that if there is an obstacle creates a movement into the opposite direction
    def looking_obstacles(self):
        detected = True
        # Hard movements
        if self.lidarcontrolstate == "Front Right Hard":
            rospy.logwarn("Obstacle: " + self.lidarcontrolstate)
            self.moving_back_left_rec_hard()
        elif self.lidarcontrolstate == "Front Left Hard":
            rospy.logwarn("Obstacle: " + self.lidarcontrolstate)
            self.moving_back_right_rec_hard()
        elif self.lidarcontrolstate == "Back Right Hard":
            rospy.logwarn("Obstacle: " + self.lidarcontrolstate)
            self.moving_front_left_rec_hard()
        elif self.lidarcontrolstate == "Back Left Hard":
            rospy.logwarn("Obstacle: " + self.lidarcontrolstate)
            self.moving_front_right_rec_hard()

        elif self.lidarcontrolstate == "Right Hard":
            rospy.logwarn("Obstacle: " + self.lidarcontrolstate)
            self.moving_left_rec_hard()
        elif self.lidarcontrolstate == "Left Hard":
            rospy.logwarn("Obstacle: " + self.lidarcontrolstate)
            self.moving_right_rec_hard()
        elif self.lidarcontrolstate == "Front Hard":
            rospy.logwarn("Obstacle: " + self.lidarcontrolstate)
            self.moving_back_rec_hard()
        elif self.lidarcontrolstate == "Back Hard":
            rospy.logwarn("Obstacle: " + self.lidarcontrolstate)
            self.moving_forward_rec_hard()

        # Normal movements
        elif self.lidarcontrolstate == "Front Right":
            rospy.logwarn("Obstacle: " + self.lidarcontrolstate)
            self.moving_back_left_rec()
        elif self.lidarcontrolstate == "Front Left":
            rospy.logwarn("Obstacle: " + self.lidarcontrolstate)
            self.moving_back_right_rec()
        elif self.lidarcontrolstate == "Back Right":
            rospy.logwarn("Obstacle: " + self.lidarcontrolstate)
            self.moving_front_left_rec()
        elif self.lidarcontrolstate == "Back Left":
            rospy.logwarn("Obstacle: " + self.lidarcontrolstate)
            self.moving_front_right_rec()

        elif self.lidarcontrolstate == "Right":
            rospy.logwarn("Obstacle: " + self.lidarcontrolstate)
            self.moving_left_rec()
        elif self.lidarcontrolstate == "Left":
            rospy.logwarn("Obstacle: " + self.lidarcontrolstate)
            self.moving_right_rec()
        elif self.lidarcontrolstate == "Front":
            rospy.logwarn("Obstacle: " + self.lidarcontrolstate)
            self.moving_back_rec()
        elif self.lidarcontrolstate == "Back":
            rospy.logwarn("Obstacle: " + self.lidarcontrolstate)
            self.moving_forward_rec()

        else:
            detected = False

        return detected

################################# PX4FLOW ########################################

    # Function that creates a movement if the px4flow detects a variation
    def looking_px4flow(self):
        if self.local_flow_x > self.flow_rate_threshold: # drone moving right, correcting moving left
            rospy.logwarn("Correcting px4flow - Moving left")
            self.moving_left_raw_hard_rec(0.15)
            self.moving_hover_raw_rec(0.15)

        elif self.local_flow_x < - self.flow_rate_threshold: # drone moving left, correcting moving right
            rospy.logwarn("Correcting px4flow - Moving Right")
            self.moving_right_raw_hard_rec(0.15)
            self.moving_hover_raw_rec(0.15)

        if self.local_flow_y > self.flow_rate_threshold: # drone moving front, correcting moving back
            rospy.logwarn("Correcting px4flow - Moving Back")
            self.moving_back_raw_hard_rec(0.15)
            self.moving_hover_raw_rec(0.15)

        elif self.local_flow_y < - self.flow_rate_threshold: # drone moving back, correcting moving front
            rospy.logwarn("Correcting px4flow - Moving Front")
            self.moving_forward_raw_hard_rec(0.15)
            self.moving_hover_raw_rec(0.15)


################################# LIFT OFF ########################################

    # Lift off of the drone divided in different steps - not used	
    def automatic_lift_off_rec(self, thrust, time_flying):
        recursions = self.calculate_recursions(time_flying)
        print(recursions)
        self.beh_type = "TAKE OFF"
	for i in range(recursions):
            print 'Recursion: ' + str(i) + '   Total Recursions taking off: ' + str(recursions)
            rospublishstring = 'Sonar down distance: ' + str(self.down_sensor_distance)
            rospy.loginfo(rospublishstring)

            if self.stop_the_drone == True:
                rospy.logfatal("Stopping the drone - Skipping take off")
                break
   
            elif self.skip_to_land == True or thrust > self.Liftoff_thrust_maximum:
                rospy.loginfo("Going to land")
                break

            elif self.beh_type == "TAKE OFF" and thrust < self.Liftoff_thrust_first: 
                print("Lifting the drone up slowly")
                rospublishstring = 'Thrust: ' + str(thrust)
                #rospy.loginfo(rospublishstring)
                print rospublishstring
                self.construct_target_attitude(0,0,0,thrust)
                thrust = thrust + self.accumulating_thrust_soft

            elif self.beh_type == "TAKE OFF" and self.down_sensor_distance <= 0.4:
                print("The drone is searching the lifting off thrust")
                rospublishstring = 'Thrust: ' + str(thrust)
                #rospy.loginfo(rospublishstring)
                print rospublishstring
                self.construct_target_attitude(0,0,0,thrust)
                if self.down_sensor_changed == True:
                    self.down_sensor_changed = False
                    thrust = thrust + 0.005
                    self.change_thrusts(thrust, thrust, thrust)

            elif self.beh_type == "TAKE OFF" and self.down_sensor_distance >= 0.4 and self.down_sensor_distance <= self.hover_sensor_altitude_min:
                print("The drone is searching the lifting off thrust")
                rospublishstring = 'Thrust: ' + str(thrust)
                #rospy.loginfo(rospublishstring)
                print rospublishstring
                self.construct_target_attitude(0,0,0,thrust)
                if self.down_sensor_changed == True:
                    self.down_sensor_changed = False
                    if self.down_sensor_distance_higher == False:
                        thrust = thrust + 0.005
                        self.change_thrusts(thrust, thrust, thrust)

            elif self.beh_type == "TAKE OFF" and self.down_sensor_distance > self.hover_sensor_altitude_min:
                self.change_thrusts(thrust + 0.015, thrust, thrust - 0.005)
                break
            
    # Lift off of the drone divided in different steps depending on the thrust and on the hight- not used
    def automatic_lift_off_rec_v2(self, thrust, time_flying):
        recursions = self.calculate_recursions(time_flying)
        print(recursions)
        self.beh_type = "TAKE OFF"
        rospy.logwarn("Take off")
        valuecount = 0
	for i in range(recursions):
            print 'Recursion: ' + str(i) + '   Total Recursions taking off: ' + str(recursions)
            rospublishstring = 'Sonar down distance: ' + str(self.down_sensor_distance)
            #rospy.loginfo(rospublishstring)
            print rospublishstring
            valuecount += 1

            if self.stop_the_drone == True:
                rospy.logfatal("Stopping the drone - Skipping take off")
                break

            elif self.skip_to_land == True or thrust > self.Liftoff_thrust_maximum:
                break

            elif self.beh_type == "TAKE OFF" and thrust < 0.54: 
                print("Lifting the drone up slowly")
                rospublishstring = 'Thrust: ' + str(thrust)
                #rospy.loginfo(rospublishstring)
                print rospublishstring
                self.construct_target_attitude(0,0,0,thrust)
                thrust = thrust + self.accumulating_thrust_soft

            elif self.beh_type == "TAKE OFF" and thrust < 0.56:
                print("The drone is searching the lifting off thrust")
                rospublishstring = 'Thrust: ' + str(thrust)
                #rospy.loginfo(rospublishstring)
                print rospublishstring
                self.construct_target_attitude(0,0,0,thrust)
                if valuecount >= 50:
                    valuecount = 0
                    thrust = thrust + 0.005
                    self.change_thrusts(thrust, thrust - 0.0035, thrust - 0.01)

            elif self.beh_type == "TAKE OFF" and self.down_sensor_distance <= 0.7:
                print("The drone is searching the lifting off thrust")
                rospublishstring = 'Thrust: ' + str(thrust)
                #rospy.loginfo(rospublishstring)
                print rospublishstring
                self.construct_target_attitude(0,0,0,thrust)
                if valuecount >= 50:
                    valuecount = 0
                    thrust = thrust + 0.0025
                    self.change_thrusts(thrust, thrust - 0.0035, thrust - 0.01)

            elif self.beh_type == "TAKE OFF" and self.down_sensor_distance > 0.7:
                self.change_thrusts(thrust, thrust - 0.0035, thrust - 0.01)
                break

################################# HOVER ########################################

    # Hover function where it is also constantly detecting obstacles or looking the variation in the px4flow
    def hover_rec(self, time_flying): 
        recursions = self.calculate_recursions(time_flying)

        thrust = float(self.hover_thrust)
        self.beh_type = 'HOVER'
        rospy.logwarn("Hovering")
        reset = 0

        print(recursions)
	for i in range(recursions):
            print 'Recursion: ' + str(i) + '   Total Recursions hovering: ' + str(recursions)
            rospublishstring = 'Sonar down distance: ' + str(self.down_sensor_distance)
            #rospy.loginfo(rospublishstring)
            print rospublishstring      
            #rospy.loginfo(self.lidarcontrolstate)
            print self.lidarcontrolstate
            self.beh_type = 'HOVER'


            if self.stop_the_drone == True:
                rospy.logfatal("Stopping the drone - Skipping hovering")
                break

            elif self.skip_to_land == True:
                rospy.logwarn("Going to land")
                break
            
            elif self.keyboardcontrolstate == "Controlled":
                time.sleep(self.Keyboard_control_time)
    
            else:

                obstacle_detected = self.looking_obstacles()

                #reset = reset + 1
                #if reset >= 50: 

                self.looking_px4flow()

                #    reset = 0

                thrust = self.calculate_thrust()

                self.construct_target_attitude(0,0,0,thrust, 0, 0, 0)
	
        print ("time of hovering has ended")


################################# MOVE RAW ########################################


#                      /// MOVE RAW ///

    # Move function where we can introduce the desired angles for our movement
    def move_raw(self, time_flying, angle_roll, angle_pitch, angle_yaw): 
        recursions = self.calculate_recursions(time_flying)

        print(recursions)
        self.beh_type = 'MOVING'
        thrust = self.Landing_thrust

	for i in range(recursions):
            print 'Recursion: ' + str(i) + '   Total Recursions moving: ' + str(recursions)
            rospublishstring = 'Sonar down distance: ' + str(self.down_sensor_distance)
            #rospy.loginfo(rospublishstring)
            print rospublishstring         

            if self.stop_the_drone == True:
                rospy.logfatal("Stopping the drone - Skipping movement")
                break

            elif self.skip_to_land == True:
                break

            elif self.keyboardcontrolstate == "Controlled":
                time.sleep(self.Keyboard_control_time)

            else:
                #thrust = self.calculate_thrust()
                thrust = 0.002
                self.construct_target_attitude(0,0,0,thrust, angle_roll, angle_pitch, angle_yaw)

################################# SIMPLE MOVEMENTS ########################################

# Simple movements introducing the correct angle for each one
 
#                      /// RIGHT MOVE ///

    def moving_right_raw_rec(self, time_flying): 

        self.move_raw(time_flying, self.angle_roll_right, 0, 0)      

#                      /// LEFT MOVE ///

    def moving_left_raw_rec(self, time_flying):

        self.move_raw(time_flying, self.angle_roll_left, 0, 0)      

#                      /// FRONT MOVE ///

    def moving_forward_raw_rec(self, time_flying):
 
        self.move_raw(time_flying, 0, self.angle_pitch_forward, 0)       

#                      /// BACK MOVE ///

    def moving_back_raw_rec(self, time_flying):
 
        self.move_raw(time_flying, 0, self.angle_pitch_back, 0)

#                       /// HOVER ///

    def moving_hover_raw_rec(self, time_flying):
 
        self.move_raw(time_flying, 0, 0, 0)

#                      /// DIAGONAL FRONT/RIGHT MOVE ///

    def moving_front_right_raw_rec(self, time_flying): 

        self.move_raw(time_flying,  self.angle_roll_right, self.angle_pitch_forward, 0)      

#                      /// DIAGONAL FRONT/LEFT MOVE ///

    def moving_front_left_raw_rec(self, time_flying):

        self.move_raw(time_flying, self.angle_roll_left, self.angle_pitch_forward, 0)      

#                      /// DIAGONAL BACK/RIGHT MOVE ///

    def moving_back_right_raw_rec(self, time_flying):
 
        self.move_raw(time_flying, self.angle_roll_right, self.angle_pitch_back, 0)       

#                      /// DIAGONAL BACK/LEFT MOVE ///

    def moving_back_left_raw_rec(self, time_flying):
 
        self.move_raw(time_flying, self.angle_roll_left, self.angle_pitch_back, 0)

################################# HARD MOVEMENTS ########################################

# Simple movements with a higher angle = more velocity

#                      /// RIGHT MOVE HARD ///

    def moving_right_raw_hard_rec(self, time_flying): 

        self.move_raw(time_flying, self.angle_roll_right_hard, 0, 0)      

#                      /// LEFT MOVE HARD///

    def moving_left_raw_hard_rec(self, time_flying):

        self.move_raw(time_flying, self.angle_roll_left_hard, 0, 0)      

#                      /// FRONT MOVE HARD///

    def moving_forward_raw_hard_rec(self, time_flying):
 
        self.move_raw(time_flying, 0, self.angle_pitch_forward_hard, 0)       

#                      /// BACK MOVE HARD///

    def moving_back_raw_hard_rec(self, time_flying):
 
        self.move_raw(time_flying, 0, self.angle_pitch_back_hard, 0)



#                      /// DIAGONAL FRONT/RIGHT MOVE HARD ///

    def moving_front_right_raw_hard_rec(self, time_flying): 

        self.move_raw(time_flying, self.angle_roll_right_hard, self.angle_pitch_forward_hard, 0)      

#                      /// DIAGONAL FRONT/LEFT MOVE HARD ///

    def moving_front_left_raw_hard_rec(self, time_flying):

        self.move_raw(time_flying, self.angle_roll_left_hard, self.angle_pitch_forward_hard, 0)      

#                      /// DIAGONAL BACK/RIGHT MOVE HARD ///

    def moving_back_right_raw_hard_rec(self, time_flying):
 
        self.move_raw(time_flying, self.angle_roll_right_hard, self.angle_pitch_back_hard, 0)       

#                      /// DIAGONAL BACK/LEFT MOVE HARD ///

    def moving_back_left_raw_hard_rec(self, time_flying):
 
        self.move_raw(time_flying, self.angle_roll_left_hard, self.angle_pitch_back_hard, 0)

################################# COMPLETE MOVEMENTS ########################################

# Each movement consists of three steeps 
# 1. movement in desired direction
# 2. movement in opposite direction 
# 3. hover

#                      /// RIGHT MOVE ///

    def moving_right_rec(self): 
        rospy.loginfo("Moving Right")
        self.moving_right_raw_rec(self.move_desired_direction_time)
        self.moving_left_raw_hard_rec(self.move_opposite_direction_time) 
        self.moving_hover_raw_rec(self.move_hover_time)      

#                      /// LEFT MOVE ///

    def moving_left_rec(self):
        rospy.loginfo("Moving Left")
        self.moving_left_raw_rec(self.move_desired_direction_time)
        self.moving_right_raw_hard_rec(self.move_opposite_direction_time)
        self.moving_hover_raw_rec(self.move_hover_time)       

#                      /// FRONT MOVE ///

    def moving_forward_rec(self):
        rospy.loginfo("Moving Front") 
        self.moving_forward_raw_rec(self.move_desired_direction_time)
        self.moving_back_raw_hard_rec(self.move_opposite_direction_time)
        self.moving_hover_raw_rec(self.move_hover_time)       

#                      /// BACK MOVE ///

    def moving_back_rec(self):
        rospy.loginfo("Moving Back")
        self.moving_back_raw_rec(self.move_desired_direction_time)
        self.moving_forward_raw_hard_rec(self.move_opposite_direction_time)
        self.moving_hover_raw_rec(self.move_hover_time)       



#                      /// FRONT / RIGHT MOVE ///

    def moving_front_right_rec(self): 
        rospy.loginfo("Moving Diagonal  Front and Right")
        self.moving_front_right_raw_rec(self.move_desired_direction_time)
        self.moving_back_left_raw_hard_rec(self.move_opposite_direction_time)
        self.moving_hover_raw_rec(self.move_hover_time)     

#                      /// FRONT / LEFT MOVE ///

    def moving_front_left_rec(self):
        rospy.loginfo("Moving Diagonal  Front and Left")
        self.moving_front_left_raw_rec(self.move_desired_direction_time)
        self.moving_back_right_raw_hard_rec(self.move_opposite_direction_time)
        self.moving_hover_raw_rec(self.move_hover_time)
      

#                      /// BACK / RIGHT MOVE ///

    def moving_back_right_rec(self):
        rospy.loginfo("Moving Diagonal  Back and Right") 
        self.moving_back_right_raw_rec(self.move_desired_direction_time)
        self.moving_front_left_raw_hard_rec(self.move_opposite_direction_time)
        self.moving_hover_raw_rec(self.move_hover_time) 
      

#                      /// BACK / LEFT MOVE ///

    def moving_back_left_rec(self):
        rospy.loginfo("Moving Diagonal Back and Left")
        self.moving_back_left_raw_rec(self.move_desired_direction_time)
        self.moving_front_right_raw_hard_rec(self.move_opposite_direction_time)
        self.moving_hover_raw_rec(self.move_hover_time) 



################################# COMPLETE MOVEMENTS HARD ########################################

# Each movement consists of three steeps 
# 1. movement in desired direction
# 2. movement in opposite direction 
# 3. hover

#                      /// RIGHT MOVE ///

    def moving_right_rec_hard(self): 
        rospy.logwarn("Moving Right")
        self.moving_right_raw_hard_rec(self.move_desired_direction_time)
        self.moving_left_raw_hard_rec(self.move_opposite_direction_time) 
        self.moving_hover_raw_rec(self.move_hover_time)      

#                      /// LEFT MOVE ///

    def moving_left_rec_hard(self):
        rospy.logwarn("Moving Left")
        self.moving_left_raw_hard_rec(self.move_desired_direction_time)
        self.moving_right_raw_hard_rec(self.move_opposite_direction_time)
        self.moving_hover_raw_rec(self.move_hover_time)       

#                      /// FRONT MOVE ///

    def moving_forward_rec_hard(self):
        rospy.logwarn("Moving Front") 
        self.moving_forward_hard_raw_rec(self.move_desired_direction_time)
        self.moving_back_raw_hard_rec(self.move_opposite_direction_time)
        self.moving_hover_raw_rec(self.move_hover_time)       

#                      /// BACK MOVE ///

    def moving_back_rec_hard(self):
        rospy.logwarn("Moving Back")
        self.moving_back_raw_hard_rec(self.move_desired_direction_time)
        self.moving_forward_raw_hard_rec(self.move_opposite_direction_time)
        self.moving_hover_raw_rec(self.move_hover_time)       



#                      /// FRONT / RIGHT MOVE ///

    def moving_front_right_rec_hard(self): 
        rospy.logwarn("Moving Diagonal Front and Right")
        self.moving_front_right_raw_hard_rec(self.move_desired_direction_time)
        self.moving_back_left_raw_hard_rec(self.move_opposite_direction_time)
        self.moving_hover_raw_rec(self.move_hover_time)     

#                      /// FRONT / LEFT MOVE ///

    def moving_front_left_rec_hard(self):
        rospy.logwarn("Moving Diagonal Front and Left")
        self.moving_front_left_raw_hard_rec(self.move_desired_direction_time)
        self.moving_back_right_raw_hard_rec(self.move_opposite_direction_time)
        self.moving_hover_raw_rec(self.move_hover_time)
      
#                      /// BACK / RIGHT MOVE ///

    def moving_back_right_rec_hard(self):
        rospy.logwarn("Moving Diagonal Back and Right") 
        self.moving_back_right_raw_hard_rec(self.move_desired_direction_time)
        self.moving_front_left_raw_hard_rec(self.move_opposite_direction_time)
        self.moving_hover_raw_rec(self.move_hover_time) 

#                      /// BACK / LEFT MOVE ///

    def moving_back_left_rec_hard(self):
        rospy.logwarn("Moving Diagonal Back and Left")
        self.moving_back_left_raw_hard_rec(self.move_desired_direction_time)
        self.moving_front_right_raw_hard_rec(self.move_opposite_direction_time)
        self.moving_hover_raw_rec(self.move_hover_time)





################################# LANDING ########################################
	
    # Landing function where it slows down the drone until it lands
    def landing_rec(self):                       # Landing phase
        self.beh_type = "LANDING"
        rospy.logwarn("Landing")
        recursions = self.calculate_recursions(self.Max_time_landing)
        print(recursions)
        thrust = self.hover_thrust
        for i in range(recursions):
            print 'Recursion: ' + str(i) + '   Total Recursions landing: ' + str(recursions)
            rospublishstring = 'Sonar down distance: ' + str(self.down_sensor_distance)
            #rospy.loginfo(rospublishstring)
            print rospublishstring 

            if self.stop_the_drone == True:
                rospy.logfatal("Stopping the drone - Skipping landing")
                break

            elif self.down_sensor_distance <= self.landing_sensor_altitude_min:
                break

            elif thrust > self.Landing_thrust: 
                print('Thrust: ' + str(thrust) + '   Landing the drone down slowly"')
                self.construct_target_attitude(0,0,0,thrust)
                thrust = thrust - self.accumulating_thrust_soft

            elif thrust <= self.Landing_thrust: 
                thrust = self.Landing_thrust
                print('Thrust: ' + str(thrust) + '   Landing the drone down slowly"')
                self.construct_target_attitude(0,0,0,thrust)
				
        self.secure_landing_phase_rec2()

    # Secure landing part (it keep sending orders of landing during a few seconds just to not disconnect the drone when the sensor stops working - 30cm)
    def secure_landing_phase_rec2(self):           
        self.beh_type = "LANDING"
        recursions = self.calculate_recursions(self.Secure_time_landing)
        thrust = self.Landing_thrust	
        for i in range(recursions):
            print 'Recursion: ' + str(i) + '   Total Recursions landing: ' + str(recursions)
            rospublishstring = 'Sonar down distance: ' + str(self.down_sensor_distance)
            #rospy.loginfo(rospublishstring)
            print rospublishstring 

            if self.stop_the_drone == True:
                rospy.logfatal("Stopping the drone - Skipping secure landing")
                break

            if thrust <= 0:
                break

            if i < 300:
                thrust = self.Landing_thrust
                print('Thrust: ' + str(thrust) + '   Smooth landing')
                self.construct_target_attitude(0,0,0,thrust)

            elif i < 1200:
                print('Thrust: ' + str(thrust) + '   Landing')
                self.construct_target_attitude(0,0,0,thrust)
                thrust = thrust - 0.00005

            else: 
                print('Thrust: ' + str(thrust) + '   Landing')
                self.construct_target_attitude(0,0,0,thrust)
                thrust = thrust - 0.005

        for i in range(20):
            
            self.arm_state = self.disarm()    # disarms the drone
            time.sleep(0.1)

    def secure_landing_phase_rec(self):           # Secure landing part - last cm - not used, just another form to land the drone
        self.beh_type = "LANDING"
        recursions = self.calculate_recursions(self.Secure_time_landing)	
        thrust = self.hover_thrust
        for i in range(recursions):
            print 'Recursion: ' + str(i) + '   Total Recursions secure landing: ' + str(recursions)
            rospublishstring = 'Sonar down distance: ' + str(self.down_sensor_distance)
            #rospy.loginfo(rospublishstring)
            print rospublishstring 

            if thrust <= 0:
                break

            elif i < 200:
                thrust = self.hover_thrust
                print('Thrust: ' + str(thrust) + '   Secure hover before landing')
                self.construct_target_attitude(0,0,0,thrust)

            elif i < 700:
                thrust = self.hover_thrust - 0.000002
                print('Thrust: ' + str(thrust) + '   Smooth landing')
                self.construct_target_attitude(0,0,0,thrust)

            else: 
                thrust = thrust - 0.000002
                print('Thrust: ' + str(thrust) + '   Landing')
                self.construct_target_attitude(0,0,0,thrust)
        for i in range(20):
            
            self.arm_state = self.disarm()    # disarms the drone
            time.sleep(0.1)	

        

 #----------------------start function---------------------------- 

    def start(self):
        for i in range(10): # Waits 5 seconds for initialization
            if self.current_heading is not None:
                break
            else:
                print("Waiting for initialization.")
                time.sleep(0.5)

        #self.cur_target_pose = self.construct_target(self.init_x, self.init_y, self.init_z, self.current_heading)
        #self.local_target_pub.publish(self.cur_target_pose)

        time.sleep(2)
        
        for i in range(20):
            
            self.arm_state = self.arm()    # arms the drone
            self.construct_target_attitude()
            self.offboard_state = self.modechnge()
            time.sleep(0.1)
            
 

 
#####################################################################################################            
if __name__ == '__main__':

    try:
        Gpio_start().start()
        time.sleep(3.5)
        print("Waiting for Advandiptera brain")
        arm = Arming_Modechng()

        # Arming + liftoff + hover
        arm.start()
        arm.print_battery_status()
        if arm.down_sensor_distance != None and arm.battery_voltage > 13:
            #arm.lift_off_rec(arm.init_thrust, arm.Liftoff_time)
            #arm.automatic_lift_off_rec(arm.init_thrust, arm.Liftoff_time)
            arm.automatic_lift_off_rec_v2(arm.init_thrust, arm.Liftoff_time)

            arm.hover_rec(arm.hover_time)
            #arm.automatic_hover_rec(arm.hover_time)
       
            # Moving
            #arm.moving_right_rec()
            #arm.moving_left_rec()
            #arm.moving_back_rec()

            # Hover + landing
            #arm.hover_rec(5)
            arm.landing_rec()
  
            #arm.moving_right_raw_rec(5)
            arm.print_battery_status()

        else:
            rospy.logerr("No sonar detected or low battery level, disarming")

    except rospy.ROSInterruptException: pass
