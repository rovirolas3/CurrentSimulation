#!/usr/bin/env python

import rospy
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget, AttitudeTarget
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import String
from quaternion import Quaternion
import time, math
import yaml
from gpio_diptera import Rpi_gpio_comm as Gpio_start
from gpio_clean import Rpi_gpio_comm_off as Gpio_stop
from sensor_msgs.msg import Range
from geometry_msgs.msg import PoseStamped, Twist


class Arming_Modechng():

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
	
                # Angles
                if key == "angle_roll_left":
                    self.angle_roll_left = value
                if key == "angle_roll_right":
                    self.angle_roll_right = value
		if key == "angle_pitch_back":
                    self.angle_pitch_back = value	
		if key == "angle_pitch_forward":
                    self.angle_pitch_forward = value

                # Percentages
                if key == "percentage_moving_desired_direction":
                    self.percentage_moving_desired_direction = value
                if key == "percentage_moving_opposite_direction":
                    self.percentage_moving_opposite_direction = value
		if key == "percentage_hovering":
                    self.percentage_hovering = value

        self.skip_to_land = False   # In case of an error it will skip al the movements and will go automatically to landing 
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
       
        # variable to know if the drone is being controlled by the keyboard
        self.keyboardcontrolstate = None

        self.printing_value = 0 
        self.current_heading = None

        rospy.init_node("Arming_safety_node")

        self.imu_sub = rospy.Subscriber("/mavros/imu/data", Imu,self.imu_callback)
        self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.local_target_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)
        #self.takeoffService = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
        self.attitude_target_pub = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
        subscribed_topic_d = "/sonarTP_D"
        self.down_sensor = rospy.Subscriber(subscribed_topic_d, Range, self.cb_down_sensor)
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.cb_local_pose)

        # Custom subscribers
        self.front_obstacle_detection_sub = rospy.Subscriber("/Front_Obstacle", Range, self.front_obstacle_detection_callback)
        self.back_obstacle_detection_sub = rospy.Subscriber("/Back_Obstacle", Range, self.back_obstacle_detection_callback)
        self.left_obstacle_detection_sub = rospy.Subscriber("/Left_Obstacle", Range, self.left_obstacle_detection_callback)
        self.right_obstacle_detection_sub = rospy.Subscriber("/Right_Obstacle", Range, self.right_obstacle_detection_callback)
        self.keyboardcontrol_target_sub = rospy.Subscriber("/custom/keyboardcontrol", String, self.keyboardcontrol_callback)

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

    def cb_down_sensor(self, msg):
        self.down_sensor_distance_old = self.down_sensor_distance
        self.down_sensor_distance = msg.range
        self.down_sensor_changed = True
        if self.down_sensor_distance_old < self.down_sensor_distance:
            self.down_sensor_distance_higher = True
        else:
            self.down_sensor_distance_higher = False

        if self.down_sensor_distance > 2:
            self.skip_to_land = True

    def imu_callback(self, msg):
        global global_imu
        self.imu = msg
        self.current_heading = self.q2yaw(self.imu.orientation) # Transforms q into degrees of yaw
        self.received_imu = True
        
    def keyboardcontrol_callback(self, msg):
        self.keyboardcontrolstate = msg.data 
        if self.keyboardcontrolstate == "Land":
            self.skip_to_land = True
        if self.keyboardcontrolstate == "Emergency land":
            self.skip_to_land = True
            self.change_thrusts(0.45, 0.45, 0.45)
 
    def euler2quaternion(self, roll = 0, pitch = 0, yaw = 0):
        qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
        qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
        qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
        qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

        return [qx, qy, qz, qw]
        
    def calculate_recursions(self, total_time):
	recursions = total_time/self.Time_between_messages
        return int(recursions)

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
		

    def change_thrusts(self, newLiftoff_thrust, newhover_thrust, newLanding_thrust):
        self.Liftoff_thrust = newLiftoff_thrust
        self.hover_thrust = newhover_thrust
        self.Landing_thrust = newLanding_thrust
        self.Moving_max_thrust = self.Liftoff_thrust
        self.Moving_min_thrust = self.Landing_thrust

#----------------------arming services----------------------------
    def arm(self):
        if self.armService(True):
            rospy.loginfo("AdvanDiptera is Armed")
            return True
        else:
            rospy.loginfo("Failed to arm AdvanDiptera")
            return False

    def disarm(self):
        if self.armService(False):
            rospy.loginfo("Advandiptera succesfully disarmed")
            return True
        else:
            rospy.loginfo("Failed to disarm AdvanDiptera")
            return False
        
#----------------------messages constructor----------------------------
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
    

    def construct_target_attitude(self, body_x = 0, body_y = 0, body_z = 0, thrust=0, roll_angle = 0, pitch_angle = 0, yaw_angle = 0):

        target_raw_attitude = AttitudeTarget()  # We will fill the following message with our values: http://docs.ros.org/api/mavros_msgs/html/msg/PositionTarget.html
        target_raw_attitude.header.stamp = rospy.Time.now()
        #target_raw_attitude.orientation = self.imu.orientation
        q = self.to_quaternion(roll_angle, pitch_angle, yaw_angle)
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

################################# AVOIDING OBSTACLE ########################################
# This functions detect the obstacle and sends the order to move the drone in the opposite direction if needed
# There are three ranges, depending what the lidar sense, it will enter to one zone or another
    def front_obstacle_detection_callback(self, msg):
        self.front_sensor_distance = msg.range
        sensor_distance = msg.range

        if (self.beh_type == 'HOVER' or self.beh_type == 'MOVING'):
 
            if sensor_distance <= self.Avoiding_obstacle_distance_min and self.stateForward != "AVOID":
                self.counterForwardAvoidObstacle = self.counterForwardAvoidObstacle + 1
                self.counterForwardBlockMovement = 0
                self.counterForwardUnblockMovement = 0
                if self.counterForwardAvoidObstacle >= 3:
                    self.stateForward = "AVOID"
                    self.blockingMovementFront = True   
                    self.avoiding_front_obstacle(self.Avoiding_obstacle_time)

            elif sensor_distance <= self.Blocking_movement_distance_min and self.stateForward != "BLOCK":
                self.counterForwardAvoidObstacle = 0
                self.counterForwardBlockMovement = self.counterForwardBlockMovement + 1
                self.counterForwardUnblockMovement = 0
                if self.counterForwardBlockMovement >= 3:
                    self.stateForward = "BLOCK"
                    self.blockingMovementFront = True

            elif sensor_distance > self.Unblocking_movement_distance_min and self.stateForward != "UNBLOCK":
                self.counterForwardAvoidObstacle = 0
                self.counterForwardBlockMovement = 0
                self.counterForwardUnblockMovement = self.counterForwardUnblockMovement + 1
                if self.counterForwardUnblockMovement >= 3:
                    self.stateForward = "UNBLOCK"
                    self.blockingMovementFront = False
           
    def back_obstacle_detection_callback(self, msg):
        self.back_sensor_distance = msg.range
        sensor_distance = msg.range

        if (self.beh_type == 'HOVER' or self.beh_type == 'MOVING'):
 
            if sensor_distance <= self.Avoiding_obstacle_distance_min and self.stateBack != "AVOID":
                self.counterBackAvoidObstacle = self.counterBackAvoidObstacle + 1
                self.counterBackBlockMovement = 0
                self.counterBackUnblockMovement = 0
                if self.counterBackAvoidObstacle >= 3:
                    self.stateBack = "AVOID"
                    self.blockingMovementBack = True   
                    self.avoiding_back_obstacle(self.Avoiding_obstacle_time)

            elif sensor_distance <= self.Blocking_movement_distance_min and self.stateBack != "BLOCK":
                self.counterBackAvoidObstacle = 0
                self.counterBackBlockMovement = self.counterBackBlockMovement + 1
                self.counterBackUnblockMovement = 0
                if self.counterBackBlockMovement >= 3:
                    self.stateBack = "BLOCK"
                    self.blockingMovementBack = True

            elif sensor_distance > self.Unblocking_movement_distance_min and self.stateBack != "UNBLOCK":
                self.counterBackAvoidObstacle = 0
                self.counterBackBlockMovement = 0
                self.counterBackUnblockMovement = self.counterBackUnblockMovement + 1
                if self.counterBackUnblockMovement >= 3:
                    self.stateBack = "UNBLOCK"
                    self.blockingMovementBack = False

    def right_obstacle_detection_callback(self, msg):
        self.right_sensor_distance = msg.range
        sensor_distance = msg.range

        if (self.beh_type == 'HOVER' or self.beh_type == 'MOVING'):
 
            if sensor_distance <= self.Avoiding_obstacle_distance_min and self.stateRight != "AVOID":
                self.counterRightAvoidObstacle = self.counterRightAvoidObstacle + 1
                self.counterRightBlockMovement = 0
                self.counterRightUnblockMovement = 0
                if self.counterRightAvoidObstacle >= 3:
                    self.stateRight = "AVOID"
                    self.blockingMovementRight = True   
                    self.avoiding_right_obstacle(self.Avoiding_obstacle_time)

            elif sensor_distance <= self.Blocking_movement_distance_min and self.stateRight != "BLOCK":
                self.counterRightAvoidObstacle = 0
                self.counterRightBlockMovement = self.counterRightBlockMovement + 1
                self.counterRightUnblockMovement = 0
                if self.counterRightBlockMovement >= 3:
                    self.stateRight = "BLOCK"
                    self.blockingMovementRight = True

            elif sensor_distance > self.Unblocking_movement_distance_min and self.stateRight != "UNBLOCK":
                self.counterRightAvoidObstacle = 0
                self.counterRightBlockMovement = 0
                self.counterRightUnblockMovement = self.counterRightUnblockMovement + 1
                if self.counterRightUnblockMovement >= 3:
                    self.stateRight = "UNBLOCK"
                    self.blockingMovementRight = False

    def left_obstacle_detection_callback(self, msg):
        self.left_sensor_distance = msg.range
        sensor_distance = msg.range

        if (self.beh_type == 'HOVER' or self.beh_type == 'MOVING'):
 
            if sensor_distance <= self.Avoiding_obstacle_distance_min and self.stateLeft != "AVOID":
                self.counterLeftAvoidObstacle = self.counterLeftAvoidObstacle + 1
                self.counterLeftBlockMovement = 0
                self.counterLeftUnblockMovement = 0
                if self.counterLeftAvoidObstacle >= 3:
                    self.stateLeft = "AVOID"
                    self.blockingMovementLeft = True   
                    self.avoiding_left_obstacle(self.Avoiding_obstacle_time)

            elif sensor_distance <= self.Blocking_movement_distance_min and self.stateLeft != "BLOCK":
                self.counterLeftAvoidObstacle = 0
                self.counterLeftBlockMovement = self.counterLeftBlockMovement + 1
                self.counterLeftUnblockMovement = 0
                if self.counterLeftBlockMovement >= 3:
                    self.stateLeft = "BLOCK"
                    self.blockingMovementLeft = True

            elif sensor_distance > self.Unblocking_movement_distance_min and self.stateLeft != "UNBLOCK":
                self.counterLeftAvoidObstacle = 0
                self.counterLeftBlockMovement = 0
                self.counterLeftUnblockMovement = self.counterLeftUnblockMovement + 1
                if self.counterLeftUnblockMovement >= 3:
                    self.stateLeft = "UNBLOCK"
                    self.blockingMovementLeft = False


################################# LIFT OFF ########################################
	
    def lift_off_rec(self, thrust, time_flying): # Not used now
        recursions = self.calculate_recursions(time_flying)
        print(recursions)
        self.beh_type = "TAKE OFF"
	for i in range(recursions):
            print 'Recursion: ' + str(i) + '   Total Recursions taking off: ' + str(recursions)
            print 'Sonar down distance: ' + str(self.down_sensor_distance)

            if self.beh_type == "TAKE OFF" and thrust < self.Liftoff_thrust: 
                print("Lifting the drone up slowly")
                print 'Thrust: ' + str(thrust)
                self.construct_target_attitude(0,0,0,thrust)
                thrust = thrust + self.accumulating_thrust_soft

            elif self.beh_type == "TAKE OFF" and thrust >= self.Liftoff_thrust and self.down_sensor_distance <= self.hover_sensor_altitude_min:
                print("The drone is lifting off at constant thrust")
                print 'Thrust: ' + str(thrust)
                thrust = self.Liftoff_thrust
                self.construct_target_attitude(0,0,0,thrust)

            else:
                break

        print ("time of lifting off has ended")




    def automatic_lift_off_rec(self, thrust, time_flying):
        recursions = self.calculate_recursions(time_flying)
        print(recursions)
        self.beh_type = "TAKE OFF"
	for i in range(recursions):
            print 'Recursion: ' + str(i) + '   Total Recursions taking off: ' + str(recursions)
            print 'Sonar down distance: ' + str(self.down_sensor_distance)

            if self.skip_to_land == True or thrust > self.Liftoff_thrust_maximum:
                print("Going to land")
                break

            elif self.beh_type == "TAKE OFF" and thrust < self.Liftoff_thrust_first: 
                print("Lifting the drone up slowly")
                print 'Thrust: ' + str(thrust)
                self.construct_target_attitude(0,0,0,thrust)
                thrust = thrust + self.accumulating_thrust_soft

            elif self.beh_type == "TAKE OFF" and self.down_sensor_distance <= 0.4:
                print("The drone is searching the lifting off thrust")
                print 'Thrust: ' + str(thrust)
                self.construct_target_attitude(0,0,0,thrust)
                if self.down_sensor_changed == True:
                    self.down_sensor_changed = False
                    thrust = thrust + 0.01
                    self.change_thrusts(thrust, thrust, thrust)

            elif self.beh_type == "TAKE OFF" and self.down_sensor_distance >= 0.4 and self.down_sensor_distance <= self.hover_sensor_altitude_min:
                print("The drone is searching the lifting off thrust")
                print 'Thrust: ' + str(thrust)
                self.construct_target_attitude(0,0,0,thrust)
                if self.down_sensor_changed == True:
                    self.down_sensor_changed = False
                    if self.down_sensor_distance_higher == False:
                        thrust = thrust + 0.01
                        self.change_thrusts(thrust, thrust, thrust)

            elif self.beh_type == "TAKE OFF" and self.down_sensor_distance > self.hover_sensor_altitude_min:
                self.change_thrusts(thrust + 0.015, thrust, thrust - 0.01)
            

################################# HOVER ########################################

    def hover_rec(self, time_flying): 
        recursions = self.calculate_recursions(time_flying)
        difference_distance = self.hover_sensor_altitude_max - self.hover_sensor_altitude_min
        difference_thrust = self.Liftoff_thrust - self.Landing_thrust
        proportional_thrust = difference_thrust / difference_distance
        self.beh_type = 'HOVER'

        print(recursions)
	for i in range(recursions):
            print 'Recursion: ' + str(i) + '   Total Recursions hovering: ' + str(recursions)
            print 'Sonar down distance: ' + str(self.down_sensor_distance)        

            if self.skip_to_land == True:
                break
            
            elif self.keyboardcontrolstate == "Controlled":
                time.sleep(self.Keyboard_control_time)

            elif self.stateForwardAvoiding == True or self.stateBackAvoiding == True or self.stateForwardAvoiding == True or self.stateBackAvoiding == True:
                print("Avoiding")

            else:
                if self.beh_type == 'HOVER' and (self.hover_sensor_altitude_max >= self.down_sensor_distance >= self.hover_sensor_altitude_min):
                    distance_more = self.down_sensor_distance - self.hover_sensor_altitude_min 
                    thrust = self.Liftoff_thrust - distance_more * proportional_thrust
                    print('Thrust: ' + str(thrust) + '   The drone is hovering')           
				
                elif self.beh_type == 'HOVER' and (self.hover_sensor_altitude_max <= self.down_sensor_distance): # We have to go down
                    thrust = self.Landing_thrust
                    print('Thrust: ' + str(thrust) + '   Recovering hover position - going down')
				
                elif self.beh_type == 'HOVER' and (self.down_sensor_distance <= self.hover_sensor_altitude_min): # We have to go up
                    thrust = self.Liftoff_thrust
                    print('Thrust: ' + str(thrust) + '   Recovering hover position - going up')

                elif self.beh_type == 'HOVER' and (self.hover_sensor_altitude_max + 0.3 <= self.down_sensor_distance): # We have to go down + change thrusts variables
                    if self.down_sensor_changed == True:
                        self.down_sensor_changed = False
                        thrust = thrust - 0.002
                        self.change_thrusts(thrust + 0.035, thrust + 0.02, thrust)
                    thrust = self.Landing_thrust
                    print('Thrust: ' + str(thrust) + '   Recovering hover position - going down')
				
                elif self.beh_type == 'HOVER' and (self.down_sensor_distance <= self.hover_sensor_altitude_min - 0.3): # We have to go up + change thrusts variables
                    if self.down_sensor_changed == True:
                        self.down_sensor_changed = False
                        thrust = thrust + 0.002
                        self.change_thrusts(thrust, thrust - 0.015, thrust - 0.035)
                    thrust = self.Liftoff_thrust
                    print('Thrust: ' + str(thrust) + '   Recovering hover position - going up')

                self.construct_target_attitude(0,0,0,thrust)
	
        print ("time of hovering has ended")


    def automatic_hover_rec(self, time_flying): # Not used
        recursions = self.calculate_recursions(time_flying)
        difference_distance = self.hover_sensor_altitude_max - self.hover_sensor_altitude_min
        difference_thrust = self.Liftoff_thrust - self.Landing_thrust
        proportional_thrust = difference_thrust / difference_distance
        self.beh_type = 'HOVER'
       
        thrust = 0
        counter = 0

        print(recursions)
	for i in range(recursions):
            print 'Recursion: ' + str(i) + '   Total Recursions hovering: ' + str(recursions)
            print 'Sonar down distance: ' + str(self.down_sensor_distance)        

            if self.skip_to_land == True:
                break
            
            elif self.keyboardcontrolstate == "Controlled":
                time.sleep(self.Keyboard_control_time)

            elif self.stateForwardAvoiding == True or self.stateBackAvoiding == True or self.stateForwardAvoiding == True or self.stateBackAvoiding == True:
                time.sleep(self.Avoiding_obstacle_time)

            elif self.beh_type == 'HOVER' and (self.hover_sensor_altitude_max >= self.down_sensor_distance >= self.hover_sensor_altitude_min):
                distance_more = self.down_sensor_distance - self.hover_sensor_altitude_min 
                thrust = self.Liftoff_thrust - distance_more * proportional_thrust
                print('Thrust: ' + str(thrust) + '   The drone is hovering')
                self.construct_target_attitude(0,0,0,thrust)   
                if i > int(recursions*0.8):
                    counter = counter + 1       
				
            elif self.beh_type == 'HOVER' and (self.hover_sensor_altitude_max <= self.down_sensor_distance): # We have to go down
                thrust = self.Landing_thrust
                print('Thrust: ' + str(thrust) + '   Recovering hover position - going down')
                self.construct_target_attitude(0,0,0,thrust)
				
            elif self.beh_type == 'HOVER' and (self.down_sensor_distance <= self.hover_sensor_altitude_min): # We have to go up
                thrust = self.Liftoff_thrust
                print('Thrust: ' + str(thrust) + '   Recovering hover position - going up')
                self.construct_target_attitude(0,0,0,thrust)

        if counter / int (recursions*0.2) > 0.8:
            self.hover_sensor_altitude_max = self.hover_sensor_altitude_max - 0.2
            self.hover_sensor_altitude_min = self.hover_sensor_altitude_min - 0.2
            self.Landing_thrust = thrust + 0.02
            self.Liftoff_thrust = thrust + 0.02
            self.hover_thrust = thrust
            print('Landing Thrust: ' + str(Landing_thrust))
            print('Liftoff Thrust: ' + str(Liftoff_thrust))
            print('Hover Thrust: ' + str(hover_thrust))
        else: 
            print ("Couldn't calculate an appropiate hover thrust")
            error = (1-counter / int (recursions*0.2))*100
            print("Error: " + str(error) + "%")
	
        print ("time of hovering has ended")

################################# MOVE RAW ########################################
# Movement in just one direction

#                      /// RIGHT MOVE RAW ///

    def moving_right_raw_rec(self, time_flying): 
        recursions = self.calculate_recursions(time_flying)
        difference_distance = self.hover_sensor_altitude_max - self.hover_sensor_altitude_min
        difference_thrust = self.Moving_max_thrust - self.Moving_min_thrust
        proportional_thrust = difference_thrust / difference_distance
        print(recursions)
        self.beh_type = 'MOVING'
        thrust = self.Moving_min_thrust

	for i in range(recursions):
            print 'Recursion: ' + str(i) + '   Total Recursions moving right: ' + str(recursions)
            print 'Sonar down distance: ' + str(self.down_sensor_distance)        

            if self.skip_to_land == True or self.blockingMovementRight == True:
                break

            elif self.keyboardcontrolstate == "Controlled":
                time.sleep(self.Keyboard_control_time)

            elif self.stateForwardAvoiding == True or self.stateBackAvoiding == True:
                print("Avoiding")

            else:
                if self.beh_type == 'MOVING' and (self.hover_sensor_altitude_max >= self.down_sensor_distance >= self.hover_sensor_altitude_min):
                    distance_more = self.down_sensor_distance - self.hover_sensor_altitude_min 
                    thrust = self.Moving_max_thrust - distance_more * proportional_thrust
                    print('Thrust: ' + str(thrust) + '   The drone is moving right')
				
                elif self.beh_type == 'MOVING' and (self.hover_sensor_altitude_max <= self.down_sensor_distance): # We have to go down
                    thrust = self.Moving_min_thrust
                    print('Thrust: ' + str(thrust) + '   Recovering altitude position - going down + moving right')
				
                elif self.beh_type == 'MOVING' and (self.down_sensor_distance <= self.hover_sensor_altitude_min): # We have to go up
                    thrust = self.Moving_max_thrust
                    print('Thrust: ' + str(thrust) + '   Recovering altitude position - going up + moving right')

                self.construct_target_attitude(0,0,0,thrust, self.angle_roll_right, 0, 0)

#                      /// LEFT MOVE RAW ///

    def moving_left_raw_rec(self, time_flying): 
        recursions = self.calculate_recursions(time_flying)
        difference_distance = self.hover_sensor_altitude_max - self.hover_sensor_altitude_min
        difference_thrust = self.Moving_max_thrust - self.Moving_min_thrust
        proportional_thrust = difference_thrust / difference_distance
        print(recursions)
        self.beh_type = 'MOVING'
        thrust = self.Moving_min_thrust

	for i in range(recursions):
            print 'Recursion: ' + str(i) + '   Total Recursions moving left: ' + str(recursions)
            print 'Sonar down distance: ' + str(self.down_sensor_distance)        

            if self.skip_to_land == True or self.blockingMovementLeft == True:
                break

            elif self.keyboardcontrolstate == "Controlled":
                time.sleep(self.Keyboard_control_time)

            elif self.stateForwardAvoiding == True or self.stateBackAvoiding == True:
                print("Avoiding")

            else:
                if self.beh_type == 'MOVING' and (self.hover_sensor_altitude_max >= self.down_sensor_distance >= self.hover_sensor_altitude_min):
                    distance_more = self.down_sensor_distance - self.hover_sensor_altitude_min 
                    thrust = self.Moving_max_thrust - distance_more * proportional_thrust
                    print('Thrust: ' + str(thrust) + '   The drone is moving left')
				
                elif self.beh_type == 'MOVING' and (self.hover_sensor_altitude_max <= self.down_sensor_distance): # We have to go down
                    thrust = self.Moving_min_thrust
                    print('Thrust: ' + str(thrust) + '   Recovering altitude position - going down + moving left')
				
                elif self.beh_type == 'MOVING' and (self.down_sensor_distance <= self.hover_sensor_altitude_min): # We have to go up
                    thrust = self.Moving_max_thrust
                    print('Thrust: ' + str(thrust) + '   Recovering altitude position - going up + moving left')

                self.construct_target_attitude(0,0,0,thrust, self.angle_roll_left, 0, 0)

#                      /// BACK MOVE RAW ///
                
    def moving_back_raw_rec(self, time_flying): 
        recursions = self.calculate_recursions(time_flying)
        difference_distance = self.hover_sensor_altitude_max - self.hover_sensor_altitude_min
        difference_thrust = self.Moving_max_thrust - self.Moving_min_thrust
        proportional_thrust = difference_thrust / difference_distance
        print(recursions)
        self.beh_type = 'MOVING'
        thrust = self.Moving_min_thrust

	for i in range(recursions):
            print 'Recursion: ' + str(i) + '   Total Recursions moving back: ' + str(recursions)
            print 'Sonar down distance: ' + str(self.down_sensor_distance)        

            if self.skip_to_land == True or self.blockingMovementBack == True:
                break

            elif self.keyboardcontrolstate == "Controlled":
                time.sleep(self.Keyboard_control_time)

            elif self.stateLeftAvoiding == True or self.stateRightAvoiding == True:
                print("Avoiding")

            else:
                if self.beh_type == 'MOVING' and (self.hover_sensor_altitude_max >= self.down_sensor_distance >= self.hover_sensor_altitude_min):
                    distance_more = self.down_sensor_distance - self.hover_sensor_altitude_min 
                    thrust = self.Moving_max_thrust - distance_more * proportional_thrust
                    print('Thrust: ' + str(thrust) + '   The drone is moving back')
				
                elif self.beh_type == 'MOVING' and (self.hover_sensor_altitude_max <= self.down_sensor_distance): # We have to go down
                    thrust = self.Moving_min_thrust
                    print('Thrust: ' + str(thrust) + '   Recovering altitude position - going down + moving back')
				
                elif self.beh_type == 'MOVING' and (self.down_sensor_distance <= self.hover_sensor_altitude_min): # We have to go up
                    thrust = self.Moving_max_thrust
                    print('Thrust: ' + str(thrust) + '   Recovering altitude position - going up + moving back')

                self.construct_target_attitude(0,0,0,thrust, 0, self.angle_pitch_back, 0)

#                      /// FORWARD MOVE RAW ///

    def moving_forward_raw_rec(self, time_flying): 
        recursions = self.calculate_recursions(time_flying)
        difference_distance = self.hover_sensor_altitude_max - self.hover_sensor_altitude_min
        difference_thrust = self.Moving_max_thrust - self.Moving_min_thrust
        proportional_thrust = difference_thrust / difference_distance
        print(recursions)
        self.beh_type = 'MOVING'
        thrust = self.Moving_min_thrust

	for i in range(recursions):
            print 'Recursion: ' + str(i) + '   Total Recursions moving forward: ' + str(recursions)
            print 'Sonar down distance: ' + str(self.down_sensor_distance)        

            if self.skip_to_land == True or self.blockingMovementFront == True:
                break

            elif self.keyboardcontrolstate == "Controlled":
                time.sleep(self.Keyboard_control_time)

            elif self.stateLeftAvoiding == True or self.stateRightAvoiding == True:
                print("Avoiding")

            else:
                if self.beh_type == 'MOVING' and (self.hover_sensor_altitude_max >= self.down_sensor_distance >= self.hover_sensor_altitude_min):
                    distance_more = self.down_sensor_distance - self.hover_sensor_altitude_min 
                    thrust = self.Moving_max_thrust - distance_more * proportional_thrust
                    print('Thrust: ' + str(thrust) + '   The drone is moving forward')
				
                elif self.beh_type == 'MOVING' and (self.hover_sensor_altitude_max <= self.down_sensor_distance): # We have to go down
                    thrust = self.Moving_min_thrust
                    print('Thrust: ' + str(thrust) + '   Recovering altitude position - going down + moving forward')
				
                elif self.beh_type == 'MOVING' and (self.down_sensor_distance <= self.hover_sensor_altitude_min): # We have to go up
                    thrust = self.Moving_max_thrust
                    print('Thrust: ' + str(thrust) + '   Recovering altitude position - going up + moving forward')

                self.construct_target_attitude(0,0,0,thrust, 0, self.angle_pitch_forward, 0)


################################# MOVE ########################################
# Each movement consists of three steeps 
# 1. movement in desired direction
# 2. movement in opposite direction 
# 3. hover

#                      /// RIGHT MOVE ///

    def moving_right_rec(self, time_flying): 

        move_desired_direction_time = time_flying * self.percentage_moving_desired_direction
        move_opposite_direction_time = time_flying * self.percentage_moving_opposite_direction
        hover_time = time_flying * self.percentage_hovering

        self.moving_right_raw_rec(move_desired_direction_time)
        self.moving_left_raw_rec(move_opposite_direction_time)       
        self.hover_rec(hover_time)

#                      /// LEFT MOVE ///

    def moving_left_rec(self, time_flying):
 
        move_desired_direction_time = time_flying * self.percentage_moving_desired_direction
        move_opposite_direction_time = time_flying * self.percentage_moving_opposite_direction
        hover_time = time_flying * self.percentage_hovering

        self.moving_left_raw_rec(move_desired_direction_time)
        self.moving_right_raw_rec(move_opposite_direction_time)       
        self.hover_rec(hover_time)

#                      /// FRONT MOVE ///

    def moving_forward_rec(self, time_flying):
 
        move_desired_direction_time = time_flying * self.percentage_moving_desired_direction
        move_opposite_direction_time = time_flying * self.percentage_moving_opposite_direction
        hover_time = time_flying * self.percentage_hovering

        self.moving_forward_raw_rec(move_desired_direction_time)
        self.moving_back_raw_rec(move_opposite_direction_time)       
        self.hover_rec(hover_time)

#                      /// BACK MOVE ///

    def moving_back_rec(self, time_flying):
 
        move_desired_direction_time = time_flying * self.percentage_moving_desired_direction
        move_opposite_direction_time = time_flying * self.percentage_moving_opposite_direction
        hover_time = time_flying * self.percentage_hovering

        self.moving_back_raw_rec(move_desired_direction_time)
        self.moving_forward_raw_rec(move_opposite_direction_time)       
        self.hover_rec(hover_time)


################################# AVOID OBSTACLE MOVE ########################################
# Blocks each movement, so the lidar only sends one time each obstacle to evit

#                      /// AVOIDING LEFT - MOVE RIGHT  ///

    def avoiding_left_obstacle(self, time_flying):
        if self.stateAvoiding == False:
            self.stateAvoiding = True
            self.stateLeftAvoiding = True
            print("Avoiding Left Obstacle")
            self.moving_right_rec(time_flying)
            self.stateLeftAvoiding = False
            self.stateAvoiding = False
   

#                      /// AVOIDING RIGHT - MOVE LEFT  ///

    def avoiding_right_obstacle(self, time_flying):
        if self.stateAvoiding == False:
            self.stateAvoiding = True
            self.stateRightAvoiding = True
            print("Avoiding Right Obstacle")
            self.moving_left_rec(time_flying)
            self.stateRightAvoiding = False
            self.stateAvoiding = False

#                      /// AVOIDING FRONT - MOVE BACK  ///

    def avoiding_front_obstacle(self, time_flying):
        if self.stateAvoiding == False:
            self.stateAvoiding = True
            self.stateForwardAvoiding = True
            print("Avoiding Front Obstacle")
            self.moving_back_rec(time_flying)
            self.stateForwardAvoiding = False
            self.stateAvoiding = False

#                      /// AVOIDING BACK - MOVE FRONT  ///

    def avoiding_back_obstacle(self, time_flying):
        if self.stateAvoiding == False:
            self.stateAvoiding = True
            self.stateBackAvoiding = True
            print("Avoiding Back Obstacle")
            self.moving_forward_rec(time_flying)
            self.stateBackAvoiding = False
            self.stateAvoiding = False



################################# LANDING ########################################
	
    def landing_rec(self):                       # Landing phase
        self.beh_type = "LANDING"
        recursions = self.calculate_recursions(self.Max_time_landing)
        print(recursions)
        thrust = self.hover_thrust
        for i in range(recursions):
            print 'Recursion: ' + str(i) + '   Total Recursions landing: ' + str(recursions)
            print 'Sonar down distance: ' + str(self.down_sensor_distance)

            if self.down_sensor_distance <= self.landing_sensor_altitude_min:
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
            print 'Sonar down distance: ' + str(self.down_sensor_distance)

            if thrust <= 0:
                break

            if i < 1000:
                thrust = self.Landing_thrust
                print('Thrust: ' + str(thrust) + '   Smooth landing')
                self.construct_target_attitude(0,0,0,thrust)

            else: 
                print('Thrust: ' + str(thrust) + '   Landing')
                self.construct_target_attitude(0,0,0,thrust)
                thrust = thrust - self.accumulating_thrust_soft

        for i in range(20):
            
            self.arm_state = self.disarm()    # disarms the drone
            time.sleep(0.1)

    def secure_landing_phase_rec(self):           # Secure landing part - last cm - not used, just another idea to land the drone
        self.beh_type = "LANDING"
        recursions = self.calculate_recursions(self.Secure_time_landing)	
        thrust = self.hover_thrust
        for i in range(recursions):
            print 'Recursion: ' + str(i) + '   Total Recursions secure landing: ' + str(recursions)
            print 'Sonar down distance: ' + str(self.down_sensor_distance)

            if thrust <= 0:
                break

            elif i < 200:
                thrust = self.hover_thrust
                print('Thrust: ' + str(thrust) + '   Secure hover before landing')
                self.construct_target_attitude(0,0,0,thrust)

            elif i < 700:
                thrust = self.hover_thrust - self.Deaccumulating_thrust
                print('Thrust: ' + str(thrust) + '   Smooth landing')
                self.construct_target_attitude(0,0,0,thrust)

            else: 
                thrust = thrust - self.accumulating_thrust_soft
                print('Thrust: ' + str(thrust) + '   Landing')
                self.construct_target_attitude(0,0,0,thrust)
        for i in range(20):
            
            self.arm_state = self.disarm()    # disarms the drone
            time.sleep(0.1)	




#----------------------change modes----------------------------

    def modechnge(self):
        #rospy.init_node("offboard_node")
        if self.flightModeService(custom_mode='OFFBOARD'):
            rospy.loginfo("succesfully changed mode to OFFBOARD")
            return True
        else:
            rospy.loginfo("failed to change mode")
            return False        

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
        #arm.lift_off_rec(arm.init_thrust, arm.Liftoff_time)
        arm.automatic_lift_off_rec(arm.init_thrust, arm.Liftoff_time)

        arm.hover_rec(arm.hover_time)
        #arm.automatic_hover_rec(arm.hover_time)
       
        # Moving
        #arm.moving_right_rec(arm.Moving_time)
        #arm.moving_left_rec(arm.Moving_time)
        #arm.moving_forward_rec(arm.Moving_time)
        #arm.moving_back_rec(arm.Moving_time)

        # Hover + landing
        #arm.hover_rec(arm.hover_time)
        arm.landing_rec()

    except rospy.ROSInterruptException: pass
