#!/usr/bin/env python

from __future__ import print_function

import roslib; roslib.load_manifest('teleop_twist_keyboard')
import rospy
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget, AttitudeTarget
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import Diptera_arm2

import sys, select, termios, tty


class keyboard_control():
    msg = """
    Abdussalam.alajami@upf.edu
    Reading from the keyboard  and Publishing to attitude mavros msg!
    ---------------------------
    Moving around:
       q    w    e
       a    s    d
       z    x    .
    For Holonomic mode (strafing), hold down the shift key:
    ---------------------------
        U    I    O
        J    K    L
        M    <    >
        t : up (+z)
        b : down (-z)
        anything else : let diptera scripts send
        q/z : increase/decrease max speeds by 10%
        b/n : increase/decrease only linear speed by 10%
        e/c : increase/decrease only angular speed by 10%
        CTRL-C to quit
        """


    def __init__(self):
        self.imu_sub = rospy.Subscriber("/mavros/imu/data", Imu,self.imu_callback)
        self.countx = 0
        self.county = 0
        self.countz = 0
        self.count_thrust = 0
        self.acc_x = 0
        self.acc_y = 0
        self.acc_z = 0
        self.acc_thrust = 0
        self.orientation.w = 0
        self.orientation.x = 0
        self.orientation.y = 0
        self.orientation.z = 0
        self.timeflying = 0
        self.movement = str
        self.moveBindings = {
            'w': (0, 0.1, 0, 0),    #pith forward
            's': (0, -0.1, 0, 0),   #pitch backward
            'd': (0.1, 0, 0, 0 ),   #roll right
            'a': (-0.1, 0, 0, 0),   #roll left
            'e': (0, 0, 0, 0.1),    #thrust desired
            'q': (0, 0, 0, -0.01),  #decend thrust soft -
            'z': (0, 0, 0.1, 0),    #change thrust + 
            'x': (0, 0, -0.1, 0),   #change thrust - 
            #'O': (1, -1, 0, 0),
            #'I': (1, 0, 0, 0),
            #'J': (0, 1, 0, 0),
            #'L': (0, -1, 0, 0),
            #'U': (1, 1, 0, 0),
            #'<': (-1, 0, 0, 0),
            #'>': (-1, -1, 0, 0),
            #'M': (-1, 1, 0, 0),
            #'t': (0, 0, 1, 0),
            #'b': (0, 0, -1, 0),
        }

        self.speedBindings = {
            #'q': (1.1, 1.1),
            #'z': (.9, .9),
            #'i': (1.1, 1),
            #'x': (.9, 1),
            #'e': (1, 1.1),
            #'c': (1, .9),
        }

        self.yamlpath = '/home/ubuntu/AdvanDiptera/src/commanding_node/params/arm_params.yaml'
        with open(self.yamlpath) as file:
            data = yaml.load(file)
            for key, value in data.items():
                if key == "Time_between_messages":
                    self.Time_between_messages = value
                if key == "angle_roll_left":
                    self.angle_roll_left = value
                if key == "angle_roll_right":
                    self.angle_roll_right = value
		if key == "angle_pitch_back":
                    self.angle_pitch_back = value	
		if key == "angle_pitch_forward":
                    self.angle_pitch_forward = value

    def imu_callback(self, msg):
        global global_imu
        self.imu = msg
        #self.current_heading = self.q2yaw(self.imu.orientation) # Transforms q into degrees of yaw
        self.received_imu = True

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key


    def vels(self, speed,turn):
        return "currently:\tspeed %s\tturn %s " % (speed,turn)

    def print_movement(self,movement ,speed):
        if movement == 'Forward':
            print("w is pressed = increasing PITCH by",speed)
        elif movement == 'Backward':
            print("s is pressed = increasing PITCH by",speed)
        elif movement == 'Right':
            print("d is pressed = increasing ROLL by",speed)
        elif movement == 'Left':
            print("a is pressed = increasing ROLL by",speed)
        elif movement == 'publishing thrust':
            print("e is pressed = publishin thrust by",speed)
        elif movement == 'change thrust down soft':
            print("q is pressed = decreasing thrust by",speed)
        elif movement == 'change thrust up':
            print("z is pressed = increasing thrust by",speed)
        elif movement == 'change thrust down':
            print("x is pressed = decreasing thrust by",speed)


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

    def calculate_recursions(self, total_time):
	recursions = total_time/self.Time_between_messages
        return int(recursions)


kb = keyboard_control()
if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    attitude_target_pub = rospy.Publisher('/mavros/setpoint_raw/attitude', AttitudeTarget, queue_size=10)
    keyboardcontrol_target_pub = rospy.Publisher('/custom/keyboardcontrol', String, queue_size=10)
    rospy.init_node('drone_keyboard')

    speed = rospy.get_param("~speed", 0.5)
    turn = rospy.get_param("~turn", 1.0)
    #x = 0
    #y = 0
    #z = 0
    #th = 0
    status = 0

    try:
        print(kb.msg)
        print(kb.vels(speed,turn))
        kb.acc_thrust = 0.5
        while(1):
            key = kb.getKey()
            if key in kb.moveBindings.keys():
                x = kb.moveBindings[key][0]
                if x == 0.1:
                    kb.movement = 'Right'
                    q = kb.to_quaternion(angle_roll_right, 0, 0)
                    kb.orientation.w = q[0]
                    kb.orientation.x = q[1]
                    kb.orientation.y = q[2]
                    kb.orientation.z = q[3]
                    kb.print_movement(kb.movement,kb.acc_x)
                    kb.timeflying = 1
                elif x == -0.1:
                    q = kb.to_quaternion(angle_roll_left, 0, 0)
                    kb.orientation.w = q[0]
                    kb.orientation.x = q[1]
                    kb.orientation.y = q[2]
                    kb.orientation.z = q[3]
                    kb.movement = 'Left'
                    kb.print_movement(kb.movement,kb.acc_x)
                    kb.timeflying = 1
                y = kb.moveBindings[key][1]
                if y == 0.1:
                    q = kb.to_quaternion(0, angle_pitch_forward, 0)
                    kb.orientation.w = q[0]
                    kb.orientation.x = q[1]
                    kb.orientation.y = q[2]
                    kb.orientation.z = q[3]
                    kb.movement = 'Forward'
                    kb.print_movement(kb.movement, kb.acc_y)
                    kb.timeflying = 1
                elif y == -0.1:
                    q = kb.to_quaternion(0, angle_pitch_back, 0)
                    kb.orientation.w = q[0]
                    kb.orientation.x = q[1]
                    kb.orientation.y = q[2]
                    kb.orientation.z = q[3]
                    kb.movement = 'Backward'
                    kb.print_movement(kb.movement, kb.acc_y)
                    kb.timeflying = 1
                z = kb.moveBindings[key][2]
                if z == 0.1: # Increasing thrust
                    kb.acc_thrust = acc_thrust + 0.1
                    kb.movement = 'change thrust up'
                    kb.print_movement(kb.movement,kb.acc_z)
                elif z == -0.1: # Decreasing thrust
                    kb.countz = kb.countz 
                    kb.acc_thrust = kb.acc_thrust - 0.1 
                    kb.movement = 'change thrust down'
                    kb.print_movement(kb.movement,kb.acc_z)
                th = kb.moveBindings[key][3]
                if th == 0.1: # print our desired thrust
                    q = kb.to_quaternion(0, 0, 0)
                    kb.orientation.w = q[0]
                    kb.orientation.x = q[1]
                    kb.orientation.y = q[2]
                    kb.orientation.z = q[3]
                    kb.acc_thrust = kb.acc_thrust + 0.1
                    kb.movement = 'publishing thrust'
                    kb.print_movement(kb.movement,kb.acc_thrust)
                    kb.timeflying = 1
                elif th == -0.01: # Decreasing thrust soft
                    kb.acc_thrust = kb.acc_thrust - 0.01
                    kb.movement = 'change thrust down soft'
                    kb.print_movement(kb.movement,kb.acc_thrust)
            elif key in kb.speedBindings.keys():
                speed = speed * kb.speedBindings[key][0]
                turn = turn * kb.speedBindings[key][1]

                print(kb.vels(speed,turn))
                if (status == 14):
                    print(kb.msg)
                status = (status + 1) % 15
            else:
                #x = 0
                #y = 0
                #z = 0
                #th = 0
                if (key == '\x03'):
                    break
            if kb.tymeflying != 0:
                recursions = kb.calculate_recursions(kb.timeflying)
          
                for i in range (recursions)
                    
                    target_raw_attitude = AttitudeTarget()
                    target_raw_attitude.header.stamp = rospy.Time.now()
                    target_raw_attitude.orientation.w = kb.orientation.w
                    target_raw_attitude.orientation.x = kb.orientation.x
                    target_raw_attitude.orientation.y = kb.orientation.y
                    target_raw_attitude.orientation.z = kb.orientation.z
                    target_raw_attitude.body_rate.x = 0  # ROLL_RATE
                    target_raw_attitude.body_rate.y = 0  # PITCH_RATE
                    target_raw_attitude.body_rate.z = 0  # YAW_RATE
                    target_raw_attitude.thrust = kb.acc_thrust
                    attitude_target_pub.publish(target_raw_attitude)
                    keyboardcontrol_target_pub.publish(String("True"))
                    time.sleep(kb.Time_between_messages) # was 0.005 (now 50hz ,500 loops ,5sec)

            for i in range (4):
                keyboardcontrol_target_pub.publish(String("False"))  
                time.sleep(kb.Time_between_messages) # was 0.005 (now 50hz ,500 loops ,5sec)           
            #twist = Twist()
            #twist.linear.x = x*speed; twist.linear.y = y*speed; twist.linear.z = z*speed
            #twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th*turn
            #pub.publish(twist)

    except Exception as e:
        print(e)

    finally:
        print("waiting for input from user to command and control")
        '''
        target_raw_attitude = AttitudeTarget()
        target_raw_attitude.header.stamp = rospy.Time.now()
        target_raw_attitude.orientation = kb.imu.orientation
        target_raw_attitude.body_rate.x = 0  # ROLL_RATE
        target_raw_attitude.body_rate.y = 0  # PITCH_RATE
        target_raw_attitude.body_rate.z = 0  # YAW_RATE
        target_raw_attitude.thrust = 
        attitude_target_pub.publish(target_raw_attitude)
        '''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
