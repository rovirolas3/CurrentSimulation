import rospy
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget
from geometry_msgs.msg import PoseStamped
import time
import yaml
from gpio_diptera import Rpi_gpio_comm as Gpio_start
from gpio_clean import Rpi_gpio_comm_off as Gpio_stop


class Arming_Modechng():

    def __init__(self):
        self.yamlpath = '/home/lybot/AdvanDiptera/src/commanding_node/params/arm_params.yaml'
        with open(self.yamlpath) as file:
            data = yaml.load(file, Loader=yaml.FullLoader)
            for key, value in data.items():
                if key == "threshold_ground_minor":
                    self.threshold_ground_minor = value

        rospy.init_node("Arming_safety_node")
        self.local_pose_sub = rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.local_pose_callback) 
        self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.local_target_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=10)


    def local_pose_callback(self, msg): 

        self.local_pose = msg
        self.local_enu_position = msg

    def disarm(self):
        if self.armService(False):
            rospy.loginfo("Advandiptera succesfully disarmed")
            return True
        else:
            rospy.loginfo("Failed to disarm AdvanDiptera")
            return False

    def modechnge(self):
        rospy.init_node("offboard_node")
        if self.flightModeService(costume_mode='OFFBOARD'):
            rospy.loginfo("succesfully changed mode to OFFBOARD")
            return True
        else:
            rospy.loginfo("failed to change mode")
            return False


    def start(self):
        for i in range(10): # Waits 5 seconds for initialization
            if self.local_pose.pose.position.z is not None:
                break
            else:
                print("Waiting for initialization.")
                time.sleep(0.5)

        while self.local_pose.pose.position.z > self.threshold_ground_minor: # If we land and we are under 0.15 in the z position...
            self.offboard_state = self.modechnge() # Calls the function offboard the will select the mode Offboard
            time.sleep(0.2) # Rate 

        self.disarm() # ... we disarm the drone

            


if __name__ == '__main__':

    disarm = Arming_Modechng()
    disarm.start()
    Gpio_stop.start()
    time.sleep(3.5)
