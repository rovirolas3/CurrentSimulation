import rospy
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
from mavros_msgs.msg import GlobalPositionTarget, State, PositionTarget
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
                if key == "construct_target":
                    self.current_heading = value       # It has to be initialized 
                if key == "initial_x_pos":
                    self.init_x = value
                if key == "initial_y_pos":
                    self.init_y = value
                if key == "initial_z_pos":
                    self.init_z = value

        rospy.init_node("Arming_safety_node")
        self.armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.flightModeService = rospy.ServiceProxy('/mavros/set_mode', SetMode)
        self.local_target_pub = rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=10)

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

    def modechnge(self):
        rospy.init_node("offboard_node")
        if self.flightModeService(costume_mode='OFFBOARD'):
            rospy.loginfo("succesfully changed mode to OFFBOARD")
            return True
        else:
            rospy.loginfo("failed to change mode")
            return False

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

    def start(self):
        for i in range(10): # Waits 5 seconds for initialization
            if self.current_heading is not None:
                break
            else:
                print("Waiting for initialization.")
                time.sleep(0.5)
        self.cur_target_pose = self.construct_target(self.init_x, self.init_y, self.init_z, self.current_heading)
        #print ("self.cur_target_pose:", self.cur_target_pose, type(self.cur_target_pose))

        for i in range(10):
            self.local_target_pub.publish(self.cur_target_pose) # Publish the drone position we initialite during the first 2 seconds
            self.arm_state = self.arm()    # arms the drone
            self.offboard_state = self.modechnge() # Calls the function offboard the will select the mode Offboard
            time.sleep(0.2)

if __name__ == '__main__':
    Gpio_start.start()
    time.sleep(3.5)
    arm = Arming_Modechng()
    arm.start()
