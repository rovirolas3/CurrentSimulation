import time
import maxSonarTTY
import sys
import signal
import geometry_msgs.msg
from sensor_msgs.msg import Range

import rospy
from std_msgs.msg import Float32


serialPort = "/dev/ttyUSB0"
maxRange = 2000  # change for 5m vs 10m sensor
sleepTime = 0.5
minMM = 9999
maxMM = 0

def signal_handler(signal, frame): # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
        signal.signal(signal.SIGINT, signal_handler)


class sonar():
    def __init__(self):
        topic_name = "/sonar1_tp"
        rospy.init_node('sonar', anonymous=True)
        self.distance_publisher = rospy.Publisher(topic_name, Range, queue_size=5)
        self.r = rospy.Rate(15)
        rospy.loginfo("Publisher with topic %s" % (topic_name))

        # --- Default message
        message = Range()
        # message.ULTRASOUND      = 1
        # message.INFRARED        = 0
        message.radiation_type = 0
        message.min_range = minMM
        message.max_range = maxMM
        self._message = message

    def dist_sendor(self):
        while True:
            mm = maxSonarTTY.measure(serialPort)
            if mm >= maxRange:
                print("no target")
                time.sleep(sleepTime)
                continue
            if mm < minMM:
                minMM = mm
            if mm > maxMM:
                maxMM = mm

            rospy.loginfo("Range [m]: distance = %4.2f  min_distance = %4.2f max_distance = %4.2f" % (mm, minMM, maxMM))
            time.sleep(sleepTime)
        data = Float32()
        data.data=mm
        self._message.range = data
        self.distance_publisher.publish(self._message)

sensor=sonar()
time.sleep(0.5)





sensor.dist_sendor()
sensor.r.sleep()