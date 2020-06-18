#!/usr/bin/env python

import time
import maxSonarTTY
from maxSonarTTY import measure
from serial import Serial
import sys
import signal
import rospy
from std_msgs.msg import Float32
from sensor_msgs.msg import Range

class sonar():

    def __init__(self):

        rospy.init_node('sonar', anonymous=True)
        topic_name_d = '/sonarTP_D'
        self.distance_publisher_down = rospy.Publisher(topic_name_d,Range, queue_size=5)
        #self.r = rospy.Rate(2)
        topic_name_f = '/sonarTP_F'
        self.distance_publisher_front = rospy.Publisher(topic_name_f, Range, queue_size=5)
        #self.r = rospy.Rate(2)
        topic_name_r = '/sonarTP_R'
        self.distance_publisher_right = rospy.Publisher(topic_name_r, Range, queue_size=5)
        #self.r = rospy.Rate(2)
        topic_name_b = '/sonarTP_B'
        self.distance_publisher_back = rospy.Publisher(topic_name_b, Range, queue_size=5)
        #self.r = rospy.Rate(2)
        topic_name_l = '/sonarTP_L'
        self.distance_publisher_left = rospy.Publisher(topic_name_l, Range, queue_size=5)
        self.r = rospy.Rate(2)
        self.serialPort0 = "/dev/ttyUSB0"
        self.serialPort1 = "/dev/ttyUSB0"
        self.serialPort2 = "/dev/ttyUSB0"
        self.serialPort3 = "/dev/ttyUSB0"
        self.serialPort4 = "/dev/ttyUSB0"

        self.maxRange = 2000  # change for 5m vs 10m sensor
        self.sleepTime = 0.1
        self.minMM = 0.3
        self.maxMM = 3
        self.mm = float
        self.frames = ['/sonarD_link','/sonarF_link','/sonarL_link','/sonarB_link','/sonarR_link']
        r = Range()
        r.header.stamp = rospy.Time.now()
        #r.header.frame_id = "/sonarD_link"
        r.radiation_type = 0
        r.field_of_view = 0.8
        self.min_range = self.minMM
        self.max_range = self.maxRange
        r.min_range = self.min_range
        r.max_range = self.max_range
        self._range = r

    def signal_handler(signal, frame):  # ctrl + c -> exit program
        print('You pressed Ctrl+C!')
        sys.exit(0)
        signal.signal(signal.SIGINT, signal_handler)


    def dist_sendor(self):
        #data = Range()
        while not rospy.is_shutdown():

            #rospy.sleep(1.0)

            for idx in self.frames:
                if(idx == '/sonarD_link'):
                    self.mm = maxSonarTTY.measure(self.serialPort0)
                    if self.mm >= self.maxRange:
                        print("no target")
                        time.sleep(self.sleepTime)
                        continue
                    # ranges = [float('NaN'), 1.0, -float('Inf'), 3.0, float('Inf')]
                    self._range.range = self.mm * 0.001
                    self._range.header.frame_id = idx
                    self.distance_publisher_down.publish(self._range)
                    #time.sleep(0.2)
                    Serial(self.serialPort0, 9600, 8, 'N', 1).flushInput()
                    Serial(self.serialPort0, 9600, 8, 'N', 1).flushOutput()
                    Serial(self.serialPort0, 9600, 8, 'N', 1).close()

                    #Serial(self.serialPort0, 9600).close()
'''
                if (idx == '/sonarF_link'):
                    self.mm = maxSonarTTY.measure(self.serialPort1)
                    if self.mm >= self.maxRange:
                        print("no target")
                        time.sleep(self.sleepTime)
                        continue
                    # ranges = [float('NaN'), 1.0, -float('Inf'), 3.0, float('Inf')]
                    self._range.range = self.mm * 0.001
                    #time.sleep(0.2)
                    self._range.header.frame_id = idx
                    self.distance_publisher_front.publish(self._range)
                    #time.sleep(0.2)
                    Serial(self.serialPort1, 9600, 8, 'N', 1).flushInput()
                    Serial(self.serialPort1, 9600, 8, 'N', 1).flushOutput()
                    Serial(self.serialPort1, 9600, 8, 'N', 1).close()

                    #measure(self.serialPort1).ser.flushInput()
                    #measure(self.serialPort1).ser.flushOutput()
                    #measure(self.serialPort1).ser.close()
                    #measure(self.serialPort)(self.serialPort1, 9600).close()


                if (idx == '/sonarL_link'):
                    self.mm = maxSonarTTY.measure(self.serialPort2)
                    if self.mm >= self.maxRange:
                        print("no target")
                        time.sleep(self.sleepTime)
                        continue
                    # ranges = [float('NaN'), 1.0, -float('Inf'), 3.0, float('Inf')]
                    self._range.range = self.mm * 0.001
                    #time.sleep(0.2)
                    self._range.header.frame_id = idx
                    self.distance_publisher_left.publish(self._range)
                    #time.sleep(0.2)
                    Serial(self.serialPort2, 9600, 8, 'N', 1).flushInput()
                    Serial(self.serialPort2, 9600, 8, 'N', 1).flushOutput()
                    Serial(self.serialPort2, 9600, 8, 'N', 1).close()

                    #measure(self.serialPort2).ser.flushInput()
                    #measure(self.serialPort2).ser.flushOutput()
                    #measure(self.serialPort2).ser.close()
                    #measure(self.serialPort)(self.serialPort2, 9600).close()


                if (idx == '/sonarB_link'):
                    self.mm = maxSonarTTY.measure(self.serialPort3)
                    if self.mm >= self.maxRange:
                        print("no target")
                        time.sleep(self.sleepTime)
                        continue
                    # ranges = [float('NaN'), 1.0, -float('Inf'), 3.0, float('Inf')]
                    self._range.range = self.mm * 0.001
                    #time.sleep(0.2)
                    self._range.header.frame_id = idx
                    self.distance_publisher_back.publish(self._range)
                    #time.sleep(0.2)
                    Serial(self.serialPort3, 9600, 8, 'N', 1).flushInput()
                    Serial(self.serialPort3, 9600, 8, 'N', 1).flushOutput()
                    Serial(self.serialPort3, 9600, 8, 'N', 1).close()

                    #measure(self.serialPort3).ser.flushInput()
                    #measure(self.serialPort3).ser.flushOutput()
                    #measure(self.serialPort3).ser.close()
                    #measure(self.serialPort)(self.serialPort3, 9600).close()


                if (idx == '/sonarR_link'):
                    self.mm = maxSonarTTY.measure(self.serialPort3)
                    if self.mm >= self.maxRange:
                            print("no target")
                            time.sleep(self.sleepTime)
                            continue
                    # ranges = [float('NaN'), 1.0, -float('Inf'), 3.0, float('Inf')]
                    self._range.range = self.mm * 0.001
                    #time.sleep(0.2)
                    self._range.header.frame_id = idx
                    self.distance_publisher_right.publish(self._range)
                    #time.sleep(0.2)
                    Serial(self.serialPort3, 9600, 8, 'N', 1).flushInput()
                    Serial(self.serialPort3, 9600, 8, 'N', 1).flushOutput()
                    Serial(self.serialPort3, 9600, 8, 'N', 1).close()

                    #measure(self.serialPort3).ser.flushInput()
                    #measure(self.serialPort3).ser.flushOutput()
                    #measure(self.serialPort3).ser.close()
                    #Serial(self.serialPort3, 9600).close()


'''
if __name__ == '__main__':
    try:
        sensor = sonar()
        sensor.dist_sendor()

    except rospy.ROSInterruptException: pass
#

#print("distance:",sensor.mm, "  min:", sensor.minMM, "max:", sensor.maxMM)

#sensor.r.sleep()
