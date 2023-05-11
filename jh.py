# !/usr/bin/env python3
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Header

class DetermineColor:
    def __init__(self):
        self.image_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.callback)
        self.color_pub = rospy.Publisher('/rotate_cmd', Header, queue_size=10)
        self.bridge = CvBridge()
        self.count=0

    def callback(self, data):
        try:
            # listen image topic
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

            # prepare rotate_cmd msg
            # DO NOT DELETE THE BELOW THREE LINES!
            msg = Header()
            msg = data.header
            msg.frame_id = '0'  # default: STOP

            # determine background color
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)  # rgb->hsv로 변경
            hist, _ = np.histogram(hsv[:,:,0], bins=180, range=[0, 180])  #hue(색상)에 대한 히스토그램
            common_color = np.argmax(hist)  #빈도수가 가장 많은 색상 추출
            # TODO
            # determine the color and assing +1, 0, or, -1 for frame_id
            if 165<= common_color <=180 or 0<= common_color <=15:  # CW (Red background)0
                msg.frame_id = '-1' # CW
            elif 90 <=common_color <=135: # CCW (Blue background)
                msg.frame_id = '+1' # CCW
            else:
                msg.frame_id = '0' # STOP
            
            # publish color_state
            self.color_pub.publish(msg)

        except CvBridgeError as e:
            print(e)


    def rospy_shutdown(self, signal, frame):
        rospy.signal_shutdown("shut down")
        sys.exit(0)

if __name__ == '__main__':
    rospy.init_node('CompressedImages1', anonymous=False)
    detector = DetermineColor()
    rospy.spin()
