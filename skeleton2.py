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
        self.colors_count={}
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
            # TODO
            # determine the color and assing +1, 0, or, -1 for frame_id
            # msg.frame_id = '+1' # CCW (Blue background)
            # msg.frame_id = '0'  # STOP
            # msg.frame_id = '-1' # CW (Red background)
            (channel_b,channel_g,channel_r)=cv2.split(image)
            channel_b=channel_b.flatten()
            channel_g=channel_g.flatten()
            channel_r=channel_r.flatten()
            for i in range(len(channel_b)):
              RGB= "(" + str(channel_r[i]+","+str(channel_g[i]) + "," + str(channel_b[i]) + ")"
              if RGB in self.colors_count:
                self.colors_count[RGB]+=1
              else:
                self.colors_count[RGB]=1
    def show_colors(self):
            for keys in sorted(self.colors_count, key=self.colors_count.__getitem__):     
              print(keys, ": ", self.colors_count[keys])
            # publish color_state
            self.color_pub.publish(msg)
          
        except CvBridgeError as e:
            print(e)


    def rospy_shutdown(self, signal, frame):
        rospy.signal_shutdown("shut down")
        sys.exit(0)

if __name__ == '__main__':
    detector = DetermineColor()
    rospy.init_node('CompressedImages1', anonymous=False)
    rospy.spin()
