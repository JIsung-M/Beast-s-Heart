
#!/usr/bin/env python3
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

    def callback(self, data):
        try:
            # listen image topic
            image = self.bridge.imgmsg_to_cv2(data, 'bgr8')

            # prepare rotate_cmd msg
            msg = Header()
            msg = data.header
            msg.frame_id = '0'  # default: STOP

            # determine background color
            bg_color = self.get_background_color(image)

            # determine the color and assign +1, 0, or -1 for frame_id
            if bg_color is not None:
                if self.is_blue(bg_color):
                    msg.frame_id = '+1'  # CCW
                elif self.is_red(bg_color):
                    msg.frame_id = '-1'  # CW

            # publish color_state
            self.color_pub.publish(msg)

        except CvBridgeError as e:
            print(e)

    def get_background_color(self, image):
        # Convert image to HSV color space
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define lower and upper thresholds for red color in HSV space
        lower_red1 = np.array([0, 50, 50])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 50, 50])
        upper_red2 = np.array([180, 255, 255])

        # Create masks for red color ranges
        mask1 = cv2.inRange(hsv_image, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv_image, lower_red2, upper_red2)

        # Combine the masks to obtain the final red color mask
        red_mask = cv2.bitwise_or(mask1, mask2)

        # Calculate the percentage of red pixels in the mask
        total_pixels = red_mask.size
        red_pixels = np.count_nonzero(red_mask)
        red_percentage = red_pixels / total_pixels

        # Set a threshold to determine if the image has a red background
        threshold = 0.1
        if red_percentage >= threshold:
            return 'red'
        else:
            return None

    def is_blue(self, color):
        # Perform blue color detection logic here
        # ...

    def rospy_shutdown(self, signal, frame):
        rospy.signal_shutdown("shut down")
        sys.exit(0)

if __name__ == '__main__':
    detector = DetermineColor()
    rospy.init_node('CompressedImages1', anonymous=False)
    rospy.spin()
