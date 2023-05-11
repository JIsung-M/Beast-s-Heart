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
        # Convert image to grayscale for easier color analysis
        gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Calculate the histogram of grayscale intensities
        hist = cv2.calcHist([gray_image], [0], None, [256], [0, 256])

        # Find the index of the most dominant color
        dominant_color_index = np.argmax(hist)

        # Map the dominant color index to the actual color
        colors = {
            0: 'black',
            255: 'white',
            # Add more color mappings as needed
        }
        dominant_color = colors.get(dominant_color_index)

        return dominant_color

    def is_blue(self, color):
        # Define the range of blue color in HSV space
        lower_blue = np.array([90, 50, 50])
        upper_blue = np.array([130, 255, 255])

        # Convert the color to the HSV color space
        hsv_color = cv2.cvtColor(np.uint8([[color]]), cv2.COLOR_BGR2HSV)

        # Check if the color falls within the blue range
        return cv2.inRange(hsv_color, lower_blue, upper_blue)
    def is_red(self, color):
        # Define the range of red color in HSV space
        lower_red = np.array([0, 50, 50])
        upper_red = np.array([10, 255, 255])

        # Convert the color to the HSV color space
        hsv_color = cv2.cvtColor(np.uint8([[color]]), cv2.COLOR_BGR2HSV)

        # Check if the color falls within the red range
        return cv2.inRange(hsv_color, lower_red, upper_red)
