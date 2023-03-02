from asyncio import events
import rospy
from gs_flight import FlightController, CallbackEvent
from gs_board import BoardManager
from gs_navigation import NavigationManager
import math
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
bridge = CvBridge()
rospy.init_node("record") 

path = "./videos/try16_img--{}.jpg"
print(1)

bridge = CvBridge()
print(2)
# i = 0
# def image_callback(data):
#     global i
#     cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image
#     cv2.imwrite(path.format(i), cv_image)
#     i+=1

# image_sub = rospy.Subscriber('/pioneer_max_camera/image_raw/', Image, image_callback)

i = 0
while True:
    rospy.sleep(0.1)
    data = rospy.wait_for_message("/pioneer_max_camera/image_raw/", Image)
    print(5)
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')
    print(cv_image.shape)
    cv2.imwrite(path.format(i), cv_image)
    i+=1
    