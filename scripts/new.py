import cv2
import rospy
from cv2 import VideoCapture
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image



rospy.init_node('opencv_example', anonymous=True)
rospy.loginfo("Hello ROS!")


bridge = CvBridge()

# Define a function to show the image in an OpenCV Window
def show_image(img):
    vid = cv2.VideoCapture(img)
    while True:
        ret, frame = vid.read()
        cv2.imshow("Image Window", frame)
        if cv2.waitKey(1) & 0xff == ord('q'):
            vid.release()
            cv2.destroyAllWindows()

# Define a callback for the Image message
def image_callback(img_msg):
    # log some info about the image topic
    rospy.loginfo(img_msg.header)

    # Try to convert the ROS Image message to a CV2 Image
    try:
        cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
    except CvBridgeError as e:
        rospy.logerr("CvBridge Error: {0}".format(e))

    # Show the converted image
    show_image(cv_image)

# Initalize a subscriber to the "/camera/rgb/image_raw" topic with the function "image_callback" as a callback
sub_image = rospy.Subscriber("/webcam/image_raw", Image, image_callback)

# Initialize an OpenCV Window named "Image Window"
cv2.namedWindow("Image Window", 1)

# Loop to keep the program from shutting down unless ROS is shut down, or CTRL+C is pressed
while not rospy.is_shutdown():
    rospy.spin()