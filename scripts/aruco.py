from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import math
import numpy as np
import cv2 
import cv2.aruco as aruco
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image
#global argument
ar_id=2
marker_size=4
calib_path="/home/bhajneet/catkin_ws/src/zebu/scripts/"
camera_matrix = np.loadtxt(calib_path+'cameraMatrix_webcam_copy.txt', delimiter=',')
camera_distortion = np.loadtxt(calib_path+'cameraDistortion_webcam_copy.txt', delimiter=',')

R_flip = np.zeros((3,3), dtype=np.float32)
R_flip[0,0]=1.0
R_flip[1,1]=-1.0
R_flip[2,2]=-1.0
# Connection IP address configuration
import argparse  
parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='127.0.0.1:14550')
args = parser.parse_args()

rospy.init_node('opencv_example', anonymous=True)

bridge = CvBridge()
# Connect to the Vehicle
vehicle = connect(args.connect, baud=921600)


def arm_and_takeoff(aTargetAltitude):

  print ("Basic pre-arm checks")

  while not vehicle.is_armable:
    print (" Waiting for vehicle to initialise...")
    time.sleep(1)
        
  print ("Arming motors")

  vehicle.mode    = VehicleMode("GUIDED")
  vehicle.armed   = True

  while not vehicle.armed:
    print (" Waiting for arming...")
    time.sleep(1)

  print ("Taking off!")
  vehicle.simple_takeoff(aTargetAltitude) 


  while True:
    print (" Altitude: ", vehicle.location.global_relative_frame.alt) 

    if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
      print ("Reached target altitude")
      break
    time.sleep(1)

def spiral(velocity_x, velocity_y, velocity_z):
    msg = vehicle.message_factory.set_position_target_global_int_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, # frame
        0b0000111111000111, # type_mask (only speeds enabled)
        0, # lat_int - X Position in WGS84 frame in 1e7 * meters
        0, # lon_int - Y Position in WGS84 frame in 1e7 * meters
        0, # alt - Altitude in meters in AMSL altitude(not WGS84 if absolute or relative)
        # altitude above terrain if GLOBAL_TERRAIN_ALT_INT
        velocity_x, # X velocity in NED frame in m/s
        velocity_y, # Y velocity in NED frame in m/s
        velocity_z, # Z velocity in NED frame in m/s
        0, 0, 0, # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink) 

    # send command to vehicle on 1 Hz cycle
    vehicle.send_mavlink(msg)
    time.sleep(0.5)



arm_and_takeoff(7)
print("Take off complete")

print("Doing Spiral now")

v_t = 1  
n = 0.1  

# Start the spiral trajectory
"""
start_time = time.time()
while True:
    current_time = time.time() - start_time
    

    v_x = v_t * math.sin(2 * math.pi * n * current_time)
    v_y = v_t * math.cos(2 * math.pi * n * current_time)
    v_z = -v_t 
    
    # Send velocity commands to the drone
    spiral(v_x, v_y, v_z)
    
    # Break the loop after a certain duration (e.g., 30 seconds)
    if current_time >= 30:
        break
"""

def image_callback(msg):
    bridge = CvBridge()
    print('Checkpoint 3')
    try:
        # Convert the ROS Image message to OpenCV format
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        print('Checkpoint 4')
    except CvBridgeError as e:
        print('Checkpoint 5')
        print(e)
        return

    # Display the image
    # spiral(3,3,0)
    arucoDict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
    arucoParams = cv2.aruco.DetectorParameters()
    detectors = cv2.aruco.ArucoDetector(arucoDict,arucoParams)
    corners, ids, rejected = detectors.detectMarkers(cv_image)
    # print("This is something: ", markerCorners , markerIds , rejectedCandidates)
    if len(corners) > 0:
      # flatten the ArUco IDs list
      ids = ids.flatten()
      # loop over the detected ArUCo corners
      for (markerCorner, markerID) in zip(corners, ids):
        # extract the marker corners (which are always returned in
        # top-left, top-right, bottom-right, and bottom-left order)
        corners = markerCorner.reshape((4, 2))
        (topLeft, topRight, bottomRight, bottomLeft) = corners
        # convert each of the (x, y)-coordinate pairs to integers
        topRight = (int(topRight[0]), int(topRight[1]))
        bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
        bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
        topLeft = (int(topLeft[0]), int(topLeft[1]))
        # cv2.line(cv_image, topLeft, topRight, (0, 255, 0), 2)
        # cv2.line(cv_image, topRight, bottomRight, (0, 255, 0), 2)
        # cv2.line(cv_image, bottomRight, bottomLeft, (0, 255, 0), 2)
        # cv2.line(cv_image, bottomLeft, topLeft, (0, 255, 0), 2)
        # compute and draw the center (x, y)-coordinates of the ArUco
        # marker
        cX = int((topLeft[0] + bottomRight[0]) / 2.0)
        cY = int((topLeft[1] + bottomRight[1]) / 2.0)
        # cv2.circle(cv_image, (cX, cY), 4, (0, 0, 255), -1)
        marker_points = np.array([
        [-marker_size/2, -marker_size/2, 0],
        [-marker_size/2, marker_size/2, 0],
        [marker_size/2, marker_size/2, 0],
        [marker_size/2, -marker_size/2, 0]
    ], dtype=np.float32)
        image_points = np.squeeze(corners)
        # draw the ArUco marker ID on the image
        cv2.putText(cv_image, str(markerID),
          (topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX,
          0.5, (0, 255, 0), 2)
        #print("[INFO] ArUco marker ID: {}".format(markerID))
        _, rvec, tvec = cv2.solvePnP(marker_points, image_points, camera_matrix, camera_distortion)
        #print(rvec, tvec)
        #aruco.drawDetectedMarkers(cv_image, corners)
        cv2.drawFrameAxes(cv_image, camera_matrix, camera_distortion, rvec, tvec, 10)
        x=tvec[0]
        y=tvec[1]
        z=tvec[2]
        target_location = LocationGlobalRelative(x, y, -z)
        vehicle.simple_goto(target_location)
        R_ct=np.matrix(cv2.Rodrigues(rvec)[0])
        R_tc=R_ct.T
        roll_marker, pitch_marker, yaw_marker = rotationMatrixToEulerAngles(R_flip*R_tc)
        #pos_camera = -R_tc*np.matrix(tvec).T
        #print("Marker X = %.1f  Y = %.1f  Z = %.1f  - fps = %.0f",tvec[0], tvec[1], tvec[2])
        
        x_cm, y_cm= camera_to_uav(x, y)
        z_cm=vehicle.location.global_relative_frame.alt*100.0
        angle_x, angle_y    = marker_position_to_angle(x_cm, y_cm, z_cm)
        send_land_message_v1(x_rad=angle_x, y_rad=angle_y, dist_m=z_cm*0.01, time_usec=time.time()*1e6)
        cv2.imshow("Camera Feed", cv_image)
        #print('Checkpoint 6')
        k = cv2.waitKey(50)
        if k==ord('q'):
          cv2.destroyAllWindows()

def rotationMatrixToEulerAngles(R):
    # Calculates rotation matrix to euler angles
    # The result is the same as MATLAB except the order
    # of the euler angles ( x and z are swapped ).
    
        def isRotationMatrix(R):
            Rt = np.transpose(R)
            shouldBeIdentity = np.dot(Rt, R)
            I = np.identity(3, dtype=R.dtype)
            n = np.linalg.norm(I - shouldBeIdentity)
            return n < 1e-6        
        assert (isRotationMatrix(R))

        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

        singular = sy < 1e-6

        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0

        return np.array([x, y, z])
def camera_to_uav(x_cam, y_cam):
    x_uav = x_cam
    y_uav = y_cam
    return(x_uav, y_uav)


def marker_position_to_angle(x, y, z):
    
    angle_x = math.atan2(x,z)
    angle_y = math.atan2(y,z)
    
    return (angle_x, angle_y)

# def send_distance_message( dist):
#     msg = vehicle.message_factory.distance_sensor_encode(
#         0,          # time since system boot, not used
#         1,          # min distance cm
#         10000,      # max distance cm
#         dist,       # current distance, must be int
#         0,          # type = laser?
#         0,          # onboard id, not used
#         mavutil.mavlink.MAV_SENSOR_ROTATION_PITCH_270, # must be set to MAV_SENSOR_ROTATION_PITCH_270 for mavlink rangefinder, represents downward facing
#         0           # covariance, not used
#     )
#     vehicle.send_mavlink(msg)

# def send_land_message_v2(x_rad=0, y_rad=0, dist_m=0, x_m=0,y_m=0,z_m=0, time_usec=0, target_num=0):
#     msg = vehicle.message_factory.landing_target_encode(
#         time_usec,          # time target data was processed, as close to sensor capture as possible
#         target_num,          # target num, not used
#         mavutil.mavlink.MAV_FRAME_BODY_NED, # frame, not used
#         x_rad,          # X-axis angular offset, in radians
#         y_rad,          # Y-axis angular offset, in radians
#         dist_m,          # distance, in meters
#         0,          # Target x-axis size, in radians
#         0,          # Target y-axis size, in radians
#         x_m,          # x	float	X Position of the landing target on MAV_FRAME
#         y_m,          # y	float	Y Position of the landing target on MAV_FRAME
#         z_m,          # z	float	Z Position of the landing target on MAV_FRAME
#         (1,0,0,0),  # q	float[4]	Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
#         2,          # type of landing target: 2 = Fiducial marker
#         1,          # position_valid boolean
#     )
#     print (msg)
#     time.sleep(0.5)

def send_land_message_v1(x_rad=0, y_rad=0, dist_m=0, time_usec=0, target_num=0):
    msg = vehicle.message_factory.landing_target_encode(
        time_usec,          # time target data was processed, as close to sensor capture as possible
        target_num,          # target num, not used
        mavutil.mavlink.MAV_FRAME_BODY_NED, # frame, not used
        x_rad,          # X-axis angular offset, in radians
        y_rad,          # Y-axis angular offset, in radians
        dist_m,          # distance, in meters
        0,          # Target x-axis size, in radians
        0,          # Target y-axis size, in radians
    )
    print (msg)
    vehicle.send_mavlink(msg)
    time.sleep(0.5)


# Initialize the ROS node
# rospy.init_node("camera_subscriber")

# Subscribe to the camera topic 
rospy.Subscriber("/webcam/image_raw", Image, image_callback)
print('Checkpoint 1')
# Spin and wait for incoming messages
rospy.spin()
print('Checkpoint 2')
print("Now let's land")
vehicle.mode = VehicleMode("RTL")
# Close vehicle object
vehicle.close()