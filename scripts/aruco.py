from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import math
import numpy as np
import cv2 
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo


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



arm_and_takeoff(50)
print("Take off complete")
"""
print("Doing Spiral now")

v_t = 1  
n = 0.1  

# Start the spiral trajectory
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

# async def run():
    
#     print('lala')
#     await drone.connect(system_address="127.0.0.1:14550")

#     print("Waiting for drone to connect...")
#     async for state in drone.core.connection_state():
#         if state.is_connected:
#             print(f"-- Connected to drone!")
#             break
        
#     print_mode_task = asyncio.ensure_future(print_mode(drone))
#     print_status_task = asyncio.ensure_future(print_status(drone))
#     running_tasks = [print_mode_task, print_status_task]

#     print("Setting mode to 'PHOTO'")
#     try:
#         await drone.camera.set_mode(Mode.PHOTO)
#     except CameraError as error:
#         print(f"Setting mode failed with error code: {error._result.result}")

#     await asyncio.sleep(2)

#     print("Taking a photo")
#     try:
#         await drone.camera.take_photo()
#     except CameraError as error:
#         print(f"Couldn't take photo: {error._result.result}")

#     # Shut down the running coroutines (here 'print_mode()' and
#     # 'print_status()')
#     for task in running_tasks:
#         task.cancel()
#         try:
#             await task
#         except asyncio.CancelledError:
#             pass
#     await asyncio.get_event_loop().shutdown_asyncgens()


# async def print_mode(drone):
#     async for mode in drone.camera.mode():
#         print(f"Camera mode: {mode}")


# async def print_status(drone):
#     async for status in drone.camera.status():
#         print(status)


# Get the world object



# asyncio.run(run())

# master = mavutil.mavlink_connection('tcp:127.0.0.1:5760')

# # Enable the camera module
# master.mav.command_long_send(
#     master.target_system, master.target_component,
#     mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE,
#     0, 1, 0, 0, 0, 0, 0, 0
# )
# print('started screaming')
# # Start the video streaming
# master.mav.command_long_send(
#     master.target_system, master.target_component,
#     mavutil.mavlink.MAV_CMD_VIDEO_START_CAPTURE,
#     0, 0, 0, 0, 0, 0, 0, 0
# )

# print('reveiving and displaying')

# # Receive and display the camera feed
# while True:
#     msg = master.recv_match(type='CAMERA_FEEDBACK', blocking=True, timeout=1)
#     if msg:
#         # Process the camera image data (e.g., display using OpenCV)
#         image_data = msg.img_data
#         image = cv2.imdecode(image_data, cv2.IMREAD_COLOR)
#         print(image)
#         cv2.imshow('Camera Feed', image)
#         cv2.waitKey(1)



# Opening camera for further operations.


# def show_image(img):
#     print('check point 1')
#     k = cv2.imshow("Image Window", img)
#     if  cv2.waitKey(3) or k == ord('q'):
#         print('check point 2')
#         cv2.destroyAllWindows()


# def image_callback(img_msg):
#     print('check point 3')
#     rospy.loginfo(img_msg.header)

#     # ROS Image message to a CV2 Image
#     try:
#         cv_image = bridge.imgmsg_to_cv2(img_msg, "passthrough")
#         print('check point 4')
#     except CvBridgeError as e:
#         rospy.logerr("CvBridge Error: {0}".format(e))
#         print('check point 5')

#     # showing the converted image
#     show_image(cv_image)
#     print('check point 6')

# # Subscriber initialization

# sub_image = rospy.Subscriber("/webcam/image_raw", Image, image_callback)
# print('check point 7')
# cv2.namedWindow("Image Window", 1)
# print('check point 8')
# # looping it
# while not rospy.is_shutdown():
#     print('check point 0')
#     rospy.spin()

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
    cv2.imshow("Camera Feed", cv_image)
    spiral(3,3,0)
    print('Checkpoint 6')
    cv2.waitKey(1)

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