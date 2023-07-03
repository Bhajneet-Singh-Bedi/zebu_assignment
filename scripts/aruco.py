from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import math

import cv2.aruco as aruco
from mavsdk import System
# import numpy as np
import cv2
import asyncio
# from mavsdk.camera import (CameraError, Mode)
# from mavsdk import System




# Connection IP address configuration
import argparse  
parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='127.0.0.1:14550')
args = parser.parse_args()

# # Connect to the Vehicle
# print ('Connecting to vehicle on: %s' % args.connect)
# drone = System()

vehicle = connect(args.connect, baud=921600)




#921600 is the baudrate that you have set in the mission plannar or qgc

# Function to arm and then takeoff to a user specified altitude
def arm_and_takeoff(aTargetAltitude):

  print ("Basic pre-arm checks")
  # Don't let the user try to arm until autopilot is ready
  while not vehicle.is_armable:
    print (" Waiting for vehicle to initialise...")
    time.sleep(1)
        
  print ("Arming motors")
  # Copter should arm in GUIDED mode
  vehicle.mode    = VehicleMode("GUIDED")
  vehicle.armed   = True

  while not vehicle.armed:
    print (" Waiting for arming...")
    time.sleep(1)

  print ("Taking off!")
  vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

  # Check that vehicle has reached takeoff altitude
  while True:
    print (" Altitude: ", vehicle.location.global_relative_frame.alt) 
    #Break and return from function just below target altitude.        
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


# async def arucoLanding():
#     drone=System()
#     await drone.connect(system_address="127.0.0.1:14550")
#     print("Setting mode to 'PHOTO'")
#     try:
#         await drone.camera.set_mode(Mode.PHOTO)
#     except CameraError as error:
#         print(f"Setting mode failed with error code: {error._result.result}")

#     print("Taking a photo")
#     try:
#         await drone.camera.take_photo()
#     except CameraError as error:
#         print(f"Couldn't take photo: {error._result.result}")


# def camera_callback(self, attr_name, value):
    # print('lala')
    # if attr_name == 'camera':
    #     # Process the received image data
    #     image_data = value
    #     print('Working fine')
    #     # Perform desired operations with the image data



# def arucoLanding():
#   cap = cv2.VideoCapture('gazebo://camera_controller')
#   if not cap.isOpened():
#     print("Failed to open camera")
#     vehicle.mode = VehicleMode("RTL")
#     vehicle.close()
#     exit()
#   while True:
#     # Capture frame-by-frame
#     ret, frame = cap.read()
#     print(ret, frame)

# Initialize the takeoff sequence to 15m
# arm_and_takeoff(2)
print("Take off complete")


print("Doing Spiral now")
#spiral(0,4,-4,5)

# Specify the parameters for the spiral trajectory
v_t = 1  # Tangential velocity (desired velocity of the drone along the spiral trajectory)
n = 0.1  # Frequency or number of revolutions of the spiral

"""
# Start the spiral trajectory
start_time = time.time()
while True:
    current_time = time.time() - start_time
    
    # Calculate the velocities in each direction
    v_x = v_t * math.sin(2 * math.pi * n * current_time)
    v_y = v_t * math.cos(2 * math.pi * n * current_time)
    v_z = -v_t 
    
    # Send velocity commands to the drone
    spiral(v_x, v_y, v_z)
    
    # Break the loop after a certain duration (e.g., 30 seconds)
    if current_time >= 30:
        break
"""
# Detect Aruco Marker for landing.
#asyncio.run(arucoLanding())

# Function to process camera frames from MAVSDK
def process_camera_frames(camera_frame):
    # Perform desired operations with the camera frame
    frame = camera_frame.frame
    # Convert frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
    # Perform ArUco marker detection
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_100)
    parameters = aruco.DetectorParameters_create()
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)
    
    # Check if the ArUco marker with ID 53 is detected
    if ids is not None and 53 in ids:
        print("Landing triggered!")
        vehicle.mode = VehicleMode("LAND")


# Connect to the MAVSDK system using MAVSDK
mavsdk_system = System()

async def connect_to_mavsdk():
    # Connect to the MAVSDK system
    await mavsdk_system.connect(system_address="udp://localhost:14540")


# Subscribe to the camera feed
camera = mavsdk_system.camera.subscribe_camera_image()


async def main():
    # Start the connection to the MAVSDK system
    await connect_to_mavsdk()

    while True:
        # Handle any incoming messages from DroneKit
        vehicle.wait_heartbeat()

        # Process the camera frames from MAVSDK
        async for camera_frame in camera:
            process_camera_frames(camera_frame)


# Create an event loop and run the main coroutine
loop = asyncio.get_event_loop()
try:
    loop.run_until_complete(main())
except KeyboardInterrupt:
    pass

print("Now let's land")
vehicle.mode = VehicleMode("RTL")
# Close vehicle object
vehicle.close()