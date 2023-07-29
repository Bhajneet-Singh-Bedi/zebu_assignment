from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time
import math
import argparse  
parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='127.0.0.1:14550')
args = parser.parse_args()

# Connect to the Vehicle
print ('Connecting to vehicle on: %s' % args.connect)
vehicle = connect(args.connect, baud=921600)
print(vehicle)
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

# Initialize the takeoff sequence to 15m
arm_and_takeoff(5)

print("Take off complete")

print("Doing Spiral now")
#spiral(0,4,-4,5)

# Specify the parameters for the spiral trajectory
v_t = 1  # Tangential velocity (desired velocity of the drone along the spiral trajectory)
n = 0.1  # Frequency or number of revolutions of the spiral

# Start the spiral trajectory
start_time = time.time()
while True:
    current_time = time.time() - start_time
    
    # Calculate the velocities in each direction
    v_x = v_t 
    v_y = v_t * math.sin(2 * math.pi * n * current_time)
    v_z = -v_t * math.cos(2 * math.pi * n * current_time)
    
    # Send velocity commands to the drone
    spiral(v_x, v_y, v_z)
    
    # Break the loop after a certain duration (e.g., 30 seconds)
    if current_time >= 30:
        break

print("Now let's land")
vehicle.mode = VehicleMode("RTL")
# Close vehicle object
vehicle.close()