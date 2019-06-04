import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil

#- Importing Tkinter: sudo apt-get install python-tk
# import Tinker as tk
import argparse

import sys,tty,termios
class _Getch:
    def __call__(self):
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(sys.stdin.fileno())
                ch = sys.stdin.read(3)
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            return ch

parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='/dev/ttyUSB0') # if the vehicle is connected to pixhawk use /dev/serial0 and to control through telementry use ttyUSB0(check for the tty port that usb is connected to)
args = parser.parse_args()

# Connect to the Vehicle
print ('Connecting to vehicle on: %s' % args.connect)
vehicle = connect(args.connect, baud=57600, wait_ready=True)

#-- Setup the commanded flying speed
gnd_speed = 5 # [m/s]


while not vehicle.is_armable:
  print("waiting to be armable")
  time.sleep(1)

print("Arming motors")
vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True

while not vehicle.armed: time.sleep(1)
    print("Taking Off")
    vehicle.simple_takeoff(altitude)

while True:
  v_alt = vehicle.location.global_relative_frame.alt
  print(">> Altitude = %.1f m"%v_alt)
  if v_alt >= altitude - 1.0:
    print("Target altitude reached")
    break
  time.sleep(1)




#-- Define arm and takeoff
def arm_and_takeoff(altitude):

   while not vehicle.is_armable:
      print("waiting to be armable")
      time.sleep(1)

   print("Arming motors")
   vehicle.mode = VehicleMode("GUIDED")
   vehicle.armed = True

   while not vehicle.armed: time.sleep(1)

   print("Taking Off")
   vehicle.simple_takeoff(altitude)

   while True:
      v_alt = vehicle.location.global_relative_frame.alt
      print(">> Altitude = %.1f m"%v_alt)
      if v_alt >= altitude - 1.0:
          print("Target altitude reached")
          break
      time.sleep(1)
      
 #-- Define the function for sending mavlink velocity command in body frame
def set_velocity_body(vehicle, vx, vy, vz):
    """ Remember: vz is positive downward!!!
    http://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html
    
    Bitmask to indicate which dimensions should be ignored by the vehicle 
    (a value of 0b0000000000000000 or 0b0000001000000000 indicates that 
    none of the setpoint dimensions should be ignored). Mapping: 
    bit 1: x,  bit 2: y,  bit 3: z, 
    bit 4: vx, bit 5: vy, bit 6: vz, 
    bit 7: ax, bit 8: ay, bit 9:
    
    
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_BODY_NED,
            0b0000111111000111, #-- BITMASK -> Consider only the velocities
            0, 0, 0,        #-- POSITION
            vx, vy, vz,     #-- VELOCITY
            0, 0, 0,        #-- ACCELERATIONS
            0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()
    
    
def get():
  arr_input = True
  while(arr_input == True):
        inkey = _Getch()
        while(1):
                k=inkey()
                if k!='':break        
        if k=='\x1b[A':
                set_velocity_body(vehicle, gnd_speed, 0, 0) #up
        elif k=='\x1b[B':
                set_velocity_body(vehicle,-gnd_speed, 0, 0) #down
        elif k=='\x1b[C':
                set_velocity_body(vehicle, 0, gnd_speed, 0) #right
        elif k=='\x1b[D':
                set_velocity_body(vehicle, 0, -gnd_speed, 0) #left
        else:
                arr_input = False
                print("Entered Unexpexted Key Exiting")

    
#---- MAIN FUNCTION
#- Takeoff
arm_and_takeoff(10)
#- Read the keyboard with tkinter
print(">> Control the drone with the arrow keys. Press r for RTL mode")
get() # calls the arrow-keys for controlling the drone
print('Enter r to return to rtl mode ')
x == raw_input()
if(x=='r'):
	print("r pressed >> Set the vehicle to RTL (Return to launch)")
	vehicle.mode = VehicleMode("RTL")
        time.sleep
vehicle.mode = VehicleMode("LAND") #setting Vehicle in Land mode
sys.exit()
# vehicle.airspeed = 3

#point1 = LocationGlobalRelative("","",20) #set gps-waypoints by setting lat-long co-ordinates in the empty string
#vehicle.simple_goto(point1,groundspeed=10) 

