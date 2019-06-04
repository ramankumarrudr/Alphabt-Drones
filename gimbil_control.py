import time
from dronekit import connect, VehicleMode, LocationGlobalRelative, Command, LocationGlobal
from pymavlink import mavutil
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('--connect', default='/dev/ttyUSB0')
args = parser.parse_args()

# Connect to the Vehicle
print ('Connecting to vehicle on: %s' % args.connect)
vehicle = connect(args.connect, baud=57600, wait_ready=True)

vehicle.gimbal.rotate(0,0,0) #pitch ,roll,yaw
time.sleep(5)
vehicle.gimbal.rotate(-90,0,0) # seting the gimbil to down
print("setting the gimbal down")
time.sleep(5)
vehicle.gimbal.rotate(90,0,0) #gimbal facing top
print("setting the gimbal up")
time.sleep(5)
vehicle.gimbal.rotate(0,0,90) #90 yaw to west
print("setting the gimbal to west")


