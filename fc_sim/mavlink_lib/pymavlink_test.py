#!/usr/bin/env python

'''
PyMAVLink Test

'''

import sys, os, time
from math import radians

os.environ["MAVLINK10"] = "1" #Set Mavlink Version 1.0

import mavlinkv10 as mavlink
import mavutil


# create a mavlink instance
mav1 = mavutil.mavlink_connection("COM12", 57600)

print("Waiting for HEARTBEAT")
mav1.wait_heartbeat()

print("Heartbeat from APM (system %u component %u)" % (mav1.target_system, mav1.target_component))
mav1.mav.command_long_send(mav1.target_system, 250,400,0,1,0,0,0,0,0,0)

#mavlink_msg_command_long_send(MAVLINK_COMM_1,1, 1,MAV_CMD_COMPONENT_ARM_DISARM,0,1,0,0,0,0,0,0); //Arm
while True:
    mav1.recv_msg()
    #Do Somethings
    time.sleep(0.01)

