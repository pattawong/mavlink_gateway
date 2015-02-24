#!/usr/bin/python
# -*- coding: utf-8 -*-

"""
MAVLink Flight Controller Simulator
author: Pattawong Pansodte
blog : mavlink-gateway.blogspot.com
"""

import sys, os, time

from math import radians
#Import MAVLink Lib Path
sys.path.append("mavlink_lib/") 
os.environ["MAVLINK10"] = "1" #Set Mavlink Version 1.0
import mavlinkv10 as mavlink
import mavutil
        

# create a mavlink instance
mav1 = mavutil.mavlink_connection("COM12", 57600)
 
print("Waiting for HEARTBEAT")
#mav1.wait_heartbeat()
 
#print("Heartbeat from APM (system %u component %u)" % (mav1.target_system, mav1.target_component ))
 
while True:
    m = mav1.recv_msg()
    #Check none empty message
    if m != None:
      #looking for msg id 70 (rc_override)
      if m.get_msgId() ==70:
        #Response
        millis = int(round(time.time() * 1000))
        print 'Response ' + str(millis)
        mav1.mav.rc_channels_override_send(1,1,1800,1700,1600,1500,1400,1300,1200,1100)  
      m = None
