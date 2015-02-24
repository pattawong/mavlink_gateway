#!/usr/bin/python
# -*- coding: utf-8 -*-

"""
MAVLink Dumpper
Dump all interested MAVLink message into file
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
mav1 = mavutil.mavlink_connection("COM5", 57600)
 
print("Waiting for HEARTBEAT")
mav1.wait_heartbeat()
 
print("Heartbeat from APM (system %u component %u)" % (mav1.target_system, mav1.target_component ))
mav1.mav.request_data_stream_send(1,1,0,5,1)

f = open('mav.out','w')

#Interested Msg id list
msg_id = [24,27,30,33,35,36,74]
while True:
    m = mav1.recv_msg()
    #Check none empty message
    if m != None:
        if m.get_msgId() in msg_id:
            print m
            f.write(str(m))
            f.write("\n")
        m = None
f.close()
