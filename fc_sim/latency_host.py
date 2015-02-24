#!/usr/bin/python
# -*- coding: utf-8 -*-

"""
MAVLink Flight Controller Simulator
author: Pattawong Pansodte
blog : mavlink-gateway.blogspot.com
"""

import sys, os, time
import datetime
import time
from math import radians
#Import MAVLink Lib Path
sys.path.append("mavlink_lib/") 
os.environ["MAVLINK10"] = "1" #Set Mavlink Version 1.0
import mavlinkv10 as mavlink
import mavutil
        
# create a mavlink instance
mav1 = mavutil.mavlink_connection("COM5", 57600)
count = 0
time_max = 0
time_min = sys.maxint
time_avg = 0
while True:
    
    #Do Somethings
    time.sleep(0.1)
    #Start Send
    time_1 = time.clock()
    mav1.mav.rc_channels_override_send(1,1,1100,1200,1300,1400,1500,1600,1700,1800)
    #Wait for Response
    m = mav1.recv_msg()
    if m != None:
      #looking for msg id 70 (rc_override)
      if m.get_msgId() ==70:
        #Response
        time_2 = time.clock()
        time_diff = (time_2-time_1)*1000
        print 'Send :  ' + str(time_1) + ' Response :  ' + str(time_2) + ' Diff : ' +str((time_2-time_1)*1000)
        count = count + 1
        time_max = max(time_max,time_diff)
        time_min = min(time_min,time_diff)
        time_avg = time_avg + time_diff
      m = None
      if count == 50:
          break
print "Testing with 50 RC_OVERRIDE MSG MAX,MIN,AVG"
print str(time_max) + "," + str(time_min) +"," + str(time_avg/50)

