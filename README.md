MAVLink Gateway
arduino application to translate MAVLink Protocol Message into custom protocol and sent to display in Portable Ground Station

Standard MAVLink Message : pixhawk.ethz.ch/mavlink/

Code at Github : https://github.com/pattawong/mavlink_gateway
Web Blog : http://mavlink-gateway.blogspot.com/


Setup Guide
1.Copy all folder inside libraries directory into Arduino libraries directory ex. C:\Program Files (x86)\Arduino\libraries

Tested on Windows 7 x64,Arduino IDE 1.0.5-r2,Arduino Mega

Code consist of 4 main part
1.read_mavlink function : Grab and Process MAVLink Message
2.read_ground function : Grab and Process Portable Ground Station Message
3.HeartbeatTimer function : Run every x ms to check telemetry status
4.send_# function : Send translated MAVLink Message to Portable Ground Station

Include Flight Controller Simulator
-Using PyMAVLink for Python Biniding
-Using PyQT for GUI
-Simulate basic MAVLink message ex.HEARTBEAT,ATTITUDE..