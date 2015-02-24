#!/usr/bin/python
# -*- coding: utf-8 -*-

"""
MAVLink Flight Controller Simulator
author: Pattawong Pansodte
blog : mavlink-gateway.blogspot.com
"""

import sys, os, time
from PyQt4 import QtGui,QtCore

import thread
from math import radians
import threading
#Import MAVLink Lib Path
sys.path.append("mavlink_lib/") 
os.environ["MAVLINK10"] = "1" #Set Mavlink Version 1.0
import mavlinkv10 as mavlink
import mavutil

#Set Display size
width = 1024
height = 768

#MAV Information
type_list = ["GENERIC","FIXED_WING","QUADROTOR","COAXIAL","HELICOPTER","ANTENNA_TRACKER","GCS","AIRSHIP","FREE_BALLOON","ROCKET","GROUND_ROVER","SURFACE_BOAT","SUBMARINE","HEXAROTOR","OCTOROTOR","TRICOPTER","FLAPPING_WING","ENUM_END"]
autopilot_list = ["GENERIC","PIXHAWK","SLUGS","ARDUPILOTMEGA","OPENPILOT","GENERIC_WAYPOINTS_ONLY","GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY","GENERIC_MISSION_FULL","INVALID","PPZ","UDB","FP","ENUM_END"]
flightmode_list = ["Stabilize","Acrobatic","Alt_Hold","Auto","Guided","Loiter","RTL","Circle","Position","Land","OF_Loiter","Drift","Sport","Flip","AutoTune","Pos_Hold","Num_Modes"]

#Init Position
atittude_x = width*0.05
atittude_y = height*0.12
vfr_hud_x = width*0.05
vfr_hud_y = height*0.5
raw_imu_x = width*0.25
raw_imu_y = height*0.12
global_position_int_x = width*0.4
global_position_int_y = height*0.12
gps_raw_int_x = width*0.25
gps_raw_int_y = height*0.65
rc_channels_raw_x = width*0.6
rc_channels_raw_y = height*0.12
servo_raw_x = width*0.8
servo_raw_y = height*0.12

#QT App
ex = None

class MAVLink_Send(threading.Thread):
    def __init__(self, mavlink_connection,qt):
        threading.Thread.__init__(self)
        self.mav1 = mavlink_connection
        self.enable_run = True
        self._finished = threading.Event()
        self._interval = 0.2
        self.qt = qt
        self.boot_time = int(round(time.time() * 1000))
        
    def setInterval(self, interval):
        """Set the number of seconds we sleep between executing our task"""
        self._interval = interval
    
    def shutdown(self):
        """Stop this thread"""
        self._finished.set()    
    def run(self):
        while self.enable_run:
            if self._finished.isSet(): return
##            self.mav1.mav.command_long_send(self.mav1.target_system, 250,400,0,1,0,0,0,0,0,0)

            mav_type = type_list.index(self.qt.combo_type.currentText())
            mav_autopilot = autopilot_list.index(self.qt.combo_autopilot.currentText())
            mav_basemode = int(self.qt.checkbox_armed.isChecked())*128 #Arm: 128
            mav_custommode = flightmode_list.index(self.qt.combo_flightmode.currentText())
            self.mav1.mav.heartbeat_send(mav_type,mav_autopilot,mav_basemode,mav_custommode,0,10)
            self.send_gps_raw_int_24()
            self.send_raw_imu_27()
            self.send_attitude_30()
            self.send_global_position_int_33()
            self.send_rc_channels_raw_35()
            self.send_servo_output_raw_36()
            self.send_vfr_74()
            print "SEND"
            # sleep for interval or until shutdown
            self._finished.wait(self._interval)
            
    def send_attitude_30(self):
        roll = float(self.qt.qle_roll.text())
        pitch = float(self.qt.qle_pitch.text())
        yaw = float(self.qt.qle_yaw.text())
        roll_speed = float(self.qt.qle_roll_speed.text())
        pitch_speed = float(self.qt.qle_pitch_speed.text())
        yaw_speed = float(self.qt.qle_yaw_speed.text())        
        self.mav1.mav.attitude_send(self.mav1.target_system,roll,pitch,yaw,roll_speed,pitch_speed,yaw_speed)
        
    def send_vfr_74(self):
        air_speed = float(self.qt.qle_airspeed.text())
        ground_speed = float(self.qt.qle_groundspeed.text())
        heading = float(self.qt.qle_heading.text())
        throttle = float(self.qt.qle_throttle.text())
        alt = float(self.qt.qle_alt.text())
        climb = float(self.qt.qle_climb.text())        
        self.mav1.mav.vfr_hud_send(air_speed,ground_speed,heading,throttle,alt,climb)

    def send_gps_raw_int_24(self):
        fix = float(self.qt.qle_raw_fix.text())
        lat = float(self.qt.qle_raw_lat.text())
        lon = float(self.qt.qle_raw_lon.text())
        alt = float(self.qt.qle_raw_alt.text())
        eph = float(self.qt.qle_raw_eph.text())
        epv = float(self.qt.qle_raw_epv.text())
        vel = float(self.qt.qle_raw_vel.text())
        cog = float(self.qt.qle_raw_cog.text())
        sat = float(self.qt.qle_sat_visible.text())
        boot_time =  int(round(time.time() * 1000)) - self.boot_time
        self.mav1.mav.gps_raw_int_send(boot_time, fix,lat,lon,alt,eph,epv,vel,cog,sat)


    def send_raw_imu_27(self):
        x_acc = float(self.qt.qle_xacc.text())
        y_acc = float(self.qt.qle_yacc.text())
        z_acc = float(self.qt.qle_zacc.text())
        x_gyro = float(self.qt.qle_xgyro.text())
        y_gyro = float(self.qt.qle_ygyro.text())
        z_gyro = float(self.qt.qle_zgyro.text())
        x_mag = float(self.qt.qle_xmag.text())
        y_mag = float(self.qt.qle_ymag.text())
        z_mag = float(self.qt.qle_zmag.text())     
        self.mav1.mav.raw_imu_send(int(round(time.time() * 1000)),x_acc,y_acc,z_acc,x_gyro,y_gyro,z_gyro,x_mag,y_mag,z_mag)         

    def send_global_position_int_33(self):
        lat = float(self.qt.qle_lat.text())
        lon = float(self.qt.qle_lon.text())
        gps_alt = float(self.qt.qle_gps_alt.text())
        rel_alt = float(self.qt.qle_relative_alt.text())
        vx = float(self.qt.qle_vx.text())
        vy = float(self.qt.qle_vy.text())
        vz = float(self.qt.qle_vz.text())
        hdg = float(self.qt.qle_hdg.text())
        boot_time =  int(round(time.time() * 1000)) - self.boot_time
        self.mav1.mav.global_position_int_send(boot_time, lat,lon, gps_alt,rel_alt, vx, vy, vz,hdg)

    def send_rc_channels_raw_35(self):
        port = float(self.qt.qle_rc_port.text())
        ch1 = float(self.qt.qle_chan1_raw.text())
        ch2 = float(self.qt.qle_chan2_raw.text())
        ch3 = float(self.qt.qle_chan3_raw.text())
        ch4 = float(self.qt.qle_chan4_raw.text())
        ch5 = float(self.qt.qle_chan5_raw.text())
        ch6 = float(self.qt.qle_chan6_raw.text())
        ch7 = float(self.qt.qle_chan7_raw.text())
        ch8 = float(self.qt.qle_chan8_raw.text())
        rssi = float(self.qt.qle_rssi.text())
        boot_time =  int(round(time.time() * 1000)) - self.boot_time
        self.mav1.mav.rc_channels_raw_send(boot_time,1,ch1,ch2,ch3,ch4,ch5,ch6,ch7,ch8,rssi)

    def send_servo_output_raw_36(self):
        port = float(self.qt.qle_servo_port.text())
        ch1 = float(self.qt.qle_servo1_raw.text())
        ch2 = float(self.qt.qle_servo2_raw.text())
        ch3 = float(self.qt.qle_servo3_raw.text())
        ch4 = float(self.qt.qle_servo4_raw.text())
        ch5 = float(self.qt.qle_servo5_raw.text())
        ch6 = float(self.qt.qle_servo6_raw.text())
        ch7 = float(self.qt.qle_servo7_raw.text())
        ch8 = float(self.qt.qle_servo8_raw.text())
        boot_time =  int(round(time.time() * 1000)) - self.boot_time
        self.mav1.mav.servo_output_raw_send(boot_time,port,ch1,ch2,ch3,ch4,ch5,ch6,ch7,ch8)
        
class FC(QtGui.QWidget):

    def __init__(self):
        
        super(FC, self).__init__()
        self.mav1 = None
        self.initUI()
        self.init_mavlink()

    def init_mavlink(self):
        # create a mavlink instance
        self.mav1 = mavutil.mavlink_connection("COM21", 57600,10)
        print("Waiting for HEARTBEAT")
        #self.mav1.wait_heartbeat()
        #print("Heartbeat from APM (system %u component %u)" % (self.mav1.target_system, self.mav1.target_component))
        self.mav1.target_system = 1
        self.mav1.target_component = 1 
        

    def handleButton(self,n):  
        if n == 0:
           print "START\n"
           self.th = MAVLink_Send(self.mav1,self)
           self.th.start()
           self.th.setInterval(1.0000/int(self.mavlink_frq.text()))
           self.th.enable_run = True
           
        elif n== 1:
           print "STOP\n"
           self.th.enable_run = False
##        elif n == 2:
##           print "n is an even number\n"
##        elif  n== 3 or n == 5 or n == 7:
##           print "n is a prime number\n"


    def initUI(self):
        #Init Color
        self.col = QtGui.QColor(0, 0, 0)       

        #System ID,Autopilot,MAV Type
        #System ID
        self.lb_system_id = QtGui.QLabel('SYSTEM ID', self)
        self.lb_system_id.move(width*0.05, height*0.04)

        self.combo_system_id = QtGui.QComboBox(self)
        for i in range(1,256):
            self.combo_system_id.addItem(str(i))
        self.combo_system_id.move(width*0.11,height*0.036)

        #MAV Type
        self.lb_mav_type = QtGui.QLabel('MAV TYPE', self)
        self.lb_mav_type.move(width*0.17, height*0.04)

        self.combo_type = QtGui.QComboBox(self)
        for mav_type in type_list:
            self.combo_type.addItem(str(mav_type))
        self.combo_type.move(width*0.22,height*0.036)

        #MAV Autopilot
        self.lb_autopilot = QtGui.QLabel('AUTOPILOT', self)
        self.lb_autopilot.move(width*0.36, height*0.04)
        
        self.combo_autopilot = QtGui.QComboBox(self)
        for autopilot in autopilot_list:
            self.combo_autopilot.addItem(str(autopilot))
        self.combo_autopilot.setFixedWidth(width*0.2)
        self.combo_autopilot.move(width*0.42,height*0.036)

        #Arm Checkbox
        self.lb_armed = QtGui.QLabel('Armed', self)
        self.lb_armed.move(width*0.64, height*0.04)
        
        self.checkbox_armed = QtGui.QCheckBox(self)
        self.checkbox_armed.move(width*0.68, height*0.04)

        #Flight Mode
        self.lb_flightmode = QtGui.QLabel('Flight Mode', self)
        self.lb_flightmode.move(width*0.71, height*0.04)
        
        self.combo_flightmode = QtGui.QComboBox(self)
        for flightmode in flightmode_list:
            self.combo_flightmode.addItem(str(flightmode))
        #self.combo_flightmode.setFixedWidth(width*0.2)
        self.combo_flightmode.move(width*0.77,height*0.036)

        
        #Start/Stop Button
        self.start_button = QtGui.QPushButton('Start', self)
        self.start_button.clicked.connect(lambda: self.handleButton(0))
        self.start_button.move(width*0.08,height*0.075)
        
        self.start_button = QtGui.QPushButton('Stop', self)
        self.start_button.clicked.connect(lambda: self.handleButton(1))
        self.start_button.move(width*0.16,height*0.075)

        #MAV Rate
        self.mavlink_frq = QtGui.QLineEdit(self)
        self.mavlink_frq.setText("5")
        self.mavlink_frq.setFixedWidth(width*0.025)
        self.mavlink_frq.move(width*0.05,height*0.075)
        self.mavlink_frq.setValidator(QtGui.QIntValidator(1, 50, self))
        
        #Attitude MSG ID 30 [Pitch,Roll,Yaw,Pitch Speed,Roll Speed,Yaw Speed]
        self.lb_attitude = QtGui.QLabel('ATTITUDE #30', self)
        self.lb_attitude.move(atittude_x, atittude_y)
        
        self.lb_pitch = QtGui.QLabel('PITCH', self)
        self.lb_pitch.move(atittude_x, atittude_y + height*0.05)

        self.lb_roll = QtGui.QLabel('ROLL', self)
        self.lb_roll.move(atittude_x, atittude_y + height*0.1)

        self.lb_yaw = QtGui.QLabel('YAW', self)
        self.lb_yaw.move(atittude_x, atittude_y + height*0.15)

        self.lb_pitch_speed = QtGui.QLabel('PITCH SPEED', self)
        self.lb_pitch_speed.move(atittude_x,atittude_y + height*0.2)

        self.lb_roll_speed = QtGui.QLabel('ROLL SPEED', self)
        self.lb_roll_speed.move(atittude_x, atittude_y + height*0.25)

        self.lb_yaw_speed = QtGui.QLabel('YAW SPEED', self)
        self.lb_yaw_speed.move(atittude_x, atittude_y + height*0.3)

        self.qle_pitch = QtGui.QLineEdit(self)
        self.qle_pitch.setInputMask("0.000")
        self.qle_pitch.setText("0.0")
        self.qle_pitch.setFixedWidth(width*0.045)
##        self.qle_pitch.setValidator(QtGui.QIntValidator(1, 65535, self))
##        self.qle_pitch.setValidator(QtGui.QDoubleValidator(0, 3.140,3, self))
##        qle.textChanged[str].connect(self.onChanged)
        self.qle_pitch.move(atittude_x+width*0.07,atittude_y + height*0.05)

        self.qle_roll = QtGui.QLineEdit(self)
        self.qle_roll.setInputMask("0.000")
        self.qle_roll.setText("0.5")
        self.qle_roll.setFixedWidth(width*0.045)
        self.qle_roll.move(atittude_x+width*0.07,atittude_y + height*0.1)

        self.qle_yaw = QtGui.QLineEdit(self)
        self.qle_yaw.setInputMask("0.000")
        self.qle_yaw.setText("1.0")
        self.qle_yaw.setFixedWidth(width*0.045)
        self.qle_yaw.move(atittude_x+width*0.07,atittude_y + height*0.15)

        self.qle_pitch_speed = QtGui.QLineEdit(self)
        self.qle_pitch_speed.setInputMask("0.000")
        self.qle_pitch_speed.setText("1.5")
        self.qle_pitch_speed.setFixedWidth(width*0.045)
##        self.qle_pitch.setValidator(QtGui.QIntValidator(1, 65535, self))
##        self.qle_pitch.setValidator(QtGui.QDoubleValidator(0, 3.140,3, self))
##        qle.textChanged[str].connect(self.onChanged)
        self.qle_pitch_speed.move(atittude_x+width*0.07,atittude_y + height*0.2)

        self.qle_roll_speed = QtGui.QLineEdit(self)
        self.qle_roll_speed.setInputMask("0.000")
        self.qle_roll_speed.setText("2.0")
        self.qle_roll_speed.setFixedWidth(width*0.045)
        self.qle_roll_speed.move(atittude_x+width*0.07,atittude_y + height*0.25)

        self.qle_yaw_speed = QtGui.QLineEdit(self)
        self.qle_yaw_speed.setInputMask("0.000")
        self.qle_yaw_speed.setText("2.5")
        self.qle_yaw_speed.setFixedWidth(width*0.045)
        self.qle_yaw_speed.move(atittude_x+width*0.07,atittude_y + height*0.3)
        
        #VFR_HUD  MSG ID 74 [Pitch,Roll,Yaw,Pitch Speed,Roll Speed,Yaw Speed]
        self.lb_vfr_hud  = QtGui.QLabel('VFR_HUD  #74', self)
        self.lb_vfr_hud.move(vfr_hud_x,vfr_hud_y)
        
        self.lb_airspeed = QtGui.QLabel('AIRSPEED', self)
        self.lb_airspeed.move(vfr_hud_x, vfr_hud_y + height*0.05)

        self.lb_groundspeed = QtGui.QLabel('GROUND SPEED', self)
        self.lb_groundspeed.move(vfr_hud_x, vfr_hud_y +height*0.1)

        self.lb_heading = QtGui.QLabel('HEADING', self)
        self.lb_heading.move(vfr_hud_x, vfr_hud_y + height*0.15)

        self.lb_throttle = QtGui.QLabel('THROTTLE', self)
        self.lb_throttle.move(vfr_hud_x,vfr_hud_y +height*0.2)

        self.lb_alt = QtGui.QLabel('ALT', self)
        self.lb_alt.move(vfr_hud_x,vfr_hud_y +height*0.25)

        self.lb_climb = QtGui.QLabel('CLIMB', self)
        self.lb_climb.move(vfr_hud_x,vfr_hud_y + height*0.3)

        self.qle_airspeed = QtGui.QLineEdit(self)
        self.qle_airspeed.setInputMask("0.000")
        self.qle_airspeed.setText("1.1")
        self.qle_airspeed.setFixedWidth(width*0.045)
        self.qle_airspeed.move(vfr_hud_x + width*0.08,vfr_hud_y + height*0.05)

        self.qle_groundspeed = QtGui.QLineEdit(self)
        self.qle_groundspeed.setInputMask("0.000")
        self.qle_groundspeed.setText("2.2")
        self.qle_groundspeed.setFixedWidth(width*0.045)
        self.qle_groundspeed.move(vfr_hud_x + width*0.08,vfr_hud_y + height*0.10)

        self.qle_heading = QtGui.QLineEdit(self)
        self.qle_heading.setValidator(QtGui.QIntValidator(0, 360, self))
        self.qle_heading.setText("3.3")
        self.qle_heading.setFixedWidth(width*0.045)
        self.qle_heading.move(vfr_hud_x + width*0.08,vfr_hud_y + height*0.15)

        self.qle_throttle = QtGui.QLineEdit(self)
        self.qle_throttle.setValidator(QtGui.QIntValidator(0, 100, self))
        self.qle_throttle.setText("4.4")
        self.qle_throttle.setFixedWidth(width*0.045)
        self.qle_throttle.move(vfr_hud_x + width*0.08,vfr_hud_y + height*0.20)

        self.qle_alt = QtGui.QLineEdit(self)
        self.qle_alt.setInputMask("0.000")
        self.qle_alt.setText("5.5")
        self.qle_alt.setFixedWidth(width*0.045)
        self.qle_alt.move(vfr_hud_x + width*0.08,vfr_hud_y + height*0.25)

        self.qle_climb = QtGui.QLineEdit(self)
        self.qle_climb.setInputMask("0.000")
        self.qle_climb.setText("6.6")
        self.qle_climb.setFixedWidth(width*0.045)
        self.qle_climb.move(vfr_hud_x + width*0.08,vfr_hud_y + height*0.30)


        #RAW_IMU MSG ID 27 [xacc,yacc,zacc,xgyro ygyro,zgyro xmag,ymag zmag]
        self.lb_attitude = QtGui.QLabel('RAW_IMU #27', self)
        self.lb_attitude.move(raw_imu_x, raw_imu_y)

        self.lb_xacc = QtGui.QLabel('X ACC', self)
        self.lb_xacc.move(raw_imu_x, raw_imu_y + height*0.05)

        self.lb_yacc = QtGui.QLabel('Y ACC', self)
        self.lb_yacc.move(raw_imu_x, raw_imu_y +height*0.1)

        self.lb_zacc = QtGui.QLabel('Z ACC', self)
        self.lb_zacc.move(raw_imu_x, raw_imu_y + height*0.15)

        self.lb_xgyro = QtGui.QLabel('X GYRO', self)
        self.lb_xgyro.move(raw_imu_x,raw_imu_y +height*0.2)

        self.lb_ygyro = QtGui.QLabel('Y GYRO', self)
        self.lb_ygyro.move(raw_imu_x,raw_imu_y +height*0.25)

        self.lb_zgyro = QtGui.QLabel('Z GYRO', self)
        self.lb_zgyro.move(raw_imu_x,raw_imu_y + height*0.3)

        self.lb_xmag = QtGui.QLabel('X MAG', self)
        self.lb_xmag.move(raw_imu_x,raw_imu_y +height*0.35)

        self.lb_ymag = QtGui.QLabel('Y MAG', self)
        self.lb_ymag.move(raw_imu_x,raw_imu_y +height*0.4)

        self.lb_zmag = QtGui.QLabel('Z MAG', self)
        self.lb_zmag.move(raw_imu_x,raw_imu_y + height*0.45)

        self.qle_xacc = QtGui.QLineEdit(self)
        self.qle_xacc.setValidator(QtGui.QIntValidator(0, 65535, self))
        self.qle_xacc.setText("0.1")
        self.qle_xacc.setFixedWidth(width*0.045)
        self.qle_xacc.move(raw_imu_x + width*0.05,raw_imu_y + height*0.05)

        self.qle_yacc = QtGui.QLineEdit(self)
        self.qle_yacc.setValidator(QtGui.QIntValidator(0, 65535, self))
        self.qle_yacc.setText("0.2")
        self.qle_yacc.setFixedWidth(width*0.045)
        self.qle_yacc.move(raw_imu_x + width*0.05,raw_imu_y + height*0.10)

        self.qle_zacc = QtGui.QLineEdit(self)
        self.qle_zacc.setValidator(QtGui.QIntValidator(0, 65535, self))
        self.qle_zacc.setText("0.3")
        self.qle_zacc.setFixedWidth(width*0.045)
        self.qle_zacc.move(raw_imu_x + width*0.05,raw_imu_y + height*0.15)

        self.qle_xgyro = QtGui.QLineEdit(self)
        self.qle_xgyro.setValidator(QtGui.QIntValidator(0, 65535, self))
        self.qle_xgyro.setText("1.1")
        self.qle_xgyro.setFixedWidth(width*0.045)
        self.qle_xgyro.move(raw_imu_x + width*0.05,raw_imu_y + height*0.20)

        self.qle_ygyro = QtGui.QLineEdit(self)
        self.qle_ygyro.setValidator(QtGui.QIntValidator(0, 65535, self))
        self.qle_ygyro.setText("1.2")
        self.qle_ygyro.setFixedWidth(width*0.045)
        self.qle_ygyro.move(raw_imu_x + width*0.05,raw_imu_y + height*0.25)

        self.qle_zgyro = QtGui.QLineEdit(self)
        self.qle_zgyro.setValidator(QtGui.QIntValidator(0, 65535, self))
        self.qle_zgyro.setText("1.3")
        self.qle_zgyro.setFixedWidth(width*0.045)
        self.qle_zgyro.move(raw_imu_x + width*0.05,raw_imu_y + height*0.30)

        self.qle_xmag = QtGui.QLineEdit(self)
        self.qle_xmag.setValidator(QtGui.QIntValidator(0, 65535, self))
        self.qle_xmag.setText("2.1")
        self.qle_xmag.setFixedWidth(width*0.045)
        self.qle_xmag.move(raw_imu_x + width*0.05,raw_imu_y + height*0.35)

        self.qle_ymag = QtGui.QLineEdit(self)
        self.qle_ymag.setValidator(QtGui.QIntValidator(0, 65535, self))
        self.qle_ymag.setText("2.2")
        self.qle_ymag.setFixedWidth(width*0.045)
        self.qle_ymag.move(raw_imu_x + width*0.05,raw_imu_y + height*0.4)

        self.qle_zmag = QtGui.QLineEdit(self)
        self.qle_zmag.setValidator(QtGui.QIntValidator(0, 65535, self))
        self.qle_zmag.setText("2.3")
        self.qle_zmag.setFixedWidth(width*0.045)
        self.qle_zmag.move(raw_imu_x + width*0.05,raw_imu_y + height*0.45)

        #GLOBAL_POSITION_INT MSG ID 33 [lat,lon,alt,relative_alt,vx,vy,vz,hdg] 
        self.lb_global_position_int = QtGui.QLabel('GLOBAL_POSITION_INT #33', self)
        self.lb_global_position_int.move(global_position_int_x, global_position_int_y)

        self.lb_lat = QtGui.QLabel('LAT', self)
        self.lb_lat.move(global_position_int_x, global_position_int_y + height*0.05)

        self.lb_lon = QtGui.QLabel('LON', self)
        self.lb_lon.move(global_position_int_x, global_position_int_y +height*0.1)

        self.lb_gps_alt = QtGui.QLabel('ALT', self)
        self.lb_gps_alt.move(global_position_int_x, global_position_int_y + height*0.15)

        self.lb_relative_alt = QtGui.QLabel('RELATIVE ALT', self)
        self.lb_relative_alt.move(global_position_int_x,global_position_int_y +height*0.2)

        self.lb_vx = QtGui.QLabel('VX', self)
        self.lb_vx.move(global_position_int_x,global_position_int_y +height*0.25)

        self.lb_vy = QtGui.QLabel('VY', self)
        self.lb_vy.move(global_position_int_x,global_position_int_y + height*0.3)

        self.lb_vz = QtGui.QLabel('VZ', self)
        self.lb_vz.move(global_position_int_x,global_position_int_y + height*0.35)

        self.lb_hdg = QtGui.QLabel('HDG', self)
        self.lb_hdg.move(global_position_int_x,global_position_int_y + height*0.4)
        
        self.qle_lat = QtGui.QLineEdit(self)
##        self.qle_lat.setInputMask("0.000")
        self.qle_lat.setText("138243103")
        self.qle_lat.setFixedWidth(width*0.045)
        self.qle_lat.move(global_position_int_x + width*0.08,global_position_int_y + height*0.05)

        self.qle_lon = QtGui.QLineEdit(self)
##        self.qle_lon.setInputMask("0.000")
        self.qle_lon.setText("1005159686")
        self.qle_lon.setFixedWidth(width*0.045)
        self.qle_lon.move(global_position_int_x + width*0.08,global_position_int_y + height*0.10)

        self.qle_gps_alt = QtGui.QLineEdit(self)
##        self.qle_gps_alt.setInputMask("0.000")
        self.qle_gps_alt.setText("15310")
        self.qle_gps_alt.setFixedWidth(width*0.045)
        self.qle_gps_alt.move(global_position_int_x + width*0.08,global_position_int_y + height*0.15)

        self.qle_relative_alt = QtGui.QLineEdit(self)
##        self.qle_relative_alt.setInputMask("0.000")
        self.qle_relative_alt.setText("-1290")
        self.qle_relative_alt.setFixedWidth(width*0.045)
        self.qle_relative_alt.move(global_position_int_x + width*0.08,global_position_int_y + height*0.20)

        self.qle_vx = QtGui.QLineEdit(self)
##        self.qle_vx.setInputMask("0.000")
        self.qle_vx.setText("-19")
        self.qle_vx.setFixedWidth(width*0.045)
        self.qle_vx.move(global_position_int_x + width*0.08,global_position_int_y + height*0.25)

        self.qle_vy = QtGui.QLineEdit(self)
##        self.qle_vy.setInputMask("0.000")
        self.qle_vy.setText("39")
        self.qle_vy.setFixedWidth(width*0.045)
        self.qle_vy.move(global_position_int_x + width*0.08,global_position_int_y + height*0.30)

        self.qle_vz = QtGui.QLineEdit(self)
##        self.qle_vz.setInputMask("0.000")
        self.qle_vz.setText("19")
        self.qle_vz.setFixedWidth(width*0.045)
        self.qle_vz.move(global_position_int_x + width*0.08,global_position_int_y + height*0.35)

        self.qle_hdg = QtGui.QLineEdit(self)
##        self.qle_hdg.setInputMask("0.000")
        self.qle_hdg.setText("10709")
        self.qle_hdg.setFixedWidth(width*0.045)
        self.qle_hdg.move(global_position_int_x + width*0.08,global_position_int_y + height*0.4)

        #GPS_RAW_INT MSG ID 24 [fix_type,lat,lon,alt,eph,epv,vel,cog,sat_visible]
        self.lb_gps_raw_int = QtGui.QLabel('GPS_RAW_INT #24', self)
        self.lb_gps_raw_int.move(gps_raw_int_x, gps_raw_int_y)

        self.lb_raw_fix = QtGui.QLabel('FIX_TYPE', self)
        self.lb_raw_fix.move(gps_raw_int_x, gps_raw_int_y + height*0.05)

        self.lb_raw_lat = QtGui.QLabel('LAT', self)
        self.lb_raw_lat.move(gps_raw_int_x, gps_raw_int_y + height*0.1)

        self.lb_raw_lon = QtGui.QLabel('LON', self)
        self.lb_raw_lon.move(gps_raw_int_x, gps_raw_int_y +height*0.15)

        self.lb_raw_alt = QtGui.QLabel('ALT', self)
        self.lb_raw_alt.move(gps_raw_int_x, gps_raw_int_y + height*0.2)

        self.lb_raw_eph = QtGui.QLabel('EPH', self)
        self.lb_raw_eph.move(gps_raw_int_x,gps_raw_int_y +height*0.25)

        self.lb_raw_epv = QtGui.QLabel('EPV', self)
        self.lb_raw_epv.move(gps_raw_int_x,gps_raw_int_y +height*0.3)

        self.lb_raw_vel = QtGui.QLabel('VEL', self)
        self.lb_raw_vel.move(gps_raw_int_x + width*0.15,gps_raw_int_y + height*0.05)

        self.lb_raw_cog = QtGui.QLabel('COG', self)
        self.lb_raw_cog.move(gps_raw_int_x + width*0.15,gps_raw_int_y + height*0.1)

        self.lb_raw_sat = QtGui.QLabel('SAT_VISIBLE', self)
        self.lb_raw_sat.move(gps_raw_int_x + width*0.15,gps_raw_int_y + height*0.15)

        self.qle_raw_fix = QtGui.QLineEdit(self)
##        self.qle_raw_fix.setInputMask("0.000")
        self.qle_raw_fix.setText("3")
        self.qle_raw_fix.setFixedWidth(width*0.045)
        self.qle_raw_fix.move(gps_raw_int_x + width*0.08,gps_raw_int_y + height*0.05)
        
        self.qle_raw_lat = QtGui.QLineEdit(self)
##        self.qle_raw_lat.setInputMask("0.000")
        self.qle_raw_lat.setText("138243103")
        self.qle_raw_lat.setFixedWidth(width*0.045)
        self.qle_raw_lat.move(gps_raw_int_x + width*0.08,gps_raw_int_y + height*0.1)

        self.qle_raw_lon = QtGui.QLineEdit(self)
##        self.qle_raw_lon.setInputMask("0.000")
        self.qle_raw_lon.setText("1005159686")
        self.qle_raw_lon.setFixedWidth(width*0.045)
        self.qle_raw_lon.move(gps_raw_int_x + width*0.08,gps_raw_int_y + height*0.15)

        self.qle_raw_alt = QtGui.QLineEdit(self)
##        self.qle_raw_alt.setInputMask("0.000")
        self.qle_raw_alt.setText("15310")
        self.qle_raw_alt.setFixedWidth(width*0.045)
        self.qle_raw_alt.move(gps_raw_int_x + width*0.08,gps_raw_int_y + height*0.2)

        self.qle_raw_eph = QtGui.QLineEdit(self)
##        self.qle_raw_eph.setInputMask("0.000")
        self.qle_raw_eph.setText("1290")
        self.qle_raw_eph.setFixedWidth(width*0.045)
        self.qle_raw_eph.move(gps_raw_int_x + width*0.08,gps_raw_int_y + height*0.25)

        self.qle_raw_epv = QtGui.QLineEdit(self)
##        self.qle_raw_epv.setInputMask("0.000")
        self.qle_raw_epv.setText("19")
        self.qle_raw_epv.setFixedWidth(width*0.045)
        self.qle_raw_epv.move(gps_raw_int_x + width*0.08,gps_raw_int_y + height*0.3)

        self.qle_raw_vel = QtGui.QLineEdit(self)
##        self.qle_raw_vel.setInputMask("0.000")
        self.qle_raw_vel.setText("39")
        self.qle_raw_vel.setFixedWidth(width*0.045)
        self.qle_raw_vel.move(gps_raw_int_x + width*0.23,gps_raw_int_y + height*0.05)

        self.qle_raw_cog = QtGui.QLineEdit(self)
##        self.qle_raw_cog.setInputMask("0.000")
        self.qle_raw_cog.setText("19")
        self.qle_raw_cog.setFixedWidth(width*0.045)
        self.qle_raw_cog.move(gps_raw_int_x + width*0.23,gps_raw_int_y + height*0.1)

        self.qle_sat_visible = QtGui.QLineEdit(self)
##        self.qle_sat_visible.setInputMask("0.000")
        self.qle_sat_visible.setText("10")
        self.qle_sat_visible.setFixedWidth(width*0.045)
        self.qle_sat_visible.move(gps_raw_int_x + width*0.23,gps_raw_int_y + height*0.15)


        #RC_CHANNELS_RAW MSG ID 35 [chan1_raw,chan2_raw,chan3_raw,chan4_raw,chan5_raw,chan6_raw,chan7_raw,chan8_raw]
        self.lb_rc_channels_raw = QtGui.QLabel('RC_CHANNELS_RAW #35', self)
        self.lb_rc_channels_raw.move(rc_channels_raw_x, rc_channels_raw_y)


        self.lb_rc_port = QtGui.QLabel('PORT', self)
        self.lb_rc_port.move(rc_channels_raw_x, rc_channels_raw_y + height*0.05)
        
        self.lb_chan1_raw = QtGui.QLabel('CHAN1_RAW', self)
        self.lb_chan1_raw.move(rc_channels_raw_x, rc_channels_raw_y + height*0.1)

        self.lb_chan2_raw = QtGui.QLabel('CHAN2_RAW', self)
        self.lb_chan2_raw.move(rc_channels_raw_x, rc_channels_raw_y +height*0.15)

        self.lb_chan3_raw = QtGui.QLabel('CHAN3_RAW', self)
        self.lb_chan3_raw.move(rc_channels_raw_x, rc_channels_raw_y + height*0.20)

        self.lb_chan4_raw = QtGui.QLabel('CHAN4_RAW', self)
        self.lb_chan4_raw.move(rc_channels_raw_x,rc_channels_raw_y +height*0.25)

        self.lb_chan5_raw = QtGui.QLabel('CHAN5_RAW', self)
        self.lb_chan5_raw.move(rc_channels_raw_x,rc_channels_raw_y +height*0.30)

        self.lb_chan6_raw = QtGui.QLabel('CHAN6_RAW', self)
        self.lb_chan6_raw.move(rc_channels_raw_x,rc_channels_raw_y + height*0.35)

        self.lb_chan7_raw = QtGui.QLabel('CHAN7_RAW', self)
        self.lb_chan7_raw.move(rc_channels_raw_x,rc_channels_raw_y + height*0.40)

        self.lb_chan8_raw = QtGui.QLabel('CHAN8_RAW', self)
        self.lb_chan8_raw.move(rc_channels_raw_x,rc_channels_raw_y + height*0.45)

        self.lb_rssi = QtGui.QLabel('RSSI', self)
        self.lb_rssi.move(rc_channels_raw_x,rc_channels_raw_y + height*0.50)

        self.qle_rc_port = QtGui.QLineEdit(self)
        self.qle_rc_port.setFixedWidth(width*0.045)
        self.qle_rc_port.setValidator(QtGui.QIntValidator(1, 10, self))
        self.qle_rc_port.setText("1")
        self.qle_rc_port.move(rc_channels_raw_x + width*0.08,rc_channels_raw_y + height*0.05)

        self.qle_chan1_raw = QtGui.QLineEdit(self)
        self.qle_chan1_raw.setFixedWidth(width*0.045)
        self.qle_chan1_raw.setValidator(QtGui.QIntValidator(1000, 2000, self))
        self.qle_chan1_raw.setText("1100")
        self.qle_chan1_raw.move(rc_channels_raw_x + width*0.08,rc_channels_raw_y + height*0.10)

        self.qle_chan2_raw = QtGui.QLineEdit(self)
        self.qle_chan2_raw.setFixedWidth(width*0.045)
        self.qle_chan2_raw.setValidator(QtGui.QIntValidator(1000, 2000, self))
        self.qle_chan2_raw.setText("1200")
        self.qle_chan2_raw.move(rc_channels_raw_x + width*0.08,rc_channels_raw_y + height*0.15)

        self.qle_chan3_raw = QtGui.QLineEdit(self)
        self.qle_chan3_raw.setFixedWidth(width*0.045)
        self.qle_chan3_raw.setValidator(QtGui.QIntValidator(1000, 2000, self))
        self.qle_chan3_raw.setText("1300")
        self.qle_chan3_raw.move(rc_channels_raw_x + width*0.08,rc_channels_raw_y + height*0.20)

        self.qle_chan4_raw = QtGui.QLineEdit(self)
        self.qle_chan4_raw.setFixedWidth(width*0.045)
        self.qle_chan4_raw.setValidator(QtGui.QIntValidator(1000, 2000, self))
        self.qle_chan4_raw.setText("1400")
        self.qle_chan4_raw.move(rc_channels_raw_x + width*0.08,rc_channels_raw_y + height*0.25)

        self.qle_chan5_raw = QtGui.QLineEdit(self)
        self.qle_chan5_raw.setFixedWidth(width*0.045)
        self.qle_chan5_raw.setValidator(QtGui.QIntValidator(1000, 2000, self))
        self.qle_chan5_raw.setText("1500")
        self.qle_chan5_raw.move(rc_channels_raw_x + width*0.08,rc_channels_raw_y + height*0.30)

        self.qle_chan6_raw = QtGui.QLineEdit(self)
        self.qle_chan6_raw.setFixedWidth(width*0.045)
        self.qle_chan6_raw.setValidator(QtGui.QIntValidator(1000, 2000, self))
        self.qle_chan6_raw.setText("1600")
        self.qle_chan6_raw.move(rc_channels_raw_x + width*0.08,rc_channels_raw_y + height*0.35)

        self.qle_chan7_raw = QtGui.QLineEdit(self)
        self.qle_chan7_raw.setFixedWidth(width*0.045)
        self.qle_chan7_raw.setValidator(QtGui.QIntValidator(1000, 2000, self))
        self.qle_chan7_raw.setText("1700")
        self.qle_chan7_raw.move(rc_channels_raw_x + width*0.08,rc_channels_raw_y + height*0.4)

        self.qle_chan8_raw = QtGui.QLineEdit(self)
        self.qle_chan8_raw.setFixedWidth(width*0.045)
        self.qle_chan8_raw.setValidator(QtGui.QIntValidator(1000, 2000, self))
        self.qle_chan8_raw.setText("1800")
        self.qle_chan8_raw.move(rc_channels_raw_x + width*0.08,rc_channels_raw_y + height*0.45)

        self.qle_rssi = QtGui.QLineEdit(self)
        self.qle_rssi.setValidator(QtGui.QIntValidator(0, 100, self))
        self.qle_rssi.setText("0")
        self.qle_rssi.setFixedWidth(width*0.045)
        self.qle_rssi.move(rc_channels_raw_x + width*0.08,rc_channels_raw_y + height*0.5)

       
        #SERVO_OUTPUT_RAW MSG ID 36 [port,servo1_raw,servo2_raw,servo3_raw,servo4_raw,servo5_raw,servo6_raw,servo7_raw,servo8_raw]
        self.lb_servo_raw = QtGui.QLabel('SERVO_OUTPUT_RAW #36', self)
        self.lb_servo_raw.move(servo_raw_x, servo_raw_y)
        
        self.lb_servo1_raw = QtGui.QLabel('SERVO1_RAW', self)
        self.lb_servo1_raw.move(servo_raw_x, servo_raw_y + height*0.05)

        self.lb_servo2_raw = QtGui.QLabel('SERVO2_RAW', self)
        self.lb_servo2_raw.move(servo_raw_x, servo_raw_y +height*0.1)

        self.lb_servo3_raw = QtGui.QLabel('SERVO3_RAW', self)
        self.lb_servo3_raw.move(servo_raw_x, servo_raw_y + height*0.15)

        self.lb_servo4_raw = QtGui.QLabel('SERVO4_RAW', self)
        self.lb_servo4_raw.move(servo_raw_x,servo_raw_y +height*0.2)

        self.lb_servo5_raw = QtGui.QLabel('SERVO5_RAW', self)
        self.lb_servo5_raw.move(servo_raw_x,servo_raw_y +height*0.25)

        self.lb_servo6_raw = QtGui.QLabel('SERVO6_RAW', self)
        self.lb_servo6_raw.move(servo_raw_x,servo_raw_y + height*0.3)

        self.lb_servo7_raw = QtGui.QLabel('SERVO7_RAW', self)
        self.lb_servo7_raw.move(servo_raw_x,servo_raw_y + height*0.35)

        self.lb_servo8_raw = QtGui.QLabel('SERVO8_RAW', self)
        self.lb_servo8_raw.move(servo_raw_x,servo_raw_y + height*0.4)

        self.lb_servo_port = QtGui.QLabel('PORT', self)
        self.lb_servo_port.move(servo_raw_x,servo_raw_y + height*0.45)
        
        self.qle_servo1_raw = QtGui.QLineEdit(self)
        self.qle_servo1_raw.setFixedWidth(width*0.045)
        self.qle_servo1_raw.setValidator(QtGui.QIntValidator(1000, 2000, self))
        self.qle_servo1_raw.setText("1800")
        self.qle_servo1_raw.move(servo_raw_x + width*0.08,servo_raw_y + height*0.05)

        self.qle_servo2_raw = QtGui.QLineEdit(self)
        self.qle_servo2_raw.setFixedWidth(width*0.045)
        self.qle_servo2_raw.setValidator(QtGui.QIntValidator(1000, 2000, self))
        self.qle_servo2_raw.setText("1700")
        self.qle_servo2_raw.move(servo_raw_x + width*0.08,servo_raw_y + height*0.10)

        self.qle_servo3_raw = QtGui.QLineEdit(self)
        self.qle_servo3_raw.setFixedWidth(width*0.045)
        self.qle_servo3_raw.setValidator(QtGui.QIntValidator(1000, 2000, self))
        self.qle_servo3_raw.setText("1600")
        self.qle_servo3_raw.move(servo_raw_x + width*0.08,servo_raw_y + height*0.15)

        self.qle_servo4_raw = QtGui.QLineEdit(self)
        self.qle_servo4_raw.setFixedWidth(width*0.045)
        self.qle_servo4_raw.setValidator(QtGui.QIntValidator(1000, 2000, self))
        self.qle_servo4_raw.setText("1500")
        self.qle_servo4_raw.move(servo_raw_x + width*0.08,servo_raw_y + height*0.20)

        self.qle_servo5_raw = QtGui.QLineEdit(self)
        self.qle_servo5_raw.setFixedWidth(width*0.045)
        self.qle_servo5_raw.setValidator(QtGui.QIntValidator(1000, 2000, self))
        self.qle_servo5_raw.setText("1400")
        self.qle_servo5_raw.move(servo_raw_x + width*0.08,servo_raw_y + height*0.25)

        self.qle_servo6_raw = QtGui.QLineEdit(self)
        self.qle_servo6_raw.setFixedWidth(width*0.045)
        self.qle_servo6_raw.setValidator(QtGui.QIntValidator(1000, 2000, self))
        self.qle_servo6_raw.setText("1300")
        self.qle_servo6_raw.move(servo_raw_x + width*0.08,servo_raw_y + height*0.30)

        self.qle_servo7_raw = QtGui.QLineEdit(self)
        self.qle_servo7_raw.setFixedWidth(width*0.045)
        self.qle_servo7_raw.setValidator(QtGui.QIntValidator(1000, 2000, self))
        self.qle_servo7_raw.setText("1200")
        self.qle_servo7_raw.move(servo_raw_x + width*0.08,servo_raw_y + height*0.35)

        self.qle_servo8_raw = QtGui.QLineEdit(self)
        self.qle_servo8_raw.setFixedWidth(width*0.045)
        self.qle_servo8_raw.setValidator(QtGui.QIntValidator(1000, 2000, self))
        self.qle_servo8_raw.setText("1100")
        self.qle_servo8_raw.move(servo_raw_x + width*0.08,servo_raw_y + height*0.4)

        self.qle_servo_port = QtGui.QLineEdit(self)
        self.qle_servo_port.setValidator(QtGui.QIntValidator(0, 100, self))
        self.qle_servo_port.setFixedWidth(width*0.045)
        self.qle_servo_port.setText("1")
        self.qle_servo_port.move(servo_raw_x + width*0.08,servo_raw_y + height*0.45)

        #Setting Font
        self.setStyleSheet('font-size: 8pt; font-family: Tahoma;')
        self.setGeometry(300, 300, width,height)
        self.setWindowTitle('MAVLink Simulator')
        self.show()     

def main():
    app = QtGui.QApplication(sys.argv)
    ex = FC()
    
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()  
