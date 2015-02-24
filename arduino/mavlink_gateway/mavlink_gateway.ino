#define MAVLINK10     // Are we listening MAVLink 1.0 or 0.9   (0.9 is obsolete now)
#define HEARTBEAT     // HeartBeat signal

//AVR Includes
#include <FastSerial.h>
#include <AP_Common.h>
#include <AP_Math.h>
#include <math.h>
#include <inttypes.h>
#include <avr/pgmspace.h>

//Arduino Include
#include "Arduino.h"
#include <GCS_MAVLink.h>
#include "SimpleTimer.h"

// Configurations
#include "custom_protocol.h"

//uIP
#include <UIPEthernet.h>

EthernetUDP udp;

#define VERSION "v3.1"

//define baudrate
#define GROUNDSTATION_SPEED 57600
#define TELEMETRY_SPEED  57600  

#define MAVLINK_COMM_NUM_BUFFERS 1
#define MAVLINK_USE_CONVENIENCE_FUNCTIONS

// this code was moved from libraries/GCS_MAVLink to allow compile
// time selection of MAVLink 1.0
BetterStream	*mavlink_comm_0_port;
BetterStream	*mavlink_comm_1_port;

mavlink_system_t mavlink_system;

#include "Mavlink_compat.h"

//#ifdef MAVLINK10
#include "../GCS_MAVLink/include/mavlink/v1.0/mavlink_types.h"
#include "../GCS_MAVLink/include/mavlink/v1.0/ardupilotmega/mavlink.h"

static int packet_drops = 0;
static int parse_error = 0;

unsigned long time_1hz;
unsigned long time_5hz;
SimpleTimer timer;

// Objects and Serial definitions
FastSerialPort0(Serial);
FastSerialPort1(Serial1);
FastSerialPort2(Serial2);

//Start Byte
byte stx = 0xFE;
//Enable/Disable Stream command from GCS
boolean stream_enable = true;
//Enable/Disable Stream to PC
boolean pc_enable = false;

typedef union {
  float floatingPoint;
  byte binary[4];
} 
binaryFloat;

void setup() 
{
  //Initialize serial
  Serial.begin(GROUNDSTATION_SPEED); //Debug USB
  Serial1.begin(TELEMETRY_SPEED); //APM TX1,RX1
  Serial2.begin(GROUNDSTATION_SPEED); //Portable Ground TX2,RX2

  //Setup Ethernet
  uint8_t mac[6] = {  
    0x00,0x01,0x02,0x03,0x04,0x05               };

  Ethernet.begin(mac,IPAddress(192,168,1,6));

  //Setup MAVLink Port
  //mavlink_comm_0_port = &Serial;
  mavlink_comm_1_port = &Serial1;

  //Setup MAVLink Timer for telemetry status checking every 1000ms 
  timer.setInterval(1000, HeartbeatTimer);

  //Initialize System ID,Component ID
  mavlink_system.sysid = 255; // System ID, 1-255
  mavlink_system.compid = 0; // Component/Subsystem ID, 1-255

  //Request MAVLink Data Stream
  request_mavlink_rates();

}

void loop() 
{
  timer.run();
  /*
  //Testing Msg Sending
   if(millis()-time>=2000)
   {
   //mavlink_msg_heartbeat_send(MAVLINK_COMM_1, MAV_TYPE_GCS, MAV_AUTOPILOT_GENERIC,10,0,MAV_STATE_STANDBY);
   //request_mavlink_rates();   //Request Data Stream
   //mavlink_msg_rc_channels_override_send(MAVLINK_COMM_1,apm_mav_system, apm_mav_component , 1100 , 1200 , 1300 , 1400 , 1500 , 1600 , 1700 , 1800 ); //Sent RC Override
   //mavlink_msg_command_long_send(MAVLINK_COMM_1,apm_mav_system, 0,MAV_CMD_DO_SET_SERVO,0,1,1500,0,0,0,0,0);  //Sent Servo Override
   //mavlink_msg_param_request_list_send(MAVLINK_COMM_1,apm_mav_system, apm_mav_component); //Request Parameter list       
   //mavlink_msg_command_long_send(MAVLINK_COMM_1,1, 1,MAV_CMD_COMPONENT_ARM_DISARM,0,1,0,0,0,0,0,0); //Arm
   //mavlink_msg_command_long_send(MAVLINK_COMM_1,apm_mav_system, apm_mav_component,MAV_CMD_DO_SET_MODE,0,MAV_MODE_STABILIZE_DISARMED,0,0,0,0,0,0); 
   //mavlink_msg_command_long_send(MAVLINK_COMM_1,apm_mav_system, apm_mav_component,176,2,80,0,0,0,0,0,0); 
   time = millis();
   }*/


  //1 Hz loop
  if(millis()-time_1hz>=1000)
  {
    // mavlink_msg_rc_channels_override_send(MAVLINK_COMM_1,gs_mav_system, gs_mav_component , 1100 , 1200 , 1300 , 1400 , 1500 , 1600 , 1700 , 1800 ); //Sent RC Override

    //Serial.println("override");
    if(pc_enable==true)
    {
      sent_msg_11_udp();
      sent_msg_12_udp();
    }
    if(stream_enable==true)
    {
      sent_msg_11();
      sent_msg_12();
    }

    time_1hz = millis();
  }

  //5 Hz loop
  if(millis()-time_5hz>=200)
  { 

    if(pc_enable==true)
    {
      sent_msg_13_udp();
      sent_msg_14_udp();
      sent_msg_15_udp();
    }
    if(stream_enable==true)
    {
      sent_msg_13();
      sent_msg_14();
      sent_msg_15();
    }
    time_5hz = millis();
  }  

  //Read and Process MAVLink Message
  read_mavlink();
  //Read and Process Portable Ground Station Message
  read_ground();
}


// Function that is called every 120ms
void HeartbeatTimer()
{
  //Missing Heart Beat
  if(gs_beat == false)
  {
    gs_beat_count++;
  }
  else
  {
    //Reset counter 
    gs_beat = false;
    gs_beat_count = 0;
  }

  //Missing more than 3 beat (>3s)
  if(gs_beat_count>=3)
  {
    gs_mavlink = false;
    Serial.println("MAVLink Down!!");
  }
  else
  {
    gs_mavlink = true;  
  }
}

void sent_float(float value)
{
  binaryFloat temp;
  temp.floatingPoint = value;
  Serial2.write(temp.binary[3]);
  Serial2.write(temp.binary[2]);
  Serial2.write(temp.binary[1]);
  Serial2.write(temp.binary[0]);
}

void sent_int_8(int value)
{
  Serial2.write(value);
}

void sent_int_16(int16_t value)
{
  /*
   Serial.print(String(value,BIN));
   Serial.println("");
   Serial.print(String((value & 0xFF00)>>8,BIN));
   Serial.print(":");
   Serial.println(String(value & 0x00FF,BIN));
   */
  Serial2.write((value & 0xFF00)>>8);
  Serial2.write(value & 0x00FF);
}
void sent_int_32(int32_t value)
{
  /*
   Serial.print(String(value,BIN));
   Serial.println("");
   Serial.print(String((value & 0xFF000000)>>24,BIN));
   Serial.print(":");
   Serial.print(String((value & 0x00FF0000)>>16,BIN));
   Serial.print(":");
   Serial.print(String((value & 0x0000FF00)>>8,BIN));
   Serial.print(":");
   Serial.println(String(value & 0x000000FF,BIN));
   */
  Serial2.write((value & 0xFF000000)>>24);
  Serial2.write((value & 0x00FF0000)>>16);
  Serial2.write((value & 0x0000FF00)>>8);
  Serial2.write(value & 0x000000FF);
}

//Sent 10 char String padding with 0x00
void sent_string(String value)
{
  if(value.length()<=10)
  {
    Serial2.print(value);
  }
  for(int i=value.length();i<10;i++)
  {
    Serial2.print((char)0x00);
  }
}


void sent_msg_11()
{
  sent_int_8(stx); //Start Byte
  sent_int_8(11); //Msg id 11
  sent_string(gs_flight_mode); //Flight Mode
  sent_int_8(gs_mav_type); //Mav Type 2:QuadRotor
  sent_int_8(gs_arm); //Arm/Disarm 0:disarm , 1;arm

    //Serial.println("Sent 11");
}

void sent_msg_12()
{
  sent_int_8(stx); //Start Byte
  sent_int_8(12); //Msg id 12
  sent_int_8(gs_fix_type); //GPS Fix Type
  sent_int_16(gs_hdop); //GPS Hdop
  sent_int_8(gs_num_sat); //GPS Visible Sat.
  sent_int_16(gs_volt_batt); //Battery Voltage

  //Serial.println("Sent 12");
}

void sent_msg_13()
{
  sent_int_8(stx); //Start Byte
  sent_int_8(13); //Msg id 13
  sent_float(gs_roll);//Roll
  sent_float(gs_pitch);//Pitch
  sent_float(gs_yaw);//Yaw

    //  Serial.println("Sent 13");
}

void sent_msg_14()
{
  sent_int_8(stx); //Start Byte
  sent_int_8(14); //Msg id 14
  sent_float(gs_alt);//Roll
  sent_int_16(gs_heading);//Pitch
  sent_float(gs_ground_speed);//Yaw

  //Serial.println("Sent 14");
}

void sent_msg_15()
{
  sent_int_8(stx); //Start Byte
  sent_int_8(15); //Msg id 15
  for(int i=0;i<=7;i++)
  {  
    sent_int_16(gs_ch_raw[i]);//Ch_Raw[i]
  } 
  //Serial.println("Sent 15");
}

//Request MAVLink Data Stream
void request_mavlink_rates()
{
  Serial.println("Requesting rates");

  const int  maxStreams = 7;
  const uint8_t MAVStreams[maxStreams] = {
    MAV_DATA_STREAM_RAW_SENSORS,
    MAV_DATA_STREAM_EXTENDED_STATUS,
    MAV_DATA_STREAM_RC_CHANNELS,
    MAV_DATA_STREAM_POSITION,
    MAV_DATA_STREAM_EXTRA1, 
    MAV_DATA_STREAM_EXTRA2,
    MAV_DATA_STREAM_EXTRA3                                                      };

  const uint16_t MAVRates[maxStreams] = {
    0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05                    };


  /*
  //Request MAVLink Data Stream
   for (int i=0; i < maxStreams; i++) {
   mavlink_msg_request_data_stream_send(MAVLINK_COMM_1,
   gs_mav_system, gs_mav_component,
   MAVStreams[i], MAVRates[i], 1); 
   }*/

  //Request All Data Stream at 5 Hz
  mavlink_msg_request_data_stream_send(MAVLINK_COMM_1,
  1, 1,
  MAV_DATA_STREAM_ALL,0x05, 1); //MAVRates[i]
}

//Collect msg from portable ground station
void read_ground()
{
  while(Serial2.available() > 0) { 
    uint8_t c = Serial2.read();
    //Serial.print(c);
    if (c==stx)
    {
      process_gs();
      clear_gs();
      gs_count = 0;
      gs_msg[gs_count] = stx;
      // Serial.println("STX");
    }
    else
    {
      gs_count++;
      gs_msg[gs_count] = c;
      //Serial.print("(");
      //Serial.print(String(c,HEX));
      //      Serial.print(")");
    }
    //In case of running out of buffer
    if(gs_count>=49)
     {
       gs_count = 0;
       clear_gs();
     }
  }

  //process_gs();
}

//Process collected msg from portable ground station
void process_gs()
{
  if(gs_msg[0]==stx)
  {
    Serial.println("Found STX ");
    /*
    Serial.print("\nRAW ");
     Serial.print(String(gs_msg[0], HEX));
     Serial.print(" ");
     Serial.print(String(gs_msg[1], HEX));
     Serial.print(" ");
     Serial.println(String(gs_msg[2], HEX));*/
    Serial.print("\ncmd_id : ");
    Serial.println(gs_msg[1]);
    switch (gs_msg[1]) {
      /*    case 0x00:
       //Start Req
       Serial.println("Req Start");
       stream_enable = true;
       
       break;
       case 0x01:
       //Stop Req
       Serial.println("Req Stop");
       stream_enable = false;
       
       break;
       */    case 0x32: //RC Override
      int rc_override[] = {
        0,0,0,0,0,0,0,0            }; 
      rc_override[2] = read_int16(gs_msg[2],gs_msg[3]);
      rc_override[3] = read_int16(gs_msg[4],gs_msg[5]);
      rc_override[1] = read_int16(gs_msg[6],gs_msg[7]);
      rc_override[0] = read_int16(gs_msg[8],gs_msg[9]);
      rc_override[2] = 0;
      rc_override[3] = 0;
      Serial.print("RC Override Recv : ");
      for(int i=0;i<8;i++)
      {
        if(rc_override[i]<=1000)
        {
          rc_override[i] = 0;
        }
        Serial.print(rc_override[i]);
        Serial.print(" ");
      }
      Serial.print("\n");
      mavlink_msg_rc_channels_override_send(MAVLINK_COMM_1,gs_mav_system, gs_mav_component , rc_override[0] , rc_override[1] , rc_override[2] , rc_override[3] , 0 , 0 , 0 , 0 ); //Sent RC Override

      break;
    }

    clear_gs();
  }
}

//Clear ground station buffer
void clear_gs()
{
  for(int i=0;i<sizeof(gs_msg);i++)
  {
    gs_msg[i]=0;
  }
}

//Read and Process MAVLink Message
void read_mavlink(){
  mavlink_message_t msg; 
  mavlink_status_t status;

  // grabing data 
  while(Serial1.available() > 0) { 
    uint8_t c = Serial1.read();

    // trying to grab msg  
    if(mavlink_parse_char(MAVLINK_COMM_1, c, &msg, &status)) {

      //Debug Message
      /*Serial.print("Msg id : ");
       Serial.println(msg.msgid);
       */
      switch(msg.msgid) {
      case MAVLINK_MSG_ID_HEARTBEAT:
        {
          gs_mav_system    = msg.sysid;
          gs_mav_component = msg.compid;
          gs_mav_type = mavlink_msg_heartbeat_get_type(&msg); //MAV Type
          gs_flight_mode = CheckFlightMode(mavlink_msg_heartbeat_get_custom_mode(&msg)); //MAV Flight mode
          gs_arm = (mavlink_msg_heartbeat_get_base_mode(&msg) & MOTORS_ARMED) >> 7;

          gs_beat = true;

          Serial.print("MAVlink HeartBeat ");
          Serial.print("MAV: ");
          Serial.print(gs_mav_type);
          Serial.print("  Modes: ");
          Serial.print(gs_flight_mode);
          Serial.print("  Armed: ");
          Serial.print(gs_arm); //isArmed
          Serial.print("  FIX: ");
          Serial.print(gs_fix_type);
          Serial.print("  Sats: ");
          Serial.print(gs_num_sat);
          Serial.print("  BatVolt: ");
          Serial.print(gs_volt_batt);
          Serial.print("  SysId: ");
          Serial.print(gs_mav_system);
          Serial.print("  ComId: ");
          Serial.println(gs_mav_component);
        }
        break;

      case MAVLINK_MSG_ID_SYS_STATUS:
        { 
          gs_volt_batt = (mavlink_msg_sys_status_get_voltage_battery(&msg) ); //1 equal 1 milli volt
          //mavlink_msg_sys_status_get_current_battery(&msg); //1 equal 10 milli amp
          //mavlink_msg_sys_status_get_battery_remaining(&msg);
        }
        break;

      case MAVLINK_MSG_ID_GPS_RAW_INT:
        {
          gs_fix_type = mavlink_msg_gps_raw_int_get_fix_type(&msg); //0:no gps,1:no fix,2:2D fix,3:3d fix
          gs_hdop = mavlink_msg_gps_raw_int_get_eph(&msg); //hdop dilution of position in cm (m*100) 
          gs_num_sat = mavlink_msg_gps_raw_int_get_satellites_visible(&msg);
        }
        break;

      case MAVLINK_MSG_ID_ATTITUDE:
        {
          gs_roll = ToDeg(mavlink_msg_attitude_get_roll(&msg));
          gs_pitch = ToDeg(mavlink_msg_attitude_get_pitch(&msg));
          gs_yaw = ToDeg(mavlink_msg_attitude_get_yaw(&msg));
          /*
          Serial.print("ATITTUDE ");
           Serial.print(ToDeg(mavlink_msg_attitude_get_pitch(&msg)));
           Serial.print(ToDeg(mavlink_msg_attitude_get_roll(&msg)));
           Serial.println(ToDeg(mavlink_msg_attitude_get_yaw(&msg)));*/
          //ToDeg(mavlink_msg_attitude_get_pitchspeed(&msg)); //Pitch Speed
          //ToDeg(mavlink_msg_attitude_get_rollspeed(&msg)); //Roll Speed
          //ToDeg(mavlink_msg_attitude_get_yawspeed(&msg)); //Yaw Speed
        }
        break;

      case MAVLINK_MSG_ID_VFR_HUD:
        {
          gs_alt = mavlink_msg_vfr_hud_get_alt(&msg);
          gs_heading = mavlink_msg_vfr_hud_get_heading(&msg);
          gs_ground_speed = mavlink_msg_vfr_hud_get_groundspeed(&msg);
          // Serial.print("VFR_HUD : ");
          // Serial.println(mavlink_msg_vfr_hud_get_alt(&msg));
        }
        break;

      case MAVLINK_MSG_ID_RC_CHANNELS_RAW :

        gs_ch_raw[0] = mavlink_msg_rc_channels_raw_get_chan1_raw(&msg);
        gs_ch_raw[1] = mavlink_msg_rc_channels_raw_get_chan2_raw(&msg);
        gs_ch_raw[2] = mavlink_msg_rc_channels_raw_get_chan3_raw(&msg);
        gs_ch_raw[3] = mavlink_msg_rc_channels_raw_get_chan4_raw(&msg);
        gs_ch_raw[4] = mavlink_msg_rc_channels_raw_get_chan5_raw(&msg);
        gs_ch_raw[5] = mavlink_msg_rc_channels_raw_get_chan6_raw(&msg);
        gs_ch_raw[6] = mavlink_msg_rc_channels_raw_get_chan7_raw(&msg);
        gs_ch_raw[7] = mavlink_msg_rc_channels_raw_get_chan8_raw(&msg);

        Serial.print("RC_CHANNELS_RAW  : ");
        Serial.print(mavlink_msg_rc_channels_raw_get_chan1_raw(&msg));
        Serial.print(",");
        Serial.print(mavlink_msg_rc_channels_raw_get_chan2_raw(&msg));
        Serial.print(",");
        Serial.print(mavlink_msg_rc_channels_raw_get_chan3_raw(&msg));
        Serial.print(",");
        Serial.print(mavlink_msg_rc_channels_raw_get_chan4_raw(&msg));
        Serial.print(",");
        Serial.print(mavlink_msg_rc_channels_raw_get_chan5_raw(&msg));
        Serial.print(",");
        Serial.print(mavlink_msg_rc_channels_raw_get_chan6_raw(&msg));
        Serial.print(",");
        Serial.print(mavlink_msg_rc_channels_raw_get_chan7_raw(&msg));
        Serial.print(",");
        Serial.println(mavlink_msg_rc_channels_raw_get_chan8_raw(&msg));
       /* if(gs_ch_raw[0]>=1600) { 
          mavlink_msg_command_long_send(MAVLINK_COMM_1,1, 1,MAV_CMD_COMPONENT_ARM_DISARM,0,1,0,0,0,0,0,0); 
          Serial.println("ARM");
        }//Arm 
        if(gs_ch_raw[0]<=1400) { 
          mavlink_msg_command_long_send(MAVLINK_COMM_1,1, 1,MAV_CMD_COMPONENT_ARM_DISARM,0,0,0,0,0,0,0,0); 
          Serial.println("DISARM");
        }//Arm 

        if(gs_ch_raw[1]>=1600) 
        {
          Serial.println("OVerride");
          mavlink_msg_rc_channels_override_send(MAVLINK_COMM_1,1, 1 , 0 , 0 , 1800 , 0 , 0 , 0 , 0 , 0 ); //Sent RC Override
        }
        if(gs_ch_raw[1]<=1400) 
        {
          Serial.println("De-override");
          mavlink_msg_rc_channels_override_send(MAVLINK_COMM_1,1, 1 , 0 , 0 , 0 , 0 , 0 , 0 , 0 , 0 ); //Sent RC Override
        }*/
        break;

      case MAVLINK_MSG_ID_COMMAND_ACK :
        Serial.print("ACK ");
        Serial.print(mavlink_msg_command_ack_get_command(&msg));
        Serial.print(":"); 
        Serial.println(mavlink_msg_command_ack_get_result(&msg));
        break;

      case MAVLINK_MSG_ID_PARAM_VALUE :
        Serial.print("Param ");
        mavlink_msg_param_value_get_param_id(&msg,gs_text);
        Serial.print(gs_text);
        Serial.print(":");
        Serial.println(mavlink_msg_param_value_get_param_value(&msg));
        break;

      case MAVLINK_MSG_ID_STATUSTEXT :
        Serial.print("STATUSTEXT ");
        Serial.print(mavlink_msg_statustext_get_severity(&msg));           
        Serial.print(",");
        //Serial.println(mavlink_msg_statustext_get_text(&msg,myText));
        break;

        //Debug
      case MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE  :
        Serial.print("RC Override ");
        Serial.print(mavlink_msg_rc_channels_override_get_target_system(&msg));           
        Serial.print(",");
        Serial.print(mavlink_msg_rc_channels_override_get_target_component(&msg));     
        Serial.print(" : ");
        Serial.print(mavlink_msg_rc_channels_override_get_chan1_raw(&msg));
        Serial.print(",");
        Serial.print(mavlink_msg_rc_channels_override_get_chan2_raw(&msg));
        Serial.print(",");
        Serial.print(mavlink_msg_rc_channels_override_get_chan3_raw(&msg));
        Serial.print(",");
        Serial.print(mavlink_msg_rc_channels_override_get_chan4_raw(&msg));
        Serial.print(",");
        Serial.print(mavlink_msg_rc_channels_override_get_chan5_raw(&msg));      
        break;
      case MAVLINK_MSG_ID_COMMAND_LONG : 
        Serial.print("COMMAND LONG  : system,component,command,confirmation,para1-7 ");
        Serial.print(mavlink_msg_command_long_get_target_system(&msg));
        Serial.print(",");
        Serial.print(mavlink_msg_command_long_get_target_component(&msg));
        Serial.print(",");
        Serial.print(mavlink_msg_command_long_get_command(&msg));
        Serial.print(",");
        Serial.print(mavlink_msg_command_long_get_confirmation(&msg));
        Serial.print(",");
        Serial.print(mavlink_msg_command_long_get_param1(&msg));
        Serial.print(",");
        Serial.print(mavlink_msg_command_long_get_param2(&msg));
        Serial.print(",");
        Serial.print(mavlink_msg_command_long_get_param3(&msg));
        Serial.print(",");
        Serial.print(mavlink_msg_command_long_get_param4(&msg));
        Serial.print(",");
        Serial.print(mavlink_msg_command_long_get_param5(&msg));
        Serial.print(",");
        Serial.print(mavlink_msg_command_long_get_param6(&msg));
        Serial.print(",");
        Serial.println(mavlink_msg_command_long_get_param7(&msg));
        break;
      default:
        //Do nothing
        break;
      }
    }
    delayMicroseconds(138); 
  }
  // Update packet drop,error counter
  packet_drops += status.packet_rx_drop_count;
  parse_error += status.parse_error;
}

void sent_float_udp(float value)
{
  binaryFloat temp;
  temp.floatingPoint = value;
  udp.write(temp.binary[3]);
  udp.write(temp.binary[2]);
  udp.write(temp.binary[1]);
  udp.write(temp.binary[0]);
}

void sent_int_8_udp(int value)
{
  udp.write(value);
}

void sent_int_16_udp(int16_t value)
{
  /*
   Serial.print(String(value,BIN));
   Serial.println("");
   Serial.print(String((value & 0xFF00)>>8,BIN));
   Serial.print(":");
   Serial.println(String(value & 0x00FF,BIN));
   */
  udp.write((value & 0xFF00)>>8);
  udp.write(value & 0x00FF);
}

void sent_msg_11_udp()
{
  udp.beginPacket(IPAddress(192,168,1,44),5000);
  udp.write(stx);
  udp.write(11);
  udp.print(gs_flight_mode);
  udp.write(gs_mav_type);
  udp.write(gs_arm);
  udp.endPacket();
  udp.flush();
  udp.stop();
}


void sent_msg_12_udp()
{

  udp.beginPacket(IPAddress(192,168,1,44),5000);
  udp.write(stx);
  udp.write(12);
  udp.write(gs_fix_type);
  sent_int_16_udp(gs_hdop);
  udp.write(gs_num_sat);
  sent_int_16_udp(gs_volt_batt);
  udp.endPacket();
  udp.flush();
  udp.stop();
}

void sent_msg_13_udp()
{
  binaryFloat temp;

  udp.beginPacket(IPAddress(192,168,1,44),5000);
  udp.write(stx);
  udp.write(13);
  sent_float_udp(gs_roll);
  sent_float_udp(gs_pitch);
  sent_float_udp(gs_yaw);
  udp.endPacket();
  udp.flush();
  udp.stop();
}


void sent_msg_14_udp()
{
  udp.beginPacket(IPAddress(192,168,1,44),5000);
  udp.write(stx);
  udp.write(14);
  sent_float_udp(gs_alt);
  sent_int_16_udp(gs_heading);
  sent_float_udp(gs_ground_speed);
  udp.endPacket();
  udp.flush();
  udp.stop();
}

void sent_msg_15_udp()
{
  udp.beginPacket(IPAddress(192,168,1,44),5000);
  udp.write(stx);
  udp.write(15);
  
  for(int i=0;i<=7;i++)
  {  
    sent_int_16_udp(gs_ch_raw[i]);
  } 
  
  udp.endPacket();
  udp.flush();
  udp.stop();
}


//Implement by Thanat Tothong
int read_int16(byte data1, byte data2)
{
  return (data1 << 8)  + data2 ;
}








