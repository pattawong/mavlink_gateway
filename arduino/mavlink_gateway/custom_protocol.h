
static uint8_t  gs_mav_system; 
static uint8_t  gs_mav_component;
static uint8_t  gs_mav_type;

//Ground Station buffer
static int gs_count =0;
static byte gs_msg[50];

//MAVLink HeartBeat Check
static int gs_beat_count = 0;
static boolean gs_beat = false;
static boolean gs_mavlink = false;

// MAVLink HeartBeat bits
#define MOTORS_ARMED 128

//Msg ID 11
static String  gs_flight_mode;
static uint8_t  gs_arm;

//Msg ID 12
static uint8_t gs_fix_type;
static uint16_t gs_hdop;
static uint8_t gs_num_sat;
static uint16_t gs_volt_batt;

//Msg ID 13
static float gs_roll;
static float gs_pitch;
static float gs_yaw;

//Msg ID 14
static float gs_alt;
static int16_t gs_heading;
static float gs_ground_speed;

//Msg ID 15
static uint16_t gs_ch_raw[8];

static char* gs_text = ""; //testing text for sending param function
String CheckFlightMode(int base_mode)
{
  if(base_mode == 0) return "Stabilize";   // Stabilize
  if(base_mode == 1) return "Acrobatic";   // Acrobatic
  if(base_mode == 2) return "Alt_Hold";   // Alt Hold
  if(base_mode == 3) return "Auto";   // Auto
  if(base_mode == 4) return "Guided";   // Guided
  if(base_mode == 5) return "Loiter";   // Loiter
  if(base_mode == 6) return "RTL";   // Return to Launch
  if(base_mode == 7) return "Circle";   // Circle
  if(base_mode == 8) return "Position";   // Position
  if(base_mode == 9) return "Land";   // Land
  if(base_mode == 10) return "OF_Loiter";  // OF_Loiter
  if(base_mode == 11) return "Drift";  // Drift 
  if(base_mode == 13) return "Sport";  // earth frame rate control
  if(base_mode == 14) return "Flip";  // flip the vehicle on the roll axis
  if(base_mode == 15) return "AutoTune";  // autotune the vehicle's roll and pitch gains
  if(base_mode == 16) return "Pos_Hold";  // position hold with manual override
  if(base_mode == 17) return "Num_Modes";  // 
}

