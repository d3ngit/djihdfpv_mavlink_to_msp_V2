
/* DJI HD FPV Mavlink to MSP
 *  Converts Ardupilot Mavlink telemetry data to MSP telemetry data compatible with the DJI HD FPV system.
 *  
 *  Arduino Nano TX to DJI Air unit RX(115200)
 *  Softwareserial RX is digital pin 8 connected to ardupilot TX telemetry port(57600)
 *  Softwareserial TX is digital pin 9 connected to ardupilot RX telemetry port(57600)
*/

#define MAH_CALIBRATION_FACTOR                                      1.166f  // used to calibrate mAh reading. Matek F405wing ~1.166
#define SPEED_IN_KILOMETERS_PER_HOUR                                        // if commented out defaults to m/s
//#define SPEED_IN_MILES_PER_HOUR
#define USE_CRAFT_NAME_FOR_ALTITUDE_AND_SPEED                               //comment out to disable
#define USE_PITCH_ROLL_ANGLE_FOR_DISTANCE_AND_DIRECTION_TO_HOME             //comment out to disable
//#define IMPERIAL_UNITS                                                    //Altitude in feet, distance to home in miles.

#include <checksum.h>
#include <mavlink.h>
#include <mavlink_helpers.h>
#include <mavlink_types.h>
#include <protocol.h>
#include <MSP.h>
#include <SoftwareSerial.h>
#include "MSP_OSD.h"

SoftwareSerial mavlinkSerial(8, 9); // RX, TX
MSP msp;

//OSD item locations
//in betaflight configurator set OSD items to your desired positions and in CLI type "set osd" to retreieve the numbers.
//234 -> not visible.  (0 - 15359) . horizontally 2048-2074(spacing 1), vertically 2048-2528(spacing 32). 26 characters X 15 lines
uint16_t osd_rssi_value_pos = 2052;
uint16_t osd_main_batt_voltage_pos = 2071;
uint16_t osd_crosshairs_pos = 234;
uint16_t osd_artificial_horizon_pos = 234;
uint16_t osd_horizon_sidebars_pos = 234;
uint16_t osd_item_timer_1_pos = 234;
uint16_t osd_item_timer_2_pos = 234;
uint16_t osd_flymode_pos = 234;
uint16_t osd_craft_name_pos = 2497;
uint16_t osd_throttle_pos_pos = 234;
uint16_t osd_vtx_channel_pos = 234;
uint16_t osd_current_draw_pos = 2103;
uint16_t osd_mah_drawn_pos = 2136;
uint16_t osd_gps_speed_pos = 234;
uint16_t osd_gps_sats_pos = 2465;
uint16_t osd_altitude_pos = 234;
uint16_t osd_roll_pids_pos = 234;
uint16_t osd_pitch_pids_pos = 234;
uint16_t osd_yaw_pids_pos = 234;
uint16_t osd_power_pos = 234;
uint16_t osd_pidrate_profile_pos = 234;
uint16_t osd_warnings_pos = 234;
uint16_t osd_avg_cell_voltage_pos = 234;
uint16_t osd_gps_lon_pos = 2113;
uint16_t osd_gps_lat_pos = 2081;
uint16_t osd_debug_pos = 234;
uint16_t osd_pitch_angle_pos = 2488;
uint16_t osd_roll_angle_pos = 2456;
uint16_t osd_main_batt_usage_pos = 234;
uint16_t osd_disarmed_pos = 234;
uint16_t osd_home_dir_pos = 234;
uint16_t osd_home_dist_pos = 234;
uint16_t osd_numerical_heading_pos = 234;
uint16_t osd_numerical_vario_pos = 234;
uint16_t osd_compass_bar_pos = 234;
uint16_t osd_esc_tmp_pos = 234;
uint16_t osd_esc_rpm_pos = 234;
uint16_t osd_remaining_time_estimate_pos = 234;
uint16_t osd_rtc_datetime_pos = 234;
uint16_t osd_adjustment_range_pos = 234;
uint16_t osd_core_temperature_pos = 234;
uint16_t osd_anti_gravity_pos = 234;
uint16_t osd_g_force_pos = 234;
uint16_t osd_motor_diag_pos = 234;
uint16_t osd_log_status_pos = 234;
uint16_t osd_flip_arrow_pos = 234;
uint16_t osd_link_quality_pos = 234;
uint16_t osd_flight_dist_pos = 234;
uint16_t osd_stick_overlay_left_pos = 234;
uint16_t osd_stick_overlay_right_pos = 234;
uint16_t osd_display_name_pos = 234;
uint16_t osd_esc_rpm_freq_pos = 234;
uint16_t osd_rate_profile_name_pos = 234;
uint16_t osd_pid_profile_name_pos = 234;
uint16_t osd_profile_name_pos = 234;
uint16_t osd_rssi_dbm_value_pos = 234;
uint16_t osd_rc_channels_pos = 234;


uint16_t previousMillisMAVLink = 0;
uint16_t next_interval_MAVLink = 1000;
uint8_t num_hbts = 60;
uint8_t num_hbts_elapsed = num_hbts;

uint16_t previousMillisMSP = 0;
uint16_t next_interval_MSP = 200;

uint8_t sysid = 199;
uint8_t compid = 50;
uint8_t type = MAV_TYPE_GCS;
uint8_t autopilot = MAV_AUTOPILOT_INVALID;
uint8_t base_mode = MAV_MODE_PREFLIGHT;
uint32_t custom_mode = 0;
uint8_t system_status = MAV_STATE_UNINIT;

uint8_t vbat = 0;
float airspeed = 0;
float groundspeed = 0;
uint16_t altitude_mav = 0;      // in meters
uint32_t altitude_msp = 0;      // EstimatedAltitudeCm
uint16_t rssi = 0;
uint8_t battery_remaining = 0;
uint32_t flightModeFlags = 0;
char craftname[12] = "";
int16_t amperage = 0;
uint16_t mAhDrawn = 0;
float f_mAhDrawn = 0.0;
uint8_t  numSat = 0;
uint8_t pid_roll[3];
uint8_t pid_pitch[3];
uint8_t pid_yaw[3]; 
int32_t gps_lon = 0;
int32_t gps_lat = 0;
int32_t gps_alt = 0;
int32_t gps_home_lon = 0;
int32_t gps_home_lat = 0;
int32_t gps_home_alt = 0;
int16_t roll_angle = 0;
int16_t pitch_angle = 0;
uint8_t is_armed = 0;
uint8_t escTemperature = 0;     // degrees celsius
uint8_t rtc_date_time[9] = {20, 20, 01, 01, 00, 00, 00, 00, 00}; //YMDHMSM
int16_t distanceToHome = 0;    // distance to home in meters
int16_t directionToHome = 0;   // direction to home in degrees
uint8_t fix_type = 0;           // < 0-1: no fix, 2: 2D fix, 3: 3D fix
uint8_t home_locked = 0;
uint8_t batteryCellCount = 4;
uint16_t batteryCapacity = 5200;
uint8_t legacyBatteryVoltage = 168;
uint8_t batteryState = 0;       // voltage color 0==white, 1==red 
uint16_t batteryVoltage = 0;
uint16_t heading = 0;
float dt = 0;
#ifdef MAH_CALIBRATION_FACTOR
float mAh_calib_factor = MAH_CALIBRATION_FACTOR;
#else
float mAh_calib_factor = 1;
#endif

void setup()
{
    Serial.begin(115200);
    msp.begin(Serial);
    mavlinkSerial.begin(57600);

}

void loop()
{

    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    mavlink_msg_heartbeat_pack(sysid,compid, &msg, type, autopilot, base_mode, custom_mode, system_status);

    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    //send heartbeat every second 
    uint16_t currentMillisMAVLink = millis();
    if (currentMillisMAVLink - previousMillisMAVLink >= next_interval_MAVLink) {
        previousMillisMAVLink = currentMillisMAVLink;

        mavlinkSerial.write(buf, len);

        //send request streams every 60 seconds
        num_hbts_elapsed++;
        if(num_hbts_elapsed >= num_hbts) {
        mavl_request_data();
        num_hbts_elapsed=0;
    }

    }

    mavl_receive();  
    
    //send msp 
    uint16_t currentMillisMSP = millis();
    if (currentMillisMSP - previousMillisMSP >= next_interval_MSP) {
        previousMillisMSP = currentMillisMSP;
        
        GPS_calculateDistanceAndDirectionToHome();
        
        send_msp_to_goggles();
        
        f_mAhDrawn += ((float)amperage * 10.0 * (millis() - dt) / 3600000.0) * mAh_calib_factor;
        mAhDrawn = (uint16_t)f_mAhDrawn;
        dt = millis();
    }
    
    if(gps_home_lat != 0 && home_locked == 0){
        GPS_calc_longitude_scaling(gps_home_lat);
        home_locked = 1;
    }
    
}

void mavl_receive() 
{

    mavlink_message_t msg;
    mavlink_status_t status;

    while(mavlinkSerial.available() > 0 ) 
    {
        
    uint8_t c = mavlinkSerial.read();

    if(mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {

      switch(msg.msgid)
      {
      
          case MAVLINK_MSG_ID_SYS_STATUS:  // SYS_STATUS
          {
            mavlink_sys_status_t sys_status;
            mavlink_msg_sys_status_decode(&msg, &sys_status);

            vbat = (uint8_t)(sys_status.voltage_battery / 100);
            battery_remaining = (uint8_t)(sys_status.battery_remaining);
            amperage = sys_status.current_battery;
            
            }
            break;
          
          case MAVLINK_MSG_ID_VFR_HUD:  // VFR_HUD
          {
            mavlink_vfr_hud_t vfr_hud;
            mavlink_msg_vfr_hud_decode(&msg, &vfr_hud);

            airspeed = vfr_hud.airspeed; //float
            groundspeed = vfr_hud.groundspeed; //float
            
            }
            break;

          case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:  // MAVLINK_MSG_ID_GLOBAL_POSITION_INT
          {
            mavlink_global_position_int_t global_position_int;
            mavlink_msg_global_position_int_decode(&msg, &global_position_int);
            
            heading = global_position_int.hdg;
            altitude_mav = (uint16_t)(global_position_int.relative_alt / 1000); //int32_t in milimeters -> converted to meters
            gps_lat = global_position_int.lat;
            gps_lon = global_position_int.lon;
            gps_alt = global_position_int.alt;                
           }
           break;

          case MAVLINK_MSG_ID_GPS_RAW_INT:  // MAVLINK_MSG_ID_GPS_RAW_INT
          {
            mavlink_gps_raw_int_t gps_raw_int;
            mavlink_msg_gps_raw_int_decode(&msg, &gps_raw_int);
            
            fix_type = gps_raw_int.fix_type;
            numSat = gps_raw_int.satellites_visible;
                
           }
           break;

          case MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN:  // MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN
          {
            mavlink_gps_global_origin_t gps_global_origin;
            mavlink_msg_gps_global_origin_decode(&msg, &gps_global_origin);
            
            gps_home_lat = gps_global_origin.latitude;
            gps_home_lon = gps_global_origin.longitude;
            gps_home_alt = gps_global_origin.altitude;
                
           }
           break;

        case MAVLINK_MSG_ID_HEARTBEAT:  // HEARTBEAT
          {
            mavlink_heartbeat_t heartbeat;
            mavlink_msg_heartbeat_decode(&msg, &heartbeat);

            //replying same heartbeat info
            base_mode = heartbeat.base_mode;
            type = heartbeat.type;
            autopilot = heartbeat.autopilot;
            custom_mode = heartbeat.custom_mode;
            system_status = heartbeat.system_status;
            
            }
            break;            
            
         case MAVLINK_MSG_ID_RC_CHANNELS_RAW:  // RC_CHANNELS_RAW
          {
            mavlink_rc_channels_raw_t rc_channels_raw;
            mavlink_msg_rc_channels_raw_decode(&msg, &rc_channels_raw);
           
            rssi = (uint16_t)map(rc_channels_raw.rssi, 0, 255, 0, 1023); //scale 0-1023
              
            }
            break; 
                                 
      default:
        break;
      }
    }

    }
}

void mavl_request_data()
{
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    const int  maxStreams = 7;
    const uint8_t MAVStreams[maxStreams] = {MAV_DATA_STREAM_RAW_SENSORS,
        MAV_DATA_STREAM_EXTENDED_STATUS,
        MAV_DATA_STREAM_RC_CHANNELS,
        MAV_DATA_STREAM_POSITION,
        MAV_DATA_STREAM_EXTRA1, 
        MAV_DATA_STREAM_EXTRA2
        };

    const uint16_t MAVRates[maxStreams] = {0x02, 0x02, 0x05, 0x02, 0x05, 0x02};

    for (int i=0; i < maxStreams; i++) {
        mavlink_msg_request_data_stream_pack(2, 200, &msg, 1, 0, MAVStreams[i], MAVRates[i], 1);
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

        mavlinkSerial.write(buf, len);
    }
}

char altstr[10];
char gsstr[10];
char tmpGSStr[4];

void send_msp_to_goggles()
{
    msp_fc_version_t fc_version;
    msp_name_t name;
    msp_status_t status;
    msp_analog_t analog;
    msp_battery_state_t battery_state;
    msp_status_ex_t status_ex;
    msp_raw_gps_t raw_gps;
    msp_comp_gps_t comp_gps;
    msp_attitude_t attitude;
    msp_altitude_t altitude;
   
                
    if(base_mode & MAV_MODE_FLAG_SAFETY_ARMED){
        status.flightModeFlags = 1;
        status_ex.flightModeFlags = 1;
    } 
    else{
        status.flightModeFlags = 0;
        status_ex.flightModeFlags = 0;
    }
    
    //MSP_FC_VERSION
    fc_version.versionMajor = 4;
    fc_version.versionMinor = 1;
    fc_version.versionPatchLevel = 1;
    msp.send(MSP_FC_VERSION, &fc_version, sizeof(fc_version));
    
    //MSP_NAME
#ifdef USE_CRAFT_NAME_FOR_ALTITUDE_AND_SPEED
    memset(altstr, 0, sizeof altstr);
    memset(gsstr, 0, sizeof gsstr);
    memset(craftname, 0, sizeof craftname);
    memset(tmpGSStr, 0, sizeof tmpGSStr);
    craftname[0] = 'A';
    tmpGSStr[0] = ' ';
    tmpGSStr[1] = 'S';
  #ifdef IMPERIAL_UNITS
    itoa((uint16_t)(altitude_mav / 0.3048), altstr, 10);
  #else  
    itoa(altitude_mav, altstr, 10);   //base 10
  #endif
  #ifdef SPEED_IN_KILOMETERS_PER_HOUR
    itoa((uint16_t)(groundspeed * 3.6), gsstr, 10);
  #elif defined(SPEED_IN_MILES_PER_HOUR)
    itoa((uint16_t)(groundspeed * 2.2369), gsstr, 10);
  #else
    itoa((uint16_t)(groundspeed), gsstr, 10);
  #endif
    strcat(craftname, altstr);
    strcat(craftname, tmpGSStr);
    strcat(craftname, gsstr);
    strncpy(name.craft_name, craftname, sizeof(name.craft_name));
#else
    strncpy(name.craft_name, craftname, sizeof(name.craft_name));
#endif
    msp.send(MSP_NAME, &name, sizeof(name));
    
    //MSP_STATUS
    msp.send(MSP_STATUS, &status, sizeof(status));
    
    //MSP_ANALOG
    analog.vbat = vbat;
    analog.rssi = rssi;
    analog.amperage = amperage;
    analog.mAhDrawn = mAhDrawn;
    msp.send(MSP_ANALOG, &analog, sizeof(analog));

    //MSP_BATTERY_STATE
    battery_state.amperage = amperage;
    battery_state.batteryVoltage = vbat;
    battery_state.mAhDrawn = mAhDrawn;
    battery_state.batteryCellCount = batteryCellCount;
    battery_state.batteryCapacity = batteryCapacity;
    battery_state.legacyBatteryVoltage = legacyBatteryVoltage;
    battery_state.batteryState = batteryState;
    msp.send(MSP_BATTERY_STATE, &battery_state, sizeof(battery_state));
    
    //MSP_STATUS_EX
    msp.send(MSP_STATUS_EX, &status_ex, sizeof(status_ex));
    
    //MSP_RAW_GPS
    raw_gps.lat = gps_lat;
    raw_gps.lon = gps_lon;
    raw_gps.numSat = numSat;
    //raw_gps.alt = (int16_t)altitude_msp;
    //raw_gps.groundSpeed = (int16_t)groundspeed;
    msp.send(MSP_RAW_GPS, &raw_gps, sizeof(raw_gps));
    
    //MSP_COMP_GPS
    comp_gps.distanceToHome = distanceToHome;
    comp_gps.directionToHome = directionToHome;
    msp.send(MSP_COMP_GPS, &comp_gps, sizeof(comp_gps));
    
    //MSP_ATTITUDE
#ifdef USE_PITCH_ROLL_ANGLE_FOR_DISTANCE_AND_DIRECTION_TO_HOME
    #ifdef IMPERIAL_UNITS
        distanceToHome = (uint16_t)((distanceToHome * 0.000621371192) * 10);  //meters to miles
        attitude.pitch = distanceToHome;
    #else
    if(distanceToHome > 1000){
        attitude.pitch = distanceToHome / 100; // switch from m to km when over 1000
    }
    else{
        attitude.pitch = distanceToHome * 10;
    }
    #endif
    attitude.roll = (directionToHome - (heading / 100)) * 10;
#else
    attitude.pitch = pitch_angle;
    attitude.roll = roll_angle;
#endif
    msp.send(MSP_ATTITUDE, &attitude, sizeof(attitude));
    
    //MSP_ALTITUDE
    altitude.estimatedActualPosition = altitude_msp; //cm
    msp.send(MSP_ALTITUDE, &altitude, sizeof(altitude));
    
    //MSP_OSD_CONFIG
    send_osd_config();
}
void send_osd_config()
{
    msp_osd_config_t msp_osd_config;    
    
    msp_osd_config.osd_item_count = 56;
    msp_osd_config.osd_stat_count = 24;
    msp_osd_config.osd_timer_count = 2;
    msp_osd_config.osd_warning_count = 16;              // 16
    msp_osd_config.osd_profile_count = 1;              // 1
    msp_osd_config.osdprofileindex = 1;                // 1
    msp_osd_config.overlay_radio_mode = 0;             //  0

    msp_osd_config.osd_rssi_value_pos = osd_rssi_value_pos;
    msp_osd_config.osd_main_batt_voltage_pos = osd_main_batt_voltage_pos;
    msp_osd_config.osd_crosshairs_pos = osd_crosshairs_pos;
    msp_osd_config.osd_artificial_horizon_pos = osd_artificial_horizon_pos;
    msp_osd_config.osd_horizon_sidebars_pos = osd_horizon_sidebars_pos;
    msp_osd_config.osd_item_timer_1_pos = osd_item_timer_1_pos;
    msp_osd_config.osd_item_timer_2_pos = osd_item_timer_2_pos;
    msp_osd_config.osd_flymode_pos = osd_flymode_pos;
    msp_osd_config.osd_craft_name_pos = osd_craft_name_pos;
    msp_osd_config.osd_throttle_pos_pos = osd_throttle_pos_pos;
    msp_osd_config.osd_vtx_channel_pos = osd_vtx_channel_pos;
    msp_osd_config.osd_current_draw_pos = osd_current_draw_pos;
    msp_osd_config.osd_mah_drawn_pos = osd_mah_drawn_pos;
    msp_osd_config.osd_gps_speed_pos = osd_gps_speed_pos;
    msp_osd_config.osd_gps_sats_pos = osd_gps_sats_pos;
    msp_osd_config.osd_altitude_pos = osd_altitude_pos;
    msp_osd_config.osd_roll_pids_pos = osd_roll_pids_pos;
    msp_osd_config.osd_pitch_pids_pos = osd_pitch_pids_pos;
    msp_osd_config.osd_yaw_pids_pos = osd_yaw_pids_pos;
    msp_osd_config.osd_power_pos = osd_power_pos;
    msp_osd_config.osd_pidrate_profile_pos = osd_pidrate_profile_pos;
    msp_osd_config.osd_warnings_pos = osd_warnings_pos;
    msp_osd_config.osd_avg_cell_voltage_pos = osd_avg_cell_voltage_pos;
    msp_osd_config.osd_gps_lon_pos = osd_gps_lon_pos;
    msp_osd_config.osd_gps_lat_pos = osd_gps_lat_pos;
    msp_osd_config.osd_debug_pos = osd_debug_pos;
    msp_osd_config.osd_pitch_angle_pos = osd_pitch_angle_pos;
    msp_osd_config.osd_roll_angle_pos = osd_roll_angle_pos;
    msp_osd_config.osd_main_batt_usage_pos = osd_main_batt_usage_pos;
    msp_osd_config.osd_disarmed_pos = osd_disarmed_pos;
    msp_osd_config.osd_home_dir_pos = osd_home_dir_pos;
    msp_osd_config.osd_home_dist_pos = osd_home_dist_pos;
    msp_osd_config.osd_numerical_heading_pos = osd_numerical_heading_pos;
    msp_osd_config.osd_numerical_vario_pos = osd_numerical_vario_pos;
    msp_osd_config.osd_compass_bar_pos = osd_compass_bar_pos;
    msp_osd_config.osd_esc_tmp_pos = osd_esc_tmp_pos;
    msp_osd_config.osd_esc_rpm_pos = osd_esc_rpm_pos;
    msp_osd_config.osd_remaining_time_estimate_pos = osd_remaining_time_estimate_pos;
    msp_osd_config.osd_rtc_datetime_pos = osd_rtc_datetime_pos;
    msp_osd_config.osd_adjustment_range_pos = osd_adjustment_range_pos;
    msp_osd_config.osd_core_temperature_pos = osd_core_temperature_pos;
    msp_osd_config.osd_anti_gravity_pos = osd_anti_gravity_pos;
    msp_osd_config.osd_g_force_pos = osd_g_force_pos;
    msp_osd_config.osd_motor_diag_pos = osd_motor_diag_pos;
    msp_osd_config.osd_log_status_pos = osd_log_status_pos;
    msp_osd_config.osd_flip_arrow_pos = osd_flip_arrow_pos;
    msp_osd_config.osd_link_quality_pos = osd_link_quality_pos;
    msp_osd_config.osd_flight_dist_pos = osd_flight_dist_pos;
    msp_osd_config.osd_stick_overlay_left_pos = osd_stick_overlay_left_pos;
    msp_osd_config.osd_stick_overlay_right_pos = osd_stick_overlay_right_pos;
    msp_osd_config.osd_display_name_pos = osd_display_name_pos;
    msp_osd_config.osd_esc_rpm_freq_pos = osd_esc_rpm_freq_pos;
    msp_osd_config.osd_rate_profile_name_pos = osd_rate_profile_name_pos;
    msp_osd_config.osd_pid_profile_name_pos = osd_pid_profile_name_pos;
    msp_osd_config.osd_profile_name_pos = osd_profile_name_pos;
    msp_osd_config.osd_rssi_dbm_value_pos = osd_rssi_dbm_value_pos;
    msp_osd_config.osd_rc_channels_pos = osd_rc_channels_pos;
    
    msp.send(MSP_OSD_CONFIG, &msp_osd_config, sizeof(msp_osd_config));
}


//from here on code from Betaflight github https://github.com/betaflight/betaflight/blob/c8b5edb415c33916c91a7ccc8bd19c7276540cd1/src/main/io/gps.c

float GPS_scaleLonDown = 1.0f;  // this is used to offset the shrinking longitude as we go towards the poles

void GPS_calc_longitude_scaling(int32_t lat)
{
    float rads = (fabsf((float)lat) / 10000000.0f) * 0.0174532925f;
    GPS_scaleLonDown = cos_approx(rads);
}

#define DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR_IN_HUNDREDS_OF_KILOMETERS 1.113195f
#define TAN_89_99_DEGREES 5729.57795f
// Get distance between two points in cm
// Get bearing from pos1 to pos2, returns an 1deg = 100 precision
void GPS_distance_cm_bearing(int32_t *currentLat1, int32_t *currentLon1, int32_t *destinationLat2, int32_t *destinationLon2, uint32_t *dist, int32_t *bearing)
{
    float dLat = *destinationLat2 - *currentLat1; // difference of latitude in 1/10 000 000 degrees
    float dLon = (float)(*destinationLon2 - *currentLon1) * GPS_scaleLonDown;
    *dist = sqrtf(sq(dLat) + sq(dLon)) * DISTANCE_BETWEEN_TWO_LONGITUDE_POINTS_AT_EQUATOR_IN_HUNDREDS_OF_KILOMETERS;

    *bearing = 9000.0f + atan2f(-dLat, dLon) * TAN_89_99_DEGREES;      // Convert the output radians to 100xdeg
    if (*bearing < 0)
        *bearing += 36000;
}

void GPS_calculateDistanceAndDirectionToHome(void)
{
     if (gps_home_lat != 0 && gps_home_lon != 0) {      // If we don't have home set, do not display anything
        uint32_t dist;
        int32_t dir;
        GPS_distance_cm_bearing(&gps_lat, &gps_lon, &gps_home_lat, &gps_home_lon, &dist, &dir);
        distanceToHome = dist / 100;
        directionToHome = dir / 100;
     } else {
         distanceToHome = 0;
         directionToHome = 0;
     }
}

#define M_PIf       3.14159265358979323846f
#define sinPolyCoef3 -1.666568107e-1f
#define sinPolyCoef5  8.312366210e-3f
#define sinPolyCoef7 -1.849218155e-4f
#define sinPolyCoef9  0

float sin_approx(float x)
{
    int32_t xint = x;
    if (xint < -32 || xint > 32) return 0.0f;                               // Stop here on error input (5 * 360 Deg)
    while (x >  M_PIf) x -= (2.0f * M_PIf);                                 // always wrap input angle to -PI..PI
    while (x < -M_PIf) x += (2.0f * M_PIf);
    if (x >  (0.5f * M_PIf)) x =  (0.5f * M_PIf) - (x - (0.5f * M_PIf));   // We just pick -90..+90 Degree
    else if (x < -(0.5f * M_PIf)) x = -(0.5f * M_PIf) - ((0.5f * M_PIf) + x);
    float x2 = x * x;
    return x + x * x2 * (sinPolyCoef3 + x2 * (sinPolyCoef5 + x2 * (sinPolyCoef7 + x2 * sinPolyCoef9)));
}

float cos_approx(float x)
{
    return sin_approx(x + (0.5f * M_PIf));
}
