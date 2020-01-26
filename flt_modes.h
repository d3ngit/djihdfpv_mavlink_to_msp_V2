#if VEHICLE_TYPE == 0
////ArduPlane
const typedef enum{
        MANUAL        = 0,
        CIRCLE        = 1,
        STABILIZE     = 2,
        TRAINING      = 3,
        ACRO          = 4,
        FLY_BY_WIRE_A = 5,
        FLY_BY_WIRE_B = 6,
        CRUISE        = 7,
        AUTOTUNE      = 8,
        AUTO          = 10,
        RTL           = 11,
        LOITER        = 12,
        TAKEOFF       = 13,
        AVOID_ADSB    = 14,
        GUIDED        = 15,
        INITIALISING  = 16,
        QSTABILIZE    = 17,
        QHOVER        = 18,
        QLOITER       = 19,
        QLAND         = 20,
        QRTL          = 21,
        QAUTOTUNE     = 22,
        QACRO         = 23,
    }arduPlaneModes_e;

const char arduPlaneModeStr[24][15] PROGMEM =  {"MANUAL",
                                                "CIRCLE",
                                                "STABILIZE",
                                                "TRAINING",
                                                "ACRO",
                                                "FLY BY WIRE A",
                                                "FLY BY WIRE B",
                                                "CRUISE",
                                                "AUTOTUNE",
                                                "",
                                                "AUTO",
                                                "RTL",
                                                "LOITER",
                                                "TAKEOFF",
                                                "AVOID_ADSB",
                                                "GUIDED",
                                                "INITIALISING",
                                                "QSTABILIZE",
                                                "QHOVER",
                                                "QLOITER",
                                                "QLAND",
                                                "QRTL",
                                                "QAUTOTUNE",
                                                "QACRO"};
#endif
#if VEHICLE_TYPE == 1
//ArduCopter
const typedef enum{
        STABILIZE =     0,  // manual airframe angle with manual throttle
        ACRO =          1,  // manual body-frame angular rate with manual throttle
        ALT_HOLD =      2,  // manual airframe angle with automatic throttle
        AUTO =          3,  // fully automatic waypoint control using mission commands
        GUIDED =        4,  // fully automatic fly to coordinate or fly at velocity/direction using GCS immediate commands
        LOITER =        5,  // automatic horizontal acceleration with automatic throttle
        RTL =           6,  // automatic return to launching point
        CIRCLE =        7,  // automatic circular flight with automatic throttle
        LAND =          9,  // automatic landing with horizontal position control
        DRIFT =        11,  // semi-automous position, yaw and throttle control
        SPORT =        13,  // manual earth-frame angular rate control with manual throttle
        FLIP =         14,  // automatically flip the vehicle on the roll axis
        AUTOTUNE =     15,  // automatically tune the vehicle's roll and pitch gains
        POSHOLD =      16,  // automatic position hold with manual override, with automatic throttle
        BRAKE =        17,  // full-brake using inertial/GPS system, no pilot input
        THROW =        18,  // throw to launch mode using inertial/GPS system, no pilot input
        AVOID_ADSB =   19,  // automatic avoidance of obstacles in the macro scale - e.g. full-sized aircraft
        GUIDED_NOGPS = 20,  // guided mode but only accepts attitude and altitude
        SMART_RTL =    21,  // SMART_RTL returns to home by retracing its steps
        FLOWHOLD  =    22,  // FLOWHOLD holds position with optical flow without rangefinder
        FOLLOW    =    23,  // follow attempts to follow another vehicle or ground station
        ZIGZAG    =    24,  // ZIGZAG mode is able to fly in a zigzag manner with predefined point A and point B
        SYSTEMID  =    25,  // System ID mode produces automated system identification signals in the controllers
    }arduCopterModes_e;

const char arduCopterModeStr[26][15] PROGMEM = {"STABILIZE",
                                                "ACRO",
                                                "ALT_HOLD",
                                                "AUTO",
                                                "GUIDED",
                                                "LOITER",
                                                "RTL",
                                                "CIRCLE",
                                                "",
                                                "LAND",
                                                "",
                                                "DRIFT",
                                                "",
                                                "SPORT",
                                                "FLIP",
                                                "AUTOTUNE",
                                                "POSHOLD",
                                                "BRAKE",
                                                "THROW",
                                                "AVOID_ADSB",
                                                "GUIDED_NOGPS",
                                                "SMART_RTL",
                                                "FLOWHOLD",
                                                "FOLLOW",
                                                "ZIGZAG",
                                                "SYSTEMID"};

#endif
#if VEHICLE_TYPE == 2
//INAV Plane
const char inavPlaneModeStr[24][15] PROGMEM =    {"MANUAL",
                                                  "POSITION_HOLD",
                                                  "ANGLE",
                                                  "",
                                                  "ACRO",
                                                  "ALTITUDE_HOLD",
                                                  "",
                                                  "",
                                                  "",
                                                  "",
                                                  "MISSION",
                                                  "RTH",
                                                  "",
                                                  "",
                                                  "",
                                                  "LAUNCH",
                                                  "",
                                                  "",
                                                  "",
                                                  "",
                                                  "",
                                                  "",
                                                  ""};

const typedef enum{
        STABILIZE       = 2,
        RTL             = 11
}arduPlaneModes_e;
#endif
#if VEHICLE_TYPE == 3
//INAV Copter
const char inavCopterModeStr[26][15] PROGMEM =   {"ANGLE",
                                                  "ACRO",
                                                  "ALTITUDE_HOLD",
                                                  "MISSION",
                                                  "",
                                                  "",
                                                  "RTH",
                                                  "",
                                                  "",
                                                  "",
                                                  "",
                                                  "",
                                                  "",
                                                  "",
                                                  "",
                                                  "",
                                                  "POSITION_HOLD",
                                                  "",
                                                  "LAUNCH",
                                                  "",
                                                  "",
                                                  "",
                                                  "",
                                                  "",
                                                  "",
                                                  ""};

const typedef enum{
        STABILIZE       = 0,  // manual airframe angle with manual throttle
        RTL             = 6   // automatic return to launching point
}arduCopterModes_e;
#endif

const typedef enum {
        ARM_ACRO_BF = (1 << 0),
        STAB_BF     = (1 << 1),
        HOR_BF      = (1 << 2),
        HEAD_BF     = (1 << 3),
        FS_BF       = (1 << 4),
        RESC_BF     = (1 << 5)
} betaflightDJIModesMask_e;

//DJI supported flightModeFlags
// 0b00000001 acro/arm
// 0b00000010 stab
// 0b00000100 hor
// 0b00001000 head
// 0b00010000 !fs!
// 0b00100000 resc
// 0b01000000 acro
// 0b10000000 acro
