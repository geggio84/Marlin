#ifndef PINS_H
#define PINS_H

#define GPIO(bank,num) ((bank*32)+num)

#define X_MS1_PIN -1
#define X_MS2_PIN -1
#define Y_MS1_PIN -1
#define Y_MS2_PIN -1
#define Z_MS1_PIN -1
#define Z_MS2_PIN -1
#define E0_MS1_PIN -1
#define E0_MS2_PIN -1
#define E1_MS1_PIN -1
#define E1_MS2_PIN -1
#define DIGIPOTSS_PIN -1

/****************************************************************************************
* BeagleBone Printer v1.0
*
****************************************************************************************/
#define X_STEP_PIN         GPIO(0,27)
#define X_DIR_PIN          GPIO(2,24)
#define X_STOP_PIN         GPIO(3,21)

#define Y_STEP_PIN         GPIO(1,12)
#define Y_DIR_PIN          GPIO(0,22)
#define Y_STOP_PIN         GPIO(1,17)

#define Z_STEP_PIN         GPIO(0,23)
#define Z_DIR_PIN          GPIO(0,26)
#define Z_STOP_PIN         GPIO(0,31)

#define E0_STEP_PIN         GPIO(1,28)
#define E0_DIR_PIN          GPIO(1,15)

#define LED_PIN            -1

#define FAN_PIN            -1
 #if FAN_PIN == 12 || FAN_PIN ==13
  #define FAN_SOFT_PWM
#endif

#ifdef NUM_SERVOS
  #define SERVO0_PIN          -1

  #if NUM_SERVOS > 1
    #define SERVO1_PIN        -1
  #endif

  #if NUM_SERVOS > 2
    #define SERVO2_PIN        -1
  #endif

  #if NUM_SERVOS > 3
    #define SERVO3_PIN        -1
  #endif
#endif

#define PS_ON_PIN          -1
#define KILL_PIN           -1

#define HEATER_0_PIN       13 // (extruder)
#define HEATER_1_PIN       -1
#define HEATER_2_PIN       -1

 #define HEATER_BED_PIN     12 // (bed)
 #define X_ENABLE_PIN       14
 #define Y_ENABLE_PIN       14
 #define Z_ENABLE_PIN       26
 #define E0_ENABLE_PIN      14

#define TEMP_0_PIN          7   // MUST USE ANALOG INPUT NUMBERING NOT DIGITAL OUTPUT NUMBERING!!!!!!!!! (pin 33 extruder)
#define TEMP_1_PIN         -1
#define TEMP_2_PIN         -1
#define TEMP_BED_PIN        6   // MUST USE ANALOG INPUT NUMBERING NOT DIGITAL OUTPUT NUMBERING!!!!!!!!! (pin 34 bed)
#define SDPOWER            -1
#define SDSS               31

//List of pins which to ignore when asked to change by gcode, 0 and 1 are RX and TX, do not mess with those!
#define _E0_PINS E0_STEP_PIN, E0_DIR_PIN, E0_ENABLE_PIN, HEATER_0_PIN,
#if EXTRUDERS > 1
  #define _E1_PINS E1_STEP_PIN, E1_DIR_PIN, E1_ENABLE_PIN, HEATER_1_PIN,
#else
  #define _E1_PINS
#endif
#if EXTRUDERS > 2
  #define _E2_PINS E2_STEP_PIN, E2_DIR_PIN, E2_ENABLE_PIN, HEATER_2_PIN,
#else
  #define _E2_PINS
#endif

#ifdef X_STOP_PIN
  #if X_HOME_DIR < 0
    #define X_MIN_PIN X_STOP_PIN
    #define X_MAX_PIN GPIO(0,3)
  #else
    #define X_MIN_PIN GPIO(0,3)
    #define X_MAX_PIN X_STOP_PIN
  #endif
#endif

#ifdef Y_STOP_PIN
  #if Y_HOME_DIR < 0
    #define Y_MIN_PIN Y_STOP_PIN
    #define Y_MAX_PIN GPIO(1,19)
  #else
    #define Y_MIN_PIN GPIO(1,19)
    #define Y_MAX_PIN Y_STOP_PIN
  #endif
#endif

#ifdef Z_STOP_PIN
  #if Z_HOME_DIR < 0
    #define Z_MIN_PIN Z_STOP_PIN
    #define Z_MAX_PIN GPIO(0,2)
  #else
    #define Z_MIN_PIN GPIO(0,2)
    #define Z_MAX_PIN Z_STOP_PIN
  #endif
#endif

#ifdef DISABLE_MAX_ENDSTOPS
#define X_MAX_PIN          -1
#define Y_MAX_PIN          -1
#define Z_MAX_PIN          -1
#endif

#ifdef DISABLE_MIN_ENDSTOPS
#define X_MIN_PIN          -1
#define Y_MIN_PIN          -1
#define Z_MIN_PIN          -1
#endif

#define SENSITIVE_PINS {0, 1, X_STEP_PIN, X_DIR_PIN, X_ENABLE_PIN, X_MIN_PIN, X_MAX_PIN, Y_STEP_PIN, Y_DIR_PIN, Y_ENABLE_PIN, Y_MIN_PIN, Y_MAX_PIN, Z_STEP_PIN, Z_DIR_PIN, Z_ENABLE_PIN, Z_MIN_PIN, Z_MAX_PIN, PS_ON_PIN, \
                        HEATER_BED_PIN, FAN_PIN,                  \
                        _E0_PINS _E1_PINS _E2_PINS             \
                        analogInputToDigitalPin(TEMP_0_PIN), analogInputToDigitalPin(TEMP_1_PIN), analogInputToDigitalPin(TEMP_2_PIN), analogInputToDigitalPin(TEMP_BED_PIN) }
#endif

