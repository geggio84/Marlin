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
#define X_STEP_PIN         GPIO(1,12)
#define X_DIR_PIN          GPIO(3,21)
#define X_MIN_PIN          GPIO(2,2)
#define X_MAX_PIN          GPIO(2,3)

#define Y_STEP_PIN         GPIO(1,13)
#define Y_DIR_PIN          GPIO(3,14)
#define Y_MIN_PIN          GPIO(2,5)
#define Y_MAX_PIN          GPIO(2,4)

#define Z_STEP_PIN         GPIO(3,17)
#define Z_DIR_PIN          GPIO(0,20)
#define Z_MIN_PIN          GPIO(0,23)
#define Z_MAX_PIN          GPIO(0,26)

#define E0_STEP_PIN         GPIO(3,16)
#define E0_DIR_PIN          GPIO(3,15)

#define STEPPER_ENABLEn_PIN GPIO(1,15)
#define STEPPER_FLAG_PIN    GPIO(1,14)

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

#define HEATER_0_PIN       -1//13 // (extruder)

 #define HEATER_BED_PIN     -1//12 // (bed)

#define TEMP_0_PIN          5
#define TEMP_1_PIN          -1//4
#define TEMP_BED_PIN        6
#define SDPOWER            -1
#define SDSS               -1//31

//List of pins which to ignore when asked to change by gcode, 0 and 1 are RX and TX, do not mess with those!
#define _E0_PINS E0_STEP_PIN, E0_DIR_PIN, HEATER_0_PIN,

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

#define SENSITIVE_PINS {0, 1, X_STEP_PIN, X_DIR_PIN, X_MIN_PIN, X_MAX_PIN, Y_STEP_PIN, Y_DIR_PIN, Y_MIN_PIN, Y_MAX_PIN, Z_STEP_PIN, Z_DIR_PIN, Z_MIN_PIN, Z_MAX_PIN, PS_ON_PIN, \
                        HEATER_BED_PIN, FAN_PIN, _E0_PINS \
                        TEMP_0_PIN, TEMP_1_PIN, TEMP_BED_PIN }
#endif

