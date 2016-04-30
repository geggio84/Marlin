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
#define X_STEP_PIN         14 // pr1_pru0_pru_r30[14]
#define X_DIR_PIN           7 // pr1_pru0_pru_r30[7]
#define X_MIN_PIN          GPIO(2,2)
#define X_MAX_PIN          GPIO(2,3)

#define Y_STEP_PIN         15 // pr1_pru0_pru_r30[15]
#define Y_DIR_PIN           0 // pr1_pru0_pru_r30[0]
#define Y_MIN_PIN          GPIO(2,5)
#define Y_MAX_PIN          GPIO(2,4)

#define Z_STEP_PIN          3 // pr1_pru0_pru_r30[3]
#define Z_DIR_PIN           6 // pr1_pru0_pru_r30[6]
#define Z_MIN_PIN          GPIO(0,23)
#define Z_MAX_PIN          GPIO(0,26)

#define E0_STEP_PIN         2 // pr1_pru0_pru_r30[2]
#define E0_DIR_PIN          1 // pr1_pru0_pru_r30[1]

#define STEPPER_ENABLEn_PIN GPIO(1,15)
#define STEPPER_FLAG_PIN    GPIO(1,14)

#define PWM_ENABLE_PIN		GPIO(0,27)

#define EXTR_LED_PIN            0
#define BED_LED_PIN            2

#define FAN_PIN            -1

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

// HEATERS PWM nr.
#define HEATER_0_PIN       "extruder" // HEATER_EXT (extruder)
#define HEATER_BED_PIN     "hotbed" // HEATER_HPB (bed)

#define TEMP_0_PIN          5
#define TEMP_1_PIN          -1//4
#define TEMP_BED_PIN        6
#define SDPOWER            -1
#define SDSS               -1//31

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

#define SENSITIVE_PINS {0, 1, X_MIN_PIN, X_MAX_PIN, Y_MIN_PIN, Y_MAX_PIN, Z_MIN_PIN, Z_MAX_PIN, PS_ON_PIN }

#endif

