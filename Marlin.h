// Tonokip RepRap firmware rewrite based off of Hydra-mmm firmware.
// License: GPL

#ifndef MARLIN_H
#define MARLIN_H

#define  FORCE_INLINE __attribute__((always_inline)) inline

#include <math.h>
#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include <sys/time.h>
#include <sys/wait.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <unistd.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>
#include <termios.h>
#include <fcntl.h>
#include <signal.h>
#include "marlin_types.h"

#include "common_pru.h"
#include "fastio.h"
#include "Configuration.h"
#include "pins.h"

#define HIGH 0x1
#define LOW  0x0

#define INPUT 0x0
#define OUTPUT 0x1
#define INPUT_PULLUP 0x2

#define constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))
#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))

typedef struct {
    FILE *file_p;
    uint32_t currpos;
    uint32_t size;
} myFILE;

#include "MarlinSerial.h"

void pinMode(uint8_t, uint8_t);
void digitalWrite(uint8_t, uint8_t);
int digitalRead(uint8_t);
int analogRead(uint8_t);
void analogWrite(uint8_t, int);
void marlin_main_kill(int);
void stepper_wait_kill(int);
void temp_read_kill(int);

int loop();
int get_command();
void process_commands();
unsigned long millis(void);

void manage_inactivity();


#define  enable_x() { if(steppers[X_AXIS].enabled == false) easySPIN_Enable(&steppers[X_AXIS].spi_device); steppers[X_AXIS].enabled = true; }
#define disable_x() { if(steppers[X_AXIS].enabled == true) easySPIN_Disable(&steppers[X_AXIS].spi_device); axis_known_position[X_AXIS] = false; steppers[X_AXIS].enabled = false; }

#define  enable_y() { if(steppers[Y_AXIS].enabled == false) easySPIN_Enable(&steppers[Y_AXIS].spi_device); steppers[Y_AXIS].enabled = true; }
#define disable_y() { if(steppers[Y_AXIS].enabled == true) easySPIN_Disable(&steppers[Y_AXIS].spi_device); axis_known_position[Y_AXIS] = false; steppers[Y_AXIS].enabled = false; }

#define  enable_z() { if(steppers[Z_AXIS].enabled == false) easySPIN_Enable(&steppers[Z_AXIS].spi_device); steppers[Z_AXIS].enabled = true; }
#define disable_z() { if(steppers[Z_AXIS].enabled == true) easySPIN_Disable(&steppers[Z_AXIS].spi_device); axis_known_position[Z_AXIS] = false; steppers[Z_AXIS].enabled = false; }

#define  enable_e0() { if(steppers[E_AXIS].enabled == false) easySPIN_Enable(&steppers[E_AXIS].spi_device); steppers[E_AXIS].enabled = true; }
#define disable_e0() { if(steppers[E_AXIS].enabled == true) easySPIN_Disable(&steppers[E_AXIS].spi_device); steppers[E_AXIS].enabled = false; }

void FlushSerialRequestResend();
void ClearToSend();

void get_coordinates();
void prepare_move();
void marlin_kill();
void Stop();

bool IsStopped();

void enquecommand(const char *cmd); //put an ASCII command at the end of the current buffer.
void enquecommand_P(const char *cmd); //put an ASCII command at the end of the current buffer, read from flash
void prepare_arc_move(char isclockwise);
void clamp_to_software_endstops(float target[3]);

void refresh_cmd_timeout(void);

void setPwmFrequency(const char *pin, int val);

#ifndef CRITICAL_SECTION_START
  #define CRITICAL_SECTION_START
  #define CRITICAL_SECTION_END
#endif //CRITICAL_SECTION_START

extern float homing_feedrate[];
extern bool axis_relative_modes[];
extern int feedmultiply;
extern int extrudemultiply; // Sets extrude multiply factor (in percent) for all extruders
extern int extruder_multiply; // sets extrude multiply factor (in percent) for each extruder individually
extern float volumetric_multiplier; // reciprocal of cross-sectional area of filament (in square millimeters), stored this way to reduce computational burden in planner
extern float current_position[NUM_AXIS] ;
extern float add_homeing[3];
extern float min_pos[3];
extern float max_pos[3];
extern bool axis_known_position[3];
extern float zprobe_zoffset;
extern int fanSpeed;
extern easySPIN_stepper steppers[4];
extern int pru_file;
extern unsigned int *check_endstops;
extern long *count_position[NUM_AXIS];
extern bool step_debug_en;

#define PRUSS_RAM2_OFFSET 0x4A310000
#define SHM_FILE "/shared_mem"

//#ifdef FAN_SOFT_PWM
//extern unsigned char fanSpeedSoftPwm;
//#endif

#ifdef FWRETRACT
extern bool autoretract_enabled;
extern bool retracted;
extern float retract_length, retract_length_swap, retract_feedrate, retract_zlift;
extern float retract_recover_length, retract_recover_length_swap, retract_recover_feedrate;
#endif

extern unsigned long starttime;
extern unsigned long stoptime;

#ifdef DIGIPOT_I2C
extern void digipot_i2c_set_current( int channel, float current );
extern void digipot_i2c_init();
#endif

#endif
