/*
  temperature.h - temperature controller
  Part of Marlin

  Copyright (c) 2011 Erik van der Zalm

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef temperature_h
#define temperature_h 

#include "Marlin.h"
#include "planner.h"
#ifdef PID_ADD_EXTRUSION_RATE
  #include "stepper.h"
#endif

// public functions
void tp_init();  //initialize the heating
void manage_heater(); //it is critical that this is called periodically.

// low level conversion routines
// do not use these routines and variables outside of temperature.cpp
//extern int target_temperature;
extern float current_temperature;
//#ifdef SHOW_TEMP_ADC_VALUES
//  extern int current_temperature_raw;
//  extern int current_temperature_bed_raw;
//#endif
//extern int target_temperature_bed;
extern float current_temperature_bed;

typedef struct {
	unsigned int soft_pwm;
	unsigned int soft_pwm_bed;
#ifdef FAN_SOFT_PWM
	unsigned int soft_pwm_fan;
	unsigned int fanSpeedSoftPwm;
#endif
	int temp_meas_ready;// = false;
	int current_temperature_raw;// = 0;
	int current_temperature_bed_raw;// = 0;
	int maxttemp_raw;// = HEATER_0_RAW_HI_TEMP;
	int minttemp_raw;// = HEATER_0_RAW_LO_TEMP;
	int bed_maxttemp_raw;// = HEATER_BED_RAW_HI_TEMP;
	int target_temperature_bed;// = 0;
	int target_temperature;// = 0;
	int Stopped;
	long Stopped_gcode_LastN;
	long gcode_LastN;
	float current_controller_temp;
} temp_struct_t;

extern temp_struct_t temp_struct;
extern temp_struct_t *TEMP_shm_addr;

//#if defined(CONTROLLERFAN_PIN)
//  extern unsigned char soft_pwm_bed;
//#endif

#ifdef PIDTEMP
  extern float Kp,Ki,Kd,Kc;
  float scalePID_i(float i);
  float scalePID_d(float d);
  float unscalePID_i(float i);
  float unscalePID_d(float d);

#endif
#ifdef PIDTEMPBED
  extern float bedKp,bedKi,bedKd;
#endif

//high level conversion routines, for use outside of temperature.cpp
//inline so that there is no performance decrease.
//deg=degreeCelsius
void temp_read_loop(void);
void temp_ISR();

FORCE_INLINE float degHotend() {
  return current_temperature;
};

#ifdef SHOW_TEMP_ADC_VALUES
  FORCE_INLINE float rawHotendTemp() {
    return TEMP_shm_addr->current_temperature_raw;
  };

  FORCE_INLINE float rawBedTemp() {
    return TEMP_shm_addr->current_temperature_bed_raw;
  };
#endif

FORCE_INLINE float degBed() {
  return current_temperature_bed;
};

FORCE_INLINE float degTargetHotend() {
  return TEMP_shm_addr->target_temperature;
};

FORCE_INLINE float degTargetBed() {
  return TEMP_shm_addr->target_temperature_bed;
};

FORCE_INLINE void setTargetHotend(const float &celsius) {
  TEMP_shm_addr->target_temperature = celsius;
};

FORCE_INLINE void setTargetBed(const float &celsius) {
  TEMP_shm_addr->target_temperature_bed = celsius;
};

FORCE_INLINE bool isHeatingHotend(){
  return TEMP_shm_addr->target_temperature > current_temperature;
};

FORCE_INLINE bool isHeatingBed() {
  return TEMP_shm_addr->target_temperature_bed > current_temperature_bed;
};

FORCE_INLINE bool isCoolingHotend() {
  return TEMP_shm_addr->target_temperature < current_temperature;
};

FORCE_INLINE bool isCoolingBed() {
  return TEMP_shm_addr->target_temperature_bed < current_temperature_bed;
};

int getHeaterPower(int heater);
void disable_heater();
void setWatch();
void updatePID();

#ifdef THERMAL_RUNAWAY_PROTECTION_PERIOD
#if THERMAL_RUNAWAY_PROTECTION_PERIOD > 0
void thermal_runaway_protection(int *state, unsigned long *timer, float temperature, float target_temperature, int heater_id, int period_seconds, int hysteresis_degc);
static int thermal_runaway_state_machine[3]; // = {0,0,0};
static unsigned long thermal_runaway_timer[3]; // = {0,0,0};
static bool thermal_runaway = false;
  #if TEMP_SENSOR_BED != 0
    static int thermal_runaway_bed_state_machine;
    static unsigned long thermal_runaway_bed_timer;
  #endif
#endif
#endif

FORCE_INLINE void autotempShutdown(){
 #ifdef AUTOTEMP
 if(autotemp_enabled)
 {
  autotemp_enabled=false;
  if(degTargetHotend()>autotemp_min)
    setTargetHotend(0);
 }
 #endif
}

void PID_autotune(float temp, int extruder, int ncycles);

#endif

