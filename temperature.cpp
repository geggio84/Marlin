/*
  temperature.c - temperature control
  Part of Marlin
  
 Copyright (C) 2011 Camiel Gubbels / Erik van der Zalm
 
 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.
 
 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.
 
 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 This firmware is a mashup between Sprinter and grbl.
  (https://github.com/kliment/Sprinter)
  (https://github.com/simen/grbl/tree)
 
 It has preliminary support for Matthew Roberts advance algorithm 
    http://reprap.org/pipermail/reprap-dev/2011-May/003323.html

 */


#include "Marlin.h"
#include "temperature.h"
#include "watchdog.h"

//===========================================================================
//=============================public variables============================
//===========================================================================
int target_temperature = 0;
int target_temperature_bed = 0;
int current_temperature_raw = 0;
float current_temperature = 0.0;
int current_temperature_bed_raw = 0;
float current_temperature_bed = 0.0;
#ifdef PIDTEMP
  float Kp=DEFAULT_Kp;
  float Ki=(DEFAULT_Ki*PID_dT);
  float Kd=(DEFAULT_Kd/PID_dT);
  #ifdef PID_ADD_EXTRUSION_RATE
    float Kc=DEFAULT_Kc;
  #endif
#endif //PIDTEMP

#ifdef PIDTEMPBED
  float bedKp=DEFAULT_bedKp;
  float bedKi=(DEFAULT_bedKi*PID_dT);
  float bedKd=(DEFAULT_bedKd/PID_dT);
#endif //PIDTEMPBED
  
#ifdef FAN_SOFT_PWM
  unsigned char fanSpeedSoftPwm;
#endif

unsigned char soft_pwm_bed;
  
#ifdef BABYSTEPPING
  volatile int babystepsTodo[3]={0,0,0};
#endif
  
//===========================================================================
//=============================private variables============================
//===========================================================================
static volatile bool temp_meas_ready = false;

#ifdef PIDTEMP
  //static cannot be external:
  static float temp_iState = 0;
  static float temp_dState = 0;
  static float pTerm;
  static float iTerm;
  static float dTerm;
  //int output;
  static float pid_error;
  static float temp_iState_min;
  static float temp_iState_max;
  // static float pid_input;
  // static float pid_output;
  static bool pid_reset;
#endif //PIDTEMP
#ifdef PIDTEMPBED
  //static cannot be external:
  static float temp_iState_bed = 0;
  static float temp_dState_bed = 0;
  static float pTerm_bed;
  static float iTerm_bed;
  static float dTerm_bed;
  //int output;
  static float pid_error_bed;
  static float temp_iState_min_bed;
  static float temp_iState_max_bed;
#else //PIDTEMPBED
	static unsigned long  previous_millis_bed_heater;
#endif //PIDTEMPBED
  static unsigned char soft_pwm;

#ifdef FAN_SOFT_PWM
  static unsigned char soft_pwm_fan;
#endif
#if defined(EXTRUDER_0_AUTO_FAN_PIN)
  static unsigned long extruder_autofan_last_check;
#endif  

// Init min and max temp with extreme values to prevent false errors during startup
static int minttemp_raw = HEATER_0_RAW_LO_TEMP;
static int maxttemp_raw = HEATER_0_RAW_HI_TEMP;
static int minttemp = 0;
static int maxttemp = 16383;
//static int bed_minttemp_raw = HEATER_BED_RAW_LO_TEMP; /* No bed mintemp error implemented?!? */
#ifdef BED_MAXTEMP
static int bed_maxttemp_raw = HEATER_BED_RAW_HI_TEMP;
#endif

static float analog2temp(int raw);
static float analog2tempBed(int raw);
static void updateTemperaturesFromRawValues();

#ifdef WATCH_TEMP_PERIOD
int watch_start_temp = 0;
unsigned long watchmillis = 0;
#endif //WATCH_TEMP_PERIOD

#ifndef SOFT_PWM_SCALE
#define SOFT_PWM_SCALE 0
#endif

//===========================================================================
//=============================   functions      ============================
//===========================================================================

void PID_autotune(float temp, int extruder, int ncycles)
{
/* TODO: FIXME */
/*  float input = 0.0;
  int cycles=0;
  bool heating = true;

  unsigned long temp_millis = millis();
  unsigned long t1=temp_millis;
  unsigned long t2=temp_millis;
  long t_high = 0;
  long t_low = 0;

  long bias, d;
  float Ku, Tu;
  float Kp, Ki, Kd;
  float max = 0, min = 10000;

  SERIAL_ECHOLN("PID Autotune start");
  
  disable_heater(); // switch off all heaters.

  if (extruder<0)
  {
     soft_pwm_bed = (MAX_BED_POWER)/2;
     bias = d = (MAX_BED_POWER)/2;
   }
   else
   {
     soft_pwm = (PID_MAX)/2;
     bias = d = (PID_MAX)/2;
  }




 for(;;) {

    if(temp_meas_ready == true) { // temp sample ready
      updateTemperaturesFromRawValues();

      input = (extruder<0)?current_temperature_bed:current_temperature;

      max=max(max,input);
      min=min(min,input);
      if(heating == true && input > temp) {
        if(millis() - t2 > 5000) { 
          heating=false;
          if (extruder<0)
            soft_pwm_bed = (bias - d) >> 1;
          else
            soft_pwm = (bias - d) >> 1;
          t1=millis();
          t_high=t1 - t2;
          max=temp;
        }
      }
      if(heating == false && input < temp) {
        if(millis() - t1 > 5000) {
          heating=true;
          t2=millis();
          t_low=t2 - t1;
          if(cycles > 0) {
            bias += (d*(t_high - t_low))/(t_low + t_high);
            bias = constrain(bias, 20 ,(extruder<0?(MAX_BED_POWER):(PID_MAX))-20);
            if(bias > (extruder<0?(MAX_BED_POWER):(PID_MAX))/2) d = (extruder<0?(MAX_BED_POWER):(PID_MAX)) - 1 - bias;
            else d = bias;

            SERIAL_PROTOCOLPGM(" bias: "); SERIAL_PROTOCOL(bias);
            SERIAL_PROTOCOLPGM(" d: "); SERIAL_PROTOCOL(d);
            SERIAL_PROTOCOLPGM(" min: "); SERIAL_PROTOCOL(min);
            SERIAL_PROTOCOLPGM(" max: "); SERIAL_PROTOCOLLN(max);
            if(cycles > 2) {
              Ku = (4.0*d)/(3.14159*(max-min)/2.0);
              Tu = ((float)(t_low + t_high)/1000.0);
              SERIAL_PROTOCOLPGM(" Ku: "); SERIAL_PROTOCOL(Ku);
              SERIAL_PROTOCOLPGM(" Tu: "); SERIAL_PROTOCOLLN(Tu);
              Kp = 0.6*Ku;
              Ki = 2*Kp/Tu;
              Kd = Kp*Tu/8;
              SERIAL_PROTOCOLLNPGM(" Classic PID ");
              SERIAL_PROTOCOLPGM(" Kp: "); SERIAL_PROTOCOLLN(Kp);
              SERIAL_PROTOCOLPGM(" Ki: "); SERIAL_PROTOCOLLN(Ki);
              SERIAL_PROTOCOLPGM(" Kd: "); SERIAL_PROTOCOLLN(Kd);
              
              //Kp = 0.33*Ku;
              //Ki = Kp/Tu;
              //Kd = Kp*Tu/3;
              //SERIAL_PROTOCOLLNPGM(" Some overshoot ");
              //SERIAL_PROTOCOLPGM(" Kp: "); SERIAL_PROTOCOLLN(Kp);
              //SERIAL_PROTOCOLPGM(" Ki: "); SERIAL_PROTOCOLLN(Ki);
              //SERIAL_PROTOCOLPGM(" Kd: "); SERIAL_PROTOCOLLN(Kd);
              //Kp = 0.2*Ku;
              //Ki = 2*Kp/Tu;
              //Kd = Kp*Tu/3;
              //SERIAL_PROTOCOLLNPGM(" No overshoot ");
              //SERIAL_PROTOCOLPGM(" Kp: "); SERIAL_PROTOCOLLN(Kp);
              //SERIAL_PROTOCOLPGM(" Ki: "); SERIAL_PROTOCOLLN(Ki);
              //SERIAL_PROTOCOLPGM(" Kd: "); SERIAL_PROTOCOLLN(Kd);
              
            }
          }
          if (extruder<0)
            soft_pwm_bed = (bias + d) >> 1;
          else
            soft_pwm = (bias + d) >> 1;
          cycles++;
          min=temp;
        }
      } 
    }
    if(input > (temp + 20)) {
      SERIAL_PROTOCOLLNPGM("PID Autotune failed! Temperature too high");
      return;
    }
    if(millis() - temp_millis > 2000) {
      int p;
      if (extruder<0){
        p=soft_pwm_bed;       
        SERIAL_PROTOCOLPGM("ok B:");
      }else{
        p=soft_pwm;
        SERIAL_PROTOCOLPGM("ok T:");
      }
			
      SERIAL_PROTOCOL(input);   
      SERIAL_PROTOCOLPGM(" @:");
      SERIAL_PROTOCOLLN(p);       

      temp_millis = millis();
    }
    if(((millis() - t1) + (millis() - t2)) > (10L*60L*1000L*2L)) {
      SERIAL_PROTOCOLLNPGM("PID Autotune failed! timeout");
      return;
    }
    if(cycles > ncycles) {
      SERIAL_PROTOCOLLNPGM("PID Autotune finished! Put the last Kp, Ki and Kd constants from above into Configuration.h");
      return;
    }
  }*/
/* TODO: FIXME */
}

void updatePID()
{
#ifdef PIDTEMP
     temp_iState_max = PID_INTEGRAL_DRIVE_MAX / Ki;
#endif
#ifdef PIDTEMPBED
  temp_iState_max_bed = PID_INTEGRAL_DRIVE_MAX / bedKi;
#endif
}
  
int getHeaterPower(int heater) {
	if (heater<0)
		return soft_pwm_bed;
  return soft_pwm;
}

#if defined(EXTRUDER_0_AUTO_FAN_PIN)

bool extruderfan_ON = false;

void checkExtruderAutoFans()
{
  // which fan pins need to be turned on?
  #if defined(EXTRUDER_0_AUTO_FAN_PIN)
    if ((current_temperature > EXTRUDER_AUTO_FAN_TEMPERATURE) && (extruderfan_ON == false)) {
		setPwmFrequency(EXTRUDER_0_AUTO_FAN_PIN, EXTRUDER_AUTO_FAN_SPEED);
		extruderfan_ON = true;
	}
    if ((current_temperature < EXTRUDER_AUTO_FAN_TEMPERATURE) && (extruderfan_ON == true)) {
		setPwmFrequency(EXTRUDER_0_AUTO_FAN_PIN, 0);
		extruderfan_ON = false;
	}
  #endif
}

#endif // any extruder auto fan pins set

void manage_heater()
{
#ifdef PIDTEMP
  float pid_input;
#endif
  float pid_output;

  if(temp_meas_ready != true)   //better readability
    return; 

  updateTemperaturesFromRawValues();

  #if defined(THERMAL_RUNAWAY_PROTECTION_PERIOD)&&(THERMAL_RUNAWAY_PROTECTION_PERIOD > 0)
    thermal_runaway_protection(&thermal_runaway_state_machine, &thermal_runaway_timer, current_temperature, target_temperature, THERMAL_RUNAWAY_PROTECTION_PERIOD, THERMAL_RUNAWAY_PROTECTION_HYSTERESIS);
  #endif

  #ifdef PIDTEMP
    pid_input = current_temperature;

    #ifndef PID_OPENLOOP
        pid_error = target_temperature - pid_input;
        if(pid_error > PID_FUNCTIONAL_RANGE) {
          pid_output = BANG_MAX;
          pid_reset = true;
        }
        else if(pid_error < -PID_FUNCTIONAL_RANGE || target_temperature == 0) {
          pid_output = 0;
          pid_reset = true;
        }
        else {
          if(pid_reset == true) {
            temp_iState = 0.0;
            pid_reset = false;
          }
          pTerm = Kp * pid_error;
          temp_iState += pid_error;
          temp_iState = constrain(temp_iState, temp_iState_min, temp_iState_max);
          iTerm = Ki * temp_iState;

          //K1 defined in Configuration.h in the PID settings
          #define K2 (1.0-K1)
          dTerm = (Kd * (pid_input - temp_dState))*K2 + (K1 * dTerm);
          pid_output = constrain(pTerm + iTerm - dTerm, 0, PID_MAX);
        }
        temp_dState = pid_input;
    #else 
          pid_output = constrain(target_temperature, 0, PID_MAX);
    #endif //PID_OPENLOOP
    #ifdef PID_DEBUG
    SERIAL_ECHO_START;
    SERIAL_ECHO(" PID_DEBUG ");
    SERIAL_ECHO(": Input ");
    SERIAL_ECHO(pid_input);
    SERIAL_ECHO(" Output ");
    SERIAL_ECHO(pid_output);
    SERIAL_ECHO(" pTerm ");
    SERIAL_ECHO(pTerm);
    SERIAL_ECHO(" iTerm ");
    SERIAL_ECHO(iTerm);
    SERIAL_ECHO(" dTerm ");
    SERIAL_ECHOLN(dTerm);  
    #endif //PID_DEBUG
  #else /* PID off */
    pid_output = 0;
    if(current_temperature < target_temperature) {
      pid_output = PID_MAX;
    }
  #endif

    // Check if temperature is within the correct range
    if((current_temperature > minttemp) && (current_temperature < maxttemp)) 
    {
      soft_pwm = (int)pid_output >> 1;
    }
    else {
      soft_pwm = 0;
    }

    #ifdef WATCH_TEMP_PERIOD
    if(watchmillis && millis() - watchmillis > WATCH_TEMP_PERIOD)
    {
        if(degHotend() < watch_start_temp + WATCH_TEMP_INCREASE)
        {
            setTargetHotend(0);
            SERIAL_ECHO_START;
            SERIAL_ECHOLN("Heating failed");
        }else{
            watchmillis = 0;
        }
    }
    #endif

  #if defined(EXTRUDER_0_AUTO_FAN_PIN)
  if(millis() - extruder_autofan_last_check > 2500)  // only need to check fan state very infrequently
  {
    checkExtruderAutoFans();
    extruder_autofan_last_check = millis();
  }  
  #endif       

  #ifndef PIDTEMPBED
  if(millis() - previous_millis_bed_heater < BED_CHECK_INTERVAL)
    return;
  previous_millis_bed_heater = millis();
  #endif

  #if TEMP_SENSOR_BED != 0
  
    #if defined(THERMAL_RUNAWAY_PROTECTION_PERIOD) && (THERMAL_RUNAWAY_PROTECTION_PERIOD > 0)
      thermal_runaway_protection(&thermal_runaway_bed_state_machine, &thermal_runaway_bed_timer, current_temperature_bed, target_temperature_bed, 9, THERMAL_RUNAWAY_PROTECTION_BED_PERIOD, THERMAL_RUNAWAY_PROTECTION_BED_HYSTERESIS);
    #endif

  #ifdef PIDTEMPBED
    pid_input = current_temperature_bed;

    #ifndef PID_OPENLOOP
		  pid_error_bed = target_temperature_bed - pid_input;
		  pTerm_bed = bedKp * pid_error_bed;
		  temp_iState_bed += pid_error_bed;
		  temp_iState_bed = constrain(temp_iState_bed, temp_iState_min_bed, temp_iState_max_bed);
		  iTerm_bed = bedKi * temp_iState_bed;

		  //K1 defined in Configuration.h in the PID settings
		  #define K2 (1.0-K1)
		  dTerm_bed= (bedKd * (pid_input - temp_dState_bed))*K2 + (K1 * dTerm_bed);
		  temp_dState_bed = pid_input;

		  pid_output = constrain(pTerm_bed + iTerm_bed - dTerm_bed, 0, MAX_BED_POWER);

    #else 
      pid_output = constrain(target_temperature_bed, 0, MAX_BED_POWER);
    #endif //PID_OPENLOOP

	  if((current_temperature_bed > BED_MINTEMP) && (current_temperature_bed < BED_MAXTEMP)) 
	  {
	    soft_pwm_bed = (int)pid_output >> 1;
	  }
	  else {
	    soft_pwm_bed = 0;
	  }

    #elif !defined(BED_LIMIT_SWITCHING)
      // Check if temperature is within the correct range
      if((current_temperature_bed > BED_MINTEMP) && (current_temperature_bed < BED_MAXTEMP))
      {
        if(current_temperature_bed >= target_temperature_bed)
        {
          soft_pwm_bed = 0;
        }
        else 
        {
          soft_pwm_bed = MAX_BED_POWER>>1;
        }
      }
      else
      {
        soft_pwm_bed = 0;
        setPwmFrequency(HEATER_BED_PIN,0);
      }
    #else //#ifdef BED_LIMIT_SWITCHING
      // Check if temperature is within the correct band
      if((current_temperature_bed > BED_MINTEMP) && (current_temperature_bed < BED_MAXTEMP))
      {
        if(current_temperature_bed > target_temperature_bed + BED_HYSTERESIS)
        {
          soft_pwm_bed = 0;
        }
        else if(current_temperature_bed <= target_temperature_bed - BED_HYSTERESIS)
        {
          soft_pwm_bed = MAX_BED_POWER>>1;
        }
      }
      else
      {
        soft_pwm_bed = 0;
        setPwmFrequency(HEATER_BED_PIN,0);
      }
    #endif
  #endif
}

// Derived from RepRap FiveD extruder::getTemperature()
// For hot end temperature measurement.
static float analog2temp(int raw) {

  float voltage = (raw / (4095.0 * OVERSAMPLENR))* 1.8;
  float res_val;
  float celsius = 0;
  uint16_t i;
  //""" Convert the voltage to a resistance value """
  if (fabs(voltage - 1.8) < 0.001)
	res_val = 10000000.0;
  else
	res_val = 4700 / ((1.8 / voltage) - 1.0);

    for (i=1; i<HEATER_0_TEMPTABLE_LEN; i++)
    {
      if (HEATER_0_TEMPTABLE[i][0] > res_val)
      {
        celsius  = HEATER_0_TEMPTABLE[i-1][1] + 
          (res_val - HEATER_0_TEMPTABLE[i-1][0]) * 
          (float)(HEATER_0_TEMPTABLE[i][1] - HEATER_0_TEMPTABLE[i-1][1]) /
          (float)(HEATER_0_TEMPTABLE[i][0] - HEATER_0_TEMPTABLE[i-1][0]);
        break;
      }
    }

    // Overflow: Set to last value in the table
    if (i == HEATER_0_TEMPTABLE_LEN) celsius = HEATER_0_TEMPTABLE[i-1][1];

    return celsius;
}

// Derived from RepRap FiveD extruder::getTemperature()
// For bed temperature measurement.
static float analog2tempBed(int raw) {
  #ifdef BED_USES_THERMISTOR
    float celsius = 0;
    uint16_t i;

  float voltage = (raw / (4095.0 * OVERSAMPLENR))* 1.8;
  float res_val;
  //""" Convert the voltage to a resistance value """
  if (fabs(voltage - 1.8) < 0.001)
	res_val = 10000000.0;
  else
	res_val = 4700 / ((1.8 / voltage) - 1.0);

    for (i=1; i<BEDTEMPTABLE_LEN; i++)
    {
      if (BEDTEMPTABLE[i][0] > res_val)
      {
        celsius  = BEDTEMPTABLE[i-1][1] + 
          (res_val - BEDTEMPTABLE[i-1][0]) * 
          (float)(BEDTEMPTABLE[i][1] - BEDTEMPTABLE[i-1][1]) /
          (float)(BEDTEMPTABLE[i][0] - BEDTEMPTABLE[i-1][0]);
        break;
      }
    }

    // Overflow: Set to last value in the table
    if (i == BEDTEMPTABLE_LEN) celsius = BEDTEMPTABLE[i-1][1];

    return celsius;
  #else
    return 0;
  #endif
}

/* Called to get the raw values into the the actual temperatures. The raw values are created in interrupt context,
    and this function is called from normal context as it is too slow to run in interrupts and will block the stepper routine otherwise */
static void updateTemperaturesFromRawValues()
{
    current_temperature = analog2temp(current_temperature_raw);
    current_temperature_bed = analog2tempBed(current_temperature_bed_raw);
    //Reset the watchdog after we know we have a temperature measurement.
    watchdog_reset();

    CRITICAL_SECTION_START;

    temp_meas_ready = false;

    CRITICAL_SECTION_END;
}

void tp_init()
{
    // populate with the first value
#ifdef PIDTEMP
    temp_iState_min = 0.0;
    temp_iState_max = PID_INTEGRAL_DRIVE_MAX / Ki;
#endif //PIDTEMP
#ifdef PIDTEMPBED
    temp_iState_min_bed = 0.0;
    temp_iState_max_bed = PID_INTEGRAL_DRIVE_MAX / bedKi;
#endif //PIDTEMPBED

  #if defined(HEATER_0_PIN)
    setPwmFrequency(HEATER_0_PIN, 0);
  #endif  
  #if defined(HEATER_BED_PIN)
    setPwmFrequency(HEATER_BED_PIN, 0);
  #endif  
  #if defined(FAN_PIN) && (FAN_PIN > -1) 
   setPwmFrequency(FAN_PIN, 0);
    #ifdef FAN_SOFT_PWM
    soft_pwm_fan = fanSpeedSoftPwm / 2;
    #endif
  #endif  

  // Set analog inputs
/* TODO: FIXME */
//  ADCSRA = 1<<ADEN | 1<<ADSC | 1<<ADIF | 0x07;
//  DIDR0 = 0;
/* TODO: FIXME */
//  #ifdef DIDR2
//    DIDR2 = 0;
//  #endif
//  #if defined(TEMP_0_PIN) && (TEMP_0_PIN > -1)
//    #if TEMP_0_PIN < 8
///* TODO: FIXME */
//       //DIDR0 |= 1 << TEMP_0_PIN; 
///* TODO: FIXME */
//    #else
//       DIDR2 |= 1<<(TEMP_0_PIN - 8); 
//    #endif
//  #endif
//  #if defined(TEMP_1_PIN) && (TEMP_1_PIN > -1)
//    #if TEMP_1_PIN < 8
//       DIDR0 |= 1<<TEMP_1_PIN; 
//    #else
//       DIDR2 |= 1<<(TEMP_1_PIN - 8); 
//    #endif
//  #endif
//  #if defined(TEMP_BED_PIN) && (TEMP_BED_PIN > -1)
//    #if TEMP_BED_PIN < 8
///* TODO: FIXME */
//       //DIDR0 |= 1<<TEMP_BED_PIN; 
///* TODO: FIXME */
//    #else
//       DIDR2 |= 1<<(TEMP_BED_PIN - 8); 
//    #endif
//  #endif
  
  // Use timer0 for temperature measurement
  // Interleave temperature interrupt with millies interrupt
/* TODO: FIXME */
  //OCR0B = 128;
  //TIMSK0 |= (1<<OCIE0B);  
  
  // Wait for temperature measurement to settle
  //delay(250);
/* TODO: FIXME */

#ifdef HEATER_0_MINTEMP
  minttemp = HEATER_0_MINTEMP;
  while(analog2temp(minttemp_raw) < HEATER_0_MINTEMP) {
#if HEATER_0_RAW_LO_TEMP < HEATER_0_RAW_HI_TEMP
    minttemp_raw += OVERSAMPLENR;
#else
    minttemp_raw -= OVERSAMPLENR;
#endif
  }
#endif //MINTEMP
#ifdef HEATER_0_MAXTEMP
  maxttemp = HEATER_0_MAXTEMP;
  while(analog2temp(maxttemp_raw) > HEATER_0_MAXTEMP) {
#if HEATER_0_RAW_LO_TEMP < HEATER_0_RAW_HI_TEMP
    maxttemp_raw -= OVERSAMPLENR;
#else
    maxttemp_raw += OVERSAMPLENR;
#endif
  }
#endif //MAXTEMP

#ifdef BED_MINTEMP
  /* No bed MINTEMP error implemented?!? */ /*
  while(analog2tempBed(bed_minttemp_raw) < BED_MINTEMP) {
#if HEATER_BED_RAW_LO_TEMP < HEATER_BED_RAW_HI_TEMP
    bed_minttemp_raw += OVERSAMPLENR;
#else
    bed_minttemp_raw -= OVERSAMPLENR;
#endif
  }
  */
#endif //BED_MINTEMP
#ifdef BED_MAXTEMP
  while(analog2tempBed(bed_maxttemp_raw) > BED_MAXTEMP) {
#if HEATER_BED_RAW_LO_TEMP < HEATER_BED_RAW_HI_TEMP
    bed_maxttemp_raw -= OVERSAMPLENR;
#else
    bed_maxttemp_raw += OVERSAMPLENR;
#endif
  }
#endif //BED_MAXTEMP
}

void setWatch() 
{  
#ifdef WATCH_TEMP_PERIOD
    if(degHotend() < degTargetHotend() - (WATCH_TEMP_INCREASE * 2))
    {
      watch_start_temp = degHotend();
      watchmillis = millis();
    }
#endif
}

#if defined(THERMAL_RUNAWAY_PROTECTION_PERIOD)&&(THERMAL_RUNAWAY_PROTECTION_PERIOD > 0)
void thermal_runaway_protection(int *state, unsigned long *timer, float temperature, float target_temperature, int heater_id, int period_seconds, int hysteresis_degc)
{
/*
      SERIAL_ECHO_START;
      SERIAL_ECHO("Thermal Thermal Runaway Running. Heater ID:");
      SERIAL_ECHO(heater_id);
      SERIAL_ECHO(" ;  State:");
      SERIAL_ECHO(*state);
      SERIAL_ECHO(" ;  Timer:");
      SERIAL_ECHO(*timer);
      SERIAL_ECHO(" ;  Temperature:");
      SERIAL_ECHO(temperature);
      SERIAL_ECHO(" ;  Target Temp:");
      SERIAL_ECHO(target_temperature);
      SERIAL_ECHOLN("");    
*/
  if ((target_temperature == 0) || thermal_runaway)
  {
    *state = 0;
    *timer = 0;
    return;
  }
  switch (*state)
  {
    case 0: // "Heater Inactive" state
      if (target_temperature > 0) *state = 1;
      break;
    case 1: // "First Heating" state
      if (temperature >= target_temperature) *state = 2;
      break;
    case 2: // "Temperature Stable" state
      if (temperature >= (target_temperature - hysteresis_degc))
      {
        *timer = millis();
      } 
      else if ( (millis() - *timer) > period_seconds*1000)
      {
        SERIAL_ERROR_START;
        SERIAL_ERRORLNPGM("Thermal Runaway, system stopped! Heater_ID: ");
        SERIAL_ERRORLN((int)heater_id);
        thermal_runaway = true;
        while(1)
        {
          disable_heater();
          disable_x();
          disable_y();
          disable_z();
          disable_e0();
          disable_e1();
          manage_heater();
        }
      }
      break;
  }
}
#endif

void disable_heater()
{
  setTargetHotend(0);
  setTargetBed(0);
  #if defined(TEMP_0_PIN) && TEMP_0_PIN > -1
  target_temperature=0;
  soft_pwm=0;
   #if defined(HEATER_0_PIN)
     setPwmFrequency(HEATER_0_PIN,0);
   #endif
  #endif

  #if defined(TEMP_BED_PIN) && TEMP_BED_PIN > -1
    target_temperature_bed=0;
    soft_pwm_bed=0;
    #if defined(HEATER_BED_PIN)
      setPwmFrequency(HEATER_BED_PIN,0);
    #endif
  #endif 
}

void max_temp_error() {
  disable_heater();
  if(IsStopped() == false) {
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM(": Extruder switched off. MAXTEMP triggered !");
  }
  #ifndef BOGUS_TEMPERATURE_FAILSAFE_OVERRIDE
  Stop();
  #endif
}

void min_temp_error() {
  disable_heater();
  if(IsStopped() == false) {
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM(": Extruder switched off. MINTEMP triggered !");
  }
  #ifndef BOGUS_TEMPERATURE_FAILSAFE_OVERRIDE
  Stop();
  #endif
}

void bed_max_temp_error(void) {

#if defined (HEATER_BED_PIN)
  setPwmFrequency(HEATER_BED_PIN, 0);
#endif

  if(IsStopped() == false) {
    SERIAL_ERROR_START;
    SERIAL_ERRORLNPGM("Temperature heated bed switched off. MAXTEMP triggered !!");
  }
  #ifndef BOGUS_TEMPERATURE_FAILSAFE_OVERRIDE
  Stop();
  #endif
}

// Timer 0 is shared with millies
/* TODO: FIXME */
void temp_ISR(void)
{
  //these variables are only accesible from the ISR, but static, so they don't lose their value
  static unsigned char temp_count = 0;
  static unsigned long raw_temp_0_value = 0;
  static unsigned long raw_temp_1_value = 0;
  static unsigned long raw_temp_bed_value = 0;
  static unsigned char soft_pwm_0;
  #if defined(HEATER_BED_PIN)
  static unsigned char soft_pwm_b;
  #endif
  
  if(soft_pwm_0 != soft_pwm){
    soft_pwm_0 = soft_pwm;
    setPwmFrequency(HEATER_0_PIN,soft_pwm);
  }

#if defined(HEATER_BED_PIN)
  if(soft_pwm_b != soft_pwm_bed){
    soft_pwm_b = soft_pwm_bed;
    setPwmFrequency(HEATER_BED_PIN,soft_pwm_bed);
  }
#endif

    #ifdef FAN_SOFT_PWM
    soft_pwm_fan = fanSpeedSoftPwm / 2;
    if(soft_pwm_fan > 0) setPwmFrequency(FAN_PIN,soft_pwm_fan);
    #endif

  char buf[256];
  int fd;
  char value[4];

	// Measure TEMP_0
	#if defined(TEMP_0_PIN) && (TEMP_0_PIN > -1)
		sprintf(buf,"/sys/bus/iio/devices/iio:device0/in_voltage%d_raw",TEMP_0_PIN);
		fd = open(buf, O_RDONLY);
		read(fd, &value, 4);
		close(fd);
		raw_temp_0_value += atoi(value);
	#endif
	// Measure TEMP_BED
	#if defined(TEMP_BED_PIN) && (TEMP_BED_PIN > -1)
		sprintf(buf,"/sys/bus/iio/devices/iio:device0/in_voltage%d_raw",TEMP_BED_PIN);
		fd = open(buf, O_RDONLY);
		read(fd, &value, 4);
		close(fd);
		raw_temp_bed_value += atoi(value);
	#endif
	// Measure TEMP_1
	#if defined(TEMP_1_PIN) && (TEMP_1_PIN > -1)
		sprintf(buf,"/sys/bus/iio/devices/iio:device0/in_voltage%d_raw",TEMP_1_PIN);
		fd = open(buf, O_RDONLY);
		read(fd, &value, 4);
		close(fd);
		raw_temp_1_value += atoi(value);
	#endif

	temp_count++;

  if(temp_count >= OVERSAMPLENR) // 8 * 16 * 1/(16000000/64/256)  = 131ms.
  {
    if (!temp_meas_ready) //Only update the raw values if they have been read. Else we could be updating them during reading.
    {
      current_temperature_raw = raw_temp_0_value;
      current_temperature_bed_raw = raw_temp_bed_value;
    }
    
    temp_meas_ready = true;
    temp_count = 0;
    raw_temp_0_value = 0;
    raw_temp_1_value = 0;
    raw_temp_bed_value = 0;

#if HEATER_0_RAW_LO_TEMP > HEATER_0_RAW_HI_TEMP
    if(current_temperature_raw <= maxttemp_raw) {
#else
    if(current_temperature_raw >= maxttemp_raw) {
#endif
        max_temp_error();
    }
#if HEATER_0_RAW_LO_TEMP > HEATER_0_RAW_HI_TEMP
    if(current_temperature_raw >= minttemp_raw) {
#else
    if(current_temperature_raw <= minttemp_raw) {
#endif
        min_temp_error();
    }

  // No bed MINTEMP error? 
#if defined(BED_MAXTEMP) && (TEMP_SENSOR_BED != 0)
# if HEATER_BED_RAW_LO_TEMP > HEATER_BED_RAW_HI_TEMP
    if(current_temperature_bed_raw <= bed_maxttemp_raw) {
#else
    if(current_temperature_bed_raw >= bed_maxttemp_raw) {
#endif
       target_temperature_bed = 0;
       bed_max_temp_error();
    }
#endif
  }
  
#ifdef BABYSTEPPING
  for(uint8_t axis=0;axis<3;axis++)
  {
    int curTodo=babystepsTodo[axis]; //get rid of volatile for performance
   
    if(curTodo>0)
    {
      babystep(axis, //fwd// true);
      babystepsTodo[axis]--; //less to do next time
    }
    else
    if(curTodo<0)
    {
      babystep(axis, //fwd// false);
      babystepsTodo[axis]++; //less to do next time
    }
  }
#endif //BABYSTEPPING
}

#ifdef PIDTEMP
// Apply the scale factors to the PID values


float scalePID_i(float i)
{
	return i*PID_dT;
}

float unscalePID_i(float i)
{
	return i/PID_dT;
}

float scalePID_d(float d)
{
    return d/PID_dT;
}

float unscalePID_d(float d)
{
	return d*PID_dT;
}

#endif //PIDTEMP


