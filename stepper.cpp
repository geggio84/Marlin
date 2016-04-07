/*
  stepper.c - stepper motor driver: executes motion plans using stepper motors
  Part of Grbl

  Copyright (c) 2009-2011 Simen Svale Skogsrud

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

/* The timer calculations of this module informed by the 'RepRap cartesian firmware' by Zack Smith
   and Philipp Tiefenbacher. */

#include "easyspin.h"
#include "Marlin.h"
#include "stepper.h"
#include "planner.h"
#include "temperature.h"
#include "language.h"
#include "cardreader.h"
#include "speed_lookuptable.h"
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#if defined(DIGIPOTSS_PIN) && DIGIPOTSS_PIN > -1
#include <SPI.h>
#endif

//===========================================================================
//=============================public variables  ============================
//===========================================================================
block_t *current_block;  // A pointer to the block currently being traced


//===========================================================================
//=============================private variables ============================
//===========================================================================
//static makes it inpossible to be called from outside of this file by extern.!

#ifdef ADVANCE
  static long advance_rate, advance, final_advance = 0;
  static long old_advance = 0;
  static long e_steps;
#endif

volatile long endstops_trigsteps[3]={0,0,0};
volatile long endstops_stepsTotal,endstops_stepsDone;
static volatile bool endstop_x_hit=false;
static volatile bool endstop_y_hit=false;
static volatile bool endstop_z_hit=false;
#ifdef ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED
bool abort_on_endstop_hit = false;
#endif
#ifdef MOTOR_CURRENT_PWM_XY_PIN
  int motor_current_setting[3] = DEFAULT_PWM_MOTOR_CURRENT;
#endif

static bool check_endstops = true;

volatile long count_position[NUM_AXIS] = { 0, 0, 0, 0};
volatile signed char count_direction[NUM_AXIS] = { 1, 1, 1, 1};

easySPIN_stepper steppers[4] = { 0, 0, 0, 0};
uint32_t easySPIN_rx_data = 0;

//===========================================================================
//=============================functions         ============================
//===========================================================================

#define CHECK_ENDSTOPS  if(check_endstops)

// Some useful constants

#define ENABLE_STEPPER_DRIVER_INTERRUPT()  do{}while(0)//TIMSK1 |= (1<<OCIE1A)
#define DISABLE_STEPPER_DRIVER_INTERRUPT() do{}while(0)//TIMSK1 &= ~(1<<OCIE1A)


void checkHitEndstops()
{
 if( endstop_x_hit || endstop_y_hit || endstop_z_hit) {
   SERIAL_ECHO_START;
   SERIAL_ECHOPGM(MSG_ENDSTOPS_HIT);
   if(endstop_x_hit) {
     SERIAL_ECHOPAIR(" X:",(float)endstops_trigsteps[X_AXIS]/axis_steps_per_unit[X_AXIS]);
   }
   if(endstop_y_hit) {
     SERIAL_ECHOPAIR(" Y:",(float)endstops_trigsteps[Y_AXIS]/axis_steps_per_unit[Y_AXIS]);
   }
   if(endstop_z_hit) {
     SERIAL_ECHOPAIR(" Z:",(float)endstops_trigsteps[Z_AXIS]/axis_steps_per_unit[Z_AXIS]);
   }
   SERIAL_ECHOLN("");
   endstop_x_hit=false;
   endstop_y_hit=false;
   endstop_z_hit=false;
#if defined(ABORT_ON_ENDSTOP_HIT_FEATURE_ENABLED) && defined(SDSUPPORT)
   if (abort_on_endstop_hit)
   {
     card.sdprinting = false;
     card.closefile();
     quickStop();
     setTargetHotend(0);
   }
#endif
 }
}

void endstops_hit_on_purpose()
{
  endstop_x_hit=false;
  endstop_y_hit=false;
  endstop_z_hit=false;
}

void enable_endstops(bool check)
{
  check_endstops = check;
}

//         __________________________
//        /|                        |\     _________________         ^
//       / |                        | \   /|               |\        |
//      /  |                        |  \ / |               | \       s
//     /   |                        |   |  |               |  \      p
//    /    |                        |   |  |               |   \     e
//   +-----+------------------------+---+--+---------------+----+    e
//   |               BLOCK 1            |      BLOCK 2          |    d
//
//                           time ----->
//
//  The trapezoid is the shape the speed curve over time. It starts at block->initial_rate, accelerates
//  first block->accelerate_until step_events_completed, then keeps going at constant speed until
//  step_events_completed reaches block->decelerate_after after which it decelerates until the trapezoid generator is reset.
//  The slope of acceleration is calculated with the leib ramp alghorithm.

void st_wake_up() {
  //  TCNT1 = 0;
  ENABLE_STEPPER_DRIVER_INTERRUPT();
}

void debug_current_block(block_t* block)
{
	printf("*** CURRENT BLOCK (%d bytes) ***\n",sizeof(block_buffer[0]));
	// Step count along each axis
	printf("- steps_x = %ld (%d bytes)\n",block->steps_x, sizeof(block->steps_x));
	printf("- steps_y = %ld (%d bytes)\n",block->steps_y, sizeof(block->steps_y));
	printf("- steps_z = %ld (%d bytes)\n",block->steps_z, sizeof(block->steps_z));
	printf("- steps_e = %ld (%d bytes)\n",block->steps_e, sizeof(block->steps_e));
	// The number of step events required to complete this block
	printf("- step_event_count = %lu (%d bytes)\n",block->step_event_count, sizeof(block->step_event_count));
	// The index of the step event on which to stop acceleration
	printf("- accelerate_until = %ld (%d bytes)\n",block->accelerate_until, sizeof(block->accelerate_until));
	// The index of the step event on which to start decelerating
	printf("- decelerate_after = %ld (%d bytes)\n",block->decelerate_after, sizeof(block->decelerate_after));
	// The acceleration rate used for acceleration calculation
	printf("- acceleration_rate = %ld (%d bytes)\n",block->acceleration_rate, sizeof(block->acceleration_rate));
	// The direction bit set for this block (refers to *_DIRECTION_BIT in config.h)
	printf("- direction_bits = %d (%d bytes)\n",block->direction_bits, sizeof(block->direction_bits));
	// The nominal speed for this block in mm/sec
	printf("- nominal_speed = %f mm/sec (%d bytes)\n",block->nominal_speed, sizeof(block->nominal_speed));
	// Entry speed at previous-current junction in mm/sec
	printf("- entry_speed = %f mm/sec (%d bytes)\n",block->entry_speed, sizeof(block->entry_speed));
	// Maximum allowable junction entry speed in mm/sec
	printf("- max_entry_speed = %f mm/sec (%d bytes)\n",block->max_entry_speed, sizeof(block->max_entry_speed));
	// The total travel of this block in mm
	printf("- millimeters = %f mm (%d bytes)\n",block->millimeters, sizeof(block->millimeters));
	// acceleration mm/sec^2
	printf("- acceleration = %f mm/sec^2 (%d bytes)\n",block->acceleration, sizeof(block->acceleration));
	// Planner flag to recalculate trapezoids on entry junction
	printf("- recalculate_flag = %d (%d bytes)\n",block->recalculate_flag, sizeof(block->recalculate_flag));
	// Planner flag for nominal speed always reached
	printf("- nominal_length_flag = %d (%d bytes)\n",block->nominal_length_flag, sizeof(block->nominal_length_flag));
	// The nominal step rate for this block in step_events/sec
	printf("- nominal_rate = %lu step_events/sec (%d bytes)\n",block->nominal_rate, sizeof(block->nominal_rate));
	// The jerk-adjusted step rate at start of block
	printf("- initial_rate = %lu (%d bytes)\n",block->initial_rate, sizeof(block->initial_rate));
	// The minimal rate at exit
	printf("- final_rate = %lu (%d bytes)\n",block->final_rate, sizeof(block->final_rate));
	// acceleration steps/sec^2
	printf("- acceleration_st = %lu steps/sec^2 (%d bytes)\n",block->acceleration_st, sizeof(block->acceleration_st));
	printf("- fan_speed = %lu (%d bytes)\n",block->fan_speed, sizeof(block->fan_speed));
	printf("- busy = %d (%d bytes)\n",block->busy, sizeof(block->busy));

	printf("*************************************************\n\n");
}

typedef struct {
	unsigned long nominal_rate;		// The nominal step rate for this block in step_events/sec 
	unsigned long initial_rate;		// The jerk-adjusted step rate at start of block  
	unsigned long final_rate;		// The minimal rate at exit
	long steps_x;
	long steps_y;
	long steps_z;
	long steps_e;					// Step count along each axis
	unsigned long step_event_count;	// The number of step events required to complete this block
	unsigned long direction_bits;	// The direction bit set for this block
	long accelerate_until;			// The index of the step event on which to stop acceleration
	long decelerate_after;			// The index of the step event on which to start decelerating
	long acceleration_rate;			// The acceleration rate used for acceleration calculation
	unsigned long enable_endstops;
} pru_stepper_block;

void stepper_wait_loop()
{
	while(1) {
		usleep(10000);
		kill(getppid(), SIGUSR1);
		usleep(10000);
		kill(getppid(), SIGUSR2);
	}
}

// "The Stepper Driver Interrupt" - This timer interrupt is the workhorse.
// It pops blocks from the block_buffer and executes them by pulsing the stepper pins appropriately.
void stepper_handler(int sign)
{
	int result = 0;
	unsigned long tmp;
	pru_stepper_block pru_block;

  // If there is no current block, attempt to pop one from the buffer
  if (current_block == NULL) {
    // Anything in the buffer?
    current_block = plan_get_current_block();
    if (current_block != NULL) {
		//debug_current_block(current_block);

		pru_block.nominal_rate = current_block->nominal_rate;
		pru_block.initial_rate = current_block->initial_rate;
		pru_block.final_rate = current_block->final_rate;
		pru_block.steps_x = current_block->steps_x;
		pru_block.steps_y = current_block->steps_y;
		pru_block.steps_z = current_block->steps_z;
		pru_block.steps_e = current_block->steps_e;
		pru_block.step_event_count = current_block->step_event_count;
		pru_block.direction_bits = current_block->direction_bits;
		pru_block.accelerate_until = current_block->accelerate_until;
		pru_block.decelerate_after = current_block->decelerate_after;
		pru_block.acceleration_rate = current_block->acceleration_rate;
		pru_block.enable_endstops = (unsigned long)check_endstops;
		//printf("SIZEOF pru_block = %d\n",sizeof(pru_block));
		/* Send 'pru_stepper_block' to the PRU through the RPMsg channel */
		result = write(pru_file, &pru_block, sizeof(pru_block));
		if(result > 0)
			printf("Message: Sent to PRU\n");

		current_block->busy = true;
    }
  }

	if (current_block != NULL) {
		/* Poll until we receive a message from the PRU and then print it */
		tmp = 0;
		result = read(pru_file, &tmp, sizeof(unsigned long));
		if(result > 0) {
			if(result == 8) {
				current_block = NULL;
				plan_discard_current_block();
			}
		}
	}

}

#ifdef ADVANCE
  unsigned char old_OCR0A;
  // Timer interrupt for E. e_steps is set in the main routine;
  // Timer 0 is shared with millies
  ISR(TIMER0_COMPA_vect)
  {
    old_OCR0A += 52; // ~10kHz interrupt (250000 / 26 = 9615kHz)
    OCR0A = old_OCR0A;
    // Set E direction (Depends on E direction + advance)
    for(unsigned char i=0; i<4;i++) {
      if (e_steps != 0) {
        //WRITE(E0_STEP_PIN, INVERT_E_STEP_PIN);
        if (e_steps < 0) {
          //WRITE(E0_DIR_PIN, INVERT_E0_DIR);
          e_steps++;
          //WRITE(E0_STEP_PIN, !INVERT_E_STEP_PIN);
        }
        else if (e_steps > 0) {
          //WRITE(E0_DIR_PIN, !INVERT_E0_DIR);
          e_steps--;
          //WRITE(E0_STEP_PIN, !INVERT_E_STEP_PIN);
        }
      }
    }
  }
#endif // ADVANCE

/* Init steppers structs */
void stepper_setup() {
	/* X stepper driver SPI Config */
	sprintf(steppers[X_AXIS].spi_device.device,"/dev/spidev%d.%d",SPI_DEVICE_BUS_NR,X_STEPPER_SPI_DEVICE);
	steppers[X_AXIS].spi_device.mode = 0;
	steppers[X_AXIS].spi_device.bits = 8;
	steppers[X_AXIS].spi_device.speed = 100000;
	steppers[X_AXIS].spi_device.delay = 100;

	/* X stepper driver config */
	steppers[X_AXIS].regs.ABS_POS = 0;
	steppers[X_AXIS].regs.EL_POS = 0;
	steppers[X_AXIS].regs.MARK = 0;
	steppers[X_AXIS].regs.TVAL = X_TVAL;
	steppers[X_AXIS].regs.T_FAST = X_T_FAST;
	steppers[X_AXIS].regs.TON_MIN = X_TON_MIN;
	steppers[X_AXIS].regs.TOFF_MIN = X_TOFF_MIN;
	steppers[X_AXIS].regs.OCD_TH = X_OCD_TH;
	steppers[X_AXIS].regs.STEP_MODE = X_STEP_MODE;
	steppers[X_AXIS].regs.ALARM_EN = X_ALARM_EN;
	steppers[X_AXIS].regs.CONFIG = X_CONFIG;

	/* Y stepper driver SPI Config */
	sprintf(steppers[Y_AXIS].spi_device.device,"/dev/spidev%d.%d",SPI_DEVICE_BUS_NR,Y_STEPPER_SPI_DEVICE);
	steppers[Y_AXIS].spi_device.mode = 0;
	steppers[Y_AXIS].spi_device.bits = 8;
	steppers[Y_AXIS].spi_device.speed = 100000;
	steppers[Y_AXIS].spi_device.delay = 100;

	/* Y stepper driver config */
	steppers[Y_AXIS].regs.ABS_POS = 0;
	steppers[Y_AXIS].regs.EL_POS = 0;
	steppers[Y_AXIS].regs.MARK = 0;
	steppers[Y_AXIS].regs.TVAL = Y_TVAL;
	steppers[Y_AXIS].regs.T_FAST = Y_T_FAST;
	steppers[Y_AXIS].regs.TON_MIN = Y_TON_MIN;
	steppers[Y_AXIS].regs.TOFF_MIN = Y_TOFF_MIN;
	steppers[Y_AXIS].regs.OCD_TH = Y_OCD_TH;
	steppers[Y_AXIS].regs.STEP_MODE = Y_STEP_MODE;
	steppers[Y_AXIS].regs.ALARM_EN = Y_ALARM_EN;
	steppers[Y_AXIS].regs.CONFIG = Y_CONFIG;

	/* Z stepper driver SPI Config */
	sprintf(steppers[Z_AXIS].spi_device.device,"/dev/spidev%d.%d",SPI_DEVICE_BUS_NR,Z_STEPPER_SPI_DEVICE);
	steppers[Z_AXIS].spi_device.mode = 0;
	steppers[Z_AXIS].spi_device.bits = 8;
	steppers[Z_AXIS].spi_device.speed = 100000;
	steppers[Z_AXIS].spi_device.delay = 100;

	/* Z stepper driver config */
	steppers[Z_AXIS].regs.ABS_POS = 0;
	steppers[Z_AXIS].regs.EL_POS = 0;
	steppers[Z_AXIS].regs.MARK = 0;
	steppers[Z_AXIS].regs.TVAL = Z_TVAL;
	steppers[Z_AXIS].regs.T_FAST = Z_T_FAST;
	steppers[Z_AXIS].regs.TON_MIN = Z_TON_MIN;
	steppers[Z_AXIS].regs.TOFF_MIN = Z_TOFF_MIN;
	steppers[Z_AXIS].regs.OCD_TH = Z_OCD_TH;
	steppers[Z_AXIS].regs.STEP_MODE = Z_STEP_MODE;
	steppers[Z_AXIS].regs.ALARM_EN = Z_ALARM_EN;
	steppers[Z_AXIS].regs.CONFIG = Z_CONFIG;

	/* E stepper driver SPI Config */
	sprintf(steppers[E_AXIS].spi_device.device,"/dev/spidev%d.%d",SPI_DEVICE_BUS_NR,E_STEPPER_SPI_DEVICE);
	steppers[E_AXIS].spi_device.mode = 0;
	steppers[E_AXIS].spi_device.bits = 8;
	steppers[E_AXIS].spi_device.speed = 100000;
	steppers[E_AXIS].spi_device.delay = 100;

	/* E stepper driver config */
	steppers[E_AXIS].regs.ABS_POS = 0;
	steppers[E_AXIS].regs.EL_POS = 0;
	steppers[E_AXIS].regs.MARK = 0;
	steppers[E_AXIS].regs.TVAL = E_TVAL;
	steppers[E_AXIS].regs.T_FAST = E_T_FAST;
	steppers[E_AXIS].regs.TON_MIN = E_TON_MIN;
	steppers[E_AXIS].regs.TOFF_MIN = E_TOFF_MIN;
	steppers[E_AXIS].regs.OCD_TH = E_OCD_TH;
	steppers[E_AXIS].regs.STEP_MODE = E_STEP_MODE;
	steppers[E_AXIS].regs.ALARM_EN = E_ALARM_EN;
	steppers[E_AXIS].regs.CONFIG = E_CONFIG;
}

void easyspin_setup(easySPIN_stepper *stepper) {

	int ret = 0;

	stepper->spi_device.spi_fd = open(stepper->spi_device.device, O_RDWR);
	if (stepper->spi_device.spi_fd < 0)
		perror("can't open device");

	/*
	 * spi mode
	 */
	ret = ioctl(stepper->spi_device.spi_fd, SPI_IOC_WR_MODE32, &stepper->spi_device.mode);
	if (ret == -1)
		perror("can't set spi mode");

	ret = ioctl(stepper->spi_device.spi_fd, SPI_IOC_RD_MODE32, &stepper->spi_device.mode);
	if (ret == -1)
		perror("can't get spi mode");

	/*
	 * bits per word
	 */
	ret = ioctl(stepper->spi_device.spi_fd, SPI_IOC_WR_BITS_PER_WORD, &stepper->spi_device.bits);
	if (ret == -1)
		perror("can't set bits per word");

	ret = ioctl(stepper->spi_device.spi_fd, SPI_IOC_RD_BITS_PER_WORD, &stepper->spi_device.bits);
	if (ret == -1)
		perror("can't get bits per word");

	/*
	 * max speed hz
	 */
	ret = ioctl(stepper->spi_device.spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &stepper->spi_device.speed);
	if (ret == -1)
		perror("can't set max speed hz");

	ret = ioctl(stepper->spi_device.spi_fd, SPI_IOC_RD_MAX_SPEED_HZ, &stepper->spi_device.speed);
	if (ret == -1)
		perror("can't get max speed hz");

	//printf("spi mode: 0x%x\n", stepper->spi_device.mode);
	//printf("bits per word: %d\n", stepper->spi_device.bits);
	//printf("max speed: %d Hz (%d KHz)\n", stepper->spi_device.speed, stepper->spi_device.speed/1000);

	//printf("##### eMotionControl_Init BEGIN #####\n");
	/* easySPIN system init */
	easySPIN_Disable(&stepper->spi_device);

	/* Program all easySPIN registers */
	easySPIN_Registers_Set(&stepper->spi_device, &stepper->regs);

	/* Read STATUS register */
	easySPIN_rx_data = easySPIN_Get_Status(&stepper->spi_device);

	/* Enable easySPIN powerstage */
	easySPIN_Disable(&stepper->spi_device);
	stepper->enabled = false;
}

void st_init()
{
  digipot_init(); //Initialize Digipot Motor Current
  microstep_init(); //Initialize Microstepping Pins

  SET_OUTPUT(STEPPER_ENABLEn_PIN);
  SET_INPUT(STEPPER_FLAG_PIN);

	/* Standby-reset deactivation */
	easySPIN_Reset(easySPIN_STBY_RESET_GPIO);
	usleep(100000);
	/* Standby-reset deactivation */
	easySPIN_ReleaseReset(easySPIN_STBY_RESET_GPIO);

	stepper_setup();
	easyspin_setup(&steppers[X_AXIS]);
	easyspin_setup(&steppers[Y_AXIS]);
	easyspin_setup(&steppers[Z_AXIS]);
	easyspin_setup(&steppers[E_AXIS]);

  //endstops and pullups

  #if defined(X_MIN_PIN) && X_MIN_PIN > -1
    SET_INPUT(X_MIN_PIN);
    #ifdef ENDSTOPPULLUP_XMIN
      WRITE(X_MIN_PIN,HIGH);
    #endif
  #endif

  #if defined(Y_MIN_PIN) && Y_MIN_PIN > -1
    SET_INPUT(Y_MIN_PIN);
    #ifdef ENDSTOPPULLUP_YMIN
      WRITE(Y_MIN_PIN,HIGH);
    #endif
  #endif

  #if defined(Z_MIN_PIN) && Z_MIN_PIN > -1
    SET_INPUT(Z_MIN_PIN);
    #ifdef ENDSTOPPULLUP_ZMIN
      WRITE(Z_MIN_PIN,HIGH);
    #endif
  #endif

  #if defined(X_MAX_PIN) && X_MAX_PIN > -1
    SET_INPUT(X_MAX_PIN);
    #ifdef ENDSTOPPULLUP_XMAX
      WRITE(X_MAX_PIN,HIGH);
    #endif
  #endif

  #if defined(Y_MAX_PIN) && Y_MAX_PIN > -1
    SET_INPUT(Y_MAX_PIN);
    #ifdef ENDSTOPPULLUP_YMAX
      WRITE(Y_MAX_PIN,HIGH);
    #endif
  #endif

  #if defined(Z_MAX_PIN) && Z_MAX_PIN > -1
    SET_INPUT(Z_MAX_PIN);
    #ifdef ENDSTOPPULLUP_ZMAX
      WRITE(Z_MAX_PIN,HIGH);
    #endif
  #endif

    disable_x();
    disable_y();
    disable_z();
    disable_e0();

  ENABLE_STEPPER_DRIVER_INTERRUPT();

  #ifdef ADVANCE
  #if defined(TCCR0A) && defined(WGM01)
    TCCR0A &= ~(1<<WGM01);
    TCCR0A &= ~(1<<WGM00);
  #endif
    e_steps = 0;
    TIMSK0 |= (1<<OCIE0A);
  #endif //ADVANCE

  enable_endstops(true); // Start with endstops active. After homing they can be disabled
/* TODO: FIXME */
  //sei();
/* TODO: FIXME */
}


// Block until all buffered steps are executed
void st_synchronize()
{
    while( blocks_queued()) {
    manage_heater();
    manage_inactivity();
  }
}

void st_set_position(const long &x, const long &y, const long &z, const long &e)
{
  CRITICAL_SECTION_START;

  count_position[X_AXIS] = x;
  count_position[Y_AXIS] = y;
  count_position[Z_AXIS] = z;
  count_position[E_AXIS] = e;

  CRITICAL_SECTION_END;
}

void st_set_e_position(const long &e)
{
  CRITICAL_SECTION_START;

  count_position[E_AXIS] = e;

  CRITICAL_SECTION_END;
}

long st_get_position(uint8_t axis)
{
  long count_pos;

  CRITICAL_SECTION_START;

  count_pos = count_position[axis];

  CRITICAL_SECTION_END;

  return count_pos;
}

#ifdef ENABLE_AUTO_BED_LEVELING
float st_get_position_mm(uint8_t axis)
{
  float steper_position_in_steps = st_get_position(axis);
  return steper_position_in_steps / axis_steps_per_unit[axis];
}
#endif  // ENABLE_AUTO_BED_LEVELING

void finishAndDisableSteppers()
{
  st_synchronize();
  disable_x();
  disable_y();
  disable_z();
  disable_e0();
}

void quickStop()
{
  DISABLE_STEPPER_DRIVER_INTERRUPT();
  while(blocks_queued())
    plan_discard_current_block();
  current_block = NULL;
  ENABLE_STEPPER_DRIVER_INTERRUPT();
}

void digitalPotWrite(int address, int value) // From Arduino DigitalPotControl example
{
  #if defined(DIGIPOTSS_PIN) && DIGIPOTSS_PIN > -1
    digitalWrite(DIGIPOTSS_PIN,LOW); // take the SS pin low to select the chip
    SPI.transfer(address); //  send in the address and value via SPI:
    SPI.transfer(value);
    digitalWrite(DIGIPOTSS_PIN,HIGH); // take the SS pin high to de-select the chip:
    //delay(10);
  #endif
}

void digipot_init() //Initialize Digipot Motor Current
{
  #if defined(DIGIPOTSS_PIN) && DIGIPOTSS_PIN > -1
    const uint8_t digipot_motor_current[] = DIGIPOT_MOTOR_CURRENT;

    SPI.begin();
    pinMode(DIGIPOTSS_PIN, OUTPUT);
    for(int i=0;i<=4;i++)
      //digitalPotWrite(digipot_ch[i], digipot_motor_current[i]);
      digipot_current(i,digipot_motor_current[i]);
  #endif
  #ifdef MOTOR_CURRENT_PWM_XY_PIN
    pinMode(MOTOR_CURRENT_PWM_XY_PIN, OUTPUT);
    pinMode(MOTOR_CURRENT_PWM_Z_PIN, OUTPUT);
    pinMode(MOTOR_CURRENT_PWM_E_PIN, OUTPUT);
    digipot_current(0, motor_current_setting[0]);
    digipot_current(1, motor_current_setting[1]);
    digipot_current(2, motor_current_setting[2]);
    //Set timer5 to 31khz so the PWM of the motor power is as constant as possible. (removes a buzzing noise)
    TCCR5B = (TCCR5B & ~(_BV(CS50) | _BV(CS51) | _BV(CS52))) | _BV(CS50);
  #endif
}

void digipot_current(uint8_t driver, int current)
{
  #if defined(DIGIPOTSS_PIN) && DIGIPOTSS_PIN > -1
    const uint8_t digipot_ch[] = DIGIPOT_CHANNELS;
    digitalPotWrite(digipot_ch[driver], current);
  #endif
  #ifdef MOTOR_CURRENT_PWM_XY_PIN
  if (driver == 0) analogWrite(MOTOR_CURRENT_PWM_XY_PIN, (long)current * 255L / (long)MOTOR_CURRENT_PWM_RANGE);
  if (driver == 1) analogWrite(MOTOR_CURRENT_PWM_Z_PIN, (long)current * 255L / (long)MOTOR_CURRENT_PWM_RANGE);
  if (driver == 2) analogWrite(MOTOR_CURRENT_PWM_E_PIN, (long)current * 255L / (long)MOTOR_CURRENT_PWM_RANGE);
  #endif
}

void microstep_init()
{
  #if defined(X_MS1_PIN) && X_MS1_PIN > -1
  const uint8_t microstep_modes[] = MICROSTEP_MODES;
  #endif

  #if defined(E1_MS1_PIN) && E1_MS1_PIN > -1
  pinMode(E1_MS1_PIN,OUTPUT);
  pinMode(E1_MS2_PIN,OUTPUT); 
  #endif

  #if defined(X_MS1_PIN) && X_MS1_PIN > -1
  pinMode(X_MS1_PIN,OUTPUT);
  pinMode(X_MS2_PIN,OUTPUT);  
  pinMode(Y_MS1_PIN,OUTPUT);
  pinMode(Y_MS2_PIN,OUTPUT);
  pinMode(Z_MS1_PIN,OUTPUT);
  pinMode(Z_MS2_PIN,OUTPUT);  
  pinMode(E0_MS1_PIN,OUTPUT);
  pinMode(E0_MS2_PIN,OUTPUT);
  for(int i=0;i<=4;i++) microstep_mode(i,microstep_modes[i]);
  #endif
}

void microstep_ms(uint8_t driver, int8_t ms1, int8_t ms2)
{
  if(ms1 > -1) switch(driver)
  {
    case 0: digitalWrite( X_MS1_PIN,ms1); break;
    case 1: digitalWrite( Y_MS1_PIN,ms1); break;
    case 2: digitalWrite( Z_MS1_PIN,ms1); break;
    case 3: digitalWrite(E0_MS1_PIN,ms1); break;
    #if defined(E1_MS1_PIN) && E1_MS1_PIN > -1
    case 4: digitalWrite(E1_MS1_PIN,ms1); break;
    #endif
  }
  if(ms2 > -1) switch(driver)
  {
    case 0: digitalWrite( X_MS2_PIN,ms2); break;
    case 1: digitalWrite( Y_MS2_PIN,ms2); break;
    case 2: digitalWrite( Z_MS2_PIN,ms2); break;
    case 3: digitalWrite(E0_MS2_PIN,ms2); break;
    #if defined(E1_MS2_PIN) && E1_MS2_PIN > -1
    case 4: digitalWrite(E1_MS2_PIN,ms2); break;
    #endif
  }
}

void microstep_mode(uint8_t driver, uint8_t stepping_mode)
{
  switch(stepping_mode)
  {
    case 1: microstep_ms(driver,MICROSTEP1); break;
    case 2: microstep_ms(driver,MICROSTEP2); break;
    case 4: microstep_ms(driver,MICROSTEP4); break;
    case 8: microstep_ms(driver,MICROSTEP8); break;
    case 16: microstep_ms(driver,MICROSTEP16); break;
  }
}

void microstep_readings()
{
      SERIAL_PROTOCOLPGM("MS1,MS2 Pins\n");
      SERIAL_PROTOCOLPGM("X: ");
      SERIAL_PROTOCOL(   digitalRead(X_MS1_PIN));
      SERIAL_PROTOCOLLN( digitalRead(X_MS2_PIN));
      SERIAL_PROTOCOLPGM("Y: ");
      SERIAL_PROTOCOL(   digitalRead(Y_MS1_PIN));
      SERIAL_PROTOCOLLN( digitalRead(Y_MS2_PIN));
      SERIAL_PROTOCOLPGM("Z: ");
      SERIAL_PROTOCOL(   digitalRead(Z_MS1_PIN));
      SERIAL_PROTOCOLLN( digitalRead(Z_MS2_PIN));
      SERIAL_PROTOCOLPGM("E0: ");
      SERIAL_PROTOCOL(   digitalRead(E0_MS1_PIN));
      SERIAL_PROTOCOLLN( digitalRead(E0_MS2_PIN));
      #if defined(E1_MS1_PIN) && E1_MS1_PIN > -1
      SERIAL_PROTOCOLPGM("E1: ");
      SERIAL_PROTOCOL(   digitalRead(E1_MS1_PIN));
      SERIAL_PROTOCOLLN( digitalRead(E1_MS2_PIN));
      #endif
}

