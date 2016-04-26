/*
 * stepper_pru.h
 *
 *  Created on: 20/mar/2016
 *      Author: Matteo
 */

#ifndef STEPPER_PRU_H_
#define STEPPER_PRU_H_

#include <stdint.h>
#include <pru_ctrl.h>
#include "../pins.h"

volatile register uint32_t __R30;
volatile register uint32_t __R31;

#ifndef PRU_SRAM
#define PRU_SRAM __far __attribute__((cregister("PRU_SHAREDMEM", __far)))
#endif

#define SHM_ADDR ((volatile uint32_t *)(0x4A310000))

#define MAX_STEP_FREQUENCY 40000 // Max step frequency for Ultimaker (5000 pps / half step)
#define F_CPU 16000000
// For now timer has a frequency of 2MHz as in Marlin original code for Arduino
// 1 / 200 MHz = 5 nsec
// 1 / 2   MHz = 500 nsec
#define TIMER_SCALE (100)
#define STEP_RATE_MIN (F_CPU / 500000)

#define FALSE 0
#define TRUE 1

#define INVERT_X_STEP_PIN FALSE
#define INVERT_Y_STEP_PIN FALSE
#define INVERT_Z_STEP_PIN FALSE
#define INVERT_E_STEP_PIN FALSE

#define INVERT_X_DIR FALSE    // for Mendel set to false, for Orca set to true
#define INVERT_Y_DIR FALSE    // for Mendel set to true, for Orca set to false
#define INVERT_Z_DIR FALSE     // for Mendel set to false, for Orca set to true
#define INVERT_E0_DIR TRUE   // for direct drive extruder v9 set to true, for geared extruder set to false

#define GPIO_DATAIN                     0x138                                   // This is the register for reading data
#define GPIO0                           0x44E07000                              // The adress of the GPIO0 bank
#define GPIO1                           0x4804C000                              // The adress of the GPIO1 bank
#define GPIO2                           0x481AC000                              // The adress of the GPIO2 bank
#define GPIO3                           0x481AE000                              // The adress of the GPIO3 bank

#define X_MIN_PIN_READ		(((*(volatile uint32_t *)(GPIO2 + GPIO_DATAIN)) & 0x4) >> 2)		// GPIO(2,2)
#define X_MAX_PIN_READ		(((*(volatile uint32_t *)(GPIO2 + GPIO_DATAIN)) & 0x8) >> 3)		// GPIO(2,3)
#define Y_MIN_PIN_READ		(((*(volatile uint32_t *)(GPIO2 + GPIO_DATAIN)) & 0x20) >> 5)	// GPIO(2,5)
#define Y_MAX_PIN_READ		(((*(volatile uint32_t *)(GPIO2 + GPIO_DATAIN)) & 0x10) >> 4)	// GPIO(2,4)
#define Z_MIN_PIN_READ		(((*(volatile uint32_t *)(GPIO0 + GPIO_DATAIN)) & 0x800000) >> 23)	// GPIO(0,23)
#define Z_MAX_PIN_READ		(((*(volatile uint32_t *)(GPIO0 + GPIO_DATAIN)) & 0x4000000) >> 26)	// GPIO(0,26)

#define READ(PIN) ( PIN )

#define WRITE(PIN,VALUE) (__R30 ^= ((int)-VALUE ^ (int)__R30) & (1 << PIN))

#define WRITE_E_STEP(v) WRITE(E0_STEP_PIN, v)
#define NORM_E_DIR() WRITE(E0_DIR_PIN, !INVERT_E0_DIR)
#define REV_E_DIR() WRITE(E0_DIR_PIN, INVERT_E0_DIR)

#define BLOCK_BUFFER_SIZE 16 // maximize block buffer

// This struct is used when buffering the setup for each linear movement "nominal" values are as specified in
// the source g-code and may never actually be reached if acceleration management is active.
typedef struct {
  // Fields used by the bresenham algorithm for tracing the line
  long steps_x, steps_y, steps_z, steps_e;  // Step count along each axis
  unsigned long step_event_count;           // The number of step events required to complete this block
  long accelerate_until;                    // The index of the step event on which to stop acceleration
  long decelerate_after;                    // The index of the step event on which to start decelerating
  long acceleration_rate;                   // The acceleration rate used for acceleration calculation
  unsigned int direction_bits;             // The direction bit set for this block (refers to *_DIRECTION_BIT in config.h)
  #ifdef ADVANCE
    long advance_rate;
    volatile long initial_advance;
    volatile long final_advance;
    float advance;
  #endif

  // Fields used by the motion planner to manage acceleration
//  float speed_x, speed_y, speed_z, speed_e;        // Nominal mm/sec for each axis
  float nominal_speed;                               // The nominal speed for this block in mm/sec
  float entry_speed;                                 // Entry speed at previous-current junction in mm/sec
  float max_entry_speed;                             // Maximum allowable junction entry speed in mm/sec
  float millimeters;                                 // The total travel of this block in mm
  float acceleration;                                // acceleration mm/sec^2
  unsigned int recalculate_flag;                    // Planner flag to recalculate trapezoids on entry junction
  unsigned int nominal_length_flag;                 // Planner flag for nominal speed always reached

  // Settings for the trapezoid generator
  unsigned long nominal_rate;                        // The nominal step rate for this block in step_events/sec
  unsigned long initial_rate;                        // The jerk-adjusted step rate at start of block
  unsigned long final_rate;                          // The minimal rate at exit
  unsigned long acceleration_st;                     // acceleration steps/sec^2
  unsigned long fan_speed;
  volatile int busy;
} block_t;

//block_t *block_buffer[BLOCK_BUFFER_SIZE];  // A ring buffer for motion instfructions
//unsigned int *block_buffer_head;           // Index of the next block to be pushed
//unsigned int *block_buffer_tail;           // Index of the block to process now
block_t *current_block;
//unsigned int *check_endstops;

enum AxisEnum {X_AXIS=0, Y_AXIS=1, Z_AXIS=2, E_AXIS=3};

unsigned short calc_timer(unsigned short);

void trapezoid_generator_reset();

unsigned char do_block();

block_t *plan_get_current_block();

void plan_discard_current_block();

#endif /* STEPPER_PRU_H_ */
