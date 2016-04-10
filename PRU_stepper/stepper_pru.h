/*
 * stepper_pru.h
 *
 *  Created on: 20/mar/2016
 *      Author: Matteo
 */

#ifndef STEPPER_PRU_H_
#define STEPPER_PRU_H_

#include <stdint.h>
#include "../pins.h"

volatile register uint32_t __R30;
volatile register uint32_t __R31;

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

typedef struct {
	unsigned long nominal_rate;		// The nominal step rate for this block in step_events/sec
	unsigned long initial_rate;		// The jerk-adjusted step rate at start of block
	unsigned long final_rate;		// The minimal rate at exit
	long steps_x;
	long steps_y;
	long steps_z;
	long steps_e;  					// Step count along each axis
	unsigned long step_event_count;	// The number of step events required to complete this block
	unsigned long direction_bits;	// The direction bit set for this block
	long accelerate_until;			// The index of the step event on which to stop acceleration
	long decelerate_after;			// The index of the step event on which to start decelerating
	long acceleration_rate;			// The acceleration rate used for acceleration calculation
	unsigned long enable_endstops;
} pru_stepper_block;

enum AxisEnum {X_AXIS=0, Y_AXIS=1, Z_AXIS=2, E_AXIS=3};

unsigned short calc_timer(unsigned short);

void trapezoid_generator_reset(pru_stepper_block *);

unsigned char do_block(pru_stepper_block *, struct pru_rpmsg_transport *, uint32_t, uint32_t);

#endif /* STEPPER_PRU_H_ */
