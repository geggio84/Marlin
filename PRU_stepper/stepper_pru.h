/*
 * stepper_pru.h
 *
 *  Created on: 20/mar/2016
 *      Author: Matteo
 */

#ifndef STEPPER_PRU_H_
#define STEPPER_PRU_H_

#include <stdint.h>

#define MAX_STEP_FREQUENCY 40000 // Max step frequency for Ultimaker (5000 pps / half step)
#define F_CPU 16000000

#define FALSE 0
#define TRUE 1

#define GPIO_DATAIN                     0x138                                   // This is the register for reading data
#define GPIO0                           0x44E07000                              // The adress of the GPIO0 bank
#define GPIO1                           0x4804C000                              // The adress of the GPIO1 bank
#define GPIO2                           0x481AC000                              // The adress of the GPIO2 bank
#define GPIO3                           0x481AE000                              // The adress of the GPIO3 bank

#define X_MIN_PIN_READ		((*(volatile uint32_t *)(GPIO2 + GPIO_DATAIN)) && 0x4)		// GPIO(2,2)
#define X_MAX_PIN_READ		((*(volatile uint32_t *)(GPIO2 + GPIO_DATAIN)) && 0x8)		// GPIO(2,3)
#define Y_MIN_PIN_READ		((*(volatile uint32_t *)(GPIO2 + GPIO_DATAIN)) && 0x20)	// GPIO(2,5)
#define Y_MAX_PIN_READ		((*(volatile uint32_t *)(GPIO2 + GPIO_DATAIN)) && 0x10)	// GPIO(2,4)
#define Z_MIN_PIN_READ		((*(volatile uint32_t *)(GPIO0 + GPIO_DATAIN)) && 0x800000)	// GPIO(0,23)
#define Z_MAX_PIN_READ		((*(volatile uint32_t *)(GPIO0 + GPIO_DATAIN)) && 0x4000000)	// GPIO(0,26)

#define READ(PIN) ( PIN )

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
