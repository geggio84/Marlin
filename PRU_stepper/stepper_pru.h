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
#include "../common_pru.h"

/* PRU0 is mailbox module user 1 */
#define MB_USER						1
/* Mbox0 - mail_u1_irq (mailbox interrupt for PRU0) is Int Number 60 */
#define MB_INT_NUMBER				60

/* Host-0 Interrupt sets bit 30 in register R31 */
#define HOST_INT					0x40000000

/* The mailboxes used for RPMsg are defined in the Linux device tree
 * PRU0 uses mailboxes 2 (From ARM) and 3 (To ARM)
 * PRU1 uses mailboxes 4 (From ARM) and 5 (To ARM)
 */
#define MB_TO_ARM_HOST				3
#define MB_FROM_ARM_HOST			2

/*
 * Using the name 'rpmsg-client-sample' will probe the RPMsg sample driver
 * found at linux-x.y.z/samples/rpmsg/rpmsg_client_sample.c
 *
 * Using the name 'rpmsg-pru' will probe the rpmsg_pru driver found
 * at linux-x.y.z/drivers/rpmsg/rpmsg_pru.c
 */
#define CHAN_NAME					"rpmsg-pru"

#define CHAN_DESC					"Channel 30"
#define CHAN_PORT					30

volatile register uint32_t __R30;
volatile register uint32_t __R31;

#ifndef PRU_SRAM
#define PRU_SRAM __far __attribute__((cregister("PRU_SHAREDMEM", __far)))
#endif

#define SHM_ADDR ((volatile uint32_t *)(0x4A310000))

#define MAX_STEP_FREQUENCY 40000 // Max step frequency for Ultimaker (5000 pps / half step)
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

block_t *current_block;
unsigned short calc_timer(unsigned short);

void trapezoid_generator_reset();

unsigned char do_block();

block_t *plan_get_current_block();

void plan_discard_current_block();

void debug_step(signed char *steps);

#endif /* STEPPER_PRU_H_ */
