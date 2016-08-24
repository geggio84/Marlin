/*
 * Copyright (C) 2015 Texas Instruments Incorporated - http://www.ti.com/ 
 *  
 *  
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions 
 * are met:
 * 
 * 	* Redistributions of source code must retain the above copyright 
 * 	  notice, this list of conditions and the following disclaimer.
 * 
 * 	* Redistributions in binary form must reproduce the above copyright
 * 	  notice, this list of conditions and the following disclaimer in the 
 * 	  documentation and/or other materials provided with the   
 * 	  distribution.
 * 
 * 	* Neither the name of Texas Instruments Incorporated nor the names of
 * 	  its contributors may be used to endorse or promote products derived
 * 	  from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pru_cfg.h>
#include <pru_intc.h>
#include <rsc_types.h>
#include <pru_virtqueue.h>
#include <pru_rpmsg.h>
#include <sys_mailbox.h>
#include "resource_table.h"
#include "stepper_pru.h"
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

#define START_REQ			"START PRU"
#define STOP_REQ			"STOP PRU"
#define START_MSG			"PRU STARTED"
#define START_ALREADY_MSG	"PRU ALREADY STARTED"
#define STOP_MSG			"PRU STOPPED"
#define STOP_ALREADY_MSG	"PRU ALREADY STOPPED"
#define VER_REQ				"VERSION PRU"
#define VER_MAJ				0
#define VER_MIN				9
#define VER_REL				0

/* 
 * Used to make sure the Linux drivers are ready for RPMsg communication
 * Found at linux-x.y.z/include/uapi/linux/virtio_config.h
 */
#define VIRTIO_CONFIG_S_DRIVER_OK	4

uint8_t payload[RPMSG_BUF_SIZE];

block_t *current_block;  // A pointer to the block currently being traced

far PRU_SRAM unsigned int counter;

/*
 * main.c
 */
void main() {
	struct pru_rpmsg_transport transport;
	uint16_t src, dst, len;
	char message[25];
	volatile uint8_t *status;
	unsigned char endstop_status;
	uint16_t pru_started = 0;

	/* allow OCP master port access by the PRU so the PRU can read external memories */
	CT_CFG.SYSCFG_bit.STANDBY_INIT = 0;

	/* clear the status of event MB_INT_NUMBER (the mailbox event) and enable the mailbox event */
	CT_INTC.SICR_bit.STS_CLR_IDX = MB_INT_NUMBER;
	CT_MBX.IRQ[MB_USER].ENABLE_SET |= 1 << (MB_FROM_ARM_HOST * 2);

	/* Make sure the Linux drivers are ready for RPMsg communication */
	status = &resourceTable.rpmsg_vdev.status;
	while (!(*status & VIRTIO_CONFIG_S_DRIVER_OK));

	/* Initialize pru_virtqueue corresponding to vring0 (PRU to ARM Host direction) */
	pru_virtqueue_init(&transport.virtqueue0, &resourceTable.rpmsg_vring0, &CT_MBX.MESSAGE[MB_TO_ARM_HOST], &CT_MBX.MESSAGE[MB_FROM_ARM_HOST]);

	/* Initialize pru_virtqueue corresponding to vring1 (ARM Host to PRU direction) */
	pru_virtqueue_init(&transport.virtqueue1, &resourceTable.rpmsg_vring1, &CT_MBX.MESSAGE[MB_TO_ARM_HOST], &CT_MBX.MESSAGE[MB_FROM_ARM_HOST]);

	counter = 10;

	/* Access PRU Shared RAM using Constant Table                    */
	/*****************************************************************/
	/* C28 defaults to 0x00000000, we need to set bits 23:8 to 0x0100 in order to have it point to 0x00010000	 */
	PRU0_CTRL.CTPPR0_bit.C28_BLK_POINTER = 0x0100;

	/* Create the RPMsg channel between the PRU and ARM user space using the transport structure. */
	while(pru_rpmsg_channel(RPMSG_NS_CREATE, &transport, CHAN_NAME, CHAN_DESC, CHAN_PORT) != PRU_RPMSG_SUCCESS);
	while(1){
		/* Check bit 31 of register R31 to see if the mailbox interrupt has occurred */
		if(__R31 & HOST_INT){
			/* Clear the mailbox interrupt */
			CT_MBX.IRQ[MB_USER].STATUS_CLR |= 1 << (MB_FROM_ARM_HOST * 2);
			/* Clear the event status, event MB_INT_NUMBER corresponds to the mailbox interrupt */
			CT_INTC.SICR_bit.STS_CLR_IDX = MB_INT_NUMBER;
			/* Use a while loop to read all of the current messages in the mailbox */
			while (CT_MBX.MSGSTATUS_bit[MB_FROM_ARM_HOST].NBOFMSG > 0) {
				/* Check to see if the message corresponds to a receive event for the PRU */
				if (CT_MBX.MESSAGE[MB_FROM_ARM_HOST] == 1) {
					/* Receive the message */
					if(pru_rpmsg_receive(&transport, &src, &dst, &message, &len) == PRU_RPMSG_SUCCESS){
						/* Decode message from Marlin Application */
						if(strcmp(message, START_REQ) == 0) {
							if (pru_started) {
								pru_rpmsg_send(&transport, dst, src, START_ALREADY_MSG, sizeof(START_ALREADY_MSG));
							} else {
								pru_started = 1;
								pru_rpmsg_send(&transport, dst, src, START_MSG, sizeof(START_MSG));
							}
						} else if (strcmp(message, STOP_REQ) == 0) {
							if (pru_started) {
								pru_started = 0;
								pru_rpmsg_send(&transport, dst, src, STOP_MSG, sizeof(STOP_MSG));
							} else {
								pru_rpmsg_send(&transport, dst, src, STOP_ALREADY_MSG, sizeof(STOP_ALREADY_MSG));
							}
						} else if (strcmp(message, VER_REQ) == 0) {
							message[2] = VER_MAJ;
							message[1] = VER_MIN;
							message[0] = VER_REL;
							pru_rpmsg_send(&transport, dst, src, message, 3);
						}
					}
				}
			}
		}

		if (pru_started) {
			counter += 1;
			if (current_block == 0) {
				// Anything in the buffer?
				current_block = plan_get_current_block();
				if (current_block != 0) {
					current_block->busy = TRUE;
					endstop_status = do_block();
					current_block = 0;
					plan_discard_current_block();
				}
			}
		}
	}
}
