#include <pru_rpmsg.h>
#include "stepper_pru.h"
#include "../pins.h"
#include "../speed_lookuptable.h"

// Variables used by The Stepper Driver Interrupt
static unsigned char out_bits;        // The next stepping-bits to be output
static long counter_x,       // Counter variables for the bresenham line tracer
            counter_y,
            counter_z,
            counter_e;
volatile static unsigned long step_events_completed; // The number of step events executed in the current block
static long acceleration_time, deceleration_time;
static unsigned short acc_step_rate; // needed for deccelaration start point
static char step_loops;
static unsigned short OCR1A_nominal;
static unsigned short step_loops_nominal;
volatile long count_position[4] = { 0, 0, 0, 0};
volatile signed char count_direction[4] = { 1, 1, 1, 1};

static unsigned char old_x_min_endstop=FALSE;
static unsigned char old_x_max_endstop=FALSE;
static unsigned char old_y_min_endstop=FALSE;
static unsigned char old_y_max_endstop=FALSE;
static unsigned char old_z_min_endstop=FALSE;
static unsigned char old_z_max_endstop=FALSE;

volatile long endstops_trigsteps[3]={0,0,0};
volatile long endstops_stepsTotal,endstops_stepsDone;
static volatile unsigned char endstop_x_hit=FALSE;
static volatile unsigned char endstop_y_hit=FALSE;
static volatile unsigned char endstop_z_hit=FALSE;

// The pullups are needed if you directly connect a mechanical endswitch between the signal and ground pins.
const unsigned char X_MIN_ENDSTOP_INVERTING = TRUE; // set to true to invert the logic of the endstop.
const unsigned char Y_MIN_ENDSTOP_INVERTING = TRUE; // set to true to invert the logic of the endstop.
const unsigned char Z_MIN_ENDSTOP_INVERTING = TRUE; // set to true to invert the logic of the endstop.
const unsigned char X_MAX_ENDSTOP_INVERTING = TRUE; // set to true to invert the logic of the endstop.
const unsigned char Y_MAX_ENDSTOP_INVERTING = TRUE; // set to true to invert the logic of the endstop.
const unsigned char Z_MAX_ENDSTOP_INVERTING = TRUE; // set to true to invert the logic of the endstop.

unsigned short calc_timer(unsigned short step_rate) {
	unsigned short timer;
	if(step_rate > MAX_STEP_FREQUENCY) step_rate = MAX_STEP_FREQUENCY;

	if(step_rate > 20000) { // If steprate > 20kHz >> step 4 times
		step_rate = (step_rate >> 2)&0x3fff;
		step_loops = 4;
	}
	else if(step_rate > 10000) { // If steprate > 10kHz >> step 2 times
		step_rate = (step_rate >> 1)&0x7fff;
		step_loops = 2;
	}
	else {
		step_loops = 1;
	}

	if(step_rate < (F_CPU/500000)) step_rate = (F_CPU/500000);
	step_rate -= (F_CPU/500000); // Correct for minimal speed
	if(step_rate >= (8*256)){ // higher step rate
		unsigned char tmp_step_rate = (step_rate & 0x00ff);
		unsigned short gain = speed_lookuptable_fast[(unsigned char)(step_rate>>8)][1];
		timer = (unsigned short)((((unsigned int)tmp_step_rate * (unsigned int)gain) & 0xFFFF0000) >> 16);
		timer = (speed_lookuptable_fast[(unsigned char)(step_rate>>8)][0]) - timer;
	}
	else { // lower step rates
		timer = speed_lookuptable_slow[(step_rate/8)][0];
		timer -= ((speed_lookuptable_slow[(step_rate/8)][1] * (unsigned char)(step_rate & 0x0007))>>3);
	}

	return timer;
}

// Initializes the trapezoid generator from the current block. Called whenever a new
// block begins.
//FORCE_INLINE void trapezoid_generator_reset() {
void trapezoid_generator_reset(pru_stepper_block *current_block) {
	deceleration_time = 0;
	// step_rate to timer interval
	OCR1A_nominal = calc_timer(current_block->nominal_rate);
	// make a note of the number of step loops required at nominal speed
	step_loops_nominal = step_loops;
	acc_step_rate = current_block->initial_rate;
	acceleration_time = calc_timer(acc_step_rate);
/* TODO: FIXME */
	//OCR1A = acceleration_time;
/* TODO: FIXME */
}

// "The Stepper Driver Interrupt" - This timer interrupt is the workhorse.
// It pops blocks from the block_buffer and executes them by pulsing the stepper pins appropriately.
unsigned char do_block(pru_stepper_block *current_block, struct pru_rpmsg_transport *transport, uint32_t src, uint32_t dst)
{
	unsigned char i;
	trapezoid_generator_reset(current_block);
	counter_x = -(current_block->step_event_count >> 1);
	counter_y = counter_x;
	counter_z = counter_x;
	counter_e = counter_x;
	step_events_completed = 0;

	do {
		// Set directions TO DO This should be done once during init of trapezoid. Endstops -> interrupt
		out_bits = current_block->direction_bits;

		// Set the direction bits
		if((out_bits & (1<<X_AXIS))!=0){
			//WRITE(X_DIR_PIN, INVERT_X_DIR);
			count_direction[X_AXIS]=-1;
		} else {
			//WRITE(X_DIR_PIN, !INVERT_X_DIR);
			count_direction[X_AXIS]=1;
		}
		if((out_bits & (1<<Y_AXIS))!=0){
			//WRITE(Y_DIR_PIN, INVERT_Y_DIR);
			count_direction[Y_AXIS]=-1;
		} else {
			//WRITE(Y_DIR_PIN, !INVERT_Y_DIR);
			count_direction[Y_AXIS]=1;
		}

		// Set direction en check limit switches
		if ((out_bits & (1<<X_AXIS)) != 0) {   // stepping along -X axis
			if(current_block->enable_endstops) {
				#if defined(X_MIN_PIN) && X_MIN_PIN > -1
				unsigned char x_min_endstop=(READ(X_MIN_PIN_READ) != X_MIN_ENDSTOP_INVERTING);
				if(x_min_endstop && old_x_min_endstop && (current_block->steps_x > 0)) {
					endstops_trigsteps[X_AXIS] = count_position[X_AXIS];
					endstop_x_hit=TRUE;
					step_events_completed = current_block->step_event_count;
				}
				old_x_min_endstop = x_min_endstop;
				#endif
			}
		} else { // +direction
			if(current_block->enable_endstops) {
				#if defined(X_MAX_PIN) && X_MAX_PIN > -1
				unsigned char x_max_endstop=(READ(X_MAX_PIN_READ) != X_MAX_ENDSTOP_INVERTING);
				if(x_max_endstop && old_x_max_endstop && (current_block->steps_x > 0)) {
					endstops_trigsteps[X_AXIS] = count_position[X_AXIS];
					endstop_x_hit=TRUE;
					step_events_completed = current_block->step_event_count;
				}
				old_x_max_endstop = x_max_endstop;
				#endif
			}
		}

		if ((out_bits & (1<<Y_AXIS)) != 0) {   // -direction
			if(current_block->enable_endstops) {
				#if defined(Y_MIN_PIN) && Y_MIN_PIN > -1
				unsigned char y_min_endstop=(READ(Y_MIN_PIN_READ) != Y_MIN_ENDSTOP_INVERTING);
				if(y_min_endstop && old_y_min_endstop && (current_block->steps_y > 0)) {
					endstops_trigsteps[Y_AXIS] = count_position[Y_AXIS];
					endstop_y_hit=TRUE;
					step_events_completed = current_block->step_event_count;
				}
				old_y_min_endstop = y_min_endstop;
				#endif
			}
		} else { // +direction
			if(current_block->enable_endstops) {
				#if defined(Y_MAX_PIN) && Y_MAX_PIN > -1
				unsigned char y_max_endstop=(READ(Y_MAX_PIN_READ) != Y_MAX_ENDSTOP_INVERTING);
				if(y_max_endstop && old_y_max_endstop && (current_block->steps_y > 0)) {
					endstops_trigsteps[Y_AXIS] = count_position[Y_AXIS];
					endstop_y_hit=TRUE;
					step_events_completed = current_block->step_event_count;
				}
				old_y_max_endstop = y_max_endstop;
				#endif
			}
		}

		if ((out_bits & (1<<Z_AXIS)) != 0) {   // -direction
			//WRITE(Z_DIR_PIN,INVERT_Z_DIR);
			count_direction[Z_AXIS]=-1;
			if(current_block->enable_endstops) {
				#if defined(Z_MIN_PIN) && Z_MIN_PIN > -1
				unsigned char z_min_endstop=(READ(Z_MIN_PIN_READ) != Z_MIN_ENDSTOP_INVERTING);
				if(z_min_endstop && old_z_min_endstop && (current_block->steps_z > 0)) {
					endstops_trigsteps[Z_AXIS] = count_position[Z_AXIS];
					endstop_z_hit=TRUE;
					step_events_completed = current_block->step_event_count;
				}
				old_z_min_endstop = z_min_endstop;
				#endif
			}
		} else { // +direction
			//WRITE(Z_DIR_PIN,!INVERT_Z_DIR);
			count_direction[Z_AXIS]=1;
			if(current_block->enable_endstops) {
				#if defined(Z_MAX_PIN) && Z_MAX_PIN > -1
				unsigned char z_max_endstop=(READ(Z_MAX_PIN_READ) != Z_MAX_ENDSTOP_INVERTING);
				if(z_max_endstop && old_z_max_endstop && (current_block->steps_z > 0)) {
					endstops_trigsteps[Z_AXIS] = count_position[Z_AXIS];
					endstop_z_hit=TRUE;
					step_events_completed = current_block->step_event_count;
				}
				old_z_max_endstop = z_max_endstop;
				#endif
			}
		}

		if ((out_bits & (1<<E_AXIS)) != 0) {  // -direction
			//REV_E_DIR();
			count_direction[E_AXIS]=-1;
		} else { // +direction
			//NORM_E_DIR();
			count_direction[E_AXIS]=1;
		}

		for(i=0; i < step_loops; i++) { // Take multiple steps per interrupt (For high speed moves)

			counter_x += current_block->steps_x;
			if (counter_x > 0) {
				//WRITE(X_STEP_PIN, !INVERT_X_STEP_PIN);
				counter_x -= current_block->step_event_count;
				count_position[X_AXIS]+=count_direction[X_AXIS];
				//WRITE(X_STEP_PIN, INVERT_X_STEP_PIN);
			}

			counter_y += current_block->steps_y;
			if (counter_y > 0) {
				//WRITE(Y_STEP_PIN, !INVERT_Y_STEP_PIN);
				counter_y -= current_block->step_event_count;
				count_position[Y_AXIS]+=count_direction[Y_AXIS];
				//WRITE(Y_STEP_PIN, INVERT_Y_STEP_PIN);
			}

			counter_z += current_block->steps_z;
			if (counter_z > 0) {
				//WRITE(Z_STEP_PIN, !INVERT_Z_STEP_PIN);
				counter_z -= current_block->step_event_count;
				count_position[Z_AXIS]+=count_direction[Z_AXIS];
				//WRITE(Z_STEP_PIN, INVERT_Z_STEP_PIN);
			}

			counter_e += current_block->steps_e;
			if (counter_e > 0) {
				//WRITE_E_STEP(!INVERT_E_STEP_PIN);
				counter_e -= current_block->step_event_count;
				count_position[E_AXIS]+=count_direction[E_AXIS];
				//WRITE_E_STEP(INVERT_E_STEP_PIN);
			}

			step_events_completed += 1;
			if(step_events_completed >= current_block->step_event_count) break;
		}

		// Calculare new timer value
		unsigned short timer;
		unsigned short step_rate;
		if (step_events_completed <= (unsigned long int)current_block->accelerate_until) {

			acc_step_rate = (((unsigned long long int)((unsigned long long int)acceleration_time * (unsigned long long int)current_block->acceleration_rate) & 0xFFFF000000) >> 24);
			acc_step_rate += current_block->initial_rate;

			// upper limit
			if(acc_step_rate > current_block->nominal_rate)
				acc_step_rate = current_block->nominal_rate;

			// step_rate to timer interval
			timer = calc_timer(acc_step_rate);
			/* TODO: FIXME */
			//OCR1A = timer;
			/* TODO: FIXME */
			acceleration_time += timer;
		} else if (step_events_completed > (unsigned long int)current_block->decelerate_after) {
			step_rate = (((unsigned long long int)((unsigned long long int)deceleration_time * (unsigned long long int)current_block->acceleration_rate) & 0xFFFF000000) >> 24);

			if(step_rate > acc_step_rate) { // Check step_rate stays positive
				step_rate = current_block->final_rate;
			} else {
				step_rate = acc_step_rate - step_rate; // Decelerate from aceleration end point.
			}

			// lower limit
			if(step_rate < current_block->final_rate)
				step_rate = current_block->final_rate;

			// step_rate to timer interval
			timer = calc_timer(step_rate);
			/* TODO: FIXME */
			//OCR1A = timer;
			/* TODO: FIXME */
			deceleration_time += timer;
		} else {
			/* TODO: FIXME */
			//OCR1A = OCR1A_nominal;
			/* TODO: FIXME */
			// ensure we're running at the correct step rate, even if we just came off an acceleration
			step_loops = step_loops_nominal;
		}

		__delay_cycles(20000000);

		while (pru_rpmsg_send(transport, src, dst, &timer, sizeof(timer)) != PRU_RPMSG_SUCCESS);
		while (pru_rpmsg_send(transport, src, dst, &acc_step_rate, sizeof(acc_step_rate)) != PRU_RPMSG_SUCCESS);
		while (pru_rpmsg_send(transport, src, dst, &step_rate, sizeof(step_rate)) != PRU_RPMSG_SUCCESS);
		// If current block is finished, reset pointer
	} while(step_events_completed < current_block->step_event_count);

	return (unsigned char)((endstop_x_hit) || (endstop_y_hit << 1) || (endstop_z_hit << 1));
}
