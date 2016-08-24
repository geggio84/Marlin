/*
  planner.h - buffers movement commands and manages the acceleration profile plan
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

// This module is to be considered a sub-module of stepper.c. Please don't include 
// this file from any other module.

#ifndef planner_h
#define planner_h

#include "Marlin.h"

#ifdef ENABLE_AUTO_BED_LEVELING
#include "vector_3.h"
#endif // ENABLE_AUTO_BED_LEVELING

typedef struct {
	block_t block_buffer[BLOCK_BUFFER_SIZE];            // A ring buffer for motion instfructions
	unsigned int block_buffer_head;           // Index of the next block to be pushed
	unsigned int block_buffer_tail;           // Index of the block to process now
	unsigned int check_endstops;
	long count_position[NUM_AXIS];
} stepper_block_t;

#ifdef ENABLE_AUTO_BED_LEVELING
// this holds the required transform to compensate for bed level
extern matrix_3x3 plan_bed_level_matrix;
#endif // #ifdef ENABLE_AUTO_BED_LEVELING

// Initialize the motion plan subsystem      
void plan_init();

// Add a new linear movement to the buffer. x, y and z is the signed, absolute target position in 
// millimaters. Feed rate specifies the speed of the motion.

#ifdef ENABLE_AUTO_BED_LEVELING
void plan_buffer_line(float x, float y, float z, const float &e, float feed_rate);

// Get the position applying the bed level matrix if enabled
vector_3 plan_get_position();
#else
void plan_buffer_line(const float &x, const float &y, const float &z, const float &e, float feed_rate);
#endif // ENABLE_AUTO_BED_LEVELING

// Set position. Used for G92 instructions.
#ifdef ENABLE_AUTO_BED_LEVELING
void plan_set_position(float x, float y, float z, const float &e);
#else
void plan_set_position(const float &x, const float &y, const float &z, const float &e);
#endif // ENABLE_AUTO_BED_LEVELING

void plan_set_e_position(const float &e);



void check_axes_activity();
uint8_t movesplanned(); //return the nr of buffered moves

extern unsigned long minsegmenttime;
extern float max_feedrate[4]; // set the max speeds
extern float axis_steps_per_unit[4];
extern unsigned long max_acceleration_units_per_sq_second[4]; // Use M201 to override by software
extern float minimumfeedrate;
extern float acceleration;         // Normal acceleration mm/s^2  THIS IS THE DEFAULT ACCELERATION for all moves. M204 SXXXX
extern float retract_acceleration; //  mm/s^2   filament pull-pack and push-forward  while standing still in the other axis M204 TXXXX
extern float max_xy_jerk; //speed than can be stopped at once, if i understand correctly.
extern float max_z_jerk;
extern float max_e_jerk;
extern float mintravelfeedrate;
extern unsigned long axis_steps_per_sqr_second[NUM_AXIS];

#ifdef AUTOTEMP
    extern bool autotemp_enabled;
    extern float autotemp_max;
    extern float autotemp_min;
    extern float autotemp_factor;
#endif

    


extern block_t *block_buffer[BLOCK_BUFFER_SIZE];            // A ring buffer for motion instfructions
extern unsigned int *block_buffer_head;           // Index of the next block to be pushed
extern unsigned int *block_buffer_tail; 
extern stepper_block_t stepper_block_buffer;
// Called when the current block is no longer needed. Discards the block and makes the memory
// availible for new blocks.    
FORCE_INLINE void plan_discard_current_block()  
{
  if (*block_buffer_head != *block_buffer_tail) {
    *block_buffer_tail = (*block_buffer_tail + 1) & (BLOCK_BUFFER_SIZE - 1);  
  }
}

// Returns true if the buffer has a queued block, false otherwise
FORCE_INLINE bool blocks_queued() 
{
  if (*block_buffer_head == *block_buffer_tail) { 
    return false; 
  }
  else
    return true;
}

#ifdef PREVENT_DANGEROUS_EXTRUDE
void set_extrude_min_temp(float temp);
#endif

void reset_acceleration_rates();
#endif
