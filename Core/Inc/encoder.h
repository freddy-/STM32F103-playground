#ifndef  _ENCODER_H_
#define  _ENCODER_H_

#include "main.h"

#define TICKS_PER_STEP 4 // each step of the encoder sends 4 events

/**
 * Get previous encoder value
 */
int16_t getPrevEncoderValue();

/**
 * Get current encoder value
 */
int16_t getEncoderValue();

/**
 * Check if the encoder has changed
 */
uint8_t isEncoderChanged();

/**
 * Get the difference between last read value and current value and reset the last value.
 */
int16_t getEncoderValueDiff();

#endif // _ENCODER_H_
