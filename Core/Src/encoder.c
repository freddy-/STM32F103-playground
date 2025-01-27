#include "encoder.h"

int16_t encoderValue = 0;
int16_t prevEncoderValue = 0;

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
  encoderValue = (int16_t)__HAL_TIM_GET_COUNTER(htim) / TICKS_PER_STEP;
}

int16_t getEncoderValue() {
  return encoderValue;
}

int16_t getPrevEncoderValue() {
  return prevEncoderValue;
}

uint8_t isEncoderChanged() {
  return prevEncoderValue != encoderValue;
}

int16_t getEncoderValueDiff() {
  int16_t diff = encoderValue - prevEncoderValue;
  prevEncoderValue = encoderValue;
  return diff;
}
