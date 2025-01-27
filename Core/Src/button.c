#include "button.h"

uint8_t buttonState = 0;
uint32_t lastButtonPress = 0;

void HAL_GPIO_EXTI_Callback(uint16_t pin) {
  uint32_t tick = HAL_GetTick();
  if (tick - lastButtonPress > 200) {
    lastButtonPress = tick;
    buttonState = 1;
  }
}

uint8_t isButonPressed() {
  // get and reset button state
  uint8_t actualState = buttonState;
  buttonState = 0;
  return actualState;
}
