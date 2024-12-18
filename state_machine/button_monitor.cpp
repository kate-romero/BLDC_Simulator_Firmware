#include "button_monitor.h"
#include <Arduino.h>
#include "my_print.h"

#define LEFT_BTN_PIN		12
#define UP_BTN_PIN		14
#define CENTER_BTN_PIN	        18
#define RIGHT_BTN_PIN		24
#define DOWN_BTN_PIN		25
#define DEBOUNCE_DELAY	        50

int buttons[] = {0, 12, 14, 18, 24, 25};
int buttonsLength = (int)(sizeof(buttons) / sizeof(*buttons));
int buttonStates[] = {0, 1, 1, 1, 1, 1};
unsigned long buttonPollTimes[] = {0, 0, 0, 0, 0, 0};
const char *buttonNames[] = {NULL, "LEFT", "UP", "CENTER", "RIGHT", "DOWN"};

void setupButtons()
{
	for(int i = 1; i < buttonsLength; i++)
  {
    pinMode(buttons[i], INPUT_PULLUP);
  }
}

int buttonMonitor()
{
  for(int i = 1; i < buttonsLength; i++)
  {
    if(debounceDetectButton(i))
    {
      lastUserInteractionTimeUS = time_us_32();
      mySerialPrint(buttonNames[i]);
      mySerialPrintln(" pressed!");
      return i;
    }
  }
  return 0;
}

int debounceDetectButton(int buttonIdx)
{
	int buttonPressed = 0;
  int state = digitalRead(buttons[buttonIdx]);
  if(state != buttonStates[buttonIdx])
  {
    int now;
    if((now = millis()) >= buttonPollTimes[buttonIdx])
    {
      buttonStates[buttonIdx] = state;
      buttonPollTimes[buttonIdx] = now + DEBOUNCE_DELAY;
			if(!state) {
				buttonPressed = 1;
			}
    }
  }
  return buttonPressed;
}
