#ifndef BUTTON_MONITOR_H
#define BUTTON_MONITOR_H

#define LEFT_BTN				1
#define UP_BTN					2
#define CENTER_BTN			3
#define RIGHT_BTN				4
#define DOWN_BTN				5

void setupButtons();

int buttonMonitor();

int debounceDetectButton(int);

#endif