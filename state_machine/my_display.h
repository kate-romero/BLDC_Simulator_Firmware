#ifndef MY_DISPLAY_H
#define MY_DISPLAY_H

#include "shared_data.h"

#define OLED_MOSI       3
#define OLED_CLK        2
#define OLED_DC         7
#define OLED_CS         5
#define OLED_RST        6
#define CURSOR_X        0
#define CURSOR_Y        0
#define STANDARD_FONT   1
#define HEADER_FONT     2
#define MODE_BOX_X      76
#define MODE_BOX_Y      0
#define POS_BOX_X       0
#define POS_BOX_Y       48
#define ITEMS_PER_SCREEN    4

#define TWOG_LOGO                   0
#define ACTUATOR_POSITION           1
#define ACTUATOR_POSITION_CONFIG    2
#define ACTUATOR_SELECT             3
#define ACTUATOR_VELOCITY           4
#define ACTUATOR_VELOCITY_CONFIG    5
#define MODE_SELECT                 6
#define MOTOR_ONLY                  7
#define MOTOR_ONLY_CONFIG           8
#define INTERCEPT_SELECT            9
#define INTERCEPT_POSITION_ACTUATOR 10
#define INTERCEPT_POSITION_MOTOR    11
#define INTERCEPT_VELOCITY_ACTUATOR 12
#define INTERCEPT_VELOCITY_MOTOR    13
#define INTERCEPT_POSITION_CONFIG   14
#define INTERCEPT_VELOCITY_CONFIG   15
#define PASSTHROUGH                 16
#define PASSTHROUGH_CONFIG          17
#define WARN_NO_SERIAL              18

// #define TEMP                0
// #define VOLTAGE             1
// #define SLP                 2
// #define BATTERY_CRITICAL    3
// #define BATTERY_LOW         4

// TODO: move to control loop only?
extern int optionCnt[];

extern int selector[];

extern int topY[];

void setupDisplay();

void hover(int, int);

void select(int, int);

void fillCheckbox(int, int, int);

void toggle(int, int);

void displayPosition(int, int, int);

void displayActuatorVelocity(int, int);

void displayInvalidCnt(int);

void displayMotorVelocity(int);

void displayBitmap(int);

void displayTempBitmap();

void displayVoltageBitmap();

void displayMV(int);

void displayTemp(float);

void displayCountsPerRev(int);

void displayThermType(int);

void displayStateMode(int);

void displaySlp();

void displayBatteryLow();

void displayBatteryCritical();

void displayClearSlp();

void displayClearBattery();

void loadCheckboxes(int);

void scroll(int, int);

#endif