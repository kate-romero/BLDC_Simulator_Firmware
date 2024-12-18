#ifndef MY_PRINT_H
#define MY_PRINT_H

#include "shared_data.h"

#define POSITION_BUFF           1
#define VELOCITY_BUFF           2
#define INVALID_CNT_BUFF        3
#define TEMP_BUFF               4

extern int serialPrint;

struct print_buff
{
  int full;
  int contents;
};

struct print_buff_temp
{
  int full;
  float contents;
};

struct print_circle_buff
{
  int full;
  int count;
  int contents;
};

extern struct print_buff printPositionBuff;

extern struct print_buff printInvalidCntBuff;

extern struct print_circle_buff printVelocityBuff;

extern struct print_buff_temp printTempBuff;

void mySerialPrint(String);

void mySerialPrintln(String);

void sendToPrinter(int, int, int);

void sendToPrinter(int, float, int);

void monitorPrintBuffs();

void emptyPrintBuffs(int);

#endif