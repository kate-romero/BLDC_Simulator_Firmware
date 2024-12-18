#include "my_print.h"
#include "my_display.h"
#include "positive_mod.h"

#define VELOCITY_COUNT_RATE   12

struct print_buff printPositionBuff = {0, 0};
struct print_buff printInvalidCntBuff = {0, 0};
struct print_circle_buff printVelocityBuff = {0, 0, 0};
struct print_buff_temp printTempBuff = {0, 0};

void mySerialPrint(String str)
{
  if(serialPrint)
  {
    Serial.print(str);
  }
}

void mySerialPrintln(String str)
{
  if(serialPrint)
  {
    Serial.println(str);
  }
}

void sendToPrinter(int buff, int message, int mode)
{
  // Serial.print("--- SENT TO BUFF: ");
  // Serial.println(buff);
  switch (buff)
  {
    case POSITION_BUFF:
      printPositionBuff.contents = message;
      printPositionBuff.full = 1;
      break;
    case VELOCITY_BUFF:
      printVelocityBuff.count = positiveMod(printVelocityBuff.count + 1, VELOCITY_COUNT_RATE);
      if(!printVelocityBuff.count)
      {
        // Serial.print("--- MESSAGE: ");
        // Serial.println(message);
        printVelocityBuff.contents = calculateRPM(message, mode);
        // Serial.print("--- CONTENTS: ");
        // Serial.println(printVelocityBuff.contents);
        printVelocityBuff.full = 1;
      }
      break;
    case INVALID_CNT_BUFF:
      printInvalidCntBuff.contents = message;
      printInvalidCntBuff.full = 1;
      break;
    default:
      Serial.print("ERROR: received unexpected buff choice: ");
      Serial.println(buff);
      break;
  }
}

void sendToPrinter(int buff, float message, int mode)
{
  switch (buff)
  {
    case TEMP_BUFF:
      printTempBuff.contents = message;
      printTempBuff.full = 1;
      break;
    default:
      Serial.print("ERROR: received unexpected buff choice: ");
      Serial.println(buff);
      break;
  }
}

void monitorPrintBuffs()
{
  if(printPositionBuff.full)
  {
    displayPosition(printPositionBuff.contents, motorOnlyConfig.countsPerRev, 1);
    printPositionBuff.full = 0;
  }
  if(printInvalidCntBuff.full)
  {
    displayInvalidCnt(printInvalidCntBuff.contents);
    printInvalidCntBuff.full = 0;
  }
  if(printVelocityBuff.full)
  {
    displayMotorVelocity(printVelocityBuff.contents);
    printVelocityBuff.full = 0;
  }
  if(printTempBuff.full)
  {
    displayTemp(printTempBuff.contents);
    printTempBuff.full = 0;
  }
}

void emptyPrintBuffs(int currScreen)
{
  printPositionBuff.contents = 0;
  printPositionBuff.full = 0;
  printVelocityBuff.count = 0;
  printVelocityBuff.contents = 0;
  printVelocityBuff.full = 0;
  printInvalidCntBuff.contents = 0;
  printInvalidCntBuff.full = 0;
  printTempBuff.contents = 0;
  printTempBuff.full = 0;
}
