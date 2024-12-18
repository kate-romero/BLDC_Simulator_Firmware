/*
 * DEPRECIATED: does not incorporate new npxManager
 */
#include "blinker.h"
#include <Adafruit_NeoPixel.h>

void update(struct blinker *blinker, uint32_t curTime)
{
	blinker->prevTime = curTime;
	blinker->lightsOn = !(blinker->lightsOn);
}

void blink(struct blinker *blinker)
{
	uint32_t now = time_us_32();
	if(!(blinker->lightsOn))
	{
		if(now >= blinker->prevTime + blinker->offDur)
		{
			for(int i = blinker->first; i < blinker->last + 1; i++)
			{
				(*(blinker->pixels)).setPixelColor(i, blinker->colorOn);
			}
			(*(blinker->pixels)).show();
			update(blinker, now);
		}
	}
	else
	{
		if(now >= blinker->prevTime + blinker->onDur)
		{
			for(int i = blinker->first; i < blinker->last + 1; i++)
			{
				(*(blinker->pixels)).setPixelColor(i, blinker->colorOff);
			}
			(*(blinker->pixels)).show();
			update(blinker, now);
		}
	}
}
