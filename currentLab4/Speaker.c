//Speaker.c

#include <stdint.h>
#include "tm4c123gh6pm.h"
#include "Speaker.h"
#include "ST7735.h"
#define PF2                     (*((volatile uint32_t *)0x40025010))

int time_alarm = (8*3600 +46*60) + 60;

int alarm = 0;
int inAlarm = 0;

int checkAlarm(int t){
	if(t == time_alarm){
		return 1;
	}
	return 0;
}
