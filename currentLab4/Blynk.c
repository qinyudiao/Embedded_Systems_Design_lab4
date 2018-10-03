// -------------------------------------------------------------------
// File name: Blynk.c
// Description: This code is used to bridge the TM4C123 board and the Blynk Application
//              via the ESP8266 WiFi board
// Author: Mark McDermott and Andrew Lynch (Arduino source)
// Converted to EE445L style Jonathan Valvano
// Orig gen date: May 21, 2018
// Last update: Sept 20, 2018
//
// Download latest Blynk library here:
//   https://github.com/blynkkk/blynk-library/releases/latest
//
//  Blynk is a platform with iOS and Android apps to control
//  Arduino, Raspberry Pi and the likes over the Internet.
//  You can easily build graphic interfaces for all your
//  projects by simply dragging and dropping widgets.
//
//   Downloads, docs, tutorials: http://www.blynk.cc
//   Sketch generator:           http://examples.blynk.cc
//   Blynk community:            http://community.blynk.cc
//
//------------------------------------------------------------------------------

// TM4C123       ESP8266-ESP01 (2 by 4 header)
// PE5 (U5TX) to Pin 1 (Rx)
// PE4 (U5RX) to Pin 5 (TX)
// PE3 output debugging
// PE2 nc
// PE1 output    Pin 7 Reset
// PE0 input     Pin 3 Rdy IO2
//               Pin 2 IO0, 10k pullup to 3.3V  
//               Pin 8 Vcc, 3.3V (separate supply from LaunchPad 
// Gnd           Pin 4 Gnd  
// Place a 4.7uF tantalum and 0.1 ceramic next to ESP8266 3.3V power pin
// Use LM2937-3.3 and two 4.7 uF capacitors to convert USB +5V to 3.3V for the ESP8266
// http://www.ti.com/lit/ds/symlink/lm2937-3.3.pdf
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "tm4c123gh6pm.h"
#include "ST7735.h"
#include "PLL.h"
#include "Timer2.h"
#include "Timer3.h"
#include "UART.h"
#include "PortF.h"
#include "esp8266.h"
#include "LCD.h"
#include "Timer.h"
#include "Systick.h"
#include "Speaker.h"

#define Factory_Time (8*3600 +46*60) - 25
#define Factory_Alarm (8*3600 +46*60) + 60

void EnableInterrupts(void);    // Defined in startup.s
void DisableInterrupts(void);   // Defined in startup.s
void WaitForInterrupt(void);    // Defined in startup.s
long StartCritical (void);    // previous I bit, disable interrupts
void EndCritical(long sr);    // restore I bit to previous value
void ButtonControl(uint32_t value, uint32_t button_num);
void PhaseControl(uint32_t phase, uint32_t tempTime);
void CheckInactiveTime(void);
void ResetToFactory(int isResetToFactory);
void PortD_Init(void);

uint32_t LED;      // VP1
uint32_t LastF;    // VP74
// These 6 variables contain the most recent Blynk to TM4C123 message
// Blynk to TM4C123 uses VP0 to VP15
char serial_buf[64];
char Pin_Number[2]   = "99";       // Initialize to invalid pin number
char Pin_Integer[8]  = "0000";     //
char Pin_Float[8]    = "0.0000";   //
uint32_t pin_num; 
uint32_t pin_int;
extern int time, secFlag;
extern char sec[], min[], hour[];
char sec_sw[2], min_sw[2];
extern uint32_t h,m,s;
int temp_t;
extern int time_alarm, alarm, inAlarm;
int lastTimePressed, timeInactive = 0, time_sw = 0, time_d = 0;
int sw_flag = 0, reset_flag = 0, d_flag = 0, isResetToFactory = 0, default_phase = 0;


typedef struct phase_t {
	char *options[5];
	int time[3];
	int8_t  highlight;    // index (out of total)l; -1 for none
	int color[6];     
	uint8_t selected;      // 0 for not selected. 1 for selected
} phase;

phase phases[7] = {
   {{"\n"}, {'\n'}, -1, {'\n'}, 0},  													// phase 0: clock display
   {{"Set Clock", "Set Alarm", "Back", "Stop Watch"}, {'\n'}, 0, {ST7735_YELLOW,ST7735_WHITE,ST7735_WHITE, ST7735_WHITE}, 0},   // phase 1: select menu
   {{"Set", "Back"}, {0, 0, 0}, 2, {ST7735_WHITE,ST7735_WHITE,ST7735_YELLOW,ST7735_WHITE,ST7735_WHITE}, 0},               // phase 2: set time
   {{"Set", "Back"}, {0, 0, 0}, 2, {ST7735_WHITE,ST7735_WHITE,ST7735_YELLOW,ST7735_WHITE,ST7735_WHITE}, 0},	              // phase 3: set alarm
	 {{"Start", "Pause", "Back"}, {0, 0, 0}, 0, {ST7735_WHITE,ST7735_WHITE,ST7735_WHITE,ST7735_WHITE,ST7735_WHITE,ST7735_YELLOW}, 0}, 	// phase 4: stop watch display
	 {{"\n"}, {'\n'}, -1, {'\n'}, 0},  													// phase 5: clock display 2
	 {{"\n"}, {'\n'}, -1, {'\n'}, 0},  													// phase 6: clock display 3
};
uint8_t phase_num = 0;

// ----------------------------------- TM4C_to_Blynk ------------------------------
// Send data to the Blynk App
// It uses Virtual Pin numbers between 70 and 99
// so that the ESP8266 knows to forward the data to the Blynk App
void TM4C_to_Blynk(uint32_t pin,uint32_t value){
  if((pin < 70)||(pin > 99)){
    return; // ignore illegal requests
  }
// your account will be temporarily halted if you send too much data
  ESP8266_OutUDec(pin);       // Send the Virtual Pin #
  ESP8266_OutChar(',');
  ESP8266_OutUDec(value);      // Send the current value
  ESP8266_OutChar(',');
  ESP8266_OutString("0.0\n");  // Null value not used in this example
}
 
 
// -------------------------   Blynk_to_TM4C  -----------------------------------
// This routine receives the Blynk Virtual Pin data via the ESP8266 and parses the
// data and feeds the commands to the TM4C.
void Blynk_to_TM4C(void){int j; char data;
// Check to see if a there is data in the RXD buffer
  if(ESP8266_GetMessage(serial_buf)){  // returns false if no message
    // Read the data from the UART5
#ifdef DEBUG1
    j = 0;
    do{
      data = serial_buf[j];
      UART_OutChar(data);        // Debug only
      j++;
    }while(data != '\n');
    UART_OutChar('\r');        
#endif
           
// Rip the 3 fields out of the CSV data. The sequence of data from the 8266 is:
// Pin #, Integer Value, Float Value.
    strcpy(Pin_Number, strtok(serial_buf, ","));
    strcpy(Pin_Integer, strtok(NULL, ","));       // Integer value that is determined by the Blynk App
    strcpy(Pin_Float, strtok(NULL, ","));         // Not used
    pin_num = atoi(Pin_Number);     // Need to convert ASCII to integer
    pin_int = atoi(Pin_Integer);  
  // ---------------------------- VP #1 ----------------------------------------
  // This VP is the LED select button
    if(pin_num == 0x01)  {  //SELECT
      LED = pin_int;
      PortF_Output(LED<<2); // Blue LED
			int temp = pin_int;
			ButtonControl(temp, 1);
			
#ifdef DEBUG3
 //     Output_Color(ST7735_CYAN);
 //     ST7735_OutString("Rcv VP1 data=");
 //     ST7735_OutUDec(LED);
 //     ST7735_OutChar('\n');
#endif
    }                               // Parse incoming data   
// ---------------------------- VP #2 ----------------------------------------
			if(pin_num == 0x02)  {	//SCROLL DOWN
				int temp = pin_int;
				ButtonControl(temp, 2);
			}
// ---------------------------- VP #3 ----------------------------------------
			if(pin_num == 0x03)  {	//SCROLL UP
				int temp = pin_int;
				ButtonControl(temp, 3);
			}
// ---------------------------- VP #4 ----------------------------------------
			if(pin_num == 0x04)  {	//MODE
				int temp = pin_int;
				ButtonControl(temp, 4);
			}
// ---------------------------- VP #0 ----------------------------------------
			if(pin_num == 0x00)  {	//FACTORY
				int temp = pin_int;
				ButtonControl(temp, 0);
			}
#ifdef DEBUG1
//    UART_OutString(" Pin_Number = ");
//    UART_OutString(Pin_Number);
//    UART_OutString("   Pin_Integer = ");
//    UART_OutString(Pin_Integer);
//    UART_OutString("   Pin_Float = ");
//    UART_OutString(Pin_Float);
//    UART_OutString("\n\r");
#endif
  }  
}

void SendInformation(void){
  uint32_t thisF;
  thisF = time;
// your account will be temporarily halted if you send too much data
  if(thisF != LastF){
    TM4C_to_Blynk(74, thisF / 3600);  // VP74
		TM4C_to_Blynk(75, (thisF % 3600) / 60);  // VP75
		TM4C_to_Blynk(76, thisF % 60);  // VP76
#ifdef DEBUG3
    Output_Color(ST7735_WHITE);
   // ST7735_OutString("Send 74 data=");
    //ST7735_OutUDec(thisF);
    //ST7735_OutChar('\n');
#endif
  }
  LastF = thisF;
}

  
int main(void){       
  PLL_Init(Bus80MHz);   // Bus clock at 80 MHz
  DisableInterrupts();  // Disable interrupts until finished with inits
  PortF_Init();
	PortD_Init();
	SysTick_Init();
  LastF = PortF_Input();
#ifdef DEBUG3
  Output_Init();        // initialize ST7735
  //ST7735_OutString("EE445L Lab 4D\nBlynk example\n");
	drawFace();
	drawHands(time);
	outputTime(time, 2);
	ST7735_DrawString(2,4,"Clock Starting...", ST7735_YELLOW);
#endif
#ifdef DEBUG1
  UART_Init(5);         // Enable Debug Serial Port
  //UART_OutString("\n\rEE445L Lab 4D\n\rBlynk example");
#endif
  ESP8266_Init();       // Enable ESP8266 Serial Port
  ESP8266_Reset();      // Reset the WiFi module
  ESP8266_SetupWiFi();  // Setup communications to Blynk Server  
  
  Timer2_Init(&Blynk_to_TM4C,800000); 
  // check for receive data from Blynk App every 10ms

  Timer3_Init(&SendInformation,40000000); 
  // Send data back to Blynk App every 1/2 second
  EnableInterrupts();

  while(1) {
		long sr = StartCritical();
		int tempTime = time;
		time = updateTime(secFlag, time);
		alarm = checkAlarm(time);
		if(time != tempTime){ // if time changed, redraw, reset flag, check alarm
         secFlag = 0;
    }
    //WaitForInterrupt(); // low power mode
		CheckInactiveTime();
		PhaseControl(phase_num, tempTime);
		EndCritical(sr);
		ResetToFactory(isResetToFactory);
	}
}

void PhaseControl(uint32_t phase, uint32_t tempTime){
			lastTimePressed = time;
	 		switch (phase) {
         case 0:
         if(time != tempTime){ // if time changed, redraw, reset flag, check alarm
            outputTime(time, 2);	
         }
         if((time % 60) == 0){ // every minute, erase hand and draw again
            eraseHands(time - 60);
            drawHands(time);
         }
         break;
				 
         case 1:
         ST7735_DrawString(6,4,phases[1].options[0], phases[1].color[0]);
         ST7735_DrawString(6,6,phases[1].options[1], phases[1].color[1]);
         ST7735_DrawString(6,8,phases[1].options[2], phases[1].color[2]);
				 ST7735_DrawString(6,10,phases[1].options[3], phases[1].color[3]);
         CheckInactiveTime();
         break;
				 
         case 2:
					 //ADCvalue = ADC0_InSeq3();
					 if (phases[2].selected) {
             if (phases[2].highlight == 2) { // hour: 1-12
               //h = ADCvalue*12/4096 + 1; // use 4096 here to avoid 13
							 h = temp_t/3600;
               if (h < 10 && h > 0) {
                  hour[0] = '0';
                  hour[1] = 48 + h;
               } else {
                  hour[0] = 48 + h/10;
                  hour[1] = 48 + h%10;
               }
							 if (h == 0) {
									hour[0] = '1';
									hour[1] = '2';
							 }
            }
            else if (phases[2].highlight == 3) { // min: 0-59
               //m = ADCvalue*60/4096; // use 4096 to avoid 60
							 m = (temp_t % 3600) / 60;
               if (m < 10) {
                  min[0] = '0';
                  min[1] = 48 + m;
               } else {
                  min[0] = 48 + m/10;
                  min[1] = 48 + m%10;
               }   
            }
            else if (phases[2].highlight == 4) { // sec: 0-59
               //s = ADCvalue*60/4096;
               s = temp_t % 60;
               if (s < 10) {
                  sec[0] = '0';
                  sec[1] = 48 + s;
               } else {
                  sec[0] = 48 + s/10;
                  sec[1] = 48 + s%10;
               }   
            }
         }
         ST7735_DrawString(8,8,phases[2].options[0], phases[2].color[0]);
         ST7735_DrawString(8,10,phases[2].options[1], phases[2].color[1]);
         ST7735_SetCursor(6, 6);
         
         ST7735_DrawString(6,6,hour,phases[2].color[2]);
         ST7735_DrawString(8,6,":",ST7735_WHITE);
         ST7735_DrawString(9,6,min,phases[2].color[3]);
         ST7735_DrawString(11,6,":",ST7735_WHITE);
         ST7735_DrawString(12,6,sec,phases[2].color[4]);
				 CheckInactiveTime();
         break;
				 
         case 3:
					 //ADCvalue = ADC0_InSeq3();
					 if (phases[3].selected) {
            if (phases[3].highlight == 2) { // hour: 1-12
               //h = ADCvalue*12/4096 + 1; // use 4096 here to avoid 13
							 h = temp_t/3600;
               if (h < 10 && h > 0) {
                  hour[0] = '0';
                  hour[1] = 48 + h;
               } else {
                  hour[0] = 48 + h/10;
                  hour[1] = 48 + h%10;
               }
							 if (h == 0) {
									hour[0] = '1';
									hour[1] = '2';
							 }							 
            }
            else if (phases[3].highlight == 3) { // min: 0-59
               //m = ADCvalue*60/4096; // use 4096 to avoid 60
							 m = (temp_t % 3600) / 60;
               if (m < 10) {
                  min[0] = '0';
                  min[1] = 48 + m;
               } else {
                  min[0] = 48 + m/10;
                  min[1] = 48 + m%10;
               }   
            }
            else if (phases[3].highlight == 4) { // sec: 0-59
               //s = ADCvalue*60/4096;
               s = temp_t % 60;
               if (s < 10) {
                  sec[0] = '0';
                  sec[1] = 48 + s;
               } else {
                  sec[0] = 48 + s/10;
                  sec[1] = 48 + s%10;
               }   
            }
         }
           ST7735_DrawString(8,8,phases[3].options[0], phases[3].color[0]);
           ST7735_DrawString(8,10,phases[3].options[1], phases[3].color[1]);
           ST7735_SetCursor(6, 6);
           ST7735_DrawString(6,6,hour,phases[3].color[2]);
           ST7735_DrawString(8,6,":",ST7735_WHITE);
           ST7735_DrawString(9,6,min,phases[3].color[3]);
           ST7735_DrawString(11,6,":",ST7735_WHITE);
           ST7735_DrawString(12,6,sec,phases[3].color[4]);
				 CheckInactiveTime();
         break;
				 
				 case 4:
				 if(time != tempTime && sw_flag == 1){ // if time changed, redraw, reset flag, check alarm
						time_sw = time - time_d;
            outputTimer(time_sw, 6);	
           }
				 ST7735_SetTextColor(ST7735_WHITE);
				 outputTimer(time_sw, 6);
				 ST7735_DrawString(8,8,phases[4].options[0], phases[4].color[0]);
         ST7735_DrawString(8,10,phases[4].options[1], phases[4].color[1]);
				 ST7735_DrawString(8,12,phases[4].options[2], phases[4].color[2]);
				 break;
					
				 case 6:
						outputTime(time, 7);
						if(time != tempTime){ // if time changed, redraw, reset flag, check alarm
							outputTime(time, 7);	
						}
         break;
				 
				 case 5:
					if(time != tempTime){
						drawFace();
						drawHands(time);
						if((time % 60) == 0){ // every minute, erase hand and draw again
							drawFace();
							eraseHands(time-60);
							drawHands(time);
						}
					}
					break;
      }
}

void ButtonControl(uint32_t value, uint32_t num){
// ********************************When PF0/SW2 is pressed******************************** // GPIO_PORTF_RIS_R&0x01
// ********************************When PF0/SW2 is pressed******************************** // GPIO_PORTF_RIS_R&0x01

// ********************************When V1 is pressed********************************
   if (value == 1 && num == 1) {	// select
      //GPIO_PORTF_IM_R &= ~0x10;     // disarm interrupt on PF4
      if(1){    // 0x10 means it was previously released; negative logic
         switch (phase_num) {
            case 0:  //enter phase 1
							phase_num  = 1;
							ST7735_FillScreen(ST7735_BLACK);   // clear the screen
            break;
            
            case 1:
            if (phases[1].highlight == 0){
               phase_num = 2;
							 temp_t = time;
               // make the initial time display current time
               getSeconds(time, sec);  // with exactly 2 digits
               getMinutes(time, min);  // with exactly 2 digits
               getHours(time, hour);  // with exactly 2 digits  
               ST7735_FillScreen(ST7735_BLACK);   // clear the screen							
            }
            else if (phases[1].highlight == 1) {
               phase_num = 3;
							 temp_t = time_alarm;
               getSeconds(time_alarm, sec);  // with exactly 2 digits
               getMinutes(time_alarm, min);  // with exactly 2 digits
               getHours(time_alarm, hour);  // with exactly 2 digits
               ST7735_FillScreen(ST7735_BLACK);   // clear the screen							
            }
            else if (phases[1].highlight == 2) {
               phase_num = default_phase;
               ST7735_FillScreen(ST7735_BLACK);   // clear the screen
							 if(default_phase == 0 || default_phase == 5){
									drawFace();
								  drawHands(time);
							 }
							 else outputTime(time, 7);
               if(default_phase == 0 ) {
									outputTime(time, 2);
							 }
            }
						/* stop watch */
						else if (phases[1].highlight == 3) {
               phase_num = 4;
               ST7735_FillScreen(ST7735_BLACK);   // clear the screen
							 outputTime(time, 2);
            }
            break;
            
            case 2:
            if (phases[2].highlight >= 2 && phases[2].highlight <= 4) {
               if (phases[2].selected == 1) {
                  phases[2].selected = 0;
                  phases[2].color[phases[2].highlight] = ST7735_YELLOW;
               }
               else if (phases[2].selected == 0) {
                  phases[2].selected = 1;
                  phases[2].color[phases[2].highlight] = ST7735_BLUE;
               }
            }
            else if (phases[2].highlight == 0) {  // save
               phase_num = 1;
               time = (h%12) * 3600 + m * 60 + s;
               ST7735_FillScreen(ST7735_BLACK);
            }
            else if (phases[2].highlight == 1) {
               phase_num = 1;
               ST7735_FillScreen(ST7735_BLACK);
            }
            break;
						
            case 3:
            if (phases[3].highlight >= 2 && phases[3].highlight <= 4) {
               if (phases[3].selected == 1) {
                  phases[3].selected = 0;
                  phases[3].color[phases[3].highlight] = ST7735_YELLOW;
               }
               else if (phases[3].selected == 0) {
                  phases[3].selected = 1;
                  phases[3].color[phases[3].highlight] = ST7735_BLUE;
               }
            }
            else if (phases[3].highlight == 0) {  // save
               phase_num = 1;
               time_alarm = (h%12) * 3600 + m * 60 + s;
               ST7735_FillScreen(ST7735_BLACK);
            }
            else if (phases[3].highlight == 1) {
               phase_num = 1;
               ST7735_FillScreen(ST7735_BLACK);
               
            }
            break;
						
						case 4:
            if (phases[4].highlight == 0){
							if(d_flag == 0){
								time_d = time;
								d_flag ++;
							}
							sw_flag = (sw_flag+1)%2;
							if(sw_flag == 1)
								time_sw = time - time_d;
                  // start the timer				
            }
            else if (phases[4].highlight == 1) {
							sw_flag = 0;
							phases[4].options[1] = "Reset";
							//time_sw = time_sw;
							reset_flag ++;
							if(reset_flag){
								time_sw = time - time_d;
							}
							if(reset_flag == 2){
								time_sw = 0;
								time_d = time;
								d_flag = 0;
								reset_flag = 0;
								phases[4].options[1] = "Pause";
							}
                 // pause or reset the timer					
            }
            else if (phases[4].highlight == 2) {
               phase_num = 1;
               ST7735_FillScreen(ST7735_BLACK);   // clear the screen
            }
						break;
						
						case 5:  //enter phase 6
							 phase_num  = 1;
							 ST7735_FillScreen(ST7735_BLACK);   // clear the screen
            break;
						
						case 6:  //enter phase 0
							 phase_num  = 1;
							 ST7735_FillScreen(ST7735_BLACK);   // clear the screen
            break;
         }
      }

   }
 
// ********************************When V2 is pressed******************************** 
   if (value == 1 && num == 2) {  // down
      //GPIO_PORTF_IM_R &= ~0x01;     // disarm interrupt on PF0
      long sr;
			if(1){    // 0x01 means it was previously released; negative logic
         switch (phase_num) {
            default: 			
							if(inAlarm == 1)
								phase_num = default_phase;
							sr = StartCritical();
							inAlarm = 0;
							EndCritical(sr);
            // alarm
            break;
            
            case 1:
            phases[1].color[phases[1].highlight] = ST7735_WHITE;
            phases[1].highlight = (phases[1].highlight+1)%4;
            phases[1].color[phases[1].highlight] = ST7735_YELLOW;
            break;
            
            case 2:
							if (phases[2].selected) {
								if (phases[2].highlight == 2) { // hour: 1-12
									temp_t -= 3600;
								}
								else if (phases[2].highlight == 3) { // min: 0-59
									temp_t -= 60;
								}
								else if (phases[2].highlight == 4) { // sec: 0-59
									 temp_t --;  
								}
								if (temp_t < 0) temp_t += 43200;
								break;
						 }
            phases[2].color[phases[2].highlight] = ST7735_WHITE;
            phases[2].highlight = (phases[2].highlight+1)%5;
            phases[2].color[phases[2].highlight] = ST7735_YELLOW;
            break;
            
            case 3:
            if (phases[3].selected){
								if (phases[3].highlight == 2) { // hour: 1-12
									temp_t -= 3600;
								}
								else if (phases[3].highlight == 3) { // min: 0-59
									temp_t -= 60;
								}
								else if (phases[3].highlight == 4) { // sec: 0-59
									 temp_t --;  
								}
								if (temp_t < 0) temp_t += 43200;
							break; 
						}
            phases[3].color[phases[3].highlight] = ST7735_WHITE;
            phases[3].highlight = (phases[3].highlight+1)%5;
            phases[3].color[phases[3].highlight] = ST7735_YELLOW;
            break;
						
						case 4:
						phases[4].color[phases[4].highlight] = ST7735_WHITE;
            phases[4].highlight = (phases[4].highlight+1)%3;
            phases[4].color[phases[4].highlight] = ST7735_YELLOW;
						
         }
      }
   }
// ********************************When V3 is pressed********************************
	 if (value == 1 && num == 3) {	// up
		  long sr;
			if(1){    // 0x01 means it was previously released; negative logic
         switch (phase_num) {
            default: 			
							if(inAlarm == 1)
								phase_num = default_phase;
							sr = StartCritical();
							inAlarm = 0;
							EndCritical(sr);
            // alarm
            break;
            
            case 1:
            phases[1].color[phases[1].highlight] = ST7735_WHITE;
            phases[1].highlight = (phases[1].highlight+3)%4;
            phases[1].color[phases[1].highlight] = ST7735_YELLOW;
            break;
            
            case 2:
            if (phases[2].selected){
								if (phases[2].highlight == 2) { // hour: 1-12
									temp_t += 3600;
									if (temp_t > 43200) temp_t -= 43200;
								}
								else if (phases[2].highlight == 3) { // min: 0-59
									temp_t += 60;
									if (temp_t > 43200) temp_t -= 3600;
								}
								else if (phases[2].highlight == 4) { // sec: 0-59
									temp_t ++;  
									if (temp_t > 43200) temp_t -= 60;
								}
								break;
						}
            phases[2].color[phases[2].highlight] = ST7735_WHITE;
            phases[2].highlight = (phases[2].highlight+4)%5;
            phases[2].color[phases[2].highlight] = ST7735_YELLOW;
            break;
            
            case 3:
            if (phases[3].selected){
								if (phases[3].highlight == 2) { // hour: 1-12
									temp_t += 3600;
									if (temp_t > 43200) temp_t -= 43200;
								}
								else if (phases[3].highlight == 3) { // min: 0-59
									temp_t += 60;
									if (temp_t > 43200) temp_t -= 3600;
								}
								else if (phases[3].highlight == 4) { // sec: 0-59
									temp_t ++;  
									if (temp_t > 43200) temp_t -= 60;
								}
							break; 
						}
            phases[3].color[phases[3].highlight] = ST7735_WHITE;
            phases[3].highlight = (phases[3].highlight+4)%5;
            phases[3].color[phases[3].highlight] = ST7735_YELLOW;
            break;
						
						case 4:
						phases[4].color[phases[4].highlight] = ST7735_WHITE;
            phases[4].highlight = (phases[4].highlight+2)%3;
            phases[4].color[phases[4].highlight] = ST7735_YELLOW;
						
         }
      }
	 }
// ********************************When V4 is pressed********************************
   if (value == 1 && num == 4) {	// mode/silence alarm
		 long sr = StartCritical();
		 if(inAlarm == 0){
				switch (phase_num) {
					  default:
								ST7735_FillScreen(ST7735_BLACK);
								EndCritical(sr);						
								break;
					
						case 0:  //enter phase 5
								phase_num = 5;
								default_phase = 5;
								ST7735_FillScreen(ST7735_BLACK); 
								drawFace();
								drawHands(time);
								EndCritical(sr);
            break;
						
						case 5:  //enter phase 6
							 phase_num  = 6;
							 default_phase = 6;
							 outputTime(time, 7);
							 ST7735_FillScreen(ST7735_BLACK);   // clear the screen
							 EndCritical(sr);
            break;
						
						case 6:  //enter phase 0
               phase_num = 0;
							 default_phase = 0;
               ST7735_FillScreen(ST7735_BLACK);   // clear the screen
							 outputTime(time, 2);
               drawFace();
               drawHands(time);
							 EndCritical(sr);
            break;
				}	
		}
		inAlarm = 0;
 }
// ********************************When V0 is pressed******************************** 
	 if (value == 1 && num == 0) {	//Factory
			isResetToFactory = 1;
			alarm = 1;
			inAlarm = 1;
	 }
}


void CheckInactiveTime(void){
		timeInactive = time - lastTimePressed;
		if(timeInactive >= 25){
						 phase_num = default_phase; 
						 ST7735_FillScreen(ST7735_BLACK);   // clear the screen
						 if(default_phase == 0 || default_phase == 5){
								drawFace();
								drawHands(time);
						 }
						 else outputTime(time, 7);
             if(default_phase == 0 ) {
								outputTime(time, 2);
						 }
		}
}


void ResetToFactory(int isFactory){
		if(isFactory){
			time = Factory_Time;
			time_alarm = Factory_Alarm;
			time_sw = 0;
			isResetToFactory = 0;
			alarm = 0;
			inAlarm = 0;
			default_phase = 0;
			phase_num = 0;
		}
}
/* initialize PortD */
void PortD_Init(void){
   SYSCTL_RCGCGPIO_R |= SYSCTL_RCGC2_GPIOD;        // 1) activate port D
   uint8_t delay = SYSCTL_RCGC2_R; ;   // allow time for clock to stabilize
   // 2) no need to unlock PD3-0
   GPIO_PORTD_AMSEL_R &= ~0x0F;      // 3) disable analog functionality on PD3-0
   GPIO_PORTD_PCTL_R &= ~0x0000FFFF; // 4) GPIO
   GPIO_PORTD_DIR_R |= 0x0F;         // 5) make PD0 out
   GPIO_PORTD_AFSEL_R &= ~0x0F;      // 6) regular port function
   GPIO_PORTD_DEN_R |= 0x0F;         // 7) enable digital I/O on PD0
}
