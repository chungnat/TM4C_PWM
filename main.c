#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "ports.h"
#include "PLL_Ports.h"
#include "PLL_Init.h"

enum D_States {D_Init, D_Here, D_Away, D_Leave, D_Arrive} D_State;

int frontCounter = 0;
int awayCounter = 0;
volatile char away = 2;

#define zero 0xE357
#define max 0xD6D7
#define trigDelay 120
#define awayThreshold 20
#define hereThreshold 5
#define distThreshold 95

int main()
{
  //Set system clock to 12MHz
  PLL_Init(PRESET3);
  D_State = D_Init;
  switchInit();
  pwmInit();
  UART0Init();
  timer0Init();
  hcsr04Init();
  portN3IntrpInit();
  timer1Init();
  while(1) {}
  return 0;
}

void pwmInit() {
  unsigned long volatile delay;
  rcgcPwm0 |=0x01;     // activate clock for PWM
  systemGPIO  |= 0x20; // Enable GPIO port F
  delay = systemGPIO;
  portFAfsel |= 0x02;  // enable alt funct on PF1
  portFDig |= 0x02;    // enable digital I/O on PF1                                     
  portFPctl = (portFPctl & 0xFFFFFF0F) + 0x00000060;  // configure PF1 as PWM1 
  portFAnalog &= ~0x02;          // disable analog functionality on PF1
  while((pwm0Ready & 0x00000001) == 0){};  // allow time for clock to stabilize

  pwmClock |= 0x100;       // use PWM divider
  pwmClock &= ~0x07;       // clear PWM divider field
  pwmClock += 0x1;         // divide by 4(12MHz / 4 = 3MHz)
  pwm0Ctrl = 0;            // re-loading down-counting mode
  //pwm0GenA = 0x0000008C; // low when match cmpA, high when match load
  pwm0GenB = 0x80C;        // low when match cmpB, high when match load

  pwm0Load = 0xEA5F;       // load period of 20ms(60000 ticks/s - 1 = 59999)
  // servo pulse width must be 1ms - 2ms(1.5ms being 90º)
  //pwm0CmpA = zero;             
  pwm0CmpB = zero;       
  pwm0Ctrl |= 0x01;      // start PWM0
  pwm0Enable |= 0x02;    // enable PWM0A/PF1 outputs

}

void pwmUpdate(int pwm) {
  pwm0Ctrl = 0;    // Disable pwm
  pwm0CmpA = pwm;  // Update Cmp values          
  pwm0CmpB = pwm;
  pwm0Ctrl |= 0x01;  // Enable pwm
}

void servoPulse() {
  pwmUpdate(max);
  delay(1500000);
  pwmUpdate(zero);
}

void switchInit() {
  volatile unsigned short delay = 0;
  systemGPIO  |= enableGpioJ ; // Enable  PortJ GPIO
  delay++; 
  delay++; 
  portJDir = 0x00;       // Set PJ0 to input
  portJDig = 0x03;       // Set PJ0 to digital port
  portJCommit = 0x03;    //writable
  portJPullUp = 0x03;    // Attach pull up resistor to PJ0
  portJIntrpSense &= ~0x3; //Intrp on edges
  portJIntrpCtrl &= ~0x3; //Intrp controlled by iev
  portJIntrpEdge = ~0x3;  //Interrupt on falling edge
  portJIntrpMask = 0x3;   //Unmask interrupt
  intrpEnable1 = 0x80000; //Enable interrupt 19 for portJ
  clearJInterrupt();
}

void clearJInterrupt() {
  portJIntrpClr = 0x3;
}

void GPIOPJ_Handler ( void ) {
  if((portJData & 0x01) != 0x01) { // if sw1 is pressed
    pwmUpdate(max);
  }
  else if((portJData & 0x02) != 0x02) { //sw2 pressed
    pwmUpdate(zero);
  }
  clearJInterrupt();
}

void hcsr04Init() {
  unsigned long volatile delay;
  systemGPIO |= enableGpioN;
  delay = systemGPIO;
  portNDir |= 0x04;  // PN2 output to trig pin
  portNDir &= ~0x08; // PN3 input for echo pin
  portNDig |= 0x0C;  // enable digital function for pin2,3
  portNAnalog &= ~0x0C; // disable analog on pn2, pn3
}

void portN3IntrpInit() {
  portNIntrpMask &= ~0x8;  //mask interrupt
  portNIntrpSense &= ~0x8; //Intrp on edges
  portNIntrpCtrl |= 0x8;   //Intrp on both edges
  intrpEnable2 = 0x200;    //Enable interrupt 73 for portN
  clearNInterrupt();
  portNIntrpMask |= 0x8;   //Unmask interrupt
}

void GPIOPN_Handler ( void ) {
  if((portNData & 0x08) == 0x08) { // if PN3 set high
    //start timer
    timer0Control = 0x1;
  }
  else { // pn3 low
    // stop timer
    // find time between edges
    // calculate distance
    timer0Control = 0x0;
    int time = timer0Value;
    char timeArray[12] = {0};
    snprintf(timeArray, sizeof timeArray, "%d", time);
    timeArray[10] = 10;
    timeArray[11] = 13;
    //printToPutty(timeArray);
    volatile double distance = ((double)time / 12.0) / 58.824;
    stateCheck(distance);
    char distArray[9] = {0};
    snprintf(distArray, sizeof distArray, "%f", distance);
    distArray[7] = 10;
    distArray[8] = 13;
    timer0Value = 0x00000000;
    printToPutty(distArray);
  }
  clearNInterrupt();
}

void clearNInterrupt() {
  portNIntrpClr = 0x8;
}

void triggerDistance() {
  portNData &= ~0x4; // make sure trig starts low
  delay(trigDelay);
  portNData |= 0x04; // set pn2 to high
  delay(trigDelay);  //delay for 10 microseconds
  portNData &= ~0x4; //set trig back to low
}

void stateCheck(double distance) {
  if(distance < distThreshold) {
    away = 0;
  }
  else {
    away = 1;
  }
  switch(D_State) {
    case D_Init:  // Initialize state to be here
      D_State = D_Here;
      break;
    case D_Here:
      if(away) {
        D_State = D_Leave;
      }
      break;
    case D_Away:
      if(!away) {
        D_State = D_Arrive;
      }
      break;
    case D_Leave:  // Transition state from here to away
      if(away) {  // if away, keep count of how long been away         
        awayCounter++;
        if(awayCounter == awayThreshold) {  // if been away for long enough, move to away state turn off monitors
          D_State = D_Away;
          servoPulse();
          awayCounter = 0;
        }
      }
      else {  // else move back to here state
        D_State = D_Here;
        awayCounter = 0;
      }
      break;
    case D_Arrive:  // Transition State from away to here
      if(away) {  // if away, then return back to away state
        D_State = D_Away;
        frontCounter = 0;
      }
      else {  // else keep count of how long we're not away
        frontCounter++;
        if(frontCounter == hereThreshold) {  // if been here for long enough, move to here state turn on monitors
          D_State = D_Here;
          servoPulse();
          frontCounter = 0;
        }
      }
      break;
    default:
      break;
  }
}

void delay(int d) {
  for(int i = 0; i < d; i++) {};
}

void timer0Init() {
  volatile unsigned char delay = 0;
  timerRunMode |= 0x1;    //Enable Timer 0
  delay++;
  delay++;
  timer0Control = 0x0;   //Disable timer 0
  timer0BitConfig = 0x00000000; //Reset bit config
  timer0BitConfig = 0x0; //set 32-bit mode
  timer0Mode = 0x12;     //Set to periodic timer mode, count up
  timer0Interval = 0x16E3600;   //Set interval to 24000000
}

void timer1Init() {
  volatile unsigned char delay = 0;
  timerRunMode |= 0x2;       // Enable Timer 1
  delay++;
  delay++;
  timer1Control = 0x0;       // Disable timer 1
  timer1BitConfig = 0x00000000; // Reset bit config
  timer1BitConfig = 0x0;     // Set 32-bit mode
  timer1Mode = 0x2;          // Set to periodic timer mode
  timer1Interval = 0x6ACFC0; // Set 1s interval 
  timer1IntrpMask = 0x1;     // Enable timer interrupts
  intrpEnable0 |= 0x200000;  // Enable interrupt 21 for timer1A
  timer1Control = 0x1;       // Start the timer
  clearTimer1Status();
}

void Timer1A_Handler( void ) {
  triggerDistance();
  clearTimer1Status();
}

void clearTimer1Status() {
    timer1IntrpClr = 0x1;
}

void UART0Init() {
  volatile unsigned char delay = 0;
  UARTEnable = 0x1;           // Enable UArt module
  delay++;
  delay++;
  systemGPIO |= enableGpioA;  // Enable gpio port for uart 
  delay++;
  delay++;
  portAAltSel = 0x3;          // let Uart use gpio port 
  portADigital = 0x3;         // Digitally enable ports
  portAPortControl = 0x11;    // set PA0 to transmit and PA1 to recieve 
  portADir = 0x00;            // reset dir register
  UART0Control = 0x0;         // disable UART0

  float brdi = (16000000.0/(16.0 * 115200.0));
  int brdiTrunc = (int)brdi;
  UART0BRIntDiv = brdiTrunc;  
  UART0BRFracDiv =(int) ((brdi - brdiTrunc) * 64 + 0.5);

  UART0LineControl = 0x60;    // 8 bit transmitted (UARTLCRH)
  UART0ClkSource   = 0x5;     // use default clock (UARTCC)
  UART0Control = 0x301;       // enable and allow transmit 
}

void printToPutty(char* message) {
  int i = 0;
  char fullMsgSent = 0;
  while(!fullMsgSent) {
    if((UART0Status & 0x8) == 0x0) { // printing start message on putty
      // printing temp
      UART0Data  = message[i];
      i++;
      if(i == strlen(message)) { // size of temp array
        i = 0;
        fullMsgSent = 1;
      }
    }
  }
}


