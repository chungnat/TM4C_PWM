/*
 * Nathan Chung
*/

#ifndef PORTS_H
#define PORTS_H
                                          
  #define enableGpioF     0x00000020 // GPIO Port F Run Mode Clock
  #define enableGpioN     0x00001000  // GPIO Port N Run Mode Clock
  #define enableGpioJ     0x00000100  // GPIO Port J Run Mode Clock
  #define enableGpioA     0x00000001  // GPIO Port A Run Mode Clock                                             

  #define systemGPIO      (*((volatile uint32_t *)0x400FE608)) //382

 // PortN GPIO: controls led 1, 2
  #define portNData       (*((volatile uint32_t *)0x400643FC))
  #define portNDir        (*((volatile uint32_t *)0x40064400))
  #define portNDig        (*((volatile uint32_t *)0x4006451C))
  #define portNAnalog     (*((volatile uint32_t *)0x40064528)) //786
  #define portNIntrpSense (*((volatile uint32_t *)0x40064404)) //761
  #define portNIntrpCtrl  (*((volatile uint32_t *)0x40064408))
  #define portNIntrpEdge  (*((volatile uint32_t *)0x4006440C))
  #define portNIntrpMask  (*((volatile uint32_t *)0x40064410))
  #define portNIntrpClr   (*((volatile uint32_t *)0x4006441C))
  #define portNIntrpStatus (*((volatile uint32_t *)0x40064418)) //768
  #define intrpEnable2    (*((volatile uint32_t *)0xE000E108)) // 154 EN2
  

  // PortJ GPIO: controls sw1,2
  #define portJData       (*((volatile uint32_t *)0x400603FC))
  #define portJDir        (*((volatile uint32_t *)0x40060400))
  #define portJDig        (*((volatile uint32_t *)0x4006051C))
  #define portJCommit     (*((volatile uint32_t *)0x40060524))
  #define portJPullUp     (*((volatile uint32_t *)0x40060510))
  #define portJIntrpSense (*((volatile uint32_t *)0x40060404)) //set respective bit to 0 to sense edge, 1 to sense level GPIOIS pg761
  #define portJIntrpCtrl  (*((volatile uint32_t *)0x40060408)) //set respective bit to 1 to intrp at both edges, 0 to let GPIOIEV control, GPIOIBE 762
  #define portJIntrpEdge  (*((volatile uint32_t *)0x4006040C)) //set respective bit to 1 to intrp at rising edge, 0 for falling edge, GPIOIEV 763
  #define portJIntrpMask  (*((volatile uint32_t *)0x40060410)) //set respective bit to 1 to unmask interrupts  GPIOIM 764
  #define portJIntrpClr   (*((volatile uint32_t *)0x4006041C)) //set respective bit to 1 to clear bit GPIOICR 769
  #define intrpEnable1    (*((volatile uint32_t *)0xE000E104)) // 154 EN1

  //Port F
  #define portFAfsel   (*((volatile uint32_t *)0x4005D420)) //770 write 1 to port# to enable alt function
  #define portFPctl    (*((volatile uint32_t *)0x4005D52C)) //788
  #define portFDig     (*((volatile uint32_t *)0x4005D51C)) //782 
  #define portFDir     (*((volatile uint32_t *)0x4005D400)) //760
  #define portFAnalog  (*((volatile uint32_t *)0x4005D528)) //786

  //PWM
  #define rcgcPwm0   (*((volatile uint32_t *)0x400FE640)) //398 write 1 to 1st bit to enable PWM module 0
  #define pwmClock   (*((volatile uint32_t *)0x40028FC8)) //1747 set 9th bit to 1 for pwm clock divder, first 3 bits control clock divider
  #define pwm0Ctrl   (*((volatile uint32_t *)0x40028040)) //1712
  #define pwm0GenA   (*((volatile uint32_t *)0x40028060)) //1724
  #define pwm0GenB   (*((volatile uint32_t *)0x40028064)) //1727
  #define pwm0Load   (*((volatile uint32_t *)0x40028050)) //1720
  #define pwm0CmpA   (*((volatile uint32_t *)0x40028058)) //1722
  #define pwm0CmpB   (*((volatile uint32_t *)0x4002805C)) //1723
  #define pwm0Enable (*((volatile uint32_t *)0x40028008)) //1686
  #define pwm0Fault  (*((volatile uint32_t *)0x40028010)) //1690
  #define pwm0Counter (*((volatile uint32_t *)0x40028054)) //1721
  #define pwm0Reset  (*((volatile uint32_t *)0x400FE540))
  #define pwm0Ready  (*((volatile uint32_t *)0x400FEA40)) //517
  #define pwm0Sync   (*((volatile uint32_t *)0x40028004)) //1685

 //Enable Timers
  #define timerRunMode   (*((volatile uint32_t *)0x400FE604))  // RCGCTIMER 380

  // Timer0 registers
  #define timer0Control     (*((volatile uint32_t *)0x4003000C))  // 986 set last bit to 0 for off, 1 to turn on
  #define timer0BitConfig   (*((volatile uint32_t *)0x40030000))  // 976 controls 16/32 bit 
  #define timer0Mode        (*((volatile uint32_t *)0x40030004))  // 981 controls one-shot/periodic 
  #define timer0Interval    (*((volatile uint32_t *)0x40030028))  // 1004 Set timer length 
  #define timer0IntrpMask   (*((volatile uint32_t *)0x40030018))  // 993
  #define timer0IntrpStatus (*((volatile uint32_t *)0x4003001C))  // 996 last bit 1 if timer timed out 
  #define timer0IntrpClr    (*((volatile uint32_t *)0x40030024))  // 1002 write 1 to last bit to clear status 
  #define timer0Value       (*((volatile uint32_t *)0x40030050))  // 1014
  #define intrpEnable0 (*((volatile uint32_t *)0xE000E100))       // EN0 154 enables an interrupt

  //Timer1 registers
  #define timer1Control     (*((volatile uint32_t *)0x4003100C))  
  #define timer1BitConfig   (*((volatile uint32_t *)0x40031000))  
  #define timer1Mode        (*((volatile uint32_t *)0x40031004))       
  #define timer1Interval    (*((volatile uint32_t *)0x40031028))   
  #define timer1IntrpMask   (*((volatile uint32_t *)0x40031018)) 
  #define timer1IntrpStatus (*((volatile uint32_t *)0x4003101C)) 
  #define timer1IntrpClr    (*((volatile uint32_t *)0x40031024))   

  // UART Setup 1172
  #define UARTEnable       (*((volatile uint32_t *)0x400FE618)) // 388 RCGCUART set last bit to 1 to enable UART0 Module
  #define portAAltSel      (*((volatile uint32_t *)0x40058420)) // 770 GPIOAFSEL set bit to 1 for alternate function
  #define portADir         (*((volatile uint32_t *)0x40058400)) // 760
  #define portADigital     (*((volatile uint32_t *)0x4005851C)) // 781
  #define portADrive       (*((volatile uint32_t *)0x40058500)) // 772 GPIODR2R
  #define portAPortControl (*((volatile uint32_t *)0x4005852C)) // 787 GPIOPCTL 7:4 control pin 1, 3:0 control pin 0 set both to 1 for  Tc, Rx respectively
  #define UART0BRIntDiv    (*((volatile uint32_t *)0x4000C024)) // 1184 UARTIBRD Integer portion of the clkdiv for baud rate
  #define UART0BRFracDiv   (*((volatile uint32_t *)0x4000C028)) // 1184 UARTFBRD Fraction portion of the clkdiv for baud rate
  #define UART0LineControl (*((volatile uint32_t *)0x4000C02C)) // 1186 UARTLCRH write 0x00 for default and update baud rate
  #define UART0Control     (*((volatile uint32_t *)0x4000C030)) // 1188 UARTCTL set 0 to disable(make sure to do in beginning) set 1 to enable
  #define UART0ClkSource   (*((volatile uint32_t *)0x4000CFC8)) // 1213 UARTCC set 0x0 to use sys clk
  #define UART0Data        (*((volatile uint32_t *)0x4000C000)) // 1175 UARTDR 
  #define UART0Status      (*((volatile uint32_t *)0x4000C018)) // 1180 UARTFR 4th bit is 1 if uart still transmitting
  #define UART0IntrpStatus (*((volatile uint32_t *)0x4000C03C)) // 1198 5th bit is 1 if data is written, 0 if data is read

  // Functions

  // Initializes pwm 
  void pwmInit();
  
  // Updates the CMP values for the pwm generator
  // Takes in an int pwm to update the cmp value
  void pwmUpdate(int pwm);

  // pulses the servo with two predefined pwm signals
  void servoPulse();
  // Initializes interrupt for pin connected to echo on hcsr04
  void portN3IntrpInit();
  // Takes in distance from hcsr04 and updates the state of the system
  // Calls servoPulse depending on the current state
  void stateCheck(double distance);

  // Intrp clear functions
  void clearJInterrupt();
  void clearNInterrupt();

  // Initializes the onboard switches and interrupts
  void switchInit();
  // Initializes the pins connected to hcsr04 periphal
  void hcsr04Init();
  // Activates trigger pin on hcsr04
  void triggerDistance();
  
  // delays for an inputted amount of time
  void delay(int d);
  
  // Initializes timers
  void timer0Init();
  void timer1Init();
  void clearTimer1Status();

  // Initializes Uart0
  void UART0Init();
  // Given a char*, sends it through the Uart char by char
  void printToPutty(char* message);

#endif