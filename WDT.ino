/*
Watch Dog Timer - reboots Arduino of it locks up

Include library: 
#include <avr/wdt.h>     // Watchdog timer, Example code: http://code.google.com/p/arduwind/source/browse/trunk/ArduWind.ino

Using WDT Functions
We always need to make sure the WDT is disabled immediately after a 
reset, otherwise it will continue to operate with default values.
So put the wdt_disable() function at the start of setup()

Later in setup() enable WDT:
WatchdogSetup(); // setup Watch Dog Timer to 8 sec
wdt_reset();

If you have some code that takes a long time to run or is using delay(), put a wdt_reset() call so the WDT doesn't time out
wdt_reset(); 

To force a restart call
WDT_ForceTimeout()
This goes into a loop and will force WDT to reboot

*/

// Watchdog timer variables
uint32_t previousWdtMillis = 0;
uint32_t wdtInterval =       0;


// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
// Initialize watchdog
// Timeout is 8 seconds
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
void WatchdogSetup(void)
{
//#define WDPS_16MS   (0<<WDP3)|(0<<WDP2)|(0<<WDP1)|(0<<WDP0) 
//#define WDPS_32MS   (0<<WDP3)|(0<<WDP2)|(0<<WDP1)|(1<<WDP0) 
//#define WDPS_64MS   (0<<WDP3)|(0<<WDP2)|(1<<WDP1)|(0<<WDP0) 
//#define WDPS_125MS  (0<<WDP3)|(0<<WDP2)|(1<<WDP1)|(1<<WDP0) 
//#define WDPS_250MS  (0<<WDP3)|(1<<WDP2)|(0<<WDP1)|(0<<WDP0) 
//#define WDPS_500MS  (0<<WDP3)|(1<<WDP2)|(0<<WDP1)|(1<<WDP0) 
//#define WDPS_1S     (0<<WDP3)|(1<<WDP2)|(1<<WDP1)|(0<<WDP0) 
//#define WDPS_2S     (0<<WDP3)|(1<<WDP2)|(1<<WDP1)|(1<<WDP0) 
//#define WDPS_4S     (1<<WDP3)|(0<<WDP2)|(0<<WDP1)|(0<<WDP0) 
  #define WDPS_8S     (1<<WDP3)|(0<<WDP2)|(0<<WDP1)|(1<<WDP0) 

  //disable interrupts
  cli();
  //reset watchdog
  wdt_reset();
	
  /* Setup Watchdog by writing to WDTCSR = WatchDog Timer Control Register */ 
  WDTCSR = (1<<WDCE)|(1<<WDE); // Set Change Enable bit and Enable Watchdog System Reset Mode. 

  // Set Watchdog prescalar as desired - 8 seconds
  WDTCSR = (1<<WDIE)|(1<<WDIF)|(1<<WDE )| WDPS_8S; 
	
  //Enable global interrupts
  sei();
	
  printf_P(PSTR("\nInitialize WDT\n\r"));
  Serial.println();
  
} // End WatchdogSetup()


// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
void WatchdogClear(void)
{
  //disable interrupts
  cli();
  //reset watchdog
  wdt_reset();
	
  // Setup Watchdog by writing to WDTCSR = WatchDog Timer Control Register 
  WDTCSR = (1<<WDCE)|(1<<WDE); // Set Change Enable bit and Enable Watchdog System Reset Mode. 	
} // End WatchdogClear()


// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
// Watchdog timeout ISR
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
ISR(WDT_vect)
{
  WatchdogSetup(); // If not there, cannot print the message before rebooting
  printf_P(PSTR("\n\nWDT REBOOTING\n\n"));
	
  delay(1000); // leave some time for printing on serial port to complete
  
  // Soft reset
  asm volatile ("  jmp 0");
  
} // End ISR()

// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
// Force the WDT to trigger by going into an infinite loop
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * 
void WDT_ForceTimeout() 
{
  printf_P(PSTR("\nWDT Force Timeout\n\r"));
  Serial.println();
  while (true){}
} // WDT_ForceTimeout()

