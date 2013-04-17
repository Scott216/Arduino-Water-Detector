/*

To Do:
Take out Tweet counter if no problems after a few months
Send a tweet if remote sensors go offline for more then a few minutes
Use I2C to send info to PanStamp weather or not any sponges are wet, if so, flash RED LED
See if you can request specific panStamp Tx data via I2C instead of sending both panStamps together in one packet
Take a look at how your handle a panStamp being offline and see if you're doing it the best way
Consider replacing Twitter with Tropo.com SMS service


====================
Water Leak Detector.  Detects water leaks using sponge and op-amp circuit. 
The resistance across a dry sponge is eseentially infinity, but when wet it varies from 30k - 150k. 
Arduino needs impedance of inputs to be 10k or less, so an op-amp is used to send a signal to the arduino when a sponge 
is wet.  A high input means it's dry, low is wet.  

For sponges that are too far to be hard wired to the op-amps, pansStamps RF devices are used.  
A panStamp should be able to runfor a couple months on one CR123 battery. 

I have two panStamps transmitting wet/dry status.  They also tranmit their ID, battery voltage and temperature.
They sleep for 8 seconds, wake up for 1/2 second to tranmit, then repeat.

The receiving panStamp is communicating with the UNO via I2C.  The UNO is the master and will make
a request for the data.  The panStamp will then transmit this as one long array of bytes.

The sketch gets time from an NPT server.  Water detect notifications and low battery are sent out via Twitter.

Sketch is low on RAM

Hardware:
* Leak detector PCB 
* Loenardo
* panStamp
* Ethernet shield R3
* 3 op-amps: MCP6004 http://search.digikey.com/us/en/products/MCP6004-I%2FP/MCP6004-I%2FP-ND/523060
* 12 470k Ohm 1/4 watt resistors for Op-amp circuit
* 2 4.7k ohm resistors for I2C pull-up
* Screw Terminal for Screw shield to add two more ground terminals
* Sponges for water detector
* 1 set arduino headers
* 3 8-position screw terminals, 0.1" pitch
* 1 2-position screw terminal for power
* Red LED
* Amper LED
* Green LED
* 2 headers for panstamp
* 2 DIN Rail clips
* Mounting hardware

V2 PCB Changes
Connect I2C to dedicated pins, not A4/A5 - done
Label LEDs - done 
Us 0.1" pitch for power screw terminal and put it on same side as other screw terminals - done
Add jumper so you can use either inputs A4/A5 or D2/D3 so you can use either an UNO or Leonardo
Don't use D4 for sensor input and leave it free to use with SD card.  Move the sensor input to a spare one on the panStamp
Use uno/leonardo to control red LED and move a 2nd input to panstamp so you have a free I/O for the LED


The Arduino is stacked on top of the Leak Detector PCB, so it must have headers like the shields have.

See this Arduino Thread forum on a discussion on reading inputs and using an OpAmp 
  http://arduino.cc/forum/index.php/topic,90671.msg682130.html#msg682130


I2C wiring
panStamp: A4 & A5
UNO: A4 (SDA) & A5 (SCL), Leonardo D2 (SDA), D3 (SCL)
PCB has 4.7k resistors from I2C lines to 3v


I2C Data packet 
== Master Bath ==
byte 0: panStamp Rx ID
byte 1: panStamp Tx ID
byte 2: Wet/Dry Status. Wet = true, Dry = false
byte 3-4: ADC Temperature from TMP36
Byte 5-6: ADC value for battery voltage

== Guest Bath ==
byte 7: panStamp Rx ID
byte 8: panStamp Tx ID
byte 9: Wet/Dry Status
byte 10-11: ADC Temperature from TMP36
Byte 12-13: ADC value for battery voltage


*/

// #define CRESTVIEW     // Comment this out when in Vermont
// #define WDT           // Comment this out to turn WDT off
#define PRINT_DEBUG   // Comment this out to turn off verbose printing 

#include <Ethernet.h>      // http://arduino.cc/en/Reference/Ethernet
#include <SPI.h>           // Allows you to communicate with SPI devices. See: http://arduino.cc/en/Reference/SPI
#include <Twitter.h>       // http://arduino.cc/playground/Code/TwitterLibrary
#include <I2C.h>           // https://github.com/rambo/I2C
#include <avr/wdt.h>       // Watchdog timer, Example code: http://code.google.com/p/arduwind/source/browse/trunk/ArduWind.ino
#include <tokens.h>        // Contains twitter token
#include "Water_Detect.h"  // Contains typedef for RemoteSensorData_t

byte mac[] = { 0x90, 0xA2, 0xDA, 0xEF, 0x46, 0x81 };  

#ifdef CRESTVIEW
  byte ip[] = { 192, 168, 216, 50 };  // Crestview
#else
  byte ip[] = { 192, 168, 46, 81 };  // Vermont
#endif

// Analog inputs 0 & 1 are configured as Digitol I/O.  
//      Name              I/O    Description
#define FirstFlBRSink     A0   // Sponge in first floor bathroom sink
#define WashingMach       A1   // Sponge next to washing machine
#define TBD1              A2   // Future use
#define TBD2              A3   // Future use
#define WaterHeater        2   // Sponge under hot water heater
#define Boiler             3   // Sponge next to boiler
#define Frig               4   // Sponge behind refrigerator
#define Dishwasher         5   // Sponge behind dishwasher
#define KitchenSink        6   // Sponge under kitchen sink
#define HotTubFilter       7   // Sponge in crawlspace by hot tub filter and pump
#define HotTubBack         8   // Sponge in crawlspace behind hot tub 
#define WaterTank          9   // Sponge in crawlspace corner by water tank
//Digital pins 10-13 are used for Ethernet

#define WET HIGH       // When a sponge is wet, the digital input is HIGH
#define DRY LOW        // When a sponge is dry, the digital input is LOW

const uint32_t MINUTE  =                   60000;  // milliseconds in a minute
const uint32_t DOUBLE_CHECK_DELAY =   2 * MINUTE;  // Delay after sensor first detects water to check again after a few minutes
const uint32_t DRYING_DELAY =       180 * MINUTE;  // 3 hour delay when sponges are drying out before twitter it is updated. Prevents extra alerts from going out

const byte packetsPerPanStamp = 7;  // Packets sent by each panStamp
const byte addrMasterBath =     1;  // panStamp device address for 2nd floor master bath
const byte addrGuestBath =      2;  // panStamp device address for 2nd floor guest bath
const byte numTransmitters =    2;  // number of panStamp transmitters on this network
const byte numWiredInputs =    12;  // Number of wired water detector inputs
bool gotI2CPacket = false;          // Flag to indicate I2C packet came in.  Sketch needs to know when I2C is working so it doesn't process bad data
uint32_t checkNTPtimer =        0;  // Countdown timer to check NTP time

#define addrSlaveI2C  21 // I2C Slave address of panStamp Rx

// Use arrays to hold input statuses.  Total inputs are wired plus wireless
byte     InputState[      numWiredInputs + numTransmitters];     // High when water is present low when it's not
byte     WetFlag[         numWiredInputs + numTransmitters];     // Used to trigger if an input goes from Dry to Wet.  Trigger will turn on WaterDetectOutput for 10 seconds.
byte     WaterDetect[     numWiredInputs + numTransmitters];     // Sensor state after delay to make sure it's really wet or dry
uint32_t DoubleCheckTime[ numWiredInputs + numTransmitters];     // Used to wait a few minutes after a sensor is triggered to check the sensor a 2nd time to see if there is still water. Like button debouncing
uint32_t WetToDryDelay[   numWiredInputs + numTransmitters];     // Delay used to let sponge dry out before indicating it's dry
//srg debug removed DoubleCheckFlag bool     DoubleCheckFlag[ numWiredInputs + numTransmitters];     // Flag prevents double check code from executing more then once.  If true, double check code won't run
uint32_t weeklyHeartbeatTimer = 0;                               // mS until Sunday at noon, at which time a tweet will go out to verify sketch is still running  static uint8_t TweetCounter;  // prevent tweets from getting to high
uint8_t TweetCounter = 0;                                        // prevent tweets from getting to high, stop tweeting after 50 tweets.  Reset every Sunday, same time as heartbeat tweet
uint32_t CheckSensorsTimer = 0;                                  // Timer to check the sensors every second
bool printStatusFlag = false;                                    // Set true if something has changed so status of all inputs can be printed to serial monitor  

// Define typedef structurs for panStamp data from remote sensors.  Typedef definition is in Water_Detect.h
RemoteSensorData_t masterBath;
RemoteSensorData_t guestBath; 

// Put input pin numbers in array so you can use loops to read wired inputs
int InputPinNum[] = {FirstFlBRSink, WashingMach, TBD1, TBD2, WaterHeater, Boiler, Frig, Dishwasher, KitchenSink, HotTubFilter, HotTubBack, WaterTank };

// Token for Twitter Account
Twitter twitter(TWITTER_TOKEN);

// Function Prototypes
void ConfigureIO(void);
void ProcessSensors(void);
bool ReadRFSensors(RemoteSensorData_t* rfsensor, byte panStampID);
void CreateTweet(byte SensorArrayPosition, bool IsWet);
int SendTweet(char msgTweet[]);
void PrintStates(); 
uint32_t WeeklyCountdownTime(uint8_t *ntpTime);
void printf_begin(void);
int serial_putc(char c, FILE *);
void software_Reset();
bool setupNTPTime();
bool getTime(uint8_t *ntpTime);

//============================================================================
/*                888                      
                  888                      
                  888                      
.d8888b   .d88b.  888888 888  888 88888b.  
88K      d8P  Y8b 888    888  888 888 "88b 
"Y8888b. 88888888 888    888  888 888  888 
     X88 Y8b.     Y88b.  Y88b 888 888 d88P 
 88888P'  "Y8888   "Y888  "Y88888 88888P"  
                                  888      
                                  888      
                                  888      
 http://patorjk.com/software/taag/#p=display&f=Colossal&t=setup  */
//============================================================================
void setup () 
{
  #ifdef WDT
    // Make sure the WDT is disabled immediately after a 
    //reset, otherwise it will continue to operate with default values.
    wdt_disable();
  #endif

  uint8_t ntpTime[6];
 
  Serial.begin(9600);
  while (!Serial && millis() < 6000) {}  // for Leonardo only, gives user 6 sec to open up serial montior
  Serial.println(F("Leak Detector"));

  // Initialize Ethernet connection and UDP
  bool EthernetOK = setupNTPTime();
  if(EthernetOK)
  {
    // Get the NTP Time
    if(getTime(ntpTime))
    {
      // Set weekly countdown timer - Sunday noon
      weeklyHeartbeatTimer = WeeklyCountdownTime(ntpTime);
      #ifdef PRINT_DEBUG
        Serial.print(F("mS until Sunday noon = "));
        Serial.println(weeklyHeartbeatTimer);
      #endif
    }
  }

  printf_begin();  // Need this so the printf_P statements work
  
  // Initialiae wire library for I2C communication
  I2c.begin();
  I2c.timeOut(30000);
  
  #ifdef PRINT_DEBUG
    Serial.println(F("Wire library initialized"));
  #endif

  // Configure inputs and ouputs
  ConfigureIO();

  // Enable WDT
  #ifdef WDT
    WatchdogSetup(); // setup Watch Dog Timer to 8 sec
    wdt_reset();
  #endif
 
  SendTweet("VT Water Leak Detector Restarted. ");  // Send startup tweet

  ProcessSensors();  // Check sensors to see if anything is wet
 
  #ifdef PRINT_DEBUG
    PrintStates();
  #endif

} // end setup()


//=========================================================================================
/*
 .d8888b.                     .d888 d8b                                  8888888 .d88888b.  
d88P  Y88b                   d88P"  Y8P                                    888  d88P" "Y88b 
888    888                   888                                           888  888     888 
888         .d88b.  88888b.  888888 888  .d88b.  888  888 888d888 .d88b.   888  888     888 
888        d88""88b 888 "88b 888    888 d88P"88b 888  888 888P"  d8P  Y8b  888  888     888 
888    888 888  888 888  888 888    888 888  888 888  888 888    88888888  888  888     888 
Y88b  d88P Y88..88P 888  888 888    888 Y88b 888 Y88b 888 888    Y8b.      888  Y88b. .d88P 
 "Y8888P"   "Y88P"  888  888 888    888  "Y88888  "Y88888 888     "Y8888 8888888 "Y88888P"  
                                             888                                            
                                        Y8b d88P                                            
                                         "Y88P"                                             
*/
// Configure Input Pins
//=========================================================================================
void ConfigureIO()
{
  // Configue input pins
  for(int i = 0; i < numWiredInputs; i++)
  {
    pinMode(InputPinNum[i], INPUT);   // Initialize digital and analog pins as digital inputs
  }

/* srg debug  
  // Read wired sensors and set input variables to the same state as sensor input
  // The reason for this is if the Arduino reboots, it will start off with the sensors
  // as dry, and if one is wet, it will send out another alert
  for(int i = 0; i < numWiredInputs; i++)
  {
    // Loop until two consecutive reading are the same
    bool firstreading = LOW;
    bool secondreading = LOW;
    
    #ifdef CRESTVIEW
      firstreading = DRY;  // If testing, force input to be dry
    #else
      do 
      {
        firstreading = !digitalRead(InputPinNum[i]);
        delay(10);
        secondreading = !digitalRead(InputPinNum[i]);
      } while (firstreading != secondreading);
    #endif

    InputState[i] = firstreading;
    if(InputState[i] == DRY)
    { // Input is dry
      WetFlag[i] = false;
      WaterDetect[i] = DRY;
      DoubleCheckFlag[i] = false;
    }
    else
    { // Input is Wet
      WetFlag[i] = true;
      WaterDetect[i] = WET;
      DoubleCheckFlag[i] = true;
    }
 
    WetToDryDelay[i] = millis() + DRYING_DELAY;
  }
  
  // Read input state from remote sensors, put values in array after wired inputs
  // Since panStamp is only going to update every 8 seconds, don't look for consecutive readings, that's done locally on the panStamp
  byte arryPositMasterBath = numWiredInputs;
  byte arryPositGuestBath = numWiredInputs + 1;
  
  ReadRFSensors( &masterBath, addrMasterBath );
  if(masterBath.IsWet == false)
  { // Input is dry
    InputState[arryPositMasterBath] = DRY;
    WetFlag[arryPositMasterBath] = false;
    WaterDetect[arryPositMasterBath] = DRY;
    DoubleCheckFlag[arryPositMasterBath] = false;
  }
  else
  { // Input is Wet
    InputState[arryPositMasterBath] = WET;
    WetFlag[arryPositMasterBath] = true;
    WaterDetect[arryPositMasterBath] = WET;
    DoubleCheckFlag[arryPositMasterBath] = true;
  }
  WetToDryDelay[arryPositMasterBath] = millis() + DRYING_DELAY;

  ReadRFSensors( &guestBath, addrGuestBath );
  if(guestBath.IsWet == false)
  { // Input is dry
    InputState[arryPositGuestBath] = DRY;
    WetFlag[arryPositGuestBath] = false;
    WaterDetect[arryPositGuestBath] = DRY;
    DoubleCheckFlag[arryPositGuestBath] = false;
  }
  else
  { // Input is Wet
    InputState[arryPositGuestBath] = WET;
    WetFlag[arryPositGuestBath] = true;
    WaterDetect[arryPositGuestBath] = WET;
    DoubleCheckFlag[arryPositGuestBath] = true;
  }
  WetToDryDelay[arryPositGuestBath] = millis() + DRYING_DELAY;
srg debug */

  // Initialiaze one shot triggers for messages
  masterBath.lowVoltMsgFlag = false; 
  guestBath.lowVoltMsgFlag =  false; 
  masterBath.lowTempMsgFlag = false;
  guestBath.lowTempMsgFlag =  false;

} // End ConfigureIO()


//============================================================================ 
/*
888
888
888
888  .d88b.   .d88b.  88888b.
888 d88""88b d88""88b 888 "88b
888 888  888 888  888 888  888
888 Y88..88P Y88..88P 888 d88P
888  "Y88P"   "Y88P"  88888P"
                      888
                      888
                      888                                                      */
// 
//============================================================================
void loop () 
{
  
  #ifdef WDT
    wdt_reset();  // Reset watchdog time
  #endif
  
  if ( (long)(millis() - CheckSensorsTimer) >= 0 )
  {
    ProcessSensors();  // Check sensors to see if anything is wet
    CheckSensorsTimer = millis() + 1000;
  }
  
  
  // Every Sunday at noon send a Tweet to indicate sketch is still running (heartbeat). Send battery voltages
  if( (long)(millis() - weeklyHeartbeatTimer) >= 0 )
  {
    TweetCounter = 0; // Reset Tweet Counter
    char tweetMsg[100]; 
    sprintf(tweetMsg, "Leak Detector: Master Bath = %d mV, Guest Bath = %d mV. ", masterBath.volts, guestBath.volts);
    SendTweet(tweetMsg);
    
    #ifdef PRINT_DEBUG
      PrintStates();
    #endif
  }  // end weekly heartbeat

  // Check time once every 2 days and reset countdown to Sunday noon
  if( (long)( millis() -  checkNTPtimer) > 0 )
  {
    checkNTPtimer = millis() + MINUTE * 60 * 48;
    uint8_t ntpTime[6];
    if( getTime(ntpTime) )
    { weeklyHeartbeatTimer = WeeklyCountdownTime(ntpTime); } // Set weekly countdown timer - Sunday noon
    else
    { 
      weeklyHeartbeatTimer = millis() + 604800000UL; 
      SendTweet("Failed to update NTP time. ");  // Send startup tweet
    } // Couldn't get time from NTP server, just add 1 week in milliseconds
  }
  
}  // end loop()


//===========================================================================================================================
/*
8888888b.                                                      .d8888b.                                                       
888   Y88b                                                    d88P  Y88b                                                      
888    888                                                    Y88b.                                                           
888   d88P 888d888 .d88b.   .d8888b .d88b.  .d8888b  .d8888b   "Y888b.    .d88b.  88888b.  .d8888b   .d88b.  888d888 .d8888b  
8888888P"  888P"  d88""88b d88P"   d8P  Y8b 88K      88K          "Y88b. d8P  Y8b 888 "88b 88K      d88""88b 888P"   88K      
888        888    888  888 888     88888888 "Y8888b. "Y8888b.       "888 88888888 888  888 "Y8888b. 888  888 888     "Y8888b. 
888        888    Y88..88P Y88b.   Y8b.          X88      X88 Y88b  d88P Y8b.     888  888      X88 Y88..88P 888          X88 
888        888     "Y88P"   "Y8888P "Y8888   88888P'  88888P'  "Y8888P"   "Y8888  888  888  88888P'  "Y88P"  888      88888P' 
*/                                                                                                                              
// Read the water sensors
//===========================================================================================================================
void ProcessSensors()
{
  char tweetMsg[100];
  #ifdef WDT
    wdt_reset();  // Reset watchdog time
  #endif

  // When sensor is dry, input is HIGH. When reading the input, reverse it so a InputState[] is LOW for a dry sensor
  for(byte i = 0; i < numWiredInputs; i++)
  {
    #ifdef CRESTVIEW 
      // If at Crestview, there are no sensors hooked up, so make them all DRY
      InputState[i] = DRY;
    #else
      // Read hard wired sensors
      // Loop until two consecutive reading are the same
      bool firstreading = DRY;
      bool secondreading = DRY;
      do 
      {
        firstreading = !digitalRead(InputPinNum[i]);
        delay(10);
        secondreading = !digitalRead(InputPinNum[i]);
      } while (firstreading != secondreading);
      InputState[i] = firstreading;
    #endif
  }

  // Read Remote Master Bath sensor
  if( ReadRFSensors( &masterBath, addrMasterBath ) )
  {
    InputState[numWiredInputs] = masterBath.IsWet;
    
    // Check for low volts
    if(masterBath.volts < 2800 && masterBath.lowVoltMsgFlag == false)
    {
      SendTweet("Low battery Master Bath.  ");
      masterBath.lowVoltMsgFlag = true;
    }
    // Reset low volts flag if volts is over 3000 mV
    if(masterBath.volts > 3000)
    {
      masterBath.lowVoltMsgFlag = false;
    }

    // Check for low temperature in Masster Bath
    if(masterBath.temp <= 40 && masterBath.temp >= 20 && masterBath.lowTempMsgFlag == false)
    {
      sprintf(tweetMsg, "Low temp Master Bath (%d F).  ", masterBath.temp);
      SendTweet(tweetMsg);
      masterBath.lowTempMsgFlag = true;
    }
    // Reset low temp flag if temp is 50 F
    if(masterBath.temp >= 50)
    {
      masterBath.lowTempMsgFlag = false;
    }
  }
  else
  {
     // panStamp in Master bath is offline
  }

  // Read Remote Guest Bath sensor
  if( ReadRFSensors( &guestBath, addrGuestBath ) )
  {
    InputState[numWiredInputs + 1] = guestBath.IsWet;

    // Check for low volts
    if(guestBath.volts < 2800 && guestBath.lowVoltMsgFlag == false)
    {
      SendTweet("Low battery in Guest Bath.  ");
      guestBath.lowVoltMsgFlag = true;
    }
    // Reset low volts flag if volts is over 3000 mV
    if(guestBath.volts > 3000)
    {
      guestBath.lowVoltMsgFlag = false;
    }

    // Check for low temperature in Guest Bath
    if(guestBath.temp <= 40 && guestBath.temp >= 20 && guestBath.lowTempMsgFlag == false)
    {
      sprintf(tweetMsg, "Low temp in Guest Bath (%d F).  ", guestBath.temp);
      SendTweet(tweetMsg);
      guestBath.lowTempMsgFlag = true;
    }
    // Reset low temp flag if temp is 50 F
    if(guestBath.temp >= 50)
    {
      guestBath.lowTempMsgFlag = false;
    }
  }
  else
  {
     // panStamp in Guest bath is offline
  }
  
  // See if any inputs have gone from Dry to Wet
  for(int i = 0; i < numWiredInputs + numTransmitters; i++)
  {
    if((InputState[i] == WET) && (WetFlag[i] == DRY ))
    {
      #ifdef PRINT_DEBUG
        PrintStates();
      #endif
      
      WetFlag[i] = WET;
      
      // One of the inputs went from Dry to Wet
      printf_P(PSTR("Water detected ID: %d\n\r"), i);

      #ifdef PRINT_DEBUG
        PrintStates();
      #endif
      
      // Set time to check again to make sure it's really wet
      DoubleCheckTime[i] = millis() + DOUBLE_CHECK_DELAY;
    }
  } // End check for Wet to Dry


  // Double check input after a delay to see it it's still wet
  for(int i = 0; i < numWiredInputs + numTransmitters; i++)
  {
//srg debug removed DoubleCheckFlag    if( (WetFlag[i] == WET) && ((long)(millis() - DoubleCheckTime[i]) >= 0) && (DoubleCheckFlag[i] == false) )
    if( (WetFlag[i] == WET) && (WaterDetect[i] == DRY) && ((long)(millis() - DoubleCheckTime[i]) >= 0) )
    {
      #ifdef PRINT_DEBUG
        PrintStates();
      #endif

      WaterDetect[i] = WET;    // Sponge is still wet after delay
//srg debug removed DoubleCheckFlag      DoubleCheckFlag[i] = true;  // Set flag so this test only happens once

      #ifdef PRINT_DEBUG
        PrintStates();
      #endif

      CreateTweet(i , WET);   // One of the inputs is still wet after DOUBLE_CHECK_DELAY.  Send Tweet out 
    }
  } // End double check for wet sponge


  // If input is dry, reset WetFlag[] 
  // A sponge isn't determined to be dry unless the input has been dry for DRYING_DELAY (3 hours)
  for(int i = 0; i < numWiredInputs + numTransmitters; i++)
  {
    if(InputState[i] == DRY) 
    {
      WetFlag[i] = DRY;
 //srg debug removed DoubleCheckFlag     DoubleCheckFlag[i] = false;
    }
    else
    {
      // If input is WET, reset drying delay
      WetToDryDelay[i] = millis() + DRYING_DELAY;
    }
  }  

 // Check for dried out sponge, after long delay
  for(int i = 0; i < numWiredInputs + numTransmitters; i++)
  {
    if( (WetFlag[i] == DRY) && (WaterDetect[i] == WET) && ((long)(millis() - WetToDryDelay[i]) >= 0))
    {
      // Sensor went from Wet To Dry
      #ifdef PRINT_DEBUG
        PrintStates();
      #endif
      
      WaterDetect[i] = DRY; 

      #ifdef PRINT_DEBUG
        PrintStates();
      #endif

      CreateTweet(i, DRY);
    }
  } // End Wet to Dry

} // ProcessSensors()


//===========================================================================================================================
/*
8888888b.                        888 8888888b.  8888888888 .d8888b.                                                       
888   Y88b                       888 888   Y88b 888       d88P  Y88b                                                      
888    888                       888 888    888 888       Y88b.                                                           
888   d88P .d88b.   8888b.   .d88888 888   d88P 8888888    "Y888b.    .d88b.  88888b.  .d8888b   .d88b.  888d888 .d8888b  
8888888P" d8P  Y8b     "88b d88" 888 8888888P"  888           "Y88b. d8P  Y8b 888 "88b 88K      d88""88b 888P"   88K      
888 T88b  88888888 .d888888 888  888 888 T88b   888             "888 88888888 888  888 "Y8888b. 888  888 888     "Y8888b. 
888  T88b Y8b.     888  888 Y88b 888 888  T88b  888       Y88b  d88P Y8b.     888  888      X88 Y88..88P 888          X88 
888   T88b "Y8888  "Y888888  "Y88888 888   T88b 888        "Y8888P"   "Y8888  888  888  88888P'  "Y88P"  888      88888P' 
*/
// Get the panStamp data
// I2C will deliver all the panStamp data to panStampData[], but this function only
// returns data for one panStamp at a time
//===========================================================================================================================
bool ReadRFSensors(RemoteSensorData_t* rfsensor, byte panStampID)
{

  #ifdef WDT
    wdt_reset();  // Reset watchdog time
  #endif
    
  int voltCalibration[3]; // voltage calibration, millivolt adjustment.  Use [3] elements because panStamp IDs are 1 and 2, there is no zero
  voltCalibration[addrMasterBath] = -71; 
  voltCalibration[addrGuestBath] =  -40; 
  
  const byte panStampOffline =  255;  // if value in panStamp Tx ID byte is 255, it means the panStamp is offline
  byte panStampData[packetsPerPanStamp * numTransmitters];     // Array to hold panstamp data sent over
  int i = 0;

  int readstatus = I2c.read(addrSlaveI2C, numTransmitters * packetsPerPanStamp , panStampData); // request data from panStamp I2C slave
  if(readstatus == 0)
  {
    gotI2CPacket = true;  // Flag to indicate sketch received I2C packet
  }

  // If we got an I2C packet, we can process it
  if(gotI2CPacket)
  {
    gotI2CPacket = false;  // Reset flag
    // First 7 bytes of data are Master bath, 2nd 7 bytes are Guest bath
    byte byteOffset = 0;
    switch (panStampID)
    {
      case addrMasterBath:
        byteOffset = 0;
        break;
      case addrGuestBath:
        byteOffset = packetsPerPanStamp;
        break;
      default:
        printf_P(PSTR("Invalid panStampID=%d\n"), panStampID );
    }
  
    // If panStamp Tx is online, calculate sensor values and return in rfsensor
    // panStamp Rx sketch will put 255 in panStamp Tx ID byte if it hasn't received data from Tx in 30 minutes
    if( panStampData[1 + byteOffset] != panStampOffline )
    { // panStamp Tx is online
    
      // Calculate millivolts 
      int millivolts;
      millivolts  = panStampData[5 + byteOffset] << 8;  
      millivolts |= panStampData[6 + byteOffset];
      millivolts = millivolts + voltCalibration[panStampID];
      if(millivolts < 0)
      { millivolts = 0;}
      
      // Calculate temperature from TMP36
      // Note: sinse reference voltage will change with the battery voltage, we need to 
      // take this into account
      int localTemp;
      localTemp  = panStampData[3 + byteOffset] << 8;  
      localTemp |= panStampData[4 + byteOffset];
      
      // Put panStamp data into rfsensor structure, only returns data for one panStamp
      // even though panStampData[] has data for all transmistters
      rfsensor->online = true;
      rfsensor->ID = panStampData[1 + byteOffset];
      rfsensor->IsWet = panStampData[2 + byteOffset];
      rfsensor->volts = millivolts;
      rfsensor->temp = localTemp;
      return true;
    }
    else
    { // panStamp Tx is offline
      rfsensor->ID = panStampID;
      rfsensor->online = false;
      return false;
    }
  } 
  else // didn't get an I2C packet
  {
    return false;
  } 

} // ReadRFSensors()


//===========================================================================================================================
/*
 .d8888b.                           888        88888888888                              888    
d88P  Y88b                          888            888                                  888    
888    888                          888            888                                  888    
888        888d888 .d88b.   8888b.  888888 .d88b.  888  888  888  888  .d88b.   .d88b.  888888 
888        888P"  d8P  Y8b     "88b 888   d8P  Y8b 888  888  888  888 d8P  Y8b d8P  Y8b 888    
888    888 888    88888888 .d888888 888   88888888 888  888  888  888 88888888 88888888 888    
Y88b  d88P 888    Y8b.     888  888 Y88b. Y8b.     888  Y88b 888 d88P Y8b.     Y8b.     Y88b.  
 "Y8888P"  888     "Y8888  "Y888888  "Y888 "Y8888  888   "Y8888888P"   "Y8888   "Y8888   "Y888 
*/
// Based on which sensor went off, and it it changed from dry to wet, or wet to dry, create message for Twitter
void CreateTweet(byte SensorArrayPosition, bool IsWet)
{
  #ifdef WDT
    wdt_reset();  // Reset watchdog time
  #endif
  
  char alertMsg[75];
  if(IsWet == WET)
  {strcpy(alertMsg, "Water Detected - "); }  // 25 characters
  else
  {strcpy(alertMsg, "Water Dried Up - "); }
  
  switch(SensorArrayPosition)
  {
    case 0:
      strcat(alertMsg, "First Fl bathroom sink. ");
      break;
    case 1:
      strcat(alertMsg, "Washing machine. ");
      break;
    case 2:
      strcat(alertMsg, "TBD1. ");
      break;
    case 3:
      strcat(alertMsg, "TBD2. ");
      break;
    case 4:
      strcat(alertMsg, "Water Heater. ");
      break;
    case 5:
      strcat(alertMsg, "Boiler. ");
      break;
    case 6:
      strcat(alertMsg, "Frig. ");
      break;
    case 7:
      strcat(alertMsg, "Dishwasher. ");
      break;
    case 8:
      strcat(alertMsg, "Kitchen Sink. ");
      break;
    case 9:
      strcat(alertMsg, "Hot Tub Pump/Filter. ");
      break;
    case 10:
      strcat(alertMsg, "Behind Hot tub. ");
      break;
    case 11:
      strcat(alertMsg, "Water Tank. ");
      break;
    case numWiredInputs:  // Master Bath
      strcat(alertMsg, "Master Bath Sink. ");
      break;
    case numWiredInputs + 1:  // Guest Bath
      strcat(alertMsg, "2nd Fl Guest Bathroom Sink.   ");  // longest text - 28 char
     break;
  }
  
   SendTweet(alertMsg);  // Send message to Twitter
  
} // CreateTweet ()


//======================================================================================
/*
 .d8888b.                         888 88888888888                              888    
d88P  Y88b                        888     888                                  888    
Y88b.                             888     888                                  888    
 "Y888b.    .d88b.  88888b.   .d88888     888  888  888  888  .d88b.   .d88b.  888888 
    "Y88b. d8P  Y8b 888 "88b d88" 888     888  888  888  888 d8P  Y8b d8P  Y8b 888    
      "888 88888888 888  888 888  888     888  888  888  888 88888888 88888888 888    
Y88b  d88P Y8b.     888  888 Y88b 888     888  Y88b 888 d88P Y8b.     Y8b.     Y88b.  
 "Y8888P"   "Y8888  888  888  "Y88888     888   "Y8888888P"   "Y8888   "Y8888   "Y888 

*/
// Send twitter text, appends the time to the message to avoid twitter blocking duplicate messages
// Example usage: 
//    strcpy(msgTweet, "Water leak by washing machine");
//    SendTweet(msgTweet);
//======================================================================================
int SendTweet(char msgTweet[])
{
  
  TweetCounter++;  // Increment tweet counter, prevents lots of Tweets going out if something is wrong.  Reset with weekly heartbeat

  // Limit number of tweets per week to 50.  Counter is set when heartbeat is executed  
  if( TweetCounter == 50 )
  { strcpy(msgTweet, "50 tweets"); } // This will overwrite tweet message coming in
  
  // Too many tweets, exit function
  if(TweetCounter > 50)
  { return 0; }  // exit function
  
  #ifdef WDT
    wdt_reset();  // Reset watchdog timer
  #endif
  
  char tweetAndTime[strlen(msgTweet) + 14];  // set char array so it can hold message and timestamp
  strcpy(tweetAndTime, msgTweet);            // copy twitter message into bigger character array
  
  // Get time from NTP Time server and append to twitter message.  This avoids duplicate tweets which may not get sent
  uint8_t getServerTime[6];
  char timebuf[13];  // char array to hold formatted time
  if(getTime(getServerTime))
  {
    if(getServerTime[4] == 1)
    { sprintf(timebuf, "%d:%02d AM", getServerTime[1], getServerTime[2]); }
    else
    { sprintf(timebuf, "%d:%02d PM", getServerTime[1], getServerTime[2]); }
    
    strcat(tweetAndTime, timebuf);  // add timestamp to twiiter message
  }
  
  delay(500);  // Thought it would be good to have a short delay after getting the time and before sending a Tweet
  
  #ifdef PRINT_DEBUG
    Serial.println(tweetAndTime);  // Print Tweet to serial monitor
    Serial.println();
  #endif
  
  #ifdef WDT
    wdt_reset();  // Reset watchdog timer
  #endif

  // Send message and timestamp to twitter
  if(twitter.post(tweetAndTime)) 
  {
    // Specify &Serial to output received response to Serial.
    // If no output is required, you can just omit the argument, e.g.  int status = twitter.wait();
    int status = twitter.wait(&Serial);
    #ifdef PRINT_DEBUG
      if(status == 200) 
      {
        Serial.println(F("\nTwitter OK."));
      } 
      else  // status != 200
      {
        Serial.print(F("Twitter failed. Err code: "));
        Serial.println(status);
      }
    #endif
    return status; 
  } 
  else 
  {
    #ifdef PRINT_DEBUG
      Serial.println(F("Twitter connection failed.\n"));
    #endif
    return 0;
  }

} // SendTweet()


//======================================================================================
/*
8888888b.          d8b          888    .d8888b.  888             888                     
888   Y88b         Y8P          888   d88P  Y88b 888             888                     
888    888                      888   Y88b.      888             888                     
888   d88P 888d888 888 88888b.  888888 "Y888b.   888888  8888b.  888888 .d88b.  .d8888b  
8888888P"  888P"   888 888 "88b 888       "Y88b. 888        "88b 888   d8P  Y8b 88K      
888        888     888 888  888 888         "888 888    .d888888 888   88888888 "Y8888b. 
888        888     888 888  888 Y88b. Y88b  d88P Y88b.  888  888 Y88b. Y8b.          X88 
888        888     888 888  888  "Y888 "Y8888P"   "Y888 "Y888888  "Y888 "Y8888   88888P' 
*/
// Print variables - remove when debugging isn't needed
//======================================================================================
void PrintStates()
{
  #ifdef WDT
    wdt_reset();  // Reset watchdog time
  #endif

  int totalInputs = numWiredInputs + numTransmitters;
  
  printf_P(PSTR("\nInputState\t"));
  for(int i = 0; i < totalInputs; i++)
  {
    Serial.print(InputState[i]);
    Serial.print("\t");
  }  
  Serial.println();  
  printf_P(PSTR("WetFlag    \t"));
  for(int i = 0; i < totalInputs; i++)
  {
    Serial.print(WetFlag[i]);
    Serial.print("\t");
  }  
  Serial.println();  

/* srg debug removed double check flag
  printf_P(PSTR("DoubleCheck\t"));
  for(int i = 0; i < totalInputs; i++)
  {
   Serial.print(DoubleCheckFlag[i]);
   Serial.print("\t");
  }  
  Serial.println();  
*/

  printf_P(PSTR("WaterDetect\t"));
  for(int i = 0; i < totalInputs; i++)
  {
    Serial.print(WaterDetect[i]);
    Serial.print("\t");
  }  
  Serial.println();

  if (masterBath.online)
  {
    Serial.println(F("\t\tIsWet\tTemp\tVolts"));
    Serial.print(F("Master Bath"));
    Serial.print(F("\t"));
    Serial.print(masterBath.IsWet);
    Serial.print(F("\t"));
    Serial.print(masterBath.temp);
    Serial.print(F("\t"));
    Serial.println(masterBath.volts);
  }
  else
  {
    Serial.println(F("Master Bath panStamp is offline"));
  }
  
  if(guestBath.online)
  {
    Serial.print("Guest Bath");
    Serial.print("\t");
    Serial.print(guestBath.IsWet);
    Serial.print("\t");
    Serial.print(guestBath.temp);
    Serial.print("\t");
    Serial.println(guestBath.volts);
  }
  else
  {
    Serial.println(F("Guest Bath panStamp is offline"));
  }

  
} // End PrintStates()



//================================================================================================================================================================================
/*
888       888                   888      888           .d8888b.                             888        888                             88888888888 d8b                        
888   o   888                   888      888          d88P  Y88b                            888        888                                 888     Y8P                        
888  d8b  888                   888      888          888    888                            888        888                                 888                                
888 d888b 888  .d88b.   .d88b.  888  888 888 888  888 888         .d88b.  888  888 88888b.  888888 .d88888  .d88b.  888  888  888 88888b.  888     888 88888b.d88b.   .d88b.  
888d88888b888 d8P  Y8b d8P  Y8b 888 .88P 888 888  888 888        d88""88b 888  888 888 "88b 888   d88" 888 d88""88b 888  888  888 888 "88b 888     888 888 "888 "88b d8P  Y8b 
88888P Y88888 88888888 88888888 888888K  888 888  888 888    888 888  888 888  888 888  888 888   888  888 888  888 888  888  888 888  888 888     888 888  888  888 88888888 
8888P   Y8888 Y8b.     Y8b.     888 "88b 888 Y88b 888 Y88b  d88P Y88..88P Y88b 888 888  888 Y88b. Y88b 888 Y88..88P Y88b 888 d88P 888  888 888     888 888  888  888 Y8b.     
888P     Y888  "Y8888   "Y8888  888  888 888  "Y88888  "Y8888P"   "Y88P"   "Y88888 888  888  "Y888 "Y88888  "Y88P"   "Y8888888P"  888  888 888     888 888  888  888  "Y8888  
                                                  888                                                                                                                         
                                             Y8b d88P                                                                                                                         
                                              "Y88P"                                                                                                                          
*/
// Returns mS until noon on next Sunday
// Used for weekly heartbeat
// ntpTime Array
// 0 - hour (12 hr format)
// 1 - hour (24 hr format)
// 2 - minute
// 3 - second
// 4 - 1 for AM, 2 for PM
// 5 - day of week.  0=Sunday
//================================================================================================================================================================================
uint32_t WeeklyCountdownTime(uint8_t *ntpTime)
{
  #ifdef WDT
    wdt_reset();  // Reset watchdog time
  #endif

  uint32_t mStoMidnight; // mS from now to midnight
  mStoMidnight =  (24 - (ntpTime[0] + 1)) * 3600000UL;  // convert hours left in the day to mS
  mStoMidnight += (60 - (ntpTime[2] + 1)) * 60000UL;    // add minutes left in current hour to mS
  mStoMidnight += (60 - ntpTime[3]) * 1000UL;     // add seconds left in current minute left to mS
  
  // Starting from midnight today, calculate how many days until Sunday starts
  int daysToSunday = 7 - ntpTime[5]  - 1;
  
  // 43200000 = mS in 12 hours - this moves time from midnight to noon on Sunday
  return (daysToSunday * 86400000UL) + mStoMidnight + 43200000UL;  
  
} // WeeklyCountdownTime()


//========================================================================================
/*
                 d8b          888     .d888        888                        d8b          
                 Y8P          888    d88P"         888                        Y8P          
                              888    888           888                                     
88888b.  888d888 888 88888b.  888888 888888        88888b.   .d88b.   .d88b.  888 88888b.  
888 "88b 888P"   888 888 "88b 888    888           888 "88b d8P  Y8b d88P"88b 888 888 "88b 
888  888 888     888 888  888 888    888           888  888 88888888 888  888 888 888  888 
888 d88P 888     888 888  888 Y88b.  888           888 d88P Y8b.     Y88b 888 888 888  888 
88888P"  888     888 888  888  "Y888 888  88888888 88888P"   "Y8888   "Y88888 888 888  888 
888                                                                       888              
888                                                                  Y8b d88P              
888                                                                   "Y88P"               
*/
// printf_P(PSTR()) stores the string in Program Memory (Flash)
// http://www.nerdkits.com/videos/printf_and_scanf/
//========================================================================================
void printf_begin(void)
{
  // fdevopen() is provided to associate a stream to a device. 
  fdevopen( &serial_putc, 0 );
} // End printf_begin()



//========================================================================================
/*
                          d8b          888                            888            
                          Y8P          888                            888            
                                       888                            888            
.d8888b   .d88b.  888d888 888  8888b.  888          88888b.  888  888 888888 .d8888b 
88K      d8P  Y8b 888P"   888     "88b 888          888 "88b 888  888 888   d88P"    
"Y8888b. 88888888 888     888 .d888888 888          888  888 888  888 888   888      
     X88 Y8b.     888     888 888  888 888          888 d88P Y88b 888 Y88b. Y88b.    
 88888P'  "Y8888  888     888 "Y888888 888 88888888 88888P"   "Y88888  "Y888 "Y8888P 
                                                    888                              
                                                    888                              
                                                    888                              
*/
// Called by printf_begin()
//========================================================================================
int serial_putc(char c, FILE *) 
{
  Serial.write( c );
  return c;
}  // End serial_putc()


//=========================================================================================================================
/*


                   .d888 888                                                   8888888b.                            888    
                  d88P"  888                                                   888   Y88b                           888    
                  888    888                                                   888    888                           888    
.d8888b   .d88b.  888888 888888 888  888  888  8888b.  888d888 .d88b.          888   d88P .d88b.  .d8888b   .d88b.  888888 
88K      d88""88b 888    888    888  888  888     "88b 888P"  d8P  Y8b         8888888P" d8P  Y8b 88K      d8P  Y8b 888    
"Y8888b. 888  888 888    888    888  888  888 .d888888 888    88888888         888 T88b  88888888 "Y8888b. 88888888 888    
     X88 Y88..88P 888    Y88b.  Y88b 888 d88P 888  888 888    Y8b.             888  T88b Y8b.          X88 Y8b.     Y88b.  
 88888P'  "Y88P"  888     "Y888  "Y8888888P"  "Y888888 888     "Y8888 88888888 888   T88b "Y8888   88888P'  "Y8888   "Y888 
*/
// Restarts program from beginning but does not reset the peripherals and registers
//=========================================================================================================================
void software_Reset() 
{
  asm volatile ("  jmp 0");  
} // End software_Reset()


//====================================================================================================
// Displays the amount of freem SRAM
//====================================================================================================
int freeRam() 
{
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}  // End freeRam

