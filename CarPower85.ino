/* 
 Simple accessories power controller for car. 
 Detects engine start (network voltage > 13.0V) and activate D0 and D1 after 3 secs.
 On engine stop deactivates D0 after 10 sec and D1 after 1 hour.
 
 Attiny85 at 1Mhz with power saving.
 
 The connections to the ATTiny are as follows:
 ATTiny    Arduino    Info
 Pin  1  - 5          RESET / Rx (Not receiving any data)
 Pin  2  - 3          Tx for serial conenction (for debug)
 Pin  3  - 4          Voltage sensor
 Pin  4  -            GND
 Pin  5  - 0          Out 1
 Pin  6  - 1          Out 2
 Pin  7  - 2          NC
 Pin  8  -            +Vcc

 To debug via SoftwareSerial use 8 MHz frequency.
 
 */

#include <avr/sleep.h>
#include <avr/interrupt.h>

// Uncomment to debug.
//#include <SoftwareSerial.h>

// Routines to set and clear bits (used in the sleep code)
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// Uncomment to debug.
//#define rxPin 5    // Use a non-existant pin as we are not interested in receiving data
//#define txPin 3
//SoftwareSerial mySerial(rxPin, txPin); // RX, TX

// Variable for the Sleep/power down modes.
volatile boolean f_wdt = 1;

void setup()  
{
  // Uncomment to debug.
  //mySerial.begin(4800);

  pinMode(4, INPUT); // PB4/A2
  pinMode(0, OUTPUT); // PB0
  pinMode(1, OUTPUT); // PB1

  digitalWrite(0, LOW);
  digitalWrite(1, LOW);

  setup_watchdog(6); // Approximately 1 second to sleep. Measured 1.05 sec for one loop.
}

boolean counterDisabled = true;
boolean enabled = false;
long counter = 0;

#define VOLT_THRESHOLD 13.0 // Voltage threshold value to detect engine start.
#define ENABLED_DELAY 3 // PB0 and PB1 to HIGH after 3 sec.
#define DISABLED_DELAY_PIN0 10 // PB0 to LOW after 10 sec.
#define DISABLED_DELAY_PIN1 3400 // PB1 to LOW after 1 hour. 3400 ~ 3600/1.05. 1.05 is a measured time for one loop.

#define VOLTAGE_CALIBRATION 19.23 // Value calibrated to particular voltage divider. Adjust for your own divider.

void loop()
{
  if (f_wdt==1) {  // wait for timed out watchdog / flag is set when a watchdog timeout occurs
    f_wdt=0;       // reset flag

    int sum = 0;
    for (int i = 0; i < 10; i++)
    {
      int adc = analogRead(2);
      sum += adc;
    }

    float f = sum / 10.0 / 1024.0 * VOLTAGE_CALIBRATION;

    boolean newEnabled = f >= VOLT_THRESHOLD;
    if (newEnabled != enabled)
    {
      enabled = newEnabled;
      counter = 0;
      counterDisabled = false;
    }

    if (!counterDisabled)
    {
      counter ++;

      if (enabled && counter > ENABLED_DELAY)
      {
        digitalWrite(0, HIGH);
        digitalWrite(1, HIGH);
        counterDisabled = true; // stop counter
      }
      else if (!enabled)
      {
        if (counter > DISABLED_DELAY_PIN0)
        {
          digitalWrite(0, LOW);
          if (counter > DISABLED_DELAY_PIN1)
          {
            digitalWrite(1, LOW);
            counterDisabled = true; // stop counter
          }
        }
      }
    }
    
    // Uncomment to debug.
    //mySerial.println(f);
    //mySerial.println(counter);

    // Go to sleep.
    system_sleep();
  }
}

// Set system into the sleep state.
// System wakes up when wtchdog is timed out.
void system_sleep() {
  cbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter OFF
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // sleep mode is set here
  sleep_enable();
  sleep_mode();                        // System actually sleeps here
  sleep_disable();                     // System continues execution here when watchdog timed out 
  sbi(ADCSRA,ADEN);                    // switch Analog to Digitalconverter ON
}

// 0=16ms, 1=32ms,2=64ms,3=128ms,4=250ms,5=500ms
// 6=1 sec,7=2 sec, 8=4 sec, 9= 8sec
void setup_watchdog(int ii) {
  byte bb;
  int ww;
  if (ii > 9 ) ii=9;
  bb=ii & 7;
  if (ii > 7) bb|= (1<<5);
  bb|= (1<<WDCE);
  ww=bb;
  MCUSR &= ~(1<<WDRF);
  // start timed sequence
  WDTCR |= (1<<WDCE) | (1<<WDE);
  // set new watchdog timeout value
  WDTCR = bb;
  WDTCR |= _BV(WDIE);
}

// Watchdog Interrupt Service / is executed when watchdog timed out
ISR(WDT_vect) {
  f_wdt=1;  // set global flag
}
