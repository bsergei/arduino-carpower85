/* 
 Accessories power controller for car.
 
 Attiny85 at 1Mhz with power saving.
 
 The connections to the ATTiny are as follows:
 ATTiny    Arduino    Info
 Pin  1  - 5          RESET
 Pin  2  - 3          Out 3 (negative)
 Pin  3  - 4          In (voltage sensor)
 Pin  4  -            GND
 Pin  5  - 0          Out 1 (negative)
 Pin  6  - 1          Out 2 (negative)
 Pin  7  - 2          In (alert) (pull-up)
 Pin  8  -            +Vcc
 
 */

#include <avr/sleep.h>
#include <avr/interrupt.h>

// Routines to set and clear bits (used in the sleep code)
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

// Variable for the Sleep/power down modes.
volatile boolean f_wdt = 1;
volatile boolean f_alarm = 0;

#define PIN_VOLTAGE 4
#define PIN_ALERT 2
#define PIN_OUT1 0
#define PIN_OUT2 1
#define PIN_OUT3 3

#define OUT_ON LOW
#define OUT_OFF HIGH

#define VOLT_THRESHOLD 13.0 // Voltage threshold value to detect engine start.
#define DELAY_TURN_ON 10 // OUT1 and OUT2 turned on after 10 secs.
#define DELAY_TURN_OFF_OUT1 10 // OUT1 turned off after 10 sec.
#define DELAY_TURN_OFF_OUT2 20 // OUT2 turned off after 1 hour. 3400 ~ 3600/1.05. 1.05 is a measured time for one loop.
//#define DELAY_TURN_OFF_OUT2 3400 // OUT2 turned off after 1 hour. 3400 ~ 3600/1.05. 1.05 is a measured time for one loop.

#define VOLTAGE_CALIBRATION 19.23 // Value calibrated to particular voltage divider. Adjust for your own divider.

void setup()  
{
  // Inputs
  pinMode(PIN_VOLTAGE, INPUT); // PB4/A2
  pinMode(PIN_ALERT, INPUT_PULLUP); // PB2
  
  // Outputs
  pinMode(PIN_OUT1, OUTPUT); // PB0
  pinMode(PIN_OUT2, OUTPUT); // PB1
  pinMode(PIN_OUT3, OUTPUT); // PB3

  // Initial OUT state
  digitalWrite(PIN_OUT1, OUT_OFF);
  digitalWrite(PIN_OUT2, OUT_OFF);
  digitalWrite(PIN_OUT3, OUT_OFF);

  setup_watchdog(6); // Approximately 1 second to sleep. Measured 1.05 sec for one loop.
  GIMSK = 0b00100000;    // turns on pin change interrupts
  PCMSK = 0b00000100;    // turn on interrupts on pins PB0, PB1, &amp; PB4
  sei();                 // enables interrupts
}

boolean counterDisabled = true;
boolean enabled = false;
unsigned long counter = 0;

void loop()
{
  if (f_wdt == 1) {  // wait for timed out watchdog / flag is set when a watchdog timeout occurs
    f_wdt = 0;       // reset flag
    
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
    
    if (f_alarm) {
      f_alarm = 0;
      if (!enabled) {
        // Turn on OUT2 from alarm.
        digitalWrite(PIN_OUT2, OUT_ON);
        counter = 0;
        counterDisabled = false;
      }
    }

    if (!counterDisabled)
    {
      counter ++;

      if (enabled && counter > DELAY_TURN_ON)
      {
        digitalWrite(PIN_OUT1, OUT_ON);
        digitalWrite(PIN_OUT2, OUT_ON);
        digitalWrite(PIN_OUT3, OUT_OFF);
        counterDisabled = true; // stop counter
      }
      else if (!enabled)
      {
        digitalWrite(PIN_OUT3, OUT_ON);
        
        if (counter > (DELAY_TURN_OFF_OUT2 + 30))
        {
          digitalWrite(PIN_OUT3, OUT_OFF);
          counterDisabled = true; // stop counter
        }
        else if (counter > DELAY_TURN_OFF_OUT2)
        {
          digitalWrite(PIN_OUT2, OUT_OFF);
        }
        else if (counter > DELAY_TURN_OFF_OUT1)
        {
          digitalWrite(PIN_OUT1, OUT_OFF);
        }
      }
    }
    
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
  //bb|= (1<<WDCE); // Error here?
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
  f_wdt = 1;  // set global flag
}

ISR(PCINT0_vect) {
  f_alarm = 1;
}
