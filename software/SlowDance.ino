/**
 * Author: Andrew Howard
 * Email: stafdehat@gmail.com
 * 
 * We're using pins in Pulse-Width-Modulation (PWM) mode, so we
 *   won't actually write anything to the OUTPUT pins directly.
 *   Instead, we set the internal Timer registers to control the
 *   frequency & duty-cycle of the pulsing-pins.  This is closer
 *   to API interaction than logic-based coding.  You'll have to
 *   consult Timer tutorials for Arduino, and/or datasheets of
 *   the chip (ie: ATMega328) for specifics on controlling PWM
 *   via Timer registers.

 * The Arduino uses Timer 0 internally for the millis() and
 *   delay() functions, so be warned that changing the frequency
 *   of this timer will cause those functions to be erroneous.
 *   Using the PWM outputs is safe if you don't change the
 *   frequency, though.
 * Ref: https://docs.arduino.cc/tutorials/generic/secrets-of-arduino-pwm

 * ATMega328 Data Sheet:
 *   https://ww1.microchip.com/downloads/aemDocuments/documents/MCU08/ProductDocuments/DataSheets/ATmega48A-PA-88A-PA-168A-PA-328-P-DS-DS40002061B.pdf
**/

/**
Pins:
  3  (D3)  - Controls the Magnet's MOSFET gate, via pulse-width modulation.
  10 (D10) - Controls the LED's MOSFET gate, via pulse-width modulation.
  4  (D4)  - Pushbutton on rotary encoder
  5  (D5)  - 1 of 2 rotary encoder pins
  6  (D6)  - 1 of 2 rotary encoder pins
**/

#include <Arduino.h>
#include "Adafruit_Debounce.h"
#include <BasicEncoder.h>
#include <TM1637Display.h>

#define DEBUG

#define CPU_FREQ 16000000L // Arduino Nano
//#define CPU_FREQ 8000000L // Adafruit Trinket 3v
//#define CPU_FREQ 16000000L // ProMicro 5V
//#define CPU_FREQ 8000000L // ProMicro 3V3
//#define CPU_FREQ 128000000L // Raspberry Pi Pico

// Note: TCNT2 is 8-bits, so has range 0-255, which in turn limits the
//   frequency to 61Hz - 7812Hz.
// In practice, anything <70Hz or >90Hz, is pointless for our puposes.
#define BASE_FREQ 8000 // 80.00 is on the spot for many flowers.
#define MIN_FREQ  7000 // 70.00 Hz
#define MAX_FREQ  9000 // 90.00 Hz

#define BASE_PHASE_SHIFT 50  // 0.50 Hz
#define MIN_PHASE_SHIFT  0   // 0.00 Hz
#define MAX_PHASE_SHIFT  500 // 5.00 Hz

#define BASE_BRIGHTNESS 5   // 0.05 (5%) duty cycle
#define MIN_BRIGHTNESS  0   // 0.00 (0%)
#define MAX_BRIGHTNESS  25  // 0.25 (25%)

#define BASE_MAG_DUTY 20 // 0.20 (20%)
#define MIN_MAG_DUTY  5  // 0.05 (5%). too low & no movement
#define MAX_MAG_DUTY  30 // 0.30 (30%). too high & magnet could overheat

// Store as int, and /10 or /100 as necessary when we use it.
// Trying to store as a float directly causes base-2 rounding errors.
int magFreqNom = BASE_FREQ;
int magFreqActual = BASE_FREQ;
int phaseShift = BASE_PHASE_SHIFT;
int ledFreq = magFreqActual - phaseShift; //LED should oscillate slower than magnet.
uint8_t magDuty = BASE_MAG_DUTY;
uint8_t ledDuty = BASE_BRIGHTNESS;


/**The microprocessor of the Arduino UNO (ATmega328P) has 3 timers:
  timer0 (8 bits) counts from 0 to 256 and controls the PWM of pins 5 and 6. It is also used by the delay(), millis() and micros() functions.
  timer1 (16 bits) counts from 0 to 65535 and is used for the PWM control of pins 9 and 10. It is also used by the Servo.h library
  timer2 (8 bits) which is used by the Tone() function and the PWM generation on pins 3 and 11.
  https://www.aranacorp.com/en/using-the-arduino-timers/
**/
// These pins are dictated by hardware/firmware of the Arduino.  In FastPWM mode,
//   OCxA will invert every 1x wavelength.  OCxB is the only one that'll oscillate
//   at a user-defined frequency, with a user-define duty cycle.
#define ledPin 10 // When Timer1=Mode7(FastPWM), Pin10=OC1B.
#define magPin 3  // When Timer2=Mode7(FastPWM), Pin3=OC2B.

const uint8_t encoderBtnPin = 12; // D12 (used to be D4, but that's shorted to D3 now)
const uint8_t encoderPinA = 5; // D5
const uint8_t encoderPinB = 6; // D6

#define CLK 7 // D7
#define DIO 8 // D8

#define MAG_FREQ_LED    14 // A0
#define PHASE_SHIFT_LED 15 // A1
#define MAG_DUTY_LED    16 // A2
#define LED_DUTY_LED    17 // A3

#define DOOR_PIN 2 // D2

uint8_t currentMode = 0;

/** Setup for the rotary encoder **/
// Create a debounce object for the button, which will be pressed when LOW
Adafruit_Debounce encoderBtn(encoderBtnPin, LOW);
BasicEncoder encoder(encoderPinA, encoderPinB);
void pciSetup(byte pin)  // Setup pin change interupt on pin
{
  *digitalPinToPCMSK(pin) |= bit(digitalPinToPCMSKbit(pin));  // enable pin
  PCIFR |= bit(digitalPinToPCICRbit(pin));                    // clear outstanding interrupt
  PCICR |= bit(digitalPinToPCICRbit(pin));                    // enable interrupt for group
}
void setup_encoders(int a, int b) {
  uint8_t old_sreg = SREG;     // save the current interrupt enable flag
  noInterrupts();
  pciSetup(a);
  pciSetup(b);
  encoder.reset();
  SREG = old_sreg;    // restore the previous interrupt enable flag state
}
ISR(PCINT2_vect) { encoder.service(); }

TM1637Display display(CLK, DIO);



/** SETUP **/
void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // Monitor rotary-encoder dial
  setup_encoders(encoderPinA,encoderPinB);
  //encoder.set_reverse();

  encoderBtn.begin(); // Monitor rotary-encoder pushbutton

  // Start magnet vibration
  pinMode(magPin, OUTPUT); //MAG: Timer2B cycle output
  mag_off();
  set_mag_freq(BASE_FREQ); // Also sets the duty
  mag_on();

  // Initialize LED strip
  pinMode(ledPin, OUTPUT); //LED: Timer1B cycle output
  led_off();
  change_phase_shift(0); // Sets Freq & Duty
  led_on();

  pinMode(MAG_FREQ_LED, OUTPUT);
  pinMode(PHASE_SHIFT_LED, OUTPUT);
  pinMode(MAG_DUTY_LED, OUTPUT);
  pinMode(LED_DUTY_LED, OUTPUT);

  pinMode(DOOR_PIN, INPUT_PULLUP);

  // Initialize 4-7 display
  display.setBrightness(0x01);
  display.clear();

  // Initialize config-panel mode to MagFreqNom(Hz)
  setMode(0);

  sei(); // Enable interrupts
}

void loop() {
  // Check if button pushed or encoder turned.
    // Update the button state, this will do the digitalRead() for us
  // must be called ~every 10ms to keep state and catch presses 
  encoderBtn.update();
  if (encoderBtn.justPressed()) {
    Serial.println("Button was just pressed!");
    switchModes();
  } else if (encoderBtn.justReleased()) {
    Serial.println("Button was just released!");
  }

  int encoder_change = encoder.get_change();
  if (encoder_change) {
    updateSettings(encoder_change);
  }

  // Show/Hide Controls - limit switch on door panel
  if(doorChanged()) {
    if(doorClosed()) {
      sleep();
    } else if (doorOpen()) {
      wake();
    }
  }

  // Add a small debouncing delay
  delay(10);

/**
  #ifdef DEBUG
    Serial.print("T(s)=");
    Serial.println(round(millis()/1000));
    Serial.print("  magDuty:        ");
    Serial.println(magDuty);
    Serial.print("  magFreqNom(Hz):    ");
    Serial.println(magFreqNom);
    Serial.print("  phaseShift(Hz): ");
    Serial.println(phaseShift);
    Serial.print("  ledDuty(%):     ");
    Serial.println(ledDuty);
    Serial.print("  OCR2A: ");
    Serial.println(OCR2A);
    delay(1000);
  #endif
  **/
} //main loop


// Disable the displays when the door's closed.
int8_t lastDoorState = LOW;
unsigned long doorTimeout = 0;  // the last time the output pin was toggled
uint8_t debounceTime = 50;
uint8_t doorChanged() {
  int8_t doorState = digitalRead(DOOR_PIN);
  if (doorState == lastDoorState) {
    return false; // No change
  }
  if (millis() < doorTimeout) {
    return false; // Too soon - wait for debounce
  }
  lastDoorState = doorState; // Remember the last read
  doorTimeout = millis() + debounceTime; // Note when we'll next care
  return true;
}
uint8_t doorClosed() {
  return (digitalRead(DOOR_PIN) == LOW);
}
uint8_t doorOpen() {
  return (digitalRead(DOOR_PIN) == HIGH);
}
void sleep() {
  uint8_t firstModePin = MAG_FREQ_LED;
  digitalWrite(firstModePin+currentMode, LOW);
  display.clear();
}
void wake() {
  uint8_t firstModePin = MAG_FREQ_LED;
  digitalWrite(firstModePin+currentMode, HIGH);
  updateDisplay();
}


void updateDisplay() {
  display.clear();
  switch(currentMode) {
    case 0: // Vibration frequency (Hz, 70.00-90.00, 8000=80.00)
      Serial.print("Vibrational frequency (Hz): ");
      Serial.println(magFreqNom/100.0);
      display.showNumberDecEx(magFreqNom, 0b01000000, false, 4, 0);
      break;
    case 1: // Phase shift (Hz, 0.00-5.00, 1=0.01)
      Serial.print("Phase shift (Hz): ");
      Serial.println(phaseShift/100.0);
      display.showNumberDecEx(phaseShift/10, 0b10000000, true, 2, 1);
      break;
    case 2: // Magnet duty (%, 0.05-0.30)
      Serial.print("Electromagnet duty cycle (%): ");
      Serial.println(magDuty);
      display.showNumberDecEx(magDuty, 0b10000000, true, 3, 1);
      break;
    case 3: // LED Brightness (duty) (%, 0.0-0.20)
      Serial.print("LED duty/brightness (%): ");
      Serial.println(ledDuty);
      display.showNumberDecEx(ledDuty, 0b10000000, true, 3, 1);
      break;
  } // END switch
}

void switchModes() {
  // 4 modes, use modulus to make it cyclic
  setMode((currentMode+1)%4);
}
void setMode(uint8_t newMode) {
  // ModeLED pins are sequential, so we can cheat with addition.
  uint8_t firstModePin = MAG_FREQ_LED;
  uint8_t priorMode = (newMode+3)%4;
  currentMode = newMode;

  #ifdef DEBUG
    Serial.print("Leaving mode ");
    Serial.println(priorMode);
    Serial.print("Entering mode ");
    Serial.println(newMode);
  #endif

  // Just to avoid any potential int-overflows, we'll restart the encoder at 0
  //   every time the mode changes.  Should be self-limited at that point.
  encoder.reset();

  // Turn off the prior mode's LED
  digitalWrite(firstModePin+priorMode, LOW);
  // Turn on the new mode's LED
  digitalWrite(firstModePin+newMode, HIGH);
  
  // Display current mode, & current value
  updateDisplay();
}


// Changes are applied LIVE.
void updateSettings(int encoder_change) {
  switch(currentMode) {
    case 0: // Vibration frequency (Hz, 70.0-90.0)
      change_mag_freq(encoder_change);
      break;
    case 1: // Phase shift (Hz, 0.0-5.0)
      change_phase_shift(encoder_change);
      break;
    case 2: // Magnet duty (%, 0.05-0.30)
      change_mag_duty(encoder_change);
      break;
    case 3: // LED Brightness (duty) (%, 0.0-0.20)
      change_led_duty(encoder_change);
      break;
  } // END switch
  updateDisplay();
}


/** Define the 4 functions that will fire when the rotary encoder
 * is turned.  Which function fires, depends on which mode you're
 * in when you turn the encoder.
 * **/
// Mode 1 - Magnet vibrational frequency
void change_mag_freq(int8_t steps) {
  int newFreq = magFreqNom + steps*25; // Every step is 25% (ie: 25, since magFreqNom is hundredths)
  if ( newFreq > MAX_FREQ ) {
    Serial.println("Max frequency exceeded");
    newFreq = MAX_FREQ;
  } else if ( newFreq < MIN_FREQ ) {
    Serial.println("Min frequency exceeded");
    newFreq = MIN_FREQ;
  }
  set_mag_freq(newFreq);
  // Update ledFreq to maintain the configured phaseShift.
  set_led_freq(magFreqActual-phaseShift);
}
// Mode 2 - Phase shift
void change_phase_shift(int8_t steps) {
  // It's confusing as hell in code, but to the end-user it's easier if we
  //   always refer to phaseShift as a positive value.  However, you get better
  //   visuals if the light blinks a hair slower than the vibration Hz, so we
  //   subtract the phaseShift from magFreq to get ledFreq.  Technically the
  //   phase shift is negative, but we store a positive value everywhere
  //   *except* in the actual calculation of OCR1A.
  // Note: We store phaseShift as hundredths for math convenience with magFreq
  //   & ledFreq, but it's adjustable by tenths.  So, steps*10 here.
  int newShift = phaseShift + steps*10;
  if ( newShift > MAX_PHASE_SHIFT ) {
    Serial.println("Max phase shift exceeded");
    newShift = MAX_PHASE_SHIFT;
  } else if ( newShift < MIN_PHASE_SHIFT ) {
    Serial.println("Min phase shift exceeded");
    newShift = MIN_PHASE_SHIFT;
  }
  phaseShift = newShift;
  set_led_freq(magFreqActual-phaseShift);
}
// Mode 3 - Magnet duty cycle (strength)
void change_mag_duty(int8_t steps) {
  int8_t newDuty = magDuty+steps;
  if ( newDuty > MAX_MAG_DUTY ) {
    Serial.println("Max magnet duty exceeded");
    newDuty = MAX_MAG_DUTY;
  } else if ( newDuty < MIN_MAG_DUTY ) {
    Serial.println("Min magnet duty exceeded");
    newDuty = MIN_MAG_DUTY;
  }
  set_mag_duty(newDuty);
}
// Mode 4 - LED duty cycle (brightness)
void change_led_duty(int8_t steps) {
  int8_t newDuty = ledDuty+steps;
  if ( newDuty > MAX_BRIGHTNESS ) {
    Serial.println("Max LED duty/brightness exceeded");
    newDuty = MAX_BRIGHTNESS;
  } else if ( newDuty < MIN_BRIGHTNESS ) {
    Serial.println("Min LED duty/brightness exceeded");
    newDuty = MIN_BRIGHTNESS;
  }
  Serial.print("Setting LED duty = ");
  Serial.println(newDuty);
  set_led_duty(newDuty);
}


/**
 * All functions below this point are interacting directly with
 * ATMega328p Timer registers.  It's insanely confusing from a
 * code-logic perspective.  It's much more similar to coding
 * against an API - the content is nonsensical unless you have
 * the documentation handy.
 * https://www.tutorialspoint.com/timer-registers-in-arduino
 * https://www.aranacorp.com/en/using-the-arduino-timers/
 * https://docs.arduino.cc/tutorials/generic/secrets-of-arduino-pwm
**/
void mag_on() {
  // TL;DR: Set Timer2 to PWM mode.
  // PWM will handle the oscillation, duty-cycle, and frequency for us.

  /** Super Efficient Method
  TCCR* are bitmasks that control Timer behaviour.
  Register |   2^7   |   2^6   |   2^5   |   2^4   |   2^3   |   2^2   |   2^1   |   2^0   |
  TCCR2A   | COM2A1  | COM2A0  | COM2B1  | COM2B0  |    -    |    -    |  WGM21  |  WGM20  |
  TCCR2B   |  FOC2A  |  FOC2B  |    -    |    -    |  WGM22  |  CS22   |  CS21   |  CS20   |

  COM2A (COM2A1,COM2A0):
    0,0 = Normal port operation, OC2A disconnected.
    0,1 = WGM22 = 0: Normal Port Operation, OC0A Disconnected.
          WGM22 = 1: Toggle OC2A on Compare Match.
    1,0 = Clear OC2A on Compare Match, set OC2A at BOTTOM, (non-inverting mode).
    1,1 = Set OC2A on Compare Match, clear OC2A at BOTTOM, (inverting mode).
  Ref: ATMega328 data sheet, Table 18-3 "Compare Output Mode, Fast PWM Mode"

  COM2B (COM2B1,COM2B0):
    0,0 = Normal port operation, OC2B disconnected.
    0,1 = Reserved
    1,0 = Clear OC2B on Compare Match, set OC2B at BOTTOM, (non-inverting mode).
    1,1 = Set OC2B on Compare Match, clear OC2B at BOTTOM, (inverting mode).
  Ref: ATMega328 data sheet, Table 18-6 "Compare Output Mode, Fast PWM Mode"

  WGM2 (WGM22,WGM21,WGM20):
    0: 0,0,0 = Normal, TCNT2=0x00->0xFF
    1: 0,0,1 = Phase-correct PWM, TCNT2=0x00->0xFF->0x00
    2: 0,1,0 = CTC, 
    3: 0,1,1 = Fast PWM, TCNT2=0x00->0xFF
    4: 1,0,0 = Reserved
    5: 1,0,1 = Phase-correct PWM, TCNT2=0x00->OCR2A->0x00
    6: 1,1,0 = Reserved
    7: 1,1,1 = Fast PWM, TCNT2=0x00->OCR2A
  Ref: ATMega328 data sheet, Table 18-8 "Waveform Generation Mode Bit Description"

  CS2:
    0,0,0 = No clock source (Timer/Counter stopped)
    0,0,1 = clkT2S/(No prescaling)
    0,1,0 = clkT2S/8 (From prescaler)
    0,1,1 = clkT2S/32 (From prescaler)
    1,0,0 = clkT2S/64 (From prescaler)
    1,0,1 = clkT2S/128 (From prescaler)
    1,1,0 = clkT2S/256 (From prescaler)
    1,1,1 = clkT2S/1024 (From prescaler)
  Ref: ATMega328 data sheet, Table 18-9 "Clock Select Bit Description"

  Goal:
    COM2A = 01 # Toggle OC2A (Pin#11,D11) on TCNT2==OCR2A
    COM2B = 10 # Clear OC2B (Pin#3,D3) on TCNT2==OCR2B, set OC2B at TCNT2=0
    WGM2 = 111 # Mode7, FastPWM, TCNT=0x00->OCR2A
    CS2 = 111 # Prescaler=1024
  **/
  TCCR2A = 0x63; // 0110 0011
  TCCR2B = 0x0f; // 0000 1111
}

void mag_off() {
  // TL;DR: Set Timer2 to Normal mode.
  // By disabling PWM, Pin10 just stays off.

  /** Super Efficient Method
  TCCR* are bitmasks that control Timer behaviour.
  Register |   2^7   |   2^6   |   2^5   |   2^4   |   2^3   |   2^2   |   2^1   |   2^0   |
  TCCR2A   | COM2A1  | COM2A0  | COM2B1  | COM2B0  |    -    |    -    |  WGM21  |  WGM20  |
  TCCR2B   |  FOC2A  |  FOC2B  |    -    |    -    |  WGM22  |  CS22   |  CS21   |  CS20   |

  COM2A (COM2A1,COM2A0):
    0,0 = Normal port operation, OC2A disconnected.
    0,1 = Toggle OC2A on Compare Match
    1,0 = Clear OC2A on Compare Match
    1,1 = Set OC2A on Compare Match
  Ref: ATMega328 data sheet, Table 18-2 "Compare Output Mode, non-PWM Mode"

  COM2B (COM2B1,COM2B0):
    0,0 = Normal port operation, OC2B disconnected.
    0,1 = Normal port operation, OC2B disconnected.
    1,0 = Clear OC2B on Compare Match
    1,1 = Set OC2B on Compare Match
  Ref: ATMega328 data sheet, Table 18-5 "Compare Output Mode, non-PWM Mode"

  Goal:
    COM2A = 01 # Toggle OC2A on Compare Match
    COM2B = 10 # Clear OC2B on Compare Match
    WGM2 = 111 # Mode 0, Normal, TCNT2=0x00->0xFF
    CS2 = 111 # Prescaler=1024
  Note: Once Mode=0, we're no longer in PWM, so we really don't care about
    the value of COM2A,COM2B,CS2.  So, we'll just re-set them to what we
    *do* want when we *are* using PWM.
  **/
  // TCCR2A = 0x60; // 0110 0000
  TCCR2A = 0b01100000;
  //TCCR2B = 0x07; // 0000 0111
  TCCR2B = 0b00000111;

  // Let's be absolutely certain that that mag stays LOW, not HIGH.
  // Don't wanna burn out the coil.
  digitalWrite(magPin, LOW);
}

void set_mag_freq(int f) {
  Serial.print("In set_mag_freq, f=");
  Serial.println(f);
  // 1s has 16-million CPU ticks, since Arduino Nano has a 16MHz CPU.
  // We set Timer2's Prescaler to 1024 in mag_on(), so TCNT2 increments
  //   once every 1024 CPU ticks.  (16mil/1024=15,625 tps)
  // To oscillate at Frequency f, we need to divide 15625 into 'f' groups.
  //   The size of those groups will be OCR2A - the upper limit of TCNT2.
  // And since TCNT2 is 0-based, we need to fix an off-by-one.
  // TL;DR:
  //   CPU_FREQ/Prescaler/(OCR2A+1) = f
  // When solving for OCR2A:
  //   OCR2A = (CPU_FREQ/Prescaler/f)-1
  // Note: TCNT2 is 8-bits, so has range 0-255, which in turn limits the
  //   frequency to 61.0-7812.5 Hz.  Also note, at 7812.5 Hz, TCNT2 would
  //   count from 0 to 1, then loop, so the duty cycle would be forced to
  //   50%.  Lower frequency means more duty-cycle granularity.
  magFreqNom = f;
  //Serial.print(round(15625/(f/100.0))-1);
  OCR2A = round(15625/(f/100.0))-1; // Set PWM to that frequency
  // With prescaler=1024, we're not super granular, so the round() function
  //   above is actually significant.  The ledFreq is much more granular
  //   though, so we *should* calculate our phaseShift from the *actual*
  //   magFreq, not the *nominal* magFreq.  So, let's calculate what the
  //   *actual* magFreq is, given what we just set in OCR2A.
  // Note: Assuming nominal==actual, in practice, resulted in a
  //   phaseShift error of +/- 0.3Hz!
  magFreqActual = round(15625/(OCR2A+1.0)*100);
  #ifdef DEBUG
    Serial.print("OCR2A:");
    Serial.println(OCR2A);
    Serial.print("magFreqActual: ");
    Serial.println(magFreqActual);
  #endif
  // Update magDuty too - it's set as a percentage of magFreq, so the change cascades.
  set_mag_duty(magDuty);
}

void set_mag_duty(uint8_t p) {
  Serial.print("In set_mag_duty, p=");
  Serial.println(p);
  // magPin turns HI at 0, and LO when TCNT==OCR2B,
  //  then cycles back to 0 (HI) when TCNT==OCR2A.
  // Set OCR2B=p% of the way from 0 to OCR2A.
  // Note: 50% duty cycle would be set_mag_duty(0.50);
  magDuty = p; // Remember it for later
  Serial.print("Setting OCR2B = ");
  Serial.println(round(p/100.0 * (OCR2A+1) - 1));
  OCR2B = round(p/100.0 * (OCR2A+1) - 1);

}

void led_on() {
  /** Super Efficient Method
  TCCR* are bitmasks that control Timer behaviour.

  Register |   2^7   |   2^6   |   2^5   |   2^4   |   2^3   |   2^2   |   2^1   |   2^0   |
  TCCR1A   | COM1A1  | COM1A0  | COM1B1  | COM1B0  |    -    |    -    |  WGM11  |  WGM10  |
  TCCR1B   |  ICNC1  |  ICES1  |    -    |  WGM13  |  WGM12  |  CS12   |  CS11   |  CS10   |
  // From ATMega328 Datasheet, sections
  //  16.11.1 "TCCR1A – Timer/Counter1 Control Register A"
  //  16.11.2 "TCCR1B – Timer/Counter1 Control Register B"

  Goal:
    COM1A = 01
    COM1B = 10
    WGM1 = 1111
    CS1 = 010
  **/
  TCCR1A = 0b01100011;
  //TCCR1A = 0x63; // 0110 0011
  TCCR1B = 0b00011010;
  //TCCR1B = 0x1A; // 0001 1010
}

void led_off() {
  /** Super Efficient Method
  Register |   2^7   |   2^6   |   2^5   |   2^4   |   2^3   |   2^2   |   2^1   |   2^0   |
  TCCR1A   | COM1A1  | COM1A0  | COM1B1  | COM1B0  |    -    |    -    |  WGM11  |  WGM10  |
  TCCR1B   |  ICNC1  |  ICES1  |    -    |  WGM13  |  WGM12  |  CS12   |  CS11   |  CS10   |

  Goal:
    COM1A = 01
    COM1B = 10
    WGM1 = 1111
    CS1 = 010
  **/
  //TCCR1A = 0x60; // 0110 0000
  TCCR1A = 0b01100000;
  //TCCR1B = 0x02; // 0000 0010
  TCCR1B = 0b00000010;

  // Leaving PWM mode should set the pin LOW, but let's be explicit
  //digitalWrite(ledPin, LOW);
}

void set_led_freq(int f) {
  Serial.print("In set_led_freq.  f=");
  Serial.println(f);
  // 1s has 16-million CPU ticks, since Arduino Nano has a 16MHz CPU.
  // We set Timer1's Prescaler to 8 in led_on(), so TCNT1 increments
  //   once every 8 CPU ticks.  (16mil/8=2mil tps)
  // To oscillate at Frequency f, we need to divide 2mil into 'f' groups.
  //   The size of those groups will be OCR1A - the upper limit of TCNT1.
  // And since TCNT1 is 0-based, we need to fix an off-by-one.
  // TL;DR:
  //   CPU_FREQ/Prescaler/(OCR1A+1) = f
  // When solving for OCR1A:
  //   OCR1A = (CPU_FREQ/Prescaler/f)-1
  // Note: TCNT1 is 16-bits, so has range 0-65535, which in turn limits the
  //   frequency to 30.6-1mil Hz.  Also note though, that at 1MHz, TCNT1
  //   would only count from 0 to 1, then loop, so we'd be forced into 50%
  //   duty cycle.  Lower frequency means more duty-cycle granularity.
  ledFreq = f;
  OCR1A = round(2000000/(f/100.0))-1;
  // LED duty is a percentage of ledFreq.  Since Freq changed, update Duty too.
  set_led_duty(ledDuty);
}

void set_led_duty(uint8_t p) {
  // ledPin turns HI at 0, and LO when TCNT==OCR1B,
  //  then cycles back to 0 (HI) when TCNT==OCR1A.
  // Set OCR1B=p% of the way from 0 to OCR1A.
  // Note: 50% duty cycle would be set_led_duty(50);
  ledDuty = p;
  if ( ledDuty == 0 ) {
    led_off();
    return;
  }
  led_on();
  OCR1B = round(p/100.0 * (OCR1A+1) - 1);
}
