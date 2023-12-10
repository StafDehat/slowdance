// TimeFrame V3.1 (modified)
// Copyright (C) 2016 Cubic-Print
// 

// Original source core here: http://www.github.com/cubic-print/timeframe
// Original video: http://youtu.be/LlGywKkifcI
// Order your DIY kit here: http://www.cubic-print.com/TimeFrame

// Modifications by Paul Hutchison, Dec 2016:
// - Glowing LED for pushbutton
// - Frame starts in "off" mode
// - Removed LED only mode
// - Additional analog input to control electromagnet duty
// - Adjustments to ranges
// * Instructable available here: http://www.instructables.com/member/SparkItUp/
// * Source code available here: https://github.com/paulh-rnd/timeframe

//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    GNU General Public License terms: <http://www.gnu.org/licenses/>.


//#define DEBUG
//uncomment to check serial monitor and see LED heartbeat
//Tactile Switch needs to be pressed longer in debug mode to change mode

// Arduino Nano has CPU speed 16 MHz
#define CPU_FREQ 16000000L

//Base frequency and trimmer Ranges
#define BASE_FREQ 80.0 //80 is on the spot for many flowers. Feel free to play with this +/-5Hz
#define MIN_PHASE_SHIFT 0.25
#define MAX_PHASE_SHIFT 2.0

#define MIN_BRIGHTNESS 0.0 // allows light to be off to reveal the full oscillating effect
#define MAX_BRIGHTNESS 20.0 // too high and flickering will occur

#define MIN_STRENGTH 5.0 //  too low no movement
#define MAX_STRENGTH 25.0 // too high and magnet will overheat
  
/**The microprocessor of the Arduino UNO (ATmega328P) has 3 timers:
  timer0 (8 bits) counts from 0 to 256 and controls the PWM of pins 5 and 6. It is also used by the delay(), millis() and micros() functions.
  timer1 (16 bits) counts from 0 to 65535 and is used for the PWM control of pins 9 and 10. It is also used by the Servo.h library
  timer2 (8 bits) which is used by the Tone() function and the PWM generation on pins 3 and 11.
  https://www.aranacorp.com/en/using-the-arduino-timers/
**/
#define ledPin 10
#define magPin 3
#define pbLedPin 5
#define pbInputPin 6


  const float pbLedMaxDuty = 64.0;
  const int LED = 13; //on board LED
  int mode_changed = 1;
  int mode = 3; //toggelt by SW button, start OFF
  bool pbReleased = true;
  //mode 1 = normal slow motion mode (power on)
  //mode 2 = distorted reality mode
  //mode 3 = off
  
  float phase_shift = 0.1; //eg. f=0.5 -> T=2 -> 2 seconds per slow motion cycle

  //Timer 2 for Magnet
  //Prescaler = 1024 = CS111 = 64us/tick
  //PIN 3
  float duty_mag = 15; //12 be carefull not overheat the magnet. better adjust force through magnet position
  float frequency_mag = BASE_FREQ;
  // 1024 is the prescaler value, set via bitmask when we define register TCCR2B
  long time_mag = round(CPU_FREQ/1024/frequency_mag);

  //Timer 1 for LED 
  //Prescaler = 8 = CS010 = 0.5 us/tick
  //PIN 9
  float duty_led = 7;
  float frequency_led = frequency_mag+phase_shift;
  // 8 is the prescaler value, set via bitmask when we define register TCCR1B
  long time_led = round(CPU_FREQ/8/frequency_led);
  
void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  pinMode(LED, OUTPUT); //Heart Beat LED
  pinMode(LED, OUTPUT); //Power button LED (pulsates slowly when off)
  pinMode(pbInputPin, INPUT); //button pin
  pinMode(A1, INPUT); // beat frequency
  pinMode(magPin, OUTPUT); //MAG: Timer 2B cycle output
  pinMode(ledPin, OUTPUT); //LED: Timer 1B cycle output 

  #ifdef DEBUG
    pinMode(9, OUTPUT); //Timer 2A half frequency at 50% duty output for debugging halbe frequenz! 50% duty
    pinMode(11, OUTPUT); //Timer 1A half frequency at 50% duty output for debuggin
  #endif

  mag_off();
  OCR2A = round(time_mag); // Timer2 overflows when it reaches OCR2A.  Hence, this sets the frequency.
  OCR2B = round(duty_mag*time_mag/100L); // This sets the duty cycle... somehow.
  led_off();
  OCR1A = round(time_led); //Hierraus frequenz output compare registers
  OCR1B = round(duty_led*time_led/100L); // This sets the duty cycle... somehow.
  
  sei();
}

void loop() {
  // Change mode when push button is pushed, don't do anything else until PB is released
  if (pbReleased && digitalRead(pbInputPin) == HIGH) //Read in switch
  {
    pbReleased = false;
    mode += 1;
    if (mode >= 4) mode = 1; //rotary menu
    mode_changed = 1;
    delay(100);
  }
  else if (!pbReleased && digitalRead(pbInputPin) == LOW)
  {
    pbReleased = true;
  }

  if (mode == 3)
  {
    if (mode_changed == 1)
    { 
      mag_off(); //mode = 3
      led_off(); //mode = 3
      mode_changed = 0;
    }//mode = 3
    
    float currentbrightness = millis()/10000.0;
    int value = (pbLedMaxDuty * 3.0/4) + pbLedMaxDuty * sin( currentbrightness * 2.0 * PI );
    if (value < 0) 
      value = 0;
    analogWrite(pbLedPin, value);
  }
  //Read in trimmer settings
  //Speed: 0.0 .. -2.0 Hz (negative because I think the effect is better when the led phase is behind the mag phase)
  phase_shift = (MAX_PHASE_SHIFT-MIN_PHASE_SHIFT)/1023L*analogRead(A1)-MAX_PHASE_SHIFT; 
  delay(3);
  duty_led = -(MAX_BRIGHTNESS-MIN_BRIGHTNESS)/1023L*analogRead(A0)+MAX_BRIGHTNESS;  //Brightness: duty_led 2..20
  delay(3);
  duty_mag = -(MAX_STRENGTH-MIN_STRENGTH)/1023L*analogRead(A2)+MAX_STRENGTH;  //Electromagnet strength: duty_mag 10..20
  frequency_led = frequency_mag*mode+phase_shift;
    
  if ((mode == 1) && (mode_changed == 1))
  {
    analogWrite(pbLedPin, pbLedMaxDuty / 4.0);
    frequency_mag = BASE_FREQ;
    mag_on();
    led_on();
    mode_changed = 0;
  }//mode = 1  
  if ((mode == 2) && (mode_changed == 1))
  {
    //frequency doubleing already done in main loop
    mode_changed = 0;
  }//mode = 2 

  time_mag = round(CPU_FREQ/1024L/frequency_mag); 
  time_led = round(CPU_FREQ/8L/frequency_led);

  OCR2A = round(time_mag); //to calculate frequency of output compare registers
  OCR2B = round(duty_mag*time_mag/100L); 
  OCR1A = round(time_led); 
  OCR1B = round(duty_led*time_led/100L);

#ifdef DEBUG
 //Heatbeat on-board LED
  digitalWrite(LED, HIGH); // LED on
  delay(300);
  digitalWrite(LED, LOW); // LED off
  delay(300); 
  digitalWrite(LED, HIGH); // LED on
  delay(200);
  digitalWrite(LED, LOW); // LED off
  delay(1200); 
  //serial print current parameters
  Serial.print("Phase Shift: "); //speed of animation
  Serial.print(phase_shift);
  Serial.print("  Force: ");
  Serial.print(duty_mag);
  Serial.print("  Freq: ");
  Serial.print(frequency_mag);
  Serial.print("  Brightness: ");
  Serial.println(duty_led);
#endif
} //main loop

// https://www.tutorialspoint.com/timer-registers-in-arduino
// https://www.aranacorp.com/en/using-the-arduino-timers/
// https://docs.arduino.cc/tutorials/generic/secrets-of-arduino-pwm
void mag_on() {
  TCCR2A = 0;
  TCCR2B = 0;

  // Waveform mode 7
  // FastPWM
  //   * Count from 0-OCR2A (mode 3 would go 0-255)
  //   * Update OCRx at BOTTOM, TOV flag set at TOP
  //TCCR2A = _BV(COM2A0) | _BV(COM2B1) | _BV(WGM21) | _BV(WGM20); // 0110 0011
  //TCCR2B = _BV(WGM22) | _BV(CS22)| _BV(CS21)| _BV(CS20); // 0000 1111

  // Set to waveform mode 7, FastPWM
  // "Fast PWM Mode with OCRA top"
  // Ref: https://docs.arduino.cc/tutorials/generic/secrets-of-arduino-pwm
  TCCR2A |= _BV(WGM21) | _BV(WGM20);
  TCCR2B |= _BV(WGM22);

  // Set prescaler to 0x111 (/1024 for Timer2):
  TCCR2B |= _BV(CS22)| _BV(CS21)| _BV(CS20);

  // ????
  // 00 = 
  // 01 = Toggle on Compare Match
  // 10 = Non-inverted PWM
  // 11 = 
  // COM2A=01, COM2B=10
  TCCR2A |= _BV(COM2A0) | _BV(COM2B1);
}

void mag_off() {
  TCCR2A = 0;
  TCCR2B = 0;
  TCCR2A = _BV(COM2A0) | _BV(COM2B1);
  TCCR2B = _BV(CS22)| _BV(CS21)| _BV(CS20); 
}

void led_on() {
TCCR1A = 0;
  TCCR1B = 0;
  TCCR1A = _BV(COM1A0) | _BV(COM1B1) | _BV(WGM11) | _BV(WGM10); 
  TCCR1B =  _BV(WGM13) | _BV(WGM12)  |  _BV(CS11);
}

void led_off() {
  TCCR1A = 0;
  TCCR1B = 0;
  TCCR1A = _BV(COM1A0) | _BV(COM1B1); 
  TCCR1B =  _BV(CS11); 
}  
