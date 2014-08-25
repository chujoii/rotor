/*
  rotor.ino --- measure distance and scan space program for robot

  Copyright (C) 2012 Roman V. Prikhodchenko



  Author: Roman V. Prikhodchenko <chujoii@gmail.com>


  This file is part of rotor.

    rotor is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    rotor is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with rotor.  If not, see <http://www.gnu.org/licenses/>.



 Keywords: robot



 Usage:


 History:
 
 Project started at 2013.05(may).31



 Code:
*/

//#include <math.h>
//#include <Stepper.h>
//#include <Servo.h>
//#include <Wire.h>


//#include <Wire.h> 
//#include <LiquidCrystal_I2C.h>

int cfg = 0;

// -------------------------------------- physical dimensions --------------------------------------

// all dimension in cm, us


// -------------------------------------- pins --------------------------------------
/* http://arduino.cc/en/Main/ArduinoBoardDuemilanove
   On boards other than the Mega, use of the Servo library disables analogWrite() (PWM) functionality on pins 9 and 10.
   
   pin  - capability     - function   (if pin without [] == NotConnected)
   [0]  - rx             - rx
   [1]  - tx             - tx
   [2]  -      ext_int 0 - encoder
   [3]  - pwm, ext_int 1 - ultrasound range sensor Echo pin
   [4]  -                - ultrasound range sensor Trig pin
   5    - pwm            -
   6    - pwm            -
   7    -                -
   8    -                -
   9    - {pwm}          -
   10   - {pwm}          -
   11   - pwm            -
   12   -                -
   [13] - led            - led

   [A0] -                - sharp_10_80
   [A1] -                - sharp_20_150
   [A2] -                - sharp_100_550
   A3   -                - 
   A4   - sda            - 
   A5   - scl            - 


*/



// -------------------------------------- const --------------


const byte pwm_min = 0;
const byte pwm_max = 255;

const unsigned long delay_us_between_step = 1000;
const int delay_ms_between_step = 100;

const int Trig_pin = 4;
const int Echo_pin = 3;
const byte Echo_interrupt = 1;
const unsigned long Trig_delay_us = 10;
const unsigned long Echo_max_delay_us = 60000;
/*
  speed_of_sound = 331.4 + 0.6T (m/s)
  Distance = pulse_width * speed_of_sound / 2
  Distance(m) = pulse_width(s) *       165.7
  Distance(m) = pulse_width(us) * 0.0001657
  Distance(cm) = pulse_width(us) *  0.01657




  for US-020 
  Trig_delay_us >= 10us
  distance 2cm-700cm
  Echo_max_delay_us = inf; // ?7m = ? fixme        for US-020 =  
  
  for US-100
  Trig_delay_us >= 5us
  distance 2cm - 350cm with termocompensation (450cm)              ==  60ms - out of range (from datasheet)
  Echo_max_delay_us = inf; // ?7m = ? fixme    
*/

const int led = 13;
const int sharp_10_80_pin = A0;
const int sharp_20_150_pin = A1;
const int sharp_100_550_pin = A2;
const int Max_int = 32767;
const int Min_int = -32768;

// -------------------------------------- other --------------


//const int i2c_lcd_addr = 0x20;
//LiquidCrystal_I2C lcd(i2c_lcd_addr,16,2);  // set the LCD address to 0x20 for a 16 chars and 2 line display

//const int i2c_btn_addr = 0x22;

//const int i2c_???_addr = 0x23; // fixme new



//volatile boolean Echo_timer_status = false; // false______/----true----\__________false
long int Trig_timer_start = 0;
volatile long int Echo_timer_start = 0;
volatile long int Echo_timer_end = 0;

int sharp_10_80_val_cm;
int sharp_20_150_val_cm;
int sharp_100_550_val_cm;
int us_sonar_val_cm;

volatile boolean changes = false;

// -------------------------------------- code --------------


void Echo_timer()
{
	if (digitalRead(Echo_pin)==HIGH){
		// start of echo
		Echo_timer_start = micros();
	} else {
		// end of echo
		Echo_timer_end = micros();
	}
	changes = true;

	digitalWrite(led, HIGH);
}


void setup() 
{



	//pinMode(digital_pin_optocouple_led, OUTPUT);
	//digitalWrite(digital_pin_optocouple_led, LOW);

		
	//pinMode(digital_pin_scanner_left, INPUT);
	//pinMode(digital_pin_scanner_right, INPUT);


	pinMode(Trig_pin, OUTPUT);
	pinMode(Echo_pin, INPUT);


	digitalWrite(Trig_pin, LOW);
	
	attachInterrupt(Echo_interrupt, Echo_timer, CHANGE);
	

	pinMode(led, OUTPUT);
	digitalWrite(led, LOW);

	Serial.begin(9600);
	delay(1000);
}



void loop()
{
	byte incomingByte = 0;
	int x;
        //if (Serial.available() > 0) {
                // read the incoming byte:
                incomingByte = Serial.read();
		
		
		digitalWrite(led, LOW);
	
		// first version without encoder
		
		// send ultrasound wave 
		
		Echo_timer_start = 0;
		Echo_timer_end = 0;
		us_sonar_val_cm = Max_int; // fixme "Infinity" ?
		digitalWrite(Trig_pin, HIGH);
		delayMicroseconds(Trig_delay_us);
		digitalWrite(Trig_pin, LOW);
		Trig_timer_start = micros();
		
		/*
		// while wait interrupt form Echo, check ADC 
		x = analogRead(sharp_10_80_pin);
		sharp_10_80_val_cm = 1.0/(0.011428869737816816   -0.00009103255938432977*x + 0.0000017050364804206691*x*x  -3.844126649774024e-9*x*x*x + 3.0899429195502907e-12*x*x*x*x);
		Serial.print(sharp_10_80_val_cm);
		Serial.print(" ");

		x = analogRead(sharp_20_150_pin);
		sharp_20_150_val_cm = 1.0/(0.009136880597455758   -0.00011375101991517708*x  + 0.0000013080880081279523*x*x -3.563346518526791e-9*x*x*x + 3.4025145623983315e-12*x*x*x*x);
		Serial.print(sharp_20_150_val_cm);
		Serial.print(" ");

		x = analogRead(sharp_100_550_pin);
		sharp_100_550_val_cm = 1.0/(-0.10187243921321469  + 0.0010199350940547584*x    -0.000003799162850464599*x*x + 6.391737720657689e-9*x*x*x -3.96057389025945e-12*x*x*x*x);
		Serial.print(sharp_100_550_val_cm);
		Serial.print(" ");
		
		


		
		while ((Echo_max_delay_us > (micros()-Trig_timer_start))     &&     ((Echo_timer_start != 0) || (Echo_timer_end !=0))){  // fixme "micros(): This number will overflow (go back to zero), after approximately 70 minutes."
			// fixme need use timer instead while
		}
	
		*/


		/*
		while (Echo_max_delay_us > (micros()-Trig_timer_start)){  // fixme "micros(): This number will overflow (go back to zero), after approximately 70 minutes."
			// fixme need use timer instead while

			if (changes) {changes = false; Serial.println(" o");}
		}
		*/

		delay(60);
		

		Serial.print(" ets="); Serial.print(Echo_timer_start);
		Serial.print(" ete="); Serial.print(Echo_timer_end);
		Serial.print(" ");
		
		if ((Echo_timer_start != 0) && (Echo_timer_end !=0)){
			us_sonar_val_cm = (Echo_timer_end - Echo_timer_start) *  0.01657;
		}
		Serial.print(us_sonar_val_cm);
		Serial.println();
				


//}
	
		delay(delay_ms_between_step);
	
	
}
