/*  =============================================================================
 *  Real-Time Interrupt Service for implementing a controller on the Arduino Mega
 *
 *  Filename:        rt_isr_arduino_mega_v1_1.ino
 *  Original Author: Steve Southward
 *  Version:         1.1
 *  Modified by:     
 *  Release Date:    11/30/2017
 *  Board:           Developed for Arduino Mega 2560 (16MHz base clock frequency) //need to change this to Due
 *
 *  Copyright  (c)   2017  Steve Southward <scsouth@vt.edu>
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *  Description of this program:
 *  This program is a working example of how to configure and run an interrupt service routine at a
 *  desired fixed sample rate, which is required for most real-time control implementations.  This
 *  program also includes examples of how to access analog and digital inputs and outputs on the
 *  Arduino Mega 2560.  Most of the code should run on the Arduino Uno and other Arduino platforms
 *  with minor changes to pin assignments.
 *
 *  Definitions of some useful Acronyms:
 *    IDE = Integrated Development Environment, the software development interface
 *    ISR = Interrupt Service Routine, the real-time control primary function
 *    RT = real-time
 *    SPI = Serial Peripheral Interface, a 4-wire serial communication bus protocol
 *    CAN = Controller Area Network, a 2-wire serial communication bus protocol
 *    UART = Universal Asynchronous Receiver/Transmitter, chip to translate between parallel & 2-wire serial
 *    I/O = Input/Output
 *    PWM = Pulse Width Modulation, these are used to drive output channels
 *    LED = Light Emitting Diode
 *    Hz = sampling frequency units, also known as "Hertz", also known as "samples/second"
 *    DC = literally means "Direct Current", but here refers to "zero frequency", or "constant"
 *    SRAM = Static Random Access Memory
 *    EEPROM = Electrically Eraseable PRogrammable Memory
 *
 *  There are three types of memory on the Arduino.
 *    1) Flash memory: this is where your program is stored.  This memory is non-volatile,
 *        which means that your program stays in this memory even after the power is turned off.
 *        The Mega has 256K of flash memory.  Flash memory can only be written ~10000 times.
 *    2) SRAM:  this is where your program creates and manipulates variables during run time.
 *        SRAM is volatile, which means that this memory is cleared when power is turned off.
 *        The Mega has 8K of SRAM memory.
 *    3) EEPROM:  this memory is also nonvolatile and can be used to store data for long-term
 *        retrieval.  The Mega has 4K of EEPROM.
 */

//  Include the header libraries

//  These header files are required to support serial communication
#include <SPI.h>
#include <SoftwareSerial.h>
#include <DueTC.h> //Library from Github

//  Include all the classes
#include "Motor.h"

/*
//  These header files support the SparkFun CAN Bus Shield (uncomment only if using a CAN Shield)
#include <Canbus.h>
#include <defaults.h>
#include <global.h>
#include <mcp2515.h>            // Microchip MCP2515 Can Controller
#include <mcp2515_defs.h>
*/


//  These Compiler Directives define compile-time Constants used in the real-time software


// Pin assignment definitions based on electrical schematic
#define ENC_LK      A0          // Left knee encoder
#define ENC_RK      A1          // Right knee encoder
#define POT_L       A2          // Left spring potentiometer
#define POT_R       A3          // Right spring potentiometer
#define ENC_LA      A4          // Left ankle encoder
#define ENC_RA      A5          // Right ankle encoder

#define LED_OUT     13          // LED pin number (on-board LED is hardwired to digital pin 13 on DUE)

/* TO-DO
#define IMU (SCL1 & SDA1)
#define MOTOR_CONTROLLER (TX3_14 & RX3_15)
#define MULTIPLEXER (SCL_21 & SDA_20)
define 4 proximity sensors and the load cells
define LED pin for the LED heartbeat
define and instantiate motor??
*/

/* NOTE:  All processors will have an upper limit for how large the sampling frequency FS can be, and this limit will
 *  depend on the amount of computation you will do in the ISR every sample, so you cannot just sample arbitrarily fast. You
 *  will have to experiment to determine the upper limit for your application, and you most likely do not need to sample
 *  as fast as possible with your hardware.  If you try to sample too fast, i.e. if FS is too big, you will "overrun the
 *  processor", which basically means that you are asking the processor to do too many computations in a fixed amount of
 *  time, which can cause your program to run incorrectly.
 */

//TO DO: PICK FS
#define FS          100        // user specified INTEGER sample rate (Hz)

/* NOTE:
 *  TIMER1 is used to set the sample rate for this program.  The TIMER1 scaling in this example code only allows
 *  accurate sample rates from 1Hz to 4000Hz.  Using this example code, FS=3000Hz requires ~65% processor usage
 */

/* NOTE:  A second-order low-pass digital filter has been included in this software as an example.  You may or may
 *  not need to use it, but it can often be very useful to clean up a signal.  Please note that this digital filter
 *  can NOT be used to provide anti-aliasing for your sampled signals!  Also note that this digital filter can ONLY
 *  be used on one signal.  Modification will be required if you want to filter more than one signal.  The digital 
 *  filter has a "unity gain at DC", which means that if you apply a constant input to the filter, the output of the 
 *  filter will give you the same constant value after any transients have decayed away.  Any sinusoidal or noise 
 *  signals that are greater than the "break frequency" will be rejected.
 *
 * Filter Design Guidelines:
 *    - The floating-point break frequency can NEVER be chosen greater than (FS/2)!!!
 *    - It is also important to make sure that the break freqency is not too small
 *    - A good rule of thumb is to always choose:  (0.001*FS) < FBREAK_LPF < (0.3*FS)
 *    - The damping ratio (ZETA_LPF) is a floating point number that must be chosen between 0 and 1
 *    - You should NEVER choose a damping ratio that is negative or equal to zero
 *    - A good rule of thumb is to always choose:  0.15 < ZETA_LPF <= 1.0
 */

#define FBREAK_LPF  1         // low-pass filter break frequency (Hz)
#define ZETA_LPF    0.707     // low-pass filter damping ratio

//#define PRINT_TO_SERIAL_MONITOR  true // uncomment this line to print to the serial monitor
//#define PRINT_TO_SERIAL_PLOTTER  true // uncomment this line to print to the serial plotter

/* NOTE: For this example code, the Mega must have FS<=300 if printing to the Serial Monitor
 *  because printing to the Serial Monitor takes processor time, and the more information that
 *  is printed, the more time will be required during each sample interval.  The amount of time
 *  required to print to the Serial Monitor can be minimized by increasing the baud rate of the
 *  serial connection.  
 *  You can NOT print to the Serial Monitor and the Serial Plotter at the same time!
 *  Please use this feature with caution!
 */

//  Declare the global variables (all functions have access to global variables)

static bool control_enable = false;
int kneeEncoderL = 0;
int kneeEncoderR = 0;
int potL = 0;
int potR = 0;
int ankleEncoderL = 0;
int ankleEncoderR = 0;
int distR1 = 0;
int distR2 = 0;
int distL1 = 0;
int distL2 = 0;
int avgDistR = 0;
int avgDistL = 0;

int currKneeEncoderL = 0;
int currKneeEncoderR = 0;
int currPotL = 0;
int currPotR = 0;
int currAnkleEncoderL = 0;
int currAnkleEncoderR = 0;
int currDistR1 = 0;
int currDistR2 = 0;
int currDistL1 = 0;
int currDistL2 = 0;
int currAvgDistR = 0;
int currAvgDistL = 0;

// TO-DO: declare motor controller, multiplexer (proximity sensors + load cells), IMU

int threshold = 1; // TO-DO: define threshold
int count = 0; // global count variable for interrupts
volatile bool led = false;

/*  ----------setup----------
 *
 *    This is the setup function which is automatically run one time during startup,
 *    which happens at powerup or reset.  This function should include all 
 *    initialization tasks.
 *
 *    Do NOT change the name of this function!
 */

void setup()
{
  //  Copyright  (c)   2017  Steve Southward <scsouth@vt.edu>

  //  Declare variables local to the setup function


  //  Initialize the serial monitor (required if you are printing to the Serial Monitor)
  //  the entered number is called the "baud rate" which means "bits per second"
  Serial.begin(115200);  // can choose {4800, 9600, 19200, 38400, 57600, 115200, 230400, or 250000}


  //  Configure the digital I/O pins we want to use
  pinMode(ENC_LK, INPUT);
  pinMode(ENC_RK, INPUT);
  pinMode(POT_L, INPUT);
  pinMode(POT_R, INPUT);
  pinMode(ENC_LA, INPUT);
  pinMode(ENC_RA, INPUT);

  pinMode(LED_OUT, OUTPUT);

  // TO-DO: configure IMU, motor controller, multiplexer


  //instantiating motors (I think this is how you do it)
  Motor motorL();
  Motor motorR();

  //  Initialize and configure the low-pass filter for operation
  digital_lpf(0, true);


  // TO-DO: Configure the TIMERS
  /*  see the following links for more details on configuring PWM Timers on the Arduino platform
   *  <http://www.engblaze.com/microcontroller-tutorial-avr-and-arduino-timer-interrupts/>
   *  <https://arduino-info.wikispaces.com/Arduino-PWM-Frequency>
   *  <https://arduinodiy.wordpress.com/2012/02/28/timer-interrupts/>
   *  <https://arduino-info.wikispaces.com/Timers-Arduino>
   *  <http://playground.arduino.cc/Main/TimerPWMCheatsheet>
   */

  noInterrupts();              // first disable all global processor interrupts

  setupTC2_Interrupt(period,tcClock,IRQ_handler);

  interrupts();                // now it is ok to enable all global processor interrupts


  /*
  //  Initialize the MCP2515 CAN Controller at 500 kbits/sec (only if you are using a CAN Shield!)
  if (Canbus.init(CANSPEED_500))
  {   // CAN speeds available: 125, 250, and 500 kbits/sec
      Serial.println("CAN Initialization Ok");
      delay(1500);
  }
  else
  {
      Serial.println("CAN Initialization FAILED");
      return;
  }
  */
}

void IRQ_handler()
{
  count++;
  if (count == 40)
  {
    if (led == false)
    {
      digitalWrite(LED_OUT, HIGH);
      count = 0;
      led = true;
    }
    else
    {
      digitalWrite(LED_OUT, LOW);
      count = 0;
      led = false;
    }
  }
}



/*  ----------ISR----------
 *
 *   This is the Interrupt Service Routine function.  This function will be called at
 *   the fixed uniform sample rate of FS Hz.  All of the real-time functional tasks
 *   that must run at a fixed clock frequency should be implemented inside this function.
 *
 *   Do NOT change the name of this function!
 */

ISR(TIMER1_COMPA_vect)
{
  //  Copyright  (c)   2017  Steve Southward <scsouth@vt.edu>

  //  Declare variables local to the ISR function
  static int samplenum = 0;

  //  Set the DIG_PROBE bit true on entry (only needed for debugging)
  digitalWrite(DIG_PROBE, true);

  //  Toggle the LED heartbeat (this will switch the LED on and off every second), operation duration: ~8.2us
  samplenum++;  //  increment the counter
  if (samplenum == (int)FS)
  { 
    //  one second has just completed
    samplenum = 0;  //  reset the counter
    digitalWrite(LED_OUT, !digitalRead(LED_OUT));  //  toggle the LED
  }


  //  Read an Analog Input signal
  //  ana_in_value will be a 10-bit integer between 0 and 1023, operation duration: ~112us
  //  zero volts input -> 0, 3.3 volts input (for due) -> 1023 
  kneeEncoderL = analogRead(ENC_LK);  
  kneeEncoderR = analogRead(ENC_RK);
  potL = analogRead(POT_L);
  potR = analogRead(POT_R);
  ankleEncoderL = analogRead(ENC_LA);
  ankleEncoderR = analogRead(ENC_RA);

  avgDistR = (distR1 + distR2)/2;
  avgDistL = (distL1 + distL2)/2;

  // TO-DO: read other serial inpts

  // TO-DO: Digitally filter the analog input signal
  //ana_in_filtered = digital_lpf((float)ana_in_value, false); //  operation duration: ~87us


  // TO-DO Generate a desired control_out signal
  if (control_enable)
  {
    // compute the control signal
    control_out = (int)(ana_in_filtered / 4);     // replace this simple example with your desired control function

    //  Note:  the value of control_out MUST be an 8-bit unsigned integer in the range of 0 to 255
    //  because the PWM output is only 8-bit, so it can only accept values between 0 and 255
    control_out = constrain(control_out, 0, 255); //  saturate the control_out value between 0 and 255
  }
  else
  {
    // control is disabled so zero the output
    control_out = 0;
  }

  //  Output the control_out signal as a PWM output
  //  for a PWM output:  0 = 0% duty cycle, 255 = 100% duty cycle, operation duration: ~16.6us
  analogWrite(PWM_OUT, (int)control_out);



#ifdef PRINT_TO_SERIAL_MONITOR
  // Only compile & execute these lines if PRINT_TO_SERIAL_MONITOR has been defined
  Serial.print("right knee encoder =");                //  Print a label to the serial monitor
  Serial.print(ENC_LK);          //  Display the value of the analog input to the serial monitor
  /* change these
  Serial.print("\tafilt=");            //  Print a label to the serial monitor
  Serial.print(ana_in_filtered, 2.4);  //  Display the filtered analog input to the serial monitor with 2 decimal place resolution
  Serial.print("\tctrl=");             //  Print a label to the serial monitor
  Serial.print((int)control_out);      //  Display the control output to the serial monitor
  Serial.print("\ten=");               //  Print a label to the serial monitor
  Serial.print(control_enable);        //  Display the value of control_enable to the serial monitor
  Serial.print("\n");                  //  Print a new line character to the serial monitor
  */
#else 
#ifdef PRINT_TO_SERIAL_PLOTTER
  // Only compile & execute these lines if PRINT_TO_SERIAL_PLOTTER has been defined
  // you can ONLY display ONE signal at a time in the Serial Plotter!
  Serial.print(ENC_LK);          // Plot the raw analog input value to the Serial Plotter
  Serial.print(",");                   // separator
  /* these will be different
  Serial.print((int)control_out*4);      // Plot the control output value to the Serial Plotter
  Serial.print(",");                   // separator
  Serial.println(ana_in_filtered);     // Plot the filtered analog input to the Serial Plotter
#endif
#endif


  /*
  //  Create a CAN message (only if you are using a CAN Shield!)
  tCAN message;       //  declare a special CAN message variable
  message.id = 0x00;  //  Set the ID of the location where the CAN message is to be sent. 11-bit system (0x00 to 0x7FF)
  message.header.rtr = 0;
  message.header.length = 8;   //Set message length to 8 bytes
  message.data[0] = 0x00;
  message.data[1] = 0x00;
  message.data[2] = 0x00;
  message.data[3] = 0x00;
  message.data[4] = 0x00;
  message.data[5] = 0x00;
  message.data[6] = 0x00;
  message.data[7] = 0x00;

  //  Send the CAN message
  mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), 0);
  if (mcp2515_send_message(&message)) {
      Serial.println("CAN Transmit OK");
  }
  else {
      Serial.println("CAN Transmit FAILED");
  }
  */


  //  Set the DIG_PROBE bit false on exit  (only needed for debugging)
  digitalWrite(DIG_PROBE, false);
}


// TO-DO: figure out how to filter multiple signals
/*  ----------digital_lpf----------
 *
 *   This is a second-order digital low-pass filter function.  This function MUST be called
 *   at a fixed sample rate FS, and can ONLY process one signal.  This function will produce
 *   a floating-point output from a floating-point input signal.  Note that this filter
 *   requires intialization in the setup() function to compute its filter coefficients.
 *
 *   Do NOT change the name of this function!
 */

float digital_lpf(float analog_meas_K, bool initialize)
{
  //  Copyright  (c)   2017  Steve Southward <scsouth@vt.edu>

  //  Define local variables
  float wT, scale;
  static float A_LPF[2], B_LPF[3];
  static float analog_meas_KM1 = 0;   // KM1 = time step k-minus-one
  static float analog_meas_KM2 = 0;   // KM2 = time step k-minus-two
  float analog_filt_K = 0;
  static float analog_filt_KM1 = 0;
  static float analog_filt_KM2 = 0;


  //  Initialize the filter coefficients
  if (initialize)
  {
    //  use a Tustin Bilinar Transform to convert to discrete time
    wT = 2 * PI * FBREAK_LPF / FS;
    scale = wT * wT + 4 * (1 + ZETA_LPF * wT);

    //  define low pass filter coefficients
    B_LPF[0] = wT * wT / scale;
    B_LPF[1] = 2 * B_LPF[0];
    B_LPF[2] = B_LPF[0];

    A_LPF[0] = 2 * (wT * wT - 4) / scale;
    A_LPF[1] = (wT * wT + 4 * (1 - ZETA_LPF * wT)) / scale;

    //  initialize the delayed values in memory
    analog_meas_KM1 = 0;
    analog_meas_KM2 = 0;
    analog_filt_KM1 = 0;
    analog_filt_KM2 = 0;
    analog_filt_K = 0;
  }
  else    //  this is the normal run-time operational mode
  {
    //  Generate the next value of the filtered output
    analog_filt_K = B_LPF[0] * analog_meas_K + B_LPF[1] * analog_meas_KM1 + B_LPF[2] * analog_meas_KM2;
    analog_filt_K = analog_filt_K - A_LPF[0] * analog_filt_KM1 - A_LPF[1] * analog_filt_KM2;

    analog_meas_KM2 = analog_meas_KM1;
    analog_meas_KM1 = analog_meas_K;
    analog_filt_KM2 = analog_filt_KM1;
    analog_filt_KM1 = analog_filt_K;
  }


  //  Output the filtered analog value
  return analog_filt_K;
}

/*  ----------loop----------
 *
 *   This is the main program loop.  The only tasks that should be in this loop are background tasks
 *   that do not need to run at the main sampling frequency FS.  Your may not have any background
 *   tasks, or you may include serial communication or diagnostic monitoring of signals here.  An
 *   example State Machine is included in the loop function below.  State Machines are commonly used
 *   to manage sequencing of different processes in a control system.
 *
 *   Do NOT change the name of this function!
 */

void loop()
{
  //  Copyright  (c)   2017  Steve Southward <scsouth@vt.edu>

  //  Define local variables
  static int state = 0;
  static int count = 0;

  bool dig_in_value = false;


  //  Run all background tasks

  //  Read a Digital Input pin, used for enabling the control process
  dig_in_value = digitalRead(DIG_IN); 
  //  dig_in_value will either be false (logical LOW) or true (logical HIGH)


  //  Evaluate the state machine
  switch (state)
  {
    case 0:   //  This is the wait-to-start state (both feet on ground)
    
      updateCurrentValues();

      // TO-DO: motor support on --> don't need to update sensor values right? because they're constantly getting updated

      while (state == 0)
      {
        if ((disL1 > threshold || distL2 > threshold) && (avgDistL > currAvgDistL))
        {
          state = 1;
        }
        else if (((disR1 > threshold || distR2 > threshold) && (avgDistR > currAvgDistR))
        {
          state = 3;
        }
        else
        {
          state = 0;
        }
      }
      
      break;
      
    case 1:    // right foot on ground, left foot rising
      
      updateCurrentValues();
      // TO-DO: turn off left motor support, set gap to threshold

      while (state == 1)
      {
        if (avgDistL < avgCurrDistL)
        {
          state = 2;
        }
        else
        {
          state = 1;
        }
      }
      
      break;

    case 2:   //  This is the initialize/enable state
      updateCurrentValues();

      // TO-DO: set left motor gap to 0

      while (state == 2)
      {
        if (disL1 > threshold || distL2 > threshold)
        {
          state = 0;
        }
        else if (avgDistL > avgCurrDistL)
        {
          state = 1;
        }
        else
        {
          state = 2;
        }
      }
      break;

      
    case 3:   //  Left foot on ground, right foot rising
      updateCurrentValues();
      // TO-DO: turn off right motor support, set gap to threshold

      while (state == 3)
      {
        if (avgDistR < avgCurrDistR)
        {
          state = 4;
        }
        else
        {
          state = 3;
        }
      }
      
      break;

    case 4: // Left foot on ground, right foot lowering

    updateCurrentValues();

    while (state == 4)
      {
        if (disR1 > threshold || distR2 > threshold)
        {
          state = 0;
        }
        else if (avgDistR > avgCurrDistR)
        {
          state = 3;
        }
        else
        {
          state = 4;
        }
      }
      break;

    default:    //  Should never be here, but just in case...
        //  move to the wait-to-start state
        state = 0;
        break;
  }
}

void updateCurrentValues()
{
  currKneeEncoderL = kneeEncoderL;
  currKneeEncoderR = kneeEncoderR;
  currPotL = potL;
  currPotR = potR;
  currAnkleEncoderL = ankleEncoderL;
  currAnkleEncoderR = ankleEncoderR;
  currDistR1 = distR1;
  currDistR2 = distR2;
  currDistL1 = distL1;
  currDistL2 = distL2;
  currAvgDistR = avgDistR;
  currAvgDistL = avgDistL;
}
