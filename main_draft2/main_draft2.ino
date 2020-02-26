/*
 * Last modified: Nazir Shamsiev
 * Date last modified: 2/26/20
 * 
 * 
 * TO-DO:
 *  #define IMU (SCL1 & SDA1)
 *  #define MOTOR_CONTROLLER (TX3_14 & RX3_15)
 *  #define MULTIPLEXER (SCL_21 & SDA_20)
 *  define 4 proximity sensors and the load cells
 *  define LED pin for the LED heartbeat
 *  define and instantiate motor??
 *  
 *  declare motor controller, multiplexer (proximity sensors + load cells), IMU
 * 
 */



//  INCLUDED CLASSES
#include "Motor.h"

//  HEADER FILES
#include <DueTC.h>                      //Library from Github (interrupts and timer configuration)


// DEFINES
#define LED_OUT     13                  // LED pin number (the on-board LED is hardwired to digital pin 13 on the Due)

// Pin assignment definitions based on electrical schematic
#define ENC_LK      A0          // Left knee encoder
#define ENC_RK      A1          // Right knee encoder
#define POT_L       A2          // Left spring potentiometer
#define POT_R       A3          // Right spring potentiometer
#define ENC_LA      A4          // Left ankle encoder
#define ENC_RA      A5          // Right ankle encoder


// GLOBAL VARIABLES
/*
 * Values from the sensors and calculations
 */
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

/*
 * Current values to be compared in the state machine
 */
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



/*
 * Global count variable which will be incremented upon entry into the interrupt handler
 * to keep track how many times we have entered the interrupt handler.
 * We can figure out how much real time has passed by seeing how many times this count was incremented
 * Ex: If the interrupt handler is called every 25 ms then when count is equal to 40, we know 
 * that 1 second has passed by. Then we will reset this value so we can continue to keep track of time.
 */
int count = 0;





void setup() {

  /*
   * The interrupt handler is triggered every 25 ms
   * These two variables are used to set up the interrupt
   */
  unsigned int period = 65535; 
  byte tcClock = 2;

  //  Initialize the serial monitor (required if you are printing to the Serial Monitor)
  //  the entered number is called the "baud rate" which means "bits per second"
  Serial.begin(115200);  // can choose {4800, 9600, 19200, 38400, 57600, 115200, 230400, or 250000}



  // Configure the digital I/O pins we want to use
  // We are using this for the onboard LED
  pinMode(LED_OUT, OUTPUT);

  pinMode(ENC_LK, INPUT);
  pinMode(ENC_RK, INPUT);
  pinMode(POT_L, INPUT);
  pinMode(POT_R, INPUT);
  pinMode(ENC_LA, INPUT);
  pinMode(ENC_RA, INPUT);
  
  // TO-DO: configure IMU, motor controller, multiplexer

  /*
   * Interrupt setup
   */
  noInterrupts();              // first disable all global processor interrupts

  setupTC2_Interrupt(period,tcClock,IRQ_handler);

  interrupts();                // now it is ok to enable all global processor interrupts

}




//IRQ HANDLER
void IRQ_handler()
{
  
  count++;

  /*
   * Toggle the LED heartbeat (this will switch the LED on and off every second)
   */
  if (count == 40)
  {
    digitalWrite(LED_OUT, !digitalRead(LED_OUT));  //  toggle the LED
    count = 0;
  }
  
}







void loop() {

  //  Define local variables
  static int state = 0;
  static int FSM_count = 0;

//  switch (state)
//  {
//    case 0:   //  This is the wait-to-start state (both feet on ground)
//    
//      updateCurrentValues();
//
//      // TO-DO: motor support on --> don't need to update sensor values right? because they're constantly getting updated
//
//      while (state == 0)
//      {
//        if ((disL1 > threshold || distL2 > threshold) && (avgDistL > currAvgDistL))
//        {
//          state = 1;
//        }
//        else if (((disR1 > threshold || distR2 > threshold) && (avgDistR > currAvgDistR))
//        {
//          state = 3;
//        }
//        else
//        {
//          state = 0;
//        }
//      }
//      
//      break;
//      
//    case 1:    // right foot on ground, left foot rising
//      
//      updateCurrentValues();
//      // TO-DO: turn off left motor support, set gap to threshold
//
//      while (state == 1)
//      {
//        if (avgDistL < avgCurrDistL)
//        {
//          state = 2;
//        }
//        else
//        {
//          state = 1;
//        }
//      }
//      
//      break;
//
//    case 2:   //  This is the initialize/enable state
//      updateCurrentValues();
//
//      // TO-DO: set left motor gap to 0
//
//      while (state == 2)
//      {
//        if (disL1 > threshold || distL2 > threshold)
//        {
//          state = 0;
//        }
//        else if (avgDistL > avgCurrDistL)
//        {
//          state = 1;
//        }
//        else
//        {
//          state = 2;
//        }
//      }
//      break;
//
//      
//    case 3:   //  Left foot on ground, right foot rising
//      updateCurrentValues();
//      // TO-DO: turn off right motor support, set gap to threshold
//
//      while (state == 3)
//      {
//        if (avgDistR < avgCurrDistR)
//        {
//          state = 4;
//        }
//        else
//        {
//          state = 3;
//        }
//      }
//      
//      break;
//
//    case 4: // Left foot on ground, right foot lowering
//
//    updateCurrentValues();
//
//    while (state == 4)
//      {
//        if (disR1 > threshold || distR2 > threshold)
//        {
//          state = 0;
//        }
//        else if (avgDistR > avgCurrDistR)
//        {
//          state = 3;
//        }
//        else
//        {
//          state = 4;
//        }
//      }
//      break;
//
//    default:    //  Should never be here, but just in case...
//        //  move to the wait-to-start state
//        state = 0;
//        break;
//  }

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
