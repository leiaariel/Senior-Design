/*
 * Last modified: Nazir Shamsiev
 * Date last modified: 3/3/20
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


// HEADER FILES
#include <DueTC.h>                                // Library from Github (interrupts and timer configuration)  URL:https://github.com/OliviliK/DueTC
#include <SparkFun_BNO080_Arduino_Library.h>      // IMU Library                                               URL:https://github.com/sparkfun/SparkFun_BNO080_Arduino_Library
#include <math.h>                                 
#include <Wire.h>                                 // For IMU setup
#include <Adafruit_VL6180X.h>                     // Adafruit VL6180X Time of Flight Micro-LIDAR Distance Sensor Breakout


// DEFINES
#define ENC_LK            A0                      // Left knee encoder
#define ENC_RK            A1                      // Right knee encoder
#define POT_L             A2                      // Left spring potentiometer
#define POT_R             A3                      // Right spring potentiometer
#define ENC_LA            A4                      // Left ankle encoder
#define ENC_RA            A5                      // Right ankle encoder

#define LED_OUT           13                      // LED pin number (the on-board LED is hardwired to digital pin 13 on the Due)

#define TCAADDR 0x70                              // TCA9548A multiplexer I2C address (0x70 by default)       URL: https://learn.adafruit.com/adafruit-tca9548a-1-to-8-i2c-multiplexer-breakout/wiring-and-test
#define DIST_PORT_SEL     1                       // Port selection on multiplexer for distance sensor  (SD_ & SC_)

#define COUNT_IMU         20                      // Count threshold for the IMU for sampling/printing frequency 
#define COUNT_DIST        15                      // Count threshold for the distance sensor sampling/printing frequency
#define COUNT_LED         40                      // Count threshold to change onboard LED toggling frequency

#define I2C_DATA_RATE     400000                  // I2C data rate (Hz)
#define IMU_DATA_UPDATE   25                      // Send data update every __(ms) 

#define NOT_STARTED       false                   // For checking if sensor has started taking data (or is ready)
#define AVAILABLE         true                    // To check if data is available

#define READY             true                    // For checking if sensors are ready to collect and process data
#define NOT_READY         false                   // If sensor is not ready to collect data 

#define BAUD_RATE         9600
          



// GLOBAL VARIABLES

/* Values from the sensors and calculations */
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

/* Current values to be compared in the state machine */
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

/* For checking when sensors should be polled */
volatile bool dist_status = false;
volatile bool IMU_status = false;                 


BNO080 myIMU;                                     // Needed IMU variable
Adafruit_VL6180X distSensor1 = Adafruit_VL6180X();// For distance sensor setup                                URL: https://learn.adafruit.com/adafruit-vl6180x-time-of-flight-micro-lidar-distance-sensor-breakout/wiring-and-test                                   

/*
 * IMPORTANT:
 * 
 * Global count variable which will be incremented upon entry into the interrupt handler
 * to keep track how many times we have entered the interrupt handler.
 * We can figure out how much real time has passed by seeing how many times this count was incremented
 * Ex: If the interrupt handler is called every 25 ms then when count is equal to 40, we know 
 * that 1 second has passed by. Then we will reset this value so we can continue to keep track of time.
 */
int count = 0;






// MAIN SETUP AND CONFIGURATION
void setup() {

  /* For I2C communication */
  Wire.begin();
  
  /* For debugging purposes */
  Serial.begin(BAUD_RATE);

  /*
   * The interrupt handler is triggered every 25 ms
   * These two variables are used to set up the interrupt
   * I don't think we can put these as a define?
   */
  unsigned int period = 65535; 
  byte tcClock = 2;

  // Configure the digital I/O pins we want to use
  pinMode(ENC_LK, INPUT);
  pinMode(ENC_RK, INPUT);
  pinMode(POT_L, INPUT);
  pinMode(POT_R, INPUT);
  pinMode(ENC_LA, INPUT);
  pinMode(ENC_RA, INPUT);
  
  /* We are using this for the onboard LED */
  pinMode(LED_OUT, OUTPUT);


  /*
   * Only one pair of the I2C ports work 
   * ports 20 and 21
   */




  // INTERRUPT SETUP

  /* First disable all global processor interrupts */
  noInterrupts();              

  setupTC2_Interrupt(period,tcClock,IRQ_handler);

  /* Now it is ok to enable all global processor interrupts */
  interrupts(); 


    /* Configure IMU */
  setupIMU();

  /* Configure distance sensor */
  setupDistance();



}





// IRQ HANDLER

void IRQ_handler()
{
  /* Increment global counter everytime you enter IRQ handler (to keep track of time) */
  count++;

  /* Toggle the LED heartbeat (this will switch the LED on and off every second) */
  if (count == COUNT_LED)
  {
    /* Toggle the LED */
    digitalWrite(LED_OUT, !digitalRead(LED_OUT));  
    count = 0;
  }


  /*
   * If counts reach appropriate threshold for 
   * each respective sensor, then we can declare them ready
   * to be polled. They will be polled and the data 
   * will be processed in the main loop
   */
  if(count == COUNT_IMU)
  {
    IMU_status = READY;
  }


  if(count == COUNT_DIST)
  {
    dist_status = READY;
  }

//  Serial.println("Count: ");
//  Serial.println(count);

}






// STATE MACHINE

void loop() {

  /*
   * Define local variables
   * state to keep track of states in state machine
   * FSM_count ???
   */
  static int state = 0;
  static int FSM_count = 0;

  /*
   * Get sensor data and process data
   */
  if(myIMU.dataAvailable() == AVAILABLE && IMU_status == READY )
  {
    readIMU();
    IMU_status = NOT_READY;
  }

  if(dist_status == READY)
  {
    readDistance();
    dist_status = NOT_READY;
  }



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





/*
 * Configures distance sensor (Adafruit VL6180x)
 */
void setupDistance()
{
    /* Wait for serial port to open on native usb devices */
    while (!Serial) 
    {
      delay(1);
    }
  
    Serial.println("Adafruit VL6180x test!");

    /* Select SD_ and SC_ ports on the multiplexer for the connected distance sensor */
    tcaselect(DIST_PORT_SEL);                                 

    /* If sensor hasn't started up, continue checking */
    if (distSensor1.begin() == NOT_STARTED) 
    {
      Serial.println("Failed to find sensor");
      while (distSensor1.begin() == NOT_STARTED);
    }

    Serial.println("Sensor found!");
}





/*
 * Read data from the distance sensor (Adafruit VL6180x)
 * Data is currently read everytime we go into the IRQ_handler
 * but we print at another rate based on a count threshold
 */
void readDistance()
{
  /*
   * Light reading from sensor
   * You can use different Gain settings to get a different range. 
   * For better results at low light,  use higher gain. 
   * For better results at high light, use a lower gain.
   * 
   * VL6180X_ALS_GAIN_1   - gain of 1x
   * VL6180X_ALS_GAIN_1_25    - gain of 1.25x
   * VL6180X_ALS_GAIN_1_67    - gain of 1.67x
   * VL6180X_ALS_GAIN_2_5    - gain of 2.5x
   * VL6180X_ALS_GAIN_5    - gain of 5x
   * VL6180X_ALS_GAIN_10    - gain of 10x
   * VL6180X_ALS_GAIN_20    - gain of 20x
   * VL6180X_ALS_GAIN_40    - gain of 40x
   */
  float lux = distSensor1.readLux(VL6180X_ALS_GAIN_5);

  /*
   * Range in millimeters. 
   * If you get 0 or a value over 200 there's likely an error.
   * Read the values first before entering if statement so there's
   * a value available to print
   */
  uint8_t range = distSensor1.readRange();
  uint8_t status = distSensor1.readRangeStatus();


  Serial.print("Lux: "); 
  Serial.println(lux);
    

  if (status == VL6180X_ERROR_NONE) 
  {
    Serial.print("Range: "); 
    Serial.println(range);
  }


  /* Error Cases */
  
  if  ((status >= VL6180X_ERROR_SYSERR_1) && (status <= VL6180X_ERROR_SYSERR_5)) 
  {
    Serial.println("System error");
  }
  else if (status == VL6180X_ERROR_ECEFAIL) 
  {
    Serial.println("ECE failure");
  }
  else if (status == VL6180X_ERROR_NOCONVERGE) 
  {
    Serial.println("No convergence");
  }
  else if (status == VL6180X_ERROR_RANGEIGNORE) 
  {
    Serial.println("Ignoring range");
  }
  else if (status == VL6180X_ERROR_SNR) 
  {
    Serial.println("Signal/Noise error");
  }
  else if (status == VL6180X_ERROR_RAWUFLOW) 
  {
    Serial.println("Raw reading underflow");
  }
  else if (status == VL6180X_ERROR_RAWOFLOW) 
  {
    Serial.println("Raw reading overflow");
  }
  else if (status == VL6180X_ERROR_RANGEUFLOW) 
  {
    Serial.println("Range reading underflow");
  }
  else if (status == VL6180X_ERROR_RANGEOFLOW) 
  {
    Serial.println("Range reading overflow");
  }
  
}



/*
 * To setup the multiplexer
 * helps to select the port on the multiplexer
 */
void tcaselect(uint8_t i) 
{
  /* Ports range from 0-7 */
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}


/*
 * Configures IMU
 */
void setupIMU()
{
  Serial.println("BNO080 Read Configuration");
  Serial.println();

  if (myIMU.begin() == NOT_STARTED)
  {
    Serial.println("BNO080 not detected at default I2C address. Check your jumpers and the hookup guide.");
    while (myIMU.begin() == NOT_STARTED);
  }


  /* Increase I2C data rate */
  Wire.setClock(I2C_DATA_RATE);

  /* Send data update every __ms */
  myIMU.enableRotationVector(IMU_DATA_UPDATE);
}




/* IMU data collection and processing  */
void readIMU()
{
  /*
   * Checks data from IMU every __ seconds based on count threshold (COUNT_IMU)
   * Can play around with this to sample more often
   * NOTES: if myIMU.enableRotationVector() is set to a low data update rate, the IMU will not be able 
   * to pause and the IMU will stop sampling for moments. Current setup has been working for constant sampling. (the mentioned function is in the setup() section)
   * If you adjust the count threshold you will need to adjust myIMU.enableRotationVector() rate so there are no pauses
   */

  float quatI = myIMU.getQuatI();
  float quatJ = myIMU.getQuatJ();
  float quatK = myIMU.getQuatK();
  float quatReal = myIMU.getQuatReal();

  /*
   * Method to convert quaternion to Euler angles -> URL:http://www.chrobotics.com/library/understanding-quaternions
   * a(quatReal) b(quatI) c(quatJ) d(quatK) for equation from link
   * Need to use atan2 instead of atan
   * These angles show the orientation of the IMU
   */
  float yaw = atan2((2*(quatReal*quatI + quatJ*quatK)),((pow(quatReal,2) - pow(quatI,2) - pow(quatJ,2) + pow(quatK,2)))) * (180/PI);
  float pitch = -asin(2*(quatI*quatK - quatReal*quatJ)) * (180/PI);
  float roll = atan2((2*(quatReal*quatK + quatI*quatJ)),((pow(quatReal,2) + pow(quatI,2) - pow(quatJ,2) - pow(quatK,2)))) * (180/PI);



  Serial.println(F("Output in form yaw, pitch, roll"));
  Serial.print(yaw, 2);
  Serial.print(F(","));
  Serial.print(pitch, 2);
  Serial.print(F(","));
  Serial.print(roll, 2);

  Serial.println();
  
}
