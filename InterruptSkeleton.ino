/*
 * Functioning Skeleton Interrupt Code
 * 
 * Current code keeps lights an onboard LED every 1 second
 * The interrupt handler is triggered every 25 ms
 * 
 */

/*
 * Install this library from the following link
 * https://github.com/OliviliK/DueTC
 * Sketch -> Add .ZIP Library -> Sketch -> Contributed libraries -> DueTC-master
 * 
 * All documentation and code can be viewed at Githib I linked if you want to explore or use the other functions
 */
#include <DueTC.h> //Library from Github



/*
 * Pin assignment definition
 * Onboard timed flash LED on board
 * This is the little LED with the L next to the programming port on the Due
 */
#define LED_OUT     13          // LED pin number (the on-board LED is hardwired to digital pin 13 on the Due)


/*
 * Global count variable which will be incremented upon entry into the interrupt handler
 * to keep track how many times we have entered the interrupt handler.
 * We can figure out how much real time has passed by seeing how many times this count was incremented
 * Ex: If the interrupt handler is called every 50 ms then when count is equal to 20, we know 
 * that 1 second has passed by. Then we will reset this value so we can continue to keep track of time.
 */
int count = 0;

/*                      
 * Can be removed, just used to turn on and off the LED on the Due board.                    
 * Used to keep track the state of the LED
 * The LED on board is just used to "visually" verify if we are 
 * accurately keeping track of time
 */
volatile bool led = false;          





/* SETUP FUNCTION */

void setup() {

  /*
   * period and tcClock are used in setupTC2_Interrupt() as parameters (the function also has a third parameter which will be discussed)
   * These two variables in combination are being used to create 
   * your target toggling frequency
   * 
   * unsigned int period can only take on the following values: 1-65535
   * byte tcClock can only take on the following values: 0-4
   * 
   * period is self explanatory
   * tcClock is the clock divider 
   * 
   * tcClock has 0-4 values (remember that MCK on the Due is 84 MHz)
   *    0: MCK/2      = 42 MHz
   *    1: MCK/8      = 10.5 MHz
   *    2: MCK/32     = 2.6 Mhz
   *    3: MCK/128    = 656 KHz
   *    4: MCK/3072   = 28 KHz
   * 
   * Here is how you would target toggle frequency: (Don't worry about the third function parameter right now)
   * 
   *      setupTC2_Interrupt(384,1,togglePin6);    // 84,000,000/(8*384)  = 27,343.75 Hz (36.57 us pulse)
   *      setupTC3_Interrupt(96,2,togglePin7);     // 84,000,000/(32*96)  = 27,343.75 Hz (36.57 us pulse)
   *      setupTC4_Interrupt(24,3,togglePin8);     // 84,000,000/(128*24) = 27,343.75 Hz (36.57 us pulse)
   *      setupTC5_Interrupt(1,4,togglePin9);      // 84,000,000/(32*96)  = 27,343.75 Hz (36.57 us pulse)
   *      
   * Basically divide the MCK by the corresponding tcClock and period to get the target toggling frequency 
   * This toggling frequency is how often the interrupt handler gets called
   */
  unsigned int period = 65535; 
  byte tcClock = 2;


  // Configure the digital I/O pins we want to use
  // We are using this for the onboard LED
  pinMode(LED_OUT, OUTPUT);




/* Configuring Interrupts */

  /*
   * Southward had this in his code and I am not sure if this is necessary for
   * our purposes. I kept it in and everything worked so
   */
  noInterrupts();              // first disable all global processor interrupts

  /*
   * This is where we actually set up the interrupt and the interrupt handler
   * I have talked about the first two parameters already (period and tcClock)
   * 
   * The third parameter is the user defined IRQ handler name
   * so whatever you write in the third parameter is the name of the handler
   * For example:
   *    if we have setupTC2_Interrupt(period,tcClock,burrito); 
   *    
   *    then we would have an IRQ handler literally named "burrito"
   *    In this case we would have a handler 
   *    
   *    void burrito(){
   *    }
   *    
   *    which would be triggered at the toggling frequency we set
   *    but for the purposes of clarity I just called the IRQ handler "IRQ_handler"
   * 
   * 
   * After this function is called, we have a functioning interrupt which will be called continuously as the code runs
   * 
   * 
   * This is the first channel for one of the 3 timer blocks in the CPU I think
   * Datasheet for reference just in case: http://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-11057-32-bit-Cortex-M3-Microcontroller-SAM3X-SAM3A_Datasheet.pdf#page=538&zoom=100,37,69
   * 
   */
  setupTC2_Interrupt(period,tcClock,IRQ_handler);


  /*
   * Again, Southward has this in his code and I kept it. I am honestly too scared to touch it
   */
  interrupts();                // now it is ok to enable all global processor interrupts
  
}







/*  ----------ISR----------
 *
 *   This is the Interrupt Service Routine function.  This function will be called at
 *   All of the real-time functional tasks
 *   
 *   This is where we want to do our data collection and processing according to Southward
 *   
 *   Since this handler is triggered every 25 ms, we can stay in this handler MAXIMUM 25 ms
 *   or else the code will stop working. This working time period will change based on what toggling
 *   frequency we choose in that setup interrupt function.
 *
 *   Do NOT change the name of this function! (this name is defined in that function I discussed earlier)
 */

void IRQ_handler() 
{

  /*
   * Increment count upon entry to handler to "track" time
   */
  count++;

  /* Since our code goes into the interrupt handler every 25 ms,
   * Then when count is incremented to 40, 1 second has passed
   * 1 second / 25 ms = 40
   */

  /*
   * Turn on and off the onboard LED every 1 second
   * If your little LED is flashing at 1 second then everything works
   * We checked this code on the oscilliscope and it works for 1 second
   */
  if(count == 40)
  {
    if(led == false)
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











void loop() {
  // put your main code here, to run repeatedly:

  /*
   * PUT STATE MACHINE HERE
   * 
   * ⠀⠀⠀⠀⠀⣤⣤
⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⣿⣿
⠀⠀⣶⠀⠀⣀⣤⣶⣤⣉⣿⣿⣤⣀
⠤⣤⣿⣤⣿⠿⠿⣿⣿⣿⣿⣿⣿⣿⣿⣀
⠀⠛⠿⠀⠀⠀⠀⠉⣿⣿⣿⣿⣿⠉⠛⠿⣿⣤
⠀⠀⠀⠀⠀⠀⠀⠀⠿⣿⣿⣿⠛⠀⠀⠀⣶⠿
⠀⠀⠀⠀⠀⠀⠀⠀⣀⣿⣿⣿⣿⣤⠀⣿⠿
⠀⠀⠀⠀⠀⠀⠀⣶⣿⣿⣿⣿⣿⣿⣿⣿
⠀⠀⠀⠀⠀⠀⠀⠿⣿⣿⣿⣿⣿⠿⠉⠉
⠀⠀⠀⠀⠀⠀⠀⠉⣿⣿⣿⣿⠿
⠀⠀⠀⠀⠀⠀⠀⠀⣿⣿⣿⠉
⠀⠀⠀⠀⠀⠀⠀⠀⣛⣿⣭⣶⣀
⠀⠀⠀⠀⠀⠀⠀⠀⣿⣿⣿⣿⣿
⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⣿⠉⠛⣿
⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⣿⠀⠀⣿⣿
⠀⠀⠀⠀⠀⠀⠀⠀⠀⣿⣉⠀⣶⠿
⠀⠀⠀⠀⠀⠀⠀⠀⣶⣿⠿
⠀⠀⠀⠀⠀⠀⠀⠛⠿⠛
   */

}
