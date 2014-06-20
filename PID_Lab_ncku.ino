
/********************************************************
 * PID Adaptive Tuning Example
 * One of the benefits of the PID library is that you can
 * change the tuning parameters at any time.  this can be
 * helpful if we want the controller to be agressive at some
 * times, and conservative at others.   in the example below
 * we set the controller to use Conservative Tuning Parameters
 * when we're near setpoint and more agressive Tuning
 * Parameters when we're farther away.
 ********************************************************/

#include <DCMotorIG42.h>
#include <PID_v1.h>


// Controlling Target
double Setpoint, Input, Output;

// PID Paremeters and Performance Index
int SamplingTime =50;  // Execute each 100 msec
double Kp=100, Ki=20, Kd=5;
// If the error within the band, the current time is Settling time
double errorBand = 0.02; // 2%
/*
 * -- Definition --
 * Rise Time: time duration from 10% to 90% of final value
 * Settling Time: time to approach the final value (within the error band)
 * Time Constant: when output value approaches 63.2% of desired value
 */
double RiseTime, SettlingTime;
// Estimate PID
double error;
// Prevent Quick Oscillation
double prevOutput=0, oscGap = 100;

// Pins of Encoder
const uint8_t encoderPinA = 18, encoderPinB =19;

// Specify the links and initial tuning parameters
PID PID_Controller(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
// Init the DC Motor
DCMotorIG42 Motor(encoderPinA, encoderPinB, 1);

// Command Parser
#define BUFSIZE 32
char commandBuffer[BUFSIZE];
boolean CheckSerial();
int serialIndex;

void setup()
{
  // initialize seiral communication
  Serial.begin(9600);
  Serial.println("system initializing...");
  //initialize the variables we're linked to
  Input = 0;
  Output = 0;
  Setpoint = 1.5;

  Serial.println("set PID mode");
  //turn the PID on
  PID_Controller.SetMode(AUTOMATIC);
  PID_Controller.SetOutputLimits(0,400);
  PID_Controller.SetSampleTime(SamplingTime);
  Serial.println("motor driver init done");

  /* The Debug Section */
  /*
  Serial.println("Motor Running Test");
  Motor.RunTest();
  Motor.stop();
*/
}

void loop()
{
    boolean settingDone = false;
    /* Head Section: Commnand Parse and Parameters Setting */
    Serial.print("PID parameters: Kp = "); Serial.print(Kp);
    Serial.print(", Ki = "); Serial.print(Ki); 
    Serial.print(", Kd = "); Serial.print(Kd);
    Serial.println(")");
    Serial.println("Please Enter which you modify"); 
    
    while (!settingDone) {

        if(CheckSerial()) {
            if (strcmp(commandBuffer, "kp") == 0) {
                Serial.print("Kp = ");
                double input = Serial.read();
                PID_Controller.SetTunings(input, Ki, Kd);
            } 
        }
    }

    /* Second part: Start Running */
    unsigned long LoopRunStart =0, LoopRunEnd=0, LoopRunDuration=0;
    LoopRunStart = millis(); // Loop Tick

    // Test: counter
    unsigned long loop_counter = 0;

  /* DC motor need pre-running to save set-up time */
  Serial.println("motor pre-running");
  Output = 20;
  Motor.setSpeed(Output);
  delay(100);
  Serial.println("<-- Enter Control Systme -->");

    /* loop stop when control success or spend too much time */
    while ( error <= errorBand || LoopRunDuration <= 50000) {   
	    // Run a period to estimate more precise speed
	    Motor.Tick();
	    // Run for one Second
	    delay(SamplingTime);
	    Motor.Tock();

	    // Read rotation speed from Encoder
	    Input = Motor.getSpeedRPS();
	    
	    // Error
	    error = abs(Setpoint - Input) / Setpoint; // in percentage
        if (error <= errorBand) {
            SettlingTime = LoopRunDuration; 
            break;
        }

	    // PID Compute
	    PID_Controller.Compute();

	    Motor.setSpeed(Output);
	    prevOutput = Output;

        // Show error values
	    Serial.println(error);

	    // wait a little while
	    delay(5);
        LoopRunEnd = millis(); // current time
        LoopRunDuration = LoopRunEnd - LoopRunStart;
        loop_counter++;
    } // End of while(duration) 
    Motor.stop();
    Serial.println("You complete the velocity controlling!");
    // Maybe Do some analysis
    // Maybe show the Total Result and Estimation
    Serial.print("  *This Run execute "); Serial.print(loop_counter); Serial.println(" times");
    Serial.print("  *The Settling Time is "); Serial.print(SettlingTime/1000.0); Serial.println(" sec");
    delay(5000);
}

/*
Checks the serial input for a string, returns true once a '\n' is seen
users can always look at the global variable "serialIndex" to see if characters have been received already
*/
boolean CheckSerial()
{
  boolean lineFound = false;
  // if there's any serial available, read it:
  while (Serial.available() > 0) {
    //Read a character as it comes in:
    //currently this will throw away anything after the buffer is full or the \n is detected
    char charBuffer = Serial.read(); 
    Serial.print(charBuffer);
      if (charBuffer == '\n') {
           commandBuffer[serialIndex] = 0; // terminate the string
           lineFound = (serialIndex > 0); // only good if we sent more than an empty line
           serialIndex=0; // reset for next line of data
         }
         else if(charBuffer == '\r') {
           // Just ignore the Carrage return, were only interested in new line
         }
         else if(serialIndex < BUFSIZE && lineFound == false) {
           /*Place the character in the string buffer:*/
           commandBuffer[serialIndex++] = charBuffer; // auto increment index
         }
  }// End of While
  return lineFound;
}// End of CheckSerial()


