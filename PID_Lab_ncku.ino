
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


//Define Variables we'll be connecting to
double Setpoint, Input, Output;

//Define the aggressive and conservative Tuning Parameters
int SamplingTime =100;  // Execute each 100 msec
double aggKp=10, aggKi=5, aggKd=2;
double Kp=98, Ki=8, Kd=5;
double tol = 2;

const uint8_t encoderPinA = 18, encoderPinB =19;

// Specify the links and initial tuning parameters
PID PID_Controller(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
// Init the DC Motor
DCMotorIG42 Motor(encoderPinA, encoderPinB, 1);

// Estimate PID
double error;
// Prevent Quick Oscillation
double prevOutput=0, oscGap = 100;

// Command Parser
#define BUFSIZE
char inputBuffer[BUFSIZE];
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
  Setpoint = 1.8;

  Serial.println("set PID mode");
  //turn the PID on
  PID_Controller.SetMode(AUTOMATIC);
  PID_Controller.SetOutputLimits(0,400);
  Serial.println("motor driver init done");

  /* The Debug Section */
  /*
  Serial.println("Motor Running Test");
  Motor.RunTest();
  Motor.stop();
*/
  /* DC motor need pre-running to save set-up time */
  Serial.println("motor pre-running");
  Output = 45;
  Motor.setSpeed(Output);
  delay(1500);
  Serial.println("<-- Enter Control Systme -->");
}

void loop()
{
    // Run a period to estimate more precise speed
    Motor.Tick();
    // Run for one Second
    delay(SamplingTime);
    Motor.Tock();

    // Read rotation speed from Encoder
    Input = Motor.getSpeedRPS();
    
    // Error
    error = abs(Setpoint - Input);

    /*
    if (abs(error) < tol) {
        PID_Controller.SetTunings(consKp, consKi, consKd);
    } else {
        PID_Controller.SetTunings(aggKp, aggKi, aggKd);
    }*/

    // PID Compute
    PID_Controller.Compute();
    /*
    if ((Output-prevOutput) >= oscGap) {
      //Output = prevOutput;
    }
    */

    Motor.setSpeed(Output);
    prevOutput = Output;
    // Show Data
    Serial.print("Input: ");
    Serial.println(Input);
    Serial.print("Setpoint: "); 
    Serial.println(Setpoint);    
    Serial.print("Error: ");
    Serial.println(error);
    Serial.print("Output: "); 
    Serial.println(Output);
    Serial.println(error);

    // wait a little while
    delay(5);
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
      if (charBuffer == '\n') {
           inputBuffer[serialIndex] = 0; // terminate the string
           lineFound = (serialIndex > 0); // only good if we sent more than an empty line
           serialIndex=0; // reset for next line of data
         }
         else if(charBuffer == '\r') {
           // Just ignore the Carrage return, were only interested in new line
         }
         else if(serialIndex < BUFSIZE && lineFound == false) {
           /*Place the character in the string buffer:*/
           inputBuffer[serialIndex++] = charBuffer; // auto increment index
         }
  }// End of While
  return lineFound;
}// End of CheckSerial()


