/*
linearStepperCalibration.ino
Written by Akira

This example can be used to calibrate a linear device driven by a stepper motor.
The hardware requirements are :
 - A stepper driver should be connected to the Arduino board (A4988 or similar)
   The main requirement is that the driver send a step only in rising edge of
   the step pin. Arduino step pin should be put in the stepPinX macro.
 - Each side of the linear device, limit switches should be provided. Limit
   switches should be grounded on one side, and the normally close (NC) pin
   should be connected to the Arduino board. Fore an Aft pin should be put in
   the ForeStopperPinX & AftStopperPinX macro.
 - You can decide where is the Fore and where is the Aft position. Keep in mind
   that Aft is always 0 position for the library, fore position could be set
   using void setMaxPosition(const uint16_t maxPosition) method. If the device
   run on the Fore side when executing a gotoPosition(0), simply inverse the two
   wire of the same phase at the driver output (i.e. for the A4988, change the
   connection of the motor, OUT1A <-> OUT1B and OUT2A <-> OUT2B pins )
 - This example run with two motors. If you want to try with only one, just
   comment out the lines containing motorY.
The software process is as following :
 - Open a serial monitor with a communication speed of 115 200 bps
 - Accurately measure the travel distance of the linear device. It is not equal
   to the distance between limit switches, because the device has its own width.
   put the value in mm in the setMaxPosition(const uint16_t maxPosition).
 - Place any value in the bool setRatioStepPerMm(const float stepPerMm) function.
   A value of 1.0f is a good start (too high value could lead to go over langage
   types limits)
 - Put a small speed (in mm/s) in void setSpeed(const uint8_t speed). This speed
   will be wrong at the get-go, as the ratio step / mm is wrong. The motor could
   block while emitting noise with a too great value. If the value is too small,
   the motor will move with a lots of vibrations.
 - The code is written for AVR Arduino board running at full steps, and for SAMX
   Arduino board running at Eighth steps. If you plan to use different divider,
   just change the setRatioStepPerMm initial value.
Once theses few steps done, you can upload the code in your Arduino board and
follow the instruction on the screen (actually it doesn't require any user
input).
The process for motorX will be:
 - The motor will run to the aft position, until it trigger the limit switch
 - After a short while, it will run to the fore position, stopped by the other
   limit switch.
 - As you inform the library of the distance between the aft and the fore
   position, we will be able to compute the right ratio step / mm. It will be
   displayed on the serial monitor. Note that this ratio depends only on the
   mechanical link (pulley diameter or thread pitch), and the microstepping setup
   You can make several tries at different speed to check if there is no missed
   steps. Once done, this ratio could be directly used in your futures program,
   as long as you didn't change mechanical ratio and microstepping setup (for
   microstepping setup, you can recompute ratio on the fly).
 - As we have now a correct ratio, speed is automatically corrected, but position
   is not well known because the setAutoCorrect() method is called after the
   stopper has been triggered. We could do a setPosition(maxPosition), but for
   demonstration purpose, we will go back to the aft and check that speed is
   correct.
 - The motor will go back to the aft (0 position). A timer is set up to measure
   the ellapsed time of the travel. Theoretical & measured time is displayed to
   check if everything is fine.
 - For demonstration purpose, the motor will then shift to the middle position,
   displaying its theoretical and measured position.
 - It will then shift 100mm to the aft.
 - Finally it will go back to middle position, using a small acceleration rate
   (see limitations for acceleration rate.)
MotorY is only for demonstration purpose :
 - It will start to move will a big acceleration rate.
 - If it doesn't reach his limit stopper, it will brake with the same acceleration
   rate, displaying its position every

Limitations:
 - PCINT : the library use pin change interrupt for limit switches.
   As we don't know which pins are used, all the vectors have been redirected
   to the same IRQ function. This could lead to overloaded vectors, which is
   illegal for µC. If you plan to use pin change interrupt in your code or
   another library, please carefully consider on which pin you place your limit
   switches and redirect only the required PCINT vector (in linearStepper.cpp).
   SoftSerial lib use the same way, you may have to tweak it if you want to run
   both library in the same program.
 - No microstepping for now. If your are driving at a fixed micro stepping rate,
   there is no issues. If you need to change the micro stepping rate during the
   execution, you will have to update the ratio Step per mm in your code,
   the library doesn't do it for you.
 - Non constant acceleration rate : the library has been written to optimize
   execution timing. To avoid heavy square root calculation, acceleration
   computation has been simplified, with a non constant acceleration rate.
   The main goal of the acceleration is to avoid missing steps, regardless of
   constant acceleration rate. Note that target position include breaking phase,
   so if you plan a small shift with a long acceleration distance, the motor
   will never reach its programmed speed (i.e. it will accelerate and
   decelerate to reach the target position).
 - To drive many motors with the same board, timing optimization have been done.
   Every variable involved in interrupt routine is optimized to ease computation
   As we use low resolution integer, it is possible that value does not match
   perfecty with what you enter. Take care to always check what is in the library,
   and not what you enter. For example, motorX.setSpeed(201) should be immediately
   checked by uint8_t actualSpeed = motorX.getSpeed(). actualSpeed will be 206.

For more information about this library please visit us at
https://github.com/akira215/linearStepper-Arduino.git

== License ==

Copyright (c) Akira. All right reserved.

This library is free software; you can redistribute it and/or
modify it under the terms of the GNU Lesser General Public
License as published by the Free Software Foundation; either
version 2.1 of the License, or (at your option) any later version.

This library is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public
License along with this library; if not, write to the Free Software
Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA 02110-1301 USA
*/
#include <Arduino.h>
#include <linearStepper.h>

// Hardware configuration
#define ForeStopperPinX  10 // Fore = motor side
#define AftStopperPinX   11 // Aft = pulley side

#define dirPinX           4 // Motor X dir pin
#define stepPinX          5 // Motor X step pin

// Hardware configuration
#define ForeStopperPinY  12 // Fore = motor side
#define AftStopperPinY   3 // Aft = pulley side ! Caution pin 13 is built-in led

#define dirPinY           7 // Motor Y dir pin
#define stepPinY          6 // Motor Y step pin

// Macro to round when casting float to int
#define ROUNDED_INT(f) ((int32_t)(f >= 0.0f ? (f + 0.5f) : (f - 0.5f)))

// linearStepper motorX(stepPinX, dirPinX, ForeStopperPinX, AftStopperPinX);
//linearStepper motorY(stepPinY, dirPinY, ForeStopperPinY, AftStopperPinY);
linearStepper* motorX;
linearStepper* motorY;

void setup()
{
  // Open serial communications and wait for port to open (PC communication)
  Serial.begin(115200);
  while (!Serial); // wait for serial port to connect. Needed for native USB port only


  motorY = new linearStepper(stepPinY, dirPinY, ForeStopperPinY, AftStopperPinY);
motorX = new linearStepper(stepPinX, dirPinX, ForeStopperPinX, AftStopperPinX);

  Serial.println("linearStepper calibration procedure");
  Serial.println("This procedure should be run only if the mechanical ratio");
  Serial.println("step/mm changed. Once done, you can write your own program,");
  Serial.println("using the ratio computed here in linearStepper::setRatioStepPerMm(float).");
  Serial.println("Before running, accurately measure the travel distance of the motor");
  Serial.println("and place the length in mm in motorX.setMaxPosition(Your length).");
  Serial.println();
  delay(2000);



/**********************************************************/
// The three following lines should be adjusted according to
// explanations at the beginning of the file
/**********************************************************/

  motorX->setMaxPosition(385);             // Place the measured distance in mm here
#if defined(__SAM3X8E__)
  motorX->setRatioStepPerMm(45.0f);  // If this ratio is changed, change first Speed set accordingly
#else
  motorX->setRatioStepPerMm(5.83376646f);  // If this ratio is changed, change first Speed set accordingly
#endif // defined(__SAM3X8E__)
  motorX->setSpeed(200);                   // Max speed is 235 mm/s

/**********************************************************/

  motorX->setAccelerationDistance(0);      // No Acceleration
  motorX->setAutoCorrect(false);           // autoCorrect is set to false for calibration purpose


  motorY->setPosition(0);                  // The current position will be considered as 0.
                                          // It will be overide once the aft limit switch will be trigger
  motorY->setMaxPosition(1000);            // Place the measured distance in mm here
  #if defined(__SAM3X8E__)
    motorY->setRatioStepPerMm(45.0f);  // If this ratio is changed, change first Speed set accordingly
  #else
    motorY->setRatioStepPerMm(5.83376646f);  // If this ratio is changed, change first Speed set accordingly
  #endif // defined(__SAM3X8E__)
  motorY->setSpeed(200);                   // Max speed is 235 mm/s
  motorY->setAccelerationDistance(50);     // Acceleration distance is 20 mm

}

void loop()
{
  motorY->gotoPosition(MAX_POS);    // Motor Y will start to move

  motorX->gotoPosition(0);
  while(motorX->isMoving());

  Serial.print("Position 0 mm - step : ");Serial.println(motorX->getCurrentStep());
  uint32_t maxPosition = motorX->getMaxPosition();
  Serial.print("Goto max position  (");Serial.print(maxPosition);Serial.println(" mm)");
  Serial.println();
  delay(750);

  motorX->gotoPosition(MAX_POS);

  while(motorX->isMoving());

  int32_t maxStep = motorX->getCurrentStep();
  uint32_t middlePosition =  ROUNDED_INT((float) maxPosition / 2.0f);
  float ratioStepPerMm = (float) maxStep / (float) maxPosition;

  Serial.print("Position ");Serial.print(maxPosition);
  Serial.print(" mm - step : ");Serial.println(maxStep);
  Serial.print("Ratio will be set to ");Serial.print(ratioStepPerMm,8);Serial.println(" steps / mm");
  Serial.println();

  motorX->setAutoCorrect();    // autoCorrection enable
  motorX->setRatioStepPerMm(ratioStepPerMm);
  delay(750);

  Serial.println("Goto min position (0 mm)"); Serial.println();

  unsigned long timeBefore, timeAfter;

  timeBefore = millis();
  motorX->gotoPosition(0);
  while(motorX->isMoving());
  // The device is correctly initialized here (ratio &  position)
  timeAfter = millis();

  uint8_t speed = motorX->getSpeed();

  Serial.print("Speed is ");Serial.print(speed);Serial.println(" mm/s");
  Serial.print("Theoretical time to run ");Serial.print(maxPosition);
  Serial.print(" mm is ");Serial.print( (float) maxPosition / (float) speed, 3 );Serial.println(" s");
  Serial.print("Ellapsed time is ");Serial.print((float)(timeAfter - timeBefore) / 1000.0f, 3);Serial.println(" s");
  Serial.println();

  delay(750);

  Serial.print("Goto middle position  (");Serial.print(middlePosition);Serial.println(" mm)");
  Serial.println();
  motorX->gotoPosition(middlePosition);

  while(motorX->isMoving());
  Serial.print("Position ");Serial.print(motorX->getPosition());
  Serial.print(" mm - step : ");Serial.println(motorX->getCurrentStep());
  delay(750);

  Serial.println("Shift to the aft - 100mm");
  Serial.println();
  motorX->move(-100);  // Motor should move to 100mm
  while(motorX->isMoving());
  delay(750);

  // To run this, DEBUG 1 should be enable, then uncomment the following lines
  // Take care that timer2 is the only one with a higher priority than UART interrupt.
  // If you try to use another timer, the daly shown will be driven by the UART (wrong timing)
  // motorX.gotoPosition(MAX_POS);
  // while(motorX.isMoving())
  // {
  //  Serial.print("Delta (µs) : ");Serial.println(motorX.getDeltaTime());
  //  delay(30);
  // }


  Serial.print("Return to middle position  (");Serial.print(middlePosition);Serial.println(" mm) with acceleration");
  motorX->setAccelerationDistance(14);      // 14mm Acceleration
  motorX->gotoPosition(middlePosition);

  while(motorX->isMoving());
  Serial.print("Position ");Serial.print(motorX->getPosition());
  Serial.print(" mm - step : ");Serial.println(motorX->getCurrentStep());
  Serial.println();

  Serial.println("Displaying braking of MotorY : ");
  while(motorY->isMoving())
  {
    Serial.print("MotorY Pos : ");Serial.println(motorY->getPosition());
    delay(100);
    motorY->stop(true);    // MotorY will stop using the acceleration set up with motorY.setAccelerationDistance(50);
  }
  Serial.println();


  while(1);
}
