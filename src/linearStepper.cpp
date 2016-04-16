/*
linearStepper.cpp
Written by Akira

This library implement all the required operation to drive stepper motor acting
on linear device, via a dedicated hardware (chipstet A4988 or similar).
The hardware should trigger one step on the rising edge (low to high level) of
the step pin.
In addition, two limit switches should be provided at each end of the device.
Limit switches should be grounded at one side and connected on the normally close
(NC) position to the Arduino board. i.e. when it is pushed, the circuit is open,
when it is realease, it tied down to the ground.
All the operations are interrupt driven in this library, so functions never
block and you don't have to provide any run() function in your main loop.
This has been done to run perfect timing when driving the motor.
Multi instance (i.e. many motors), are supported, it could be increase using
the MAX_INSTANCES macro. You don't have to decrease it, even you drive less
motor than the current value : the implementation of the class is done so that
only the memory is allocated only for the motor you are using in your code.
Take care when running many instance on low speed µC that is could drive to
continuously interrupt the main loop (no remaining time for other operations)
Timing & speed are very important for stepper operations.
Please take a look at the wiki page of the library if you have to tweak timings.

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

#include <linearStepper.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <Arduino.h>

/**************************************************************
 *  Macro (should not be visible for the user)
**************************************************************/
#define FORE            LOW   // If the motor is running the wrong way
#define AFT             HIGH  // consider permutting wire on the same phase

#define MAX_STEP        INT32_MAX
#define MIN_STEP        INT32_MIN
#define TICK_INTERRUPT  104.0f// Interrupt is fired every 104µs
                              // depend of µC freq, prescaler & OCR2A register
                              // Could be checked using DEBUG 1 macro
// Macro to round when casting float to int
#define ROUNDED_INT(f) ((int32_t)(f >= 0.0f ? (f + 0.5f) : (f - 0.5f)))

/**************************************************************
 *  Statics initialization
**************************************************************/
linearStepper* linearStepper::_instance[MAX_INSTANCES] = {0};
uint8_t linearStepper::_nbInstances = 0;

// Interrupt service routine for pin change as we don't know which port is
// to be used we take it for all port. Others libs will throw an error since
// the interrupt will be redefined.
// If required, comment the unused lines to allow other libs to access to the
// commented vector (pins should obviously not be connected to the commented
// register)
#if defined(PCINT0_vect)
ISR(PCINT0_vect)
{  linearStepper::handle_PCINT_isr(); }
#endif

#if defined(PCINT1_vect)
ISR(PCINT1_vect, ISR_ALIASOF(PCINT0_vect));
#endif

#if defined(PCINT2_vect)
ISR(PCINT2_vect, ISR_ALIASOF(PCINT0_vect));
#endif

#if defined(PCINT3_vect)
ISR(PCINT3_vect, ISR_ALIASOF(PCINT0_vect));
#endif

// Interrupt service routine that call the static interrupt handler in the class
ISR(TIMER2_COMPA_vect)
{  linearStepper::handle_Timer_isr(); }

// Static interrupt handler. It only call the private interrupt handler
// of each instance
void linearStepper::handle_PCINT_isr()
{
  for(uint8_t i=0;i<_nbInstances;i++)
    _instance[i]->limitSwitch_isr();
}

void linearStepper::handle_Timer_isr()
{
  for(uint8_t i=0;i<_nbInstances;i++)
    _instance[i]->timer_isr();
}

/**************************************************************
 *  Constructor / Destructor
**************************************************************/
// Only the pins are required, interrputs are enable at the end of it
linearStepper::linearStepper(const uint8_t stepPin, const uint8_t dirPin,
                            const uint8_t foreLSPin, const uint8_t aftLSPin):
_forePin(foreLSPin), _aftPin(aftLSPin), _autoCorrect(true),
_dirPin(dirPin), _counter(1), _speed(100), _currentStep(0),
_currentDir(0), _currentTarget(0), _counterAcceleration(0),
_acceleration(0), _brake(false), _maxStep(100), _stepPerMm(1)
{
  if(_nbInstances == MAX_INSTANCES)
  {
    free(this);
    return; // don't create the instance !
  }

  _instance[_nbInstances] = this;
  _nbInstances++;

  cli();          // disable all interrupts

  // Limit Switch initalization : they will drive pin change interrupts
  // stopper should be grounded, NC connected to Arduino
  // switch is pushed => pin HIGH

  // Enable the PCINT for the entire port here, but never disable it
  // (others might also need it, so we disable the interrupt by using
  // the per-pin PCMSK register).

  // Precalculate the read register and value, so it
  // can be used inside the ISR without costing too much time.
  if (digitalPinToPCICR(foreLSPin))
  {
    pinMode(foreLSPin, INPUT_PULLUP);    // stopper should be grounded, NC connected to Arduino

    *digitalPinToPCICR(foreLSPin) |= _BV(digitalPinToPCICRbit(foreLSPin));    // enable interrupt for the whole port
    *digitalPinToPCMSK(foreLSPin) |=_BV(digitalPinToPCMSKbit(foreLSPin));     // enable interrupt for the correct bit in the port

    _forePortRegister = portInputRegister(digitalPinToPort(foreLSPin));        // setup the port of fore pin
    _foreBitMask = digitalPinToBitMask(foreLSPin);                             // determine the bit for the fore pin in the port
  }
  if (digitalPinToPCICR(aftLSPin))
  {
    pinMode(aftLSPin, INPUT_PULLUP);     // switch is pushed => pin HIGH

    *digitalPinToPCICR(aftLSPin) |= _BV(digitalPinToPCICRbit(aftLSPin)); // enable interrupt for the whole port
    *digitalPinToPCMSK(aftLSPin) |= _BV(digitalPinToPCMSKbit(aftLSPin)); // enable interrupt for the correct bit in the port

    _aftPortRegister = portInputRegister(digitalPinToPort(aftLSPin));    // setup the port of fore pin
    _aftBitMask = digitalPinToBitMask(aftLSPin);                         // determine the bit for the fore pin in the
  }

  // Using Timer2 as it has the best interrupt priority
  TCCR2A = 0;
  TCCR2B = 0;

  OCR2A = 13;
  //TCCR2A |= (1 << WGM21);   // Set to CTC Mode
  TCCR2B |= (1 << WGM22); // Set to CTC Mode (datasheet error !)
  TCCR2B |= (1 << CS22); // set up timer with prescaler = 64
  TIMSK2 |= (1 << OCIE2A); //Set interrupt on compare match channel A


  // Precalculate the write register and value, so it
  // can be used inside the ISR without costing too much time.
  pinMode(stepPin, OUTPUT);    // use Arduino function since no timing required
  _stepBitMask = digitalPinToBitMask(stepPin);
  _stepPortRegister = portOutputRegister(digitalPinToPort(stepPin));
  *_stepPortRegister &= ~ _stepBitMask; // Pin to LOW

  // Setup the dir pin
  pinMode(_dirPin, OUTPUT);    // use Arduino function since no timing required
  digitalWrite(_dirPin, LOW);

  sei(); // enable all interrupts

}

// Destructor
// It remove the instance from the Static array
linearStepper::~linearStepper()
{
  // Remove the interrupts flags, don't disable port interrupt as other may required it
  *digitalPinToPCMSK(_forePin) &= ~_BV(digitalPinToPCMSKbit(_forePin));
  *digitalPinToPCMSK(_aftPin) &= ~_BV(digitalPinToPCMSKbit(_aftPin));

  // remove the pointer from the static array and shift the array
  uint8_t n = 0;
  for(uint8_t i=0;i<_nbInstances;i++)
  {
    if(_instance[i] != this)
      _instance[n++] = _instance[i];
  }
  // finally reduce the array
  _nbInstances--;
}


/**************************************************************
 *  Linear operations
**************************************************************/
// Convert the physical positon to steps for the motor
void linearStepper::setMaxPosition(const uint16_t maxPosition)
{
  uint8_t oldSREG = SREG;
  cli();
  _maxStep = ROUNDED_INT( (float) maxPosition * _stepPerMm );
  SREG = oldSREG;
}

// This method is used to initialize a position of the device. Motor will not move
void linearStepper::setPosition(const uint16_t position)
{
  uint8_t oldSREG = SREG;
  cli();
  _currentStep = ROUNDED_INT( (float) position * _stepPerMm );
  SREG = oldSREG;
}

uint16_t  linearStepper::getMaxPosition() const
{
  uint8_t oldSREG = SREG;
  // create a local variable to avoid interrupt issues
  cli();
  uint16_t maxPosition = ROUNDED_INT( (float) _maxStep / _stepPerMm );
  SREG = oldSREG;
  return maxPosition;
}

uint16_t linearStepper::getPosition() const
{
  uint8_t oldSREG = SREG;
  // create a local variable to avoid interrupt issues
  cli();
  uint16_t position = ROUNDED_INT( (float) _currentStep / _stepPerMm );

  SREG = oldSREG;
  return position;
}
// This function automatically recompute the required values. It return false
// only if the argument is 0.
bool linearStepper::setRatioStepPerMm(const float stepPerMm)
{
  if(stepPerMm)
  {
    uint8_t oldSREG = SREG;
    cli(); // Disable interrupts
    // Backup required values which use _stepPerMm
    uint16_t maxPosition = getMaxPosition();
    uint16_t position = getPosition();
    uint8_t speed = getSpeed();
    uint8_t accel = getAccelerationDistance();

    _stepPerMm = stepPerMm;

    setMaxPosition(maxPosition);    // We need to recompute maxPosition
    setPosition(position);          // We need to recompute position
    setSpeed(speed);                // We need to recompute speed
    setAccelerationDistance(accel); // We need to recompute speed

    SREG = oldSREG;
    return true;
  }

  return false;
}

int32_t linearStepper::getCurrentStep() const
{
  uint8_t oldSREG = SREG;
  // create a local variable to avoid interrupt issues
  cli();
  int32_t currentStep = _currentStep;
  SREG = oldSREG;
  return currentStep;
}

void linearStepper::setSpeed(const uint8_t speed) // speed in mm/s
{
  uint8_t oldSREG = SREG;
  cli();      //Disable interrupts

  if (speed > 0)
  {
    // Compute _speed. +0.5f is added to round the value when casting
    float fSpeed =  (1000000.0f / ( TICK_INTERRUPT *
                          _stepPerMm * (float) speed )) + 0.5f;
    if(fSpeed > (float) UINT8_MAX)
      _speed = UINT8_MAX;
    else if (fSpeed < 1.0f)
      _speed = 1;   // This could never happen as speed max is 255!
    else
      _speed = (uint8_t) fSpeed;

  }
  else
    _speed = UINT8_MAX;

  SREG = oldSREG;
}
// To avoid long root computing, acceleration is simplified as non const
// acceleration. Give the distance (mm) of the acceleration instead.
void linearStepper::setAccelerationDistance(const uint8_t distOfAccel)
{
  uint8_t oldSREG = SREG;
  cli();      //Disable interrupts

  // Compute _speed. +0.5f is added to round the value when casting
  float fAcceleration =  ( distOfAccel * _stepPerMm )  + 0.5f;

  if(fAcceleration > (float) UINT8_MAX - 1)
    _acceleration = UINT8_MAX - 1;    // -1 is required since we compare from 1
  else if (fAcceleration < 1.0f)
    _acceleration = 0;                // disable acceleration
  else
    _acceleration = (uint8_t) fAcceleration + 1;

  SREG = oldSREG;
}

uint8_t linearStepper::getAccelerationDistance() const
{
  uint8_t oldSREG = SREG;
  cli();      //Disable interrupts

  uint8_t acceleration = ROUNDED_INT( (float) _acceleration / _stepPerMm );
  SREG = oldSREG;

  return acceleration;
}


uint8_t linearStepper::getSpeed() const
{
  uint8_t oldSREG = SREG;
  cli();      //Disable interrupts
  uint8_t speed = (uint8_t) ( (1000000.0f / ( TICK_INTERRUPT *
                          _stepPerMm * (float) _speed )) + 0.5f);

  SREG = oldSREG;
  return speed;
}
//The only function to start the motor
void linearStepper::gotoPosition(const uint16_t position)
{
  _currentDir = 0; // stop the motor to avoid going over during computation
  uint8_t oldSREG = SREG;
  cli();

  int32_t targerStep = ROUNDED_INT( (float) position * _stepPerMm );

  if(targerStep >= _maxStep)
    targerStep = MAX_STEP;
  else if (targerStep == 0 ) // could not be negative as position is unsigned
    targerStep = MIN_STEP;

  //_currentTarget = targerStep ; // code without acceleration
  if(_acceleration)
  {
    if(abs(targerStep - _currentStep) <= 2 * (int32_t) _acceleration)
    {
      // We don't have enough space to accelerate and brake
      // we place the target at the middle position so that we will do the job
      _currentTarget =  ROUNDED_INT((float)(targerStep - _currentStep) / 2.0f);
    }
    else
      _currentTarget = targerStep - (_acceleration - 1); // Once the target is reached,
                                                         // the brake will put us where we want
  }
  else
    _currentTarget = targerStep ;

  if (_currentStep > _currentTarget)
  {
    if(*_aftPortRegister & _aftBitMask)
    {
      _currentStep = 0; // we are at the aft position !
      SREG = oldSREG;
      return;
    }
    _counter = 1; // the motor will start immedialely !
    digitalWrite(_dirPin, AFT );
    _counterAcceleration = _acceleration;
    _currentDir = -1;   // this will start the motor to move
  }
  else if (_currentStep < _currentTarget)
  {
    if(*_forePortRegister & _foreBitMask) // we are already at the maxFore position
    {
      if(_autoCorrect)
        _currentStep = _maxStep; // we are at the fore position !
      SREG = oldSREG;
      return;
    }
    _counter = 1; // the motor will start immedialely !
    digitalWrite(_dirPin, FORE );
    _counterAcceleration = _acceleration;
    _currentDir = 1;    // this will start the motor to move
  }
  // if _currentStep = targetStep there is nothing to do
  SREG = oldSREG;
}

void linearStepper::move(const int16_t offset)
{
  _currentDir = 0; // stop the motor to avoid going over during computation
  uint8_t oldSREG = SREG;
  cli();

  int32_t targetPos = getPosition() + offset;
  if (targetPos > MAX_POS)
    targetPos = MAX_POS;
  else if (targetPos < 0)
    targetPos = 0;

  gotoPosition(targetPos);
  SREG = oldSREG;
}
// default value for smooth is false. If the motor is running at a high speed,
// it could lead to missing steps.
void linearStepper::stop(bool smooth)
{
  if((!isMoving())||(_brake))	// We are already stopped or in a breaking phase
    return;
  if(smooth)
  {
    _counterAcceleration = 1 ; // We need to initiate the accel
    _brake = true;             // Testing _acceleration is not required
  }
  else
    _currentDir = 0;
}

/**************************************************************
 *  Class interrupt routine
**************************************************************/
// Private interrupt handler for timer overflow
// The steps are generated inside this function
void linearStepper::timer_isr()
{
  if(_currentDir == 0)  // The motor should not move
    return;

  _counter--;
  if(_counter != 0)
    return;

  // if we are here counter is 0
  _counter = _speed + _counterAcceleration; // speed min 7

  if(_counterAcceleration)
  {
    if(_brake)
    {
      _counterAcceleration++;
      if(_counterAcceleration > _acceleration)
      {
        _brake = false;
        _currentDir = 0;  // Stop the motor (no more step after here)
        return;           // We need to return, if not an uncount step occur
      }
    }
    else
      _counterAcceleration--;
  }


#if DEBUG == 1
  previousTime = currentTime;
  currentTime = micros();
#endif // DEBUG
  // steps occurred only on the rising low-to-high
  *_stepPortRegister &= ~ _stepBitMask; // Pin to LOW
  *_stepPortRegister |= _stepBitMask; // Pin to HIGH

  _currentStep += _currentDir;
  if (_currentStep == _currentTarget)  // we reach our goal
  {
    if(_acceleration)
    {
      _brake = true;          // Set the brake !
      _counterAcceleration++; // We need to initiate the accel
    }
    else
      _currentDir = 0;    // Stop the motor
  }
}

// Private interrupt handler for pin change interrupt
// It deals with limit switch operations
void linearStepper::limitSwitch_isr()
{
  if(*_forePortRegister & _foreBitMask) // don't take care of deboucing
    if(_currentDir == 1)  // Only allow fore move
    {
      _currentDir = 0; // stop the motor
      // _autoCorrect should be set to 0 only for calibration
      if(_autoCorrect)
        _currentStep = _maxStep; // we are at the fore position !
    }

  if(*_aftPortRegister & _aftBitMask)
    if(_currentDir == -1) // Only allow aft move
    {
      _currentDir = 0; // stop the motor
      _currentStep = 0; // we are at the aft position !
    }
}

/**************************************************************
 *  Debug routine (to check interrupt frequency)
**************************************************************/
#if DEBUG == 1
  // static initialization
  volatile unsigned long linearStepper::currentTime = 0;
  volatile unsigned long linearStepper::previousTime = 0;

  unsigned long linearStepper::getDeltaTime()
  {
    uint8_t oldSREG = SREG;
    // disable interrupts while we read timer0_millis or we might get an
    // inconsistent value (e.g. in the middle of a write to timer0_millis)
    cli();
    unsigned long result = currentTime - previousTime;

    SREG = oldSREG;
    return result;
  }
#endif // DEBUG
