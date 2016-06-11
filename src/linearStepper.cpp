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
#if defined(__SAM3X8E__)
	#define TICK_INTERRUPT  25.0f	// Interrupt is fired every 25µs
                              	// depend of µC freq,
#else
	#define TICK_INTERRUPT  104.0f// Interrupt is fired every 104µs
                              	// depend of µC freq, prescaler & OCR2A register
                              	// Could be checked using DEBUG 1 macro
#endif // defined(__SAM3X8E__)

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
#if defined(__SAM3X8E__)

void PIOA_Handler(void) {
	uint32_t isr __attribute__((unused));
  isr= PIOA->PIO_ISR;
  linearStepper::handle_PCINT_isr();
}

void PIOB_Handler(void) {
  uint32_t isr __attribute__((unused));
	isr = PIOB->PIO_ISR;
	linearStepper::handle_PCINT_isr();
}

void PIOC_Handler(void) {
  uint32_t isr __attribute__((unused));
	isr = PIOC->PIO_ISR;
	linearStepper::handle_PCINT_isr();
}

void PIOD_Handler(void) {
  uint32_t isr __attribute__((unused));
	isr = PIOD->PIO_ISR;
  linearStepper::handle_PCINT_isr();
}
#endif // defined(__SAM3X8E__)

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
#if defined(__SAM3X8E__)
	void TC3_Handler(){
	  TC_GetStatus(TC1, 0);
	  linearStepper::handle_Timer_isr();
	}
#else
	ISR(TIMER2_COMPA_vect)
	{  linearStepper::handle_Timer_isr(); }
#endif // defined(__SAM3X8E__)

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

  noInterrupts();          // disable all interrupts

  // Limit Switch initalization : they will drive pin change interrupts
  // stopper should be grounded, NC connected to Arduino
  // switch is pushed => pin HIGH

  // Enable the PCINT for the entire port here, but never disable it
  // (others might also need it, so we disable the interrupt by using
  // the per-pin PCMSK register).

  // Precalculate the read register and value, so it
  // can be used inside the ISR without costing too much time.
	setupInterruptOnPinChange(foreLSPin);
  _forePortRegister = portInputRegister(digitalPinToPort(foreLSPin));        // setup the port of fore pin
  _foreBitMask = digitalPinToBitMask(foreLSPin);                             // determine the bit for the fore pin in the port

  setupInterruptOnPinChange(aftLSPin);
  _aftPortRegister = portInputRegister(digitalPinToPort(aftLSPin));    // setup the port of fore pin
  _aftBitMask = digitalPinToBitMask(aftLSPin);                         // determine the bit for the fore pin in the

	setupTimer();

  // Precalculate the write register and value, so it
  // can be used inside the ISR without costing too much time.
  pinMode(stepPin, OUTPUT);    // use Arduino function since no timing required
  _stepBitMask = digitalPinToBitMask(stepPin);
  _stepPortRegister = portOutputRegister(digitalPinToPort(stepPin));
  *_stepPortRegister &= ~ _stepBitMask; // Pin to LOW

  // Setup the dir pin
  pinMode(_dirPin, OUTPUT);    // use Arduino function since no timing required
  digitalWrite(_dirPin, LOW);

  interrupts(); // enable all interrupts

}

// Destructor
// It remove the instance from the Static array
linearStepper::~linearStepper()
{
  // Remove the interrupts flags, don't disable port interrupt as other may required it
#if defined(__SAM3X8E__)
	g_APinDescription[_forePin].pPort->PIO_IDR = g_APinDescription[_forePin].ulPin;
	g_APinDescription[_aftPin].pPort->PIO_IDR = g_APinDescription[_forePin].ulPin;
#else // AVR code
  *digitalPinToPCMSK(_forePin) &= ~_BV(digitalPinToPCMSKbit(_forePin));
  *digitalPinToPCMSK(_aftPin) &= ~_BV(digitalPinToPCMSKbit(_aftPin));
#endif // defined(__SAM3X8E__)
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
 *  Private setup methods
**************************************************************/

void linearStepper::setupInterruptOnPinChange(const uint8_t pin)
{
	  pinMode(pin, INPUT_PULLUP);    // stopper should be grounded, NC connected to Arduino

#if defined(__SAM3X8E__)
  //Enable or disable write protect of PMC registers.
  pmc_set_writeprotect(false);

  Pio *pio = g_APinDescription[pin].pPort;
	uint32_t mask = g_APinDescription[pin].ulPin;
  IRQn irq;

  // //Enable the specified peripheral clock & configure irq
  if (pio == PIOA)
  {
    pmc_enable_periph_clk(ID_PIOA);
    irq = PIOA_IRQn;
  }
  if (pio == PIOB)
  {
    pmc_enable_periph_clk(ID_PIOB);
    irq = PIOB_IRQn;
  }
  if (pio == PIOC)
  {
    pmc_enable_periph_clk(ID_PIOC);
    irq = PIOC_IRQn;
  }
  if (pio == PIOD)
  {
    pmc_enable_periph_clk(ID_PIOD);
    irq = PIOD_IRQn;
  }
  // Configure the interrupt mode
  // Disable additional interrupt mode (detects both rising and falling edges)
  // mode = change
  pio->PIO_AIMDR = mask;
  // Enable interrupt
	pio->PIO_IER = mask;

  NVIC_SetPriority(irq, (uint32_t) irq);  // Priority is defined by irq to avoid complex tests
  NVIC_EnableIRQ(irq);
#else // AVR Code
	*digitalPinToPCICR(pin) |= _BV(digitalPinToPCICRbit(pin));    // enable interrupt for the whole port
	*digitalPinToPCMSK(pin) |=_BV(digitalPinToPCMSKbit(pin));     // enable interrupt for the correct bit in the port
#endif // defined(__SAM3X8E__)
}

void linearStepper::setupTimer()
{
#if defined(__SAM3X8E__)
		// For ARM porcessor, send an interrupt every 25µs (40kHz)
		int FREQ_Hz = 40000;
		startTimer(TC1, 0, TC3_IRQn, FREQ_Hz);
#else // AVR code
		// For AVR porcessor, send an interrupt every 104µs (10kHz)
		// Using Timer2 as it has the best interrupt priority
		TCCR2A = 0;
		TCCR2B = 0;

		OCR2A = 13;
		//TCCR2A |= (1 << WGM21);   // Set to CTC Mode
		TCCR2B |= (1 << WGM22); // Set to CTC Mode (datasheet error !)
		TCCR2B |= (1 << CS22); // set up timer with prescaler = 64
		TIMSK2 |= (1 << OCIE2A); //Set interrupt on compare match channel A
#endif // defined(__SAM3X8E__)
}

#if defined(__SAM3X8E__)
void linearStepper::startTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency)
{
    //Enable or disable write protect of PMC registers.
    pmc_set_writeprotect(false);
    //Enable the specified peripheral clock.
    pmc_enable_periph_clk((uint32_t)irq);

    //TC_Configure(tc, channel, TC_CMR_WAVE|TC_CMR_WAVSEL_UP_RC|TC_CMR_TCCLKS_TIMER_CLOCK2);
    tc->TC_CHANNEL[channel].TC_CMR = TC_CMR_WAVE  | TC_CMR_WAVSEL_UP_RC | TC_CMR_TCCLKS_TIMER_CLOCK2;

    // Setup values in counting registers
    uint32_t rc = VARIANT_MCK/8/frequency;
    tc->TC_CHANNEL[channel].TC_RA = rc/2; //TC_SetRA(tc, channel, rc/2);
    tc->TC_CHANNEL[channel].TC_RC = rc; //TC_SetRC(tc, channel, rc);

    // Software trigger & Start timer
    tc->TC_CHANNEL[channel].TC_CCR = TC_CCR_CLKEN  | TC_CCR_SWTRG ;   //TC_Start(tc, channel);

    // Position the interrupt we need
    tc->TC_CHANNEL[channel].TC_IER = TC_IER_CPCS;  // Enable RC compare interrupt
    tc->TC_CHANNEL[channel].TC_IDR = ~TC_IER_CPCS; // disable other interrupts

    // Finally start the interrupt
    NVIC_SetPriority(irq, 32);
    NVIC_EnableIRQ(irq);
}
#endif // defined(__SAM3X8E__)
/**************************************************************
 *  Linear operations
**************************************************************/
// Convert the physical positon to steps for the motor
void linearStepper::setMaxPosition(const uint16_t maxPosition)
{
#ifndef __SAM3X8E__
  uint8_t oldSREG = SREG;
	cli();
#endif

  _maxStep = ROUNDED_INT( (float) maxPosition * _stepPerMm );
#ifndef __SAM3X8E__
  SREG = oldSREG;
#endif
}

// This method is used to initialize a position of the device. Motor will not move
void linearStepper::setPosition(const uint16_t position)
{
#ifndef __SAM3X8E__
	uint8_t oldSREG = SREG;
	cli();
#endif

  _currentStep = ROUNDED_INT( (float) position * _stepPerMm );
#ifndef __SAM3X8E__
  SREG = oldSREG;
#endif
}

uint16_t  linearStepper::getMaxPosition() const
{
#ifndef __SAM3X8E__
  uint8_t oldSREG = SREG;
	cli();
#endif

  // create a local variable to avoid interrupt issues
  uint16_t maxPosition = ROUNDED_INT( (float) _maxStep / _stepPerMm );
#ifndef __SAM3X8E__
  SREG = oldSREG;
#endif
  return maxPosition;
}

uint16_t linearStepper::getPosition() const
{
#ifndef __SAM3X8E__
  uint8_t oldSREG = SREG;
	cli();
#endif
  // create a local variable to avoid interrupt issues
  uint16_t position = ROUNDED_INT( (float) _currentStep / _stepPerMm );

#ifndef __SAM3X8E__
	SREG = oldSREG;
#endif
  return position;
}
// This function automatically recompute the required values. It return false
// only if the argument is 0.
bool linearStepper::setRatioStepPerMm(const float stepPerMm)
{
  if(stepPerMm)
  {
	#ifndef __SAM3X8E__
		uint8_t oldSREG = SREG;
		cli();
	#endif

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

	#ifndef __SAM3X8E__
	  SREG = oldSREG;
	#endif
    return true;
  }

  return false;
}

int32_t linearStepper::getCurrentStep() const
{
#ifndef __SAM3X8E__
	uint8_t oldSREG = SREG;
	cli();
#endif
  // create a local variable to avoid interrupt issues
  int32_t currentStep = _currentStep;
#ifndef __SAM3X8E__
	SREG = oldSREG;
#endif
  return currentStep;
}

void linearStepper::setSpeed(const uint8_t speed) // speed in mm/s
{
#ifndef __SAM3X8E__
	uint8_t oldSREG = SREG;
	cli();
#endif

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

#ifndef __SAM3X8E__
	SREG = oldSREG;
#endif
}
// To avoid long root computing, acceleration is simplified as non const
// acceleration. Give the distance (mm) of the acceleration instead.
void linearStepper::setAccelerationDistance(const uint8_t distOfAccel)
{
#ifndef __SAM3X8E__
	uint8_t oldSREG = SREG;
	cli();      //Disable interrupts
#endif

  // Compute _speed. +0.5f is added to round the value when casting
  float fAcceleration =  ( distOfAccel * _stepPerMm )  + 0.5f;

  if(fAcceleration > (float) UINT8_MAX - 1)
    _acceleration = UINT8_MAX - 1;    // -1 is required since we compare from 1
  else if (fAcceleration < 1.0f)
    _acceleration = 0;                // disable acceleration
  else
    _acceleration = (uint8_t) fAcceleration + 1;

#ifndef __SAM3X8E__
	SREG = oldSREG;
#endif
}

uint8_t linearStepper::getAccelerationDistance() const
{
#ifndef __SAM3X8E__
	uint8_t oldSREG = SREG;
	cli();      //Disable interrupts
#endif


  uint8_t acceleration = ROUNDED_INT( (float) _acceleration / _stepPerMm );
#ifndef __SAM3X8E__
	SREG = oldSREG;
#endif

  return acceleration;
}


uint8_t linearStepper::getSpeed() const
{
#ifndef __SAM3X8E__
	uint8_t oldSREG = SREG;
	cli();
#endif

  uint8_t speed = (uint8_t) ( (1000000.0f / ( TICK_INTERRUPT *
                          _stepPerMm * (float) _speed )) + 0.5f);

#ifndef __SAM3X8E__
  SREG = oldSREG;
#endif
  return speed;
}
//The only function to start the motor
void linearStepper::gotoPosition(const uint16_t position)
{
  _currentDir = 0; // stop the motor to avoid going over during computation
#ifndef __SAM3X8E__
  uint8_t oldSREG = SREG;
	cli();
#endif

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
    {
      if(targerStep == MIN_STEP)
        _currentTarget = MIN_STEP;
      else
        _currentTarget = targerStep - (_acceleration - 1); // Once the target is reached,
                                                           // the brake will put us where we want
    }
  }
  else
    _currentTarget = targerStep ;

  if (_currentStep > _currentTarget)
  {
    if(*_aftPortRegister & _aftBitMask)
    {
      _currentStep = 0; // we are at the aft position !
		#ifndef __SAM3X8E__
		  SREG = oldSREG;
		#endif
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
			#ifndef __SAM3X8E__
			  SREG = oldSREG;
			#endif
      return;
    }
    _counter = 1; // the motor will start immedialely !
    digitalWrite(_dirPin, FORE );
    _counterAcceleration = _acceleration;
    _currentDir = 1;    // this will start the motor to move
  }
  // if _currentStep = targetStep there is nothing to do
#ifndef __SAM3X8E__
  SREG = oldSREG;
#endif
}

void linearStepper::move(const int16_t offset)
{
  _currentDir = 0; // stop the motor to avoid going over during computation
#ifndef __SAM3X8E__
  uint8_t oldSREG = SREG;
	cli();
#endif

  int32_t targetPos = getPosition() + offset;
  if (targetPos > MAX_POS)
    targetPos = MAX_POS;
  else if (targetPos < 0)
    targetPos = 0;

  gotoPosition(targetPos);
#ifndef __SAM3X8E__
  SREG = oldSREG;
#endif
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
		#ifndef (__SAM3X8E__)
		  uint8_t oldSREG = SREG;
			cli();
		#endif
    // disable interrupts while we read timer0_millis or we might get an
    // inconsistent value (e.g. in the middle of a write to timer0_millis)
    unsigned long result = currentTime - previousTime;
		#ifndef (__SAM3X8E__)
    	SREG = oldSREG;
		#endif
    return result;
  }
#endif // DEBUG
