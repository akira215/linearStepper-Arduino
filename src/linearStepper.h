/*
linearStepper.h
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

#ifndef linearStepper_h
#define linearStepper_h

#define MAX_INSTANCES   10   // Shall be <=255 as the counter is define as uint8_t
#define DEBUG           0     // Used to debug timing using unsigned long getDeltaTime();

#define MAX_POS         UINT16_MAX

#include <Arduino.h>

// Define a data type to avoid 32bit calculation when using 8bit AVR
#if defined(__SAM3X8E__)
  typedef RwReg RegData;
#else  // AVR code
  typedef   uint8_t  RegData;
#endif // defined(__SAM3X8E__)

class linearStepper
{
private:
  // Class operation (required by accessing members during interrputs)
  static linearStepper* _instance[MAX_INSTANCES];
  static uint8_t _nbInstances;

  // Limit switches operations
  uint8_t _forePin;         // We keep it only for destruction !
  RegData _foreBitMask;
  volatile RegData *_forePortRegister;
  uint8_t _aftPin;
  RegData _aftBitMask;
  volatile RegData *_aftPortRegister;
  bool _autoCorrect;

  // Motor pins
  uint8_t _dirPin;          // As dir pin is not used during move, std Arduino cmd are used
  RegData _stepBitMask;
  volatile RegData *_stepPortRegister;
  RegData _dirBitMask;
  volatile RegData *_dirPortRegister;

  // Moving operation
  volatile uint16_t _counter;      // This couter is used during ISR to count the number of trigger ISR
  volatile uint8_t _speed;         // _counter is initialized using this value (the less it is, the faster the motor run)
  volatile int32_t _currentStep;   // Could be negative if go over limit switch
  volatile int8_t  _currentDir;    // should be -1, 0 or 1
  volatile int32_t _currentTarget; // Steps to be reach

  // Acceletation operations
  volatile uint8_t _counterAcceleration; // used in the ISR to count the number of trigger
  volatile uint8_t _acceleration; // This is the number of steps to slow down
  volatile bool _brake;           // true only during breaking phases

  //Linear operations
  volatile int32_t _maxStep;     // min position is always 0
  float _stepPerMm;              // store the ratio step per millimeter

  // Private Methods
  void setupInterruptOnPinChange(const uint8_t pin);
  void setupTimer();
  #if defined(__SAM3X8E__)
  void startTimer(Tc *tc, uint32_t channel, IRQn_Type irq, uint32_t frequency);
  #endif // defined(__SAM3X8E__)
protected:
  void inline limitSwitch_isr() __attribute__((__always_inline__));
  void inline timer_isr() __attribute__((__always_inline__));

public:
  linearStepper(const uint8_t stepPin, const uint8_t dirPin,
                const uint8_t foreLSPin, const uint8_t aftLSPin);

  ~linearStepper();

  static uint8_t getNumOfInstances() { return _nbInstances; }

  void setAutoCorrect(bool enable = true) { _autoCorrect = enable; }

  void setMaxPosition(const uint16_t maxPosition); // maxPosition in mm, min is always 0
  void setPosition(const uint16_t position);       // If you know the current position of the motor
  bool setRatioStepPerMm(const float stepPerMm);   // step per mm return false only if stepPerMm = 0
  void setSpeed(const uint8_t speed);              // speed in mm/s

  void setAccelerationDistance(const uint8_t distOfAccel);  // Distance of acceleration in mm

  void gotoPosition(const uint16_t position);     // absolute position (mm)
  void move(const int16_t offset); // Offset (mm) is relative to actual position (fore = positive, aft = negative)
  void stop(bool smooth = false);

  bool isMoving()           const  { return (_currentDir !=0); }
  bool isForeLStriggered()  const  { return *_forePortRegister & _foreBitMask; }
  bool isAftLStriggered()   const  { return *_aftPortRegister & _aftBitMask; }

  uint16_t  getPosition() const;
  uint16_t  getMaxPosition() const;
  int32_t   getCurrentStep() const;   // Only for information, it is not required
  uint8_t   getSpeed() const;   // speed in mm/s : actual speed setting, it is NOT the present speed
                                // i.e. : it will return a speed even if the motor is stopped
  uint8_t   getAccelerationDistance() const;
  // Global static ISR routine. Public only for interrupt access handler.
  // Shall not be called directly
  static inline void handle_PCINT_isr() __attribute__((__always_inline__));
  static inline void handle_Timer_isr() __attribute__((__always_inline__));

  // Debugging function. deltaTime is time ellapsed between two steps
  #if DEBUG == 1
    unsigned long getDeltaTime();
    static volatile unsigned long currentTime;
    static volatile unsigned long previousTime;
  #endif // DEBUG
};

#endif // LinearStepper_h
