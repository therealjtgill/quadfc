#ifndef REMOTESTARTUP
#define REMOTESTARTUP

typedef enum OperationMode {UNINITIALIZED, FLIGHT, NOFLIGHT} OperationMode;

// rxPulses is an array of four integers representing received pulse widths.
// Changes the mode of operation based on input from the remote control
// receiver.
OperationMode checkOperationMode(
   const int * rxPulses,
   const OperationMode & currentMode,
   const double & timeSinceModeChange_ms
)
{
   if (timeSinceModeChange_ms < 750)
   {
      return currentMode;
   }
   if (currentMode == FLIGHT || currentMode == NOFLIGHT)
   {
      if (
            rxPulses[0] <= 1200 
         && rxPulses[1] <= 1200
         && rxPulses[2] <= 1200
         && rxPulses[3] <= 1200
      )
      {
         if (currentMode == FLIGHT)
         {
            return NOFLIGHT;
         }
         else
         {
            return FLIGHT;
         }
      }
   }
   else
   {
      if (
            rxPulses[0] >= 1200 
         || rxPulses[1] >= 1200
         || rxPulses[2] >= 1200
         || rxPulses[3] >= 1200
      )
      {
         return NOFLIGHT;
      }
   }

   return currentMode;
}

typedef enum TuningMode {MOTOR1=0, MOTOR2=1, MOTOR3=2, MOTOR4=3, BLANK=4, TUNINGOFF=5, ALLMOTORS=6} TuningMode;

// rxPulses is an array of four integers representing received pulse widths.
// rxPulses[0] --> right joystick, right/left
// rxPulses[1] --> right joystick, up/down
// rxPulses[2] --> left joystick,  up/down
// rxPulses[3] --> left joystick,  right/left
// Always want the left joystick to be in the bottom left position.
// Add one with left joystick being in bottom right corner to test buzz for all
// motors.
TuningMode checkTuningMode(
   const int * rxPulses,
   const TuningMode & currentMode,
   const double & timeSinceModeChange_ms
)
{
   if (timeSinceModeChange_ms < 750)
   {
      return currentMode;
   }

   if (
         rxPulses[0] <= 1200 
      && rxPulses[1] <= 1200
      && rxPulses[2] <= 1200
      && rxPulses[3] >= 1700
   )
   {
      return MOTOR1;
   }
   else if (
         rxPulses[0] <= 1200 
      && rxPulses[1] >= 1700
      && rxPulses[2] <= 1200
      && rxPulses[3] >= 1700
   )
   {
      return MOTOR2;
   }
   else if (
         rxPulses[0] >= 1700 
      && rxPulses[1] >= 1700
      && rxPulses[2] <= 1200
      && rxPulses[3] >= 1700
   )
   {
      return MOTOR3;
   }
   else if (
         rxPulses[0] >= 1700 
      && rxPulses[1] <= 1200
      && rxPulses[2] <= 1200
      && rxPulses[3] >= 1700
   )
   {
      return MOTOR4;
   }
   else if (
         rxPulses[0] <= 1200
      && rxPulses[1] <= 1200
      && rxPulses[2] <= 1200
      && rxPulses[3] <= 1200
   )
   {
      return TUNINGOFF;
   }
   else if (
         rxPulses[0] <= 1700
      && rxPulses[1] <= 1700
      && rxPulses[2] <= 1200
      && rxPulses[3] >= 1200
   )
   {
      return ALLMOTORS;
   }
   return currentMode;
}

#endif
