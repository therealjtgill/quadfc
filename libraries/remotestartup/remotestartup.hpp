#ifndef REMOTESTARTUP
#define REMOTESTARTUP

typedef enum OperationMode {UNINITIALIZED, FLIGHT, NOFLIGHT} OperationMode;

OperationMode checkOperationMode(const int * motorPulses, const OperationMode & currentMode)
{
   if (currentMode == FLIGHT || currentMode == NOFLIGHT)
   {
      if (
         motorPulses[0] <= 1200 
         && motorPulses[1] <= 1200
         && motorPulses[2] <= 1200
         && motorPulses[3] <= 1200
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
         motorPulses[0] >= 1200 
         || motorPulses[1] >= 1200
         || motorPulses[2] >= 1200
         || motorPulses[3] >= 1200
      )
      {
         return NOFLIGHT;
      }
   }
   
   
}

#endif
