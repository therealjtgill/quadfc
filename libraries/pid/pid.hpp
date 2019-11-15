#ifndef PIDHPP
#define PIDHPP

template <class numeric_T>
class PID
{
   private:
      float kP_;
      float kI_;
      float kD_;
      numeric_T errorPrev_;
      numeric_T errorIntegral_;
      numeric_T yPrev_;
      numeric_T yMin_;
      numeric_T yMax_;

   public:
      PID (float kP, float kI, float kD, numeric_T x, numeric_T u, numeric_T outMin, numeric_T outMax)
         :  yMin_(outMin),
            yMax_(outMax),
            yPrev_(0)
      {
         kP_ = kP;
         kI_ = kI;
         kD_ = kD;
         // xPrev_ = x;
         // uPrev_ = u;
         errorPrev_ = (x - u);
      }

      PID (float kP, float kI, float kD, numeric_T x, numeric_T u)
         :  yMin_(-4096),
            yMax_(4096),
            yPrev_(0)
      {
         kP_ = kP;
         kI_ = kI;
         kD_ = kD;
         //xPrev_ = x;
         //uPrev_ = u;
         errorPrev_ = (x - u);
      }

      ~PID (void)
      {

      }

      numeric_T filter (const numeric_T & x, const numeric_T & u)
      {
         numeric_T y = 0;
         numeric_T error = (x - u);

         errorIntegral_ += error;

         errorIntegral_ = max(min(errorIntegral_, yMax_), yMin_);
         y = kP_*error + kD_*(error - errorPrev_) + kI_*errorIntegral_;
 
         y = max(min(y, yMax_), yMin_);
         yPrev_ = y;
         
         errorPrev_ = error;

         return y;
      }

};


/////////////////////////////////////////////////
// clamp
/////////////////////////////////////////////////
template <typename T>
T clamp(T val, T min_val, T max_val)
{
  return max(min(val, max_val), min_val);
}

#endif
