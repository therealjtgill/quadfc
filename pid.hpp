#ifndef PIDHPP
#define PIDHPP

template <class numeric_T>
class PID
{
   private:
      float kP_;
      float kI_;
      float kD_;
      numeric_T xPrev_;
      numeric_T xIntegral_;
      numeric_T uPrev_;
      numeric_T uIntegral_;
      numeric_T yPrev_;
      numeric_T yMin_;
      numeric_T yMax_;

   public:
      PID (float kP, float kI, float kD, numeric_T x, numeric_T u, numeric_T outMin, numeric_T outMax)
         :  xIntegral_(0),
            uIntegral_(0),
            yMin_(outMin),
            yMax_(outMax),
            yPrev_(0)
      {
         kP_ = kP;
         kI_ = kI;
         kD_ = kD;
         xPrev_ = x;
         uPrev_ = u;
      }

      PID (float kP, float kI, float kD, numeric_T x, numeric_T u)
         :  xIntegral_(0),
            uIntegral_(0),
            yMin_(-4096),
            yMax_(4096),
            yPrev_(0)
      {
         kP_ = kP;
         kI_ = kI;
         kD_ = kD;
         xPrev_ = x;
         uPrev_ = u;
      }

      ~PID (void)
      {

      }

      numeric_T filter (const numeric_T & x, const numeric_T & u)
      {
         numeric_T y = 0;
         xIntegral_ += x;
         uIntegral_ += u;

         if (!(yPrev_ > yMax_ && (x - u) > 0) || !(yPrev_ < yMin_ && (x - u) < 0))
         {
            y = kP_*(x - u) + kD_*((x - xPrev_) - (u - uPrev_)) + kI_*(xIntegral_ - uIntegral_);
         }
         else
         {
            y = kP_*(x - u) + kD_*((x - xPrev_) - (u - uPrev_));
         }
         y = max(min(y, yMax_), yMin_);
         yPrev_ = y;
         xPrev_ = x;
         uPrev_ = u;

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
