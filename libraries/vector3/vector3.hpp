#ifndef VECTOR3
#define VECTOR3

#include <math.h>
#include <iomanip>
#include <iostream>
#include <assert.h>

class Vector3
{
   public:

      Vector3 (float x, float y, float z)
         : pad_(1e-16)
         , name_("None")
#ifdef MATHDEBUGGING
         , logger_("quaterniondebug.log", "Quaternion")
#endif
      {
         Initialize(x, y, z);
      }

      Vector3 (const Vector3 & in)
         : pad_(1e-16)
         , name_("None")
#ifdef MATHDEBUGGING
         , logger_("quaterniondebug.log", "Quaternion")
#endif
      {
         Initialize(in[0], in[1], in[2]);
      }

      Vector3 (const float in[3])
         : pad_(1e-16)
         , name_("None")
#ifdef MATHDEBUGGING
         , logger_("quaterniondebug.log", "Quaternion")
#endif
      {
         Initialize(in[0], in[1], in[2]);
      }

      Vector3 (void)
         : pad_(1e-16)
         , name_("None")
#ifdef MATHDEBUGGING
         , logger_("quaterniondebug.log", "Quaternion")
#endif
      {
         Initialize(0.0, 0.0, 0.0);
      }

      void Initialize (float x, float y, float z)
      {
         base_[0] = x;
         base_[1] = y;
         base_[2] = z;
         CalculateMagnitudeAndDirection();
      }

      Vector3 crossProduct (const Vector3 & a) const
      {
         Vector3 c(base_[1]*a[2] - base_[2]*a[1], base_[2]*a[0] - base_[0]*a[2], base_[0]*a[1] - base_[1]*a[0]);
         return c;
      }

      Vector3 unitVector (void) const
      {
         Vector3 dir;
         float mag = magnitude();
         dir[0] = base_[0]/(mag + pad_);
         dir[1] = base_[1]/(mag + pad_);
         dir[2] = base_[2]/(mag + pad_);
         return dir;
      }

      float magnitude (void) const
      {
         float mag = sqrt(base_[0]*base_[0] + base_[1]*base_[1] + base_[2]*base_[2]);
         return mag;
      }

      float dot (const Vector3 & a) const
      {
         return base_[0]*a[0] + base_[1]*a[1] + base_[2]*a[2];
      }

      Vector3 & operator= (const Vector3 & a)
      {
         if (&a == this)
         {
            return *this;
         }

         Initialize(a[0], a[1], a[2]);
         return *this;
      }

      const Vector3 operator= (const Vector3 & a) const
      {
         if (&a == this)
         {
            return *this;
         }
         Vector3 result(a[0], a[1], a[2]);
         return result;
      }

      float & operator[] (unsigned int i)
      {
         assert (i >= 0 && i < 3);
         return base_[i];
      }

      const float & operator[] (unsigned int i) const
      {
         assert (i >= 0 && i < 3);
         return base_[i];
      }

      Vector3 operator+ (const Vector3 & a) const
      {
         return Vector3(base_[0] + a[0], base_[1] + a[1], base_[2] + a[2]);
      }

      Vector3 & operator+= (const Vector3 & a)
      {
         for (unsigned int i = 0; i < 3; ++i)
         {
            base_[i] += a[i];
         }
         CalculateMagnitudeAndDirection();
         return *this;
      }

      Vector3 operator- (const Vector3 & a) const
      {
         Vector3 result(base_[0] - a[0], base_[1] - a[1], base_[2] - a[2]);
         return result;
      }

      Vector3 & operator-= (const Vector3 & a)
      {
         for (unsigned int i = 0; i < 3; ++i)
         {
            base_[i] -= a[i];
         }
         CalculateMagnitudeAndDirection();
         return *this;
      }

      Vector3 operator* (const Vector3 & a) const
      {
         Vector3 result(base_[0]*a[0], base_[1]*a[1], base_[2]*a[2]);
         return result;
      }

      Vector3 operator* (const float & v) const
      {
         return Vector3(base_[0]*v, base_[1]*v, base_[2]*v);
      }

      Vector3 & operator*= (const float & v)
      {
         base_[0] *= v;
         base_[1] *= v;
         base_[2] *= v;
         return *this;
      }

      Vector3 & operator/= (const float & v)
      {
         base_[0] /= v;
         base_[1] /= v;
         base_[2] /= v;
         return *this;
      }

      Vector3 operator/ (const float & v)
      {
         Vector3 result(*this);
         return result*(1.0/v);
      }

      Vector3 operator& (const Vector3 & v)
      {
         return Vector3(base_[0]*v[0], base_[1]*v[1], base_[2]*v[2]);
      }

      const std::string & getName (void) const
      {
         return name_;
      }

      bool setName (const std::string & s)
      {
         bool ret = false;
         if (name_ == "None")
         {
            ret = true;
            name_ = s;
         }
         return ret;
      }

   private:
      const float pad_;

      float base_[3];

      float mag_;

      float dir_[3];

      std::string name_;

#ifdef MATHDEBUGGING
      Logger logger_;
#endif

      void CalculateMagnitudeAndDirection (void)
      {
         mag_ = sqrt(base_[0]*base_[0] + base_[1]*base_[1] + base_[2]*base_[2]);
         dir_[0] = base_[0]/(mag_ + pad_);
         dir_[1] = base_[1]/(mag_ + pad_);
         dir_[2] = base_[2]/(mag_ + pad_);
      }

};

std::ostream & operator << (std::ostream & out, const Vector3 & a)
{
   for (unsigned int j = 0; j < 3; ++j)
   {
      out << std::setw(10) << std::right << a[j];
      if (j < 2)
      {
         out << " ";
      }
      else
      {
         //out << "\n";
      }
   }
   return out;
}

Vector3 operator* (const float & v, const Vector3 & a)
{
   return a*v;
}

#endif