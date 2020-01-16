#ifndef QUATERNION
#define QUATERNION

#include "vector3.hpp"
#include "matrix33.hpp"

// Quaternion implementation where the scalar part is the first element, and
// the vector part is the second element.

class Quaternion
{
   public:

      Quaternion (void)
#ifdef MATHDEBUGGING
         , logger_("quaterniondebug.log", "Quaternion")
#endif
      {
         Initialize(0.0, 0.0, 0.0, 0.0);
      }

      Quaternion (const float & a0, const Vector3 & b)
#ifdef MATHDEBUGGING
         , logger_("quaterniondebug.log", "Quaternion")
#endif
      {
         Initialize(a0, b[0], b[1], b[2]);
      }

      Quaternion (const Quaternion & a)
#ifdef MATHDEBUGGING
         , logger_("quaterniondebug.log", "Quaternion")
#endif
      {
         Initialize(a[0], a[1], a[2], a[3]);
      }

      Quaternion (float a0, float a1, float a2, float a3)
#ifdef MATHDEBUGGING
         , logger_("quaterniondebug.log", "Quaternion")
#endif
      {
         Initialize(a0, a1, a2, a3);
      }

      void Initialize (float a0, float a1, float a2, float a3)
      {
         scalar_ = a0;
         vector_[0] = a1;
         vector_[1] = a2;
         vector_[2] = a3;
      }

      float magnitude (void) const
      {
         return sqrt(scalar_*scalar_ + vector_.magnitude()*vector_.magnitude());;
      }

      float magnitudeSquared (void) const
      {
         return scalar_*scalar_ + vector_.magnitude()*vector_.magnitude();
      }

      Vector3 & vector (void)
      {
         return vector_;
      }

      const Vector3 & vector (void) const
      {
         return vector_;
      }

      float & scalar (void)
      {
         return scalar_;
      }

      const float & scalar (void) const
      {
         return scalar_;
      }

      Quaternion conjugate (void) const
      {
         static Quaternion result;
         result.scalar() = scalar();
         result.vector() = -1.0*vector();
         return result;
      }

      Matrix33 rotationMatrix (void) const
      {
         Quaternion unit = (*this)/this->magnitude();
         float q0 = unit[0];
         float q1 = unit[1];
         float q2 = unit[2];
         float q3 = unit[3];
         Matrix33 R;
         R[0][0] = q0*q0 + q1*q1 - q2*q2 - q3*q3;
         R[0][1] = 2*(q1*q2 + q0*q3);
         R[0][2] = 2*(q1*q3 - q0*q2);
         R[1][0] = 2*(q1*q2 - q0*q3);
         R[1][1] = q0*q0 - q1*q1 + q2*q2 - q3*q3;
         R[1][2] = 2*(q2*q3 + q0*q1);
         R[2][0] = 2*(q1*q3 + q0*q2);
         R[2][1] = 2*(q2*q3 - q0*q1);
         R[2][2] = q0*q0 - q1*q1 - q2*q2 + q3*q3;

         return R;
      }

      float & operator[] (unsigned int i)
      {
         if (i == 0)
         {
            return scalar_;
         }
         else
         {
            return vector_[i - 1];
         }
      }

      const float & operator[] (unsigned int i) const
      {
         if (i == 0)
         {
            return scalar_;
         }
         else
         {
            return vector_[i - 1];
         }
      }

      Quaternion & operator= (const Quaternion & a)
      {
         if (&a == this)
         {
            return *this;
         }

         Initialize(a[0], a[1], a[2], a[3]);
         return *this;
      }

      const Quaternion operator= (const Quaternion & a) const
      {
         if (&a == this)
         {
            return *this;
         }
         static Quaternion result;
         result.scalar() = a[0];
         result[1] = a[1];
         result[2] = a[2];
         result[3] = a[3];
         return result;
      }

      Quaternion operator* (float b) const
      {
         static Quaternion result;
         for (unsigned int i = 0; i < 4; ++i)
         {
            result[i] = (*this)[i] * b;
         }
         return result;
      }

      Quaternion operator* (const Quaternion & b) const
      {
         static Quaternion result;
         float a0 = scalar_;
         float b0 = b[0];
         Vector3 aVec = vector_;
         Vector3 bVec = b.vector();
         result[0] = a0*b0 - aVec.dot(bVec);
         result.vector() = a0*bVec + b0*aVec + aVec.crossProduct(bVec);
         //std::cout << result.vector() << std::endl;
         //std::cout << "a0: " << a0 << " b0: " << b0 << std::endl;
         //std::cout << "avec: " << aVec << " bvec: " << bVec << std::endl;
         //std::cout << "a0*bvec: " << a0*bVec << " b0*avec: " << b0*aVec << std::endl;
         //std::cout << "this mag < 1: " << ((*this).magnitude() < 1.0) << " incoming mag < 1: " << (b.magnitude() < 1.0) << std::endl;
         //std::cout << "quat mult mag: " << result.magnitude() << " less than 1? " << (result.magnitude() < 1.0) << std::endl;
         return result;
      }

      Quaternion & operator*= (float b)
      {
         for (unsigned int i = 0; i < 4; ++i)
         {
            (*this)[i] *= b;
         }
         return *this;
      }

      Quaternion & operator*= (const Quaternion b)
      {
         Vector3 aVec = vector_;
         Vector3 bVec = b.vector();
         float a0 = scalar_;
         float b0 = b[0];
         scalar_ = a0*b0 - aVec.dot(bVec);
         vector_ = a0*bVec + b0*aVec + aVec.crossProduct(bVec);
         return *this;
      }

      Quaternion operator/ (const float b) const
      {
         Quaternion result(*this);
         for (unsigned int i = 0; i < 4; ++i)
         {
            result[i] *= b;
         }
         return result;
      }

      Quaternion & operator/= (const float b)
      {
         for (unsigned int i = 0; i < 4; ++i)
         {
            (*this)[i] *= b;
         }
         return *this;
      }

      Quaternion operator+ (const Quaternion & b)
      {
         Quaternion result;
         result.scalar() = scalar_ + b.scalar();
         result.vector() = vector_ + b.vector();
         return result;
      }

      Quaternion & operator+= (const Quaternion & b)
      {
         scalar_ += b.scalar();
         vector_ += b.vector();
         return *this;
      }

      Quaternion operator- (const Quaternion & b) const
      {
         Quaternion result;
         result.scalar() = scalar_ - b.scalar();
         result.vector() = vector_ - b.vector();
         return result;
      }

      Quaternion & operator-= (const Quaternion & b)
      {
         scalar_ -= b.scalar();
         vector_ -= b.vector();
         return *this;
      }

      Quaternion operator~ (void) const
      {
         Quaternion result;
         result = conjugate()/magnitude();
         return result;
      }

   private:

      float scalar_;

      Vector3 vector_;

#ifdef MATHDEBUGGING
      Logger logger_;
#endif

};

Quaternion operator* (const float & a, const Quaternion b)
{
   Quaternion result(b);
   return result*a;
}

Quaternion exp (const Vector3 & u)
{
   Quaternion result;
   float magnitude = u.magnitude();
   //result.scalar() = cos(magnitude);
   result.scalar() = 1 - magnitude*magnitude/2.;
   //result.vector() = sin(magnitude)*u/magnitude;
   result.vector() = (1 - magnitude*magnitude/6.)*u;
   //std::cout << "Magnitude of exponentiated quaternion: " << result.magnitude() << std::endl;
   //result = result/result.magnitude();
   return result;
}

// This is only valid for unit quaternions.
Quaternion exp (const Quaternion & u)
{
   return exp(u.vector());
}

/*
std::ostream & operator << (std::ostream & out, const Quaternion & a)
{
   for (unsigned int j = 0; j < 4; ++j)
   {
      out << std::setw(10) << std::right << a[j];
      if (j < 3)
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
*/

#endif
