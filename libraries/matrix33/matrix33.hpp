#ifndef MATRIX33
#define MATRIX33

#include <assert.h>
//#include <iostream>
//#include <iomanip>
#include "vector3.hpp"

class Matrix33
{
   public:

      Matrix33 (float a00, float a01, float a02,
                float a10, float a11, float a12,
                float a20, float a21, float a22)
//         : name_("None")
#ifdef MATHDEBUGGING
         , logger_("matrixdebug.log", "Matrix")
#endif
      {
         Initialize (a00, a01, a02,
                     a10, a11, a12,
                     a20, a21, a22);
      }

      Matrix33 (const Matrix33 & M)
//         : name_("None")
#ifdef MATHDEBUGGING
         , logger_("matrixdebug.log", "Matrix")
#endif
      {
         for (unsigned int i = 0; i < 3; ++i)
         {
            for (unsigned int j = 0; j < 3; ++j)
            {
               base_[i][j] = M[i][j];
            }
         }
         CalculateDeterminant();
      }

      Matrix33 (void)
//         : name_("None")
#ifdef MATHDEBUGGING
         , logger_("matrixdebug.log", "Matrix")
#endif
      {
         Initialize (0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0,
                     0.0, 0.0, 0.0);
      }

      void Initialize (float a00, float a01, float a02,
                       float a10, float a11, float a12,
                       float a20, float a21, float a22)
      {
         base_[0][0] = a00; base_[0][1] = a01; base_[0][2] = a02;
         base_[1][0] = a10; base_[1][1] = a11; base_[1][2] = a12;
         base_[2][0] = a20; base_[2][1] = a21; base_[2][2] = a22;

         CalculateDeterminant();
      }

      Matrix33 transpose (void)
      {
         Matrix33 T(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
         for (unsigned int i = 0; i < 3; ++i)
         {
            for (unsigned int j = 0; j < 3; ++j)
            {
               T[i][j] = base_[j][i];
            }
         }

         return T;
      }

      float determinant (void) const
      {
         float det = base_[0][0]*base_[1][1]*base_[2][2] +
                     base_[0][1]*base_[1][2]*base_[2][0] +
                     base_[0][2]*base_[1][0]*base_[2][1] -
                     base_[2][0]*base_[1][1]*base_[0][2] -
                     base_[2][1]*base_[1][2]*base_[0][0] -
                     base_[2][2]*base_[1][0]*base_[0][1];
         return det;
      }

      Matrix33 & operator= (const Matrix33 & M)
      {
         // Just sets all row vectors equal to each other.
         for (unsigned int i = 0; i < 3; ++i)
         {
            base_[i] = M[i];
         }
         CalculateDeterminant();
         return *this;
      }

      Vector3 & operator[] (unsigned int i)
      {
         return base_[i];
      }

      const Vector3 & operator[] (unsigned int i) const
      {
         return base_[i];
      }

      Matrix33 operator* (const float & v) const
      {
         Matrix33 result;
         for (unsigned int i = 0; i < 3; ++i)
         {
            result[i] = base_[i]*v;
         }
         return result;
      }

      Vector3 operator* (const Vector3 & a) const
      {
         Vector3 result;
         for (unsigned int i = 0; i < 3; ++i)
         {
            for (unsigned int j = 0; j < 3; ++j)
            {
               result[i] += base_[i][j]*a[j];
            }
         }
         return result;
      }

      Matrix33 operator* (const Matrix33 & M) const
      {
         Matrix33 result(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
         for (unsigned int i = 0; i < 3; ++i)
         {
            for (unsigned int j = 0; j < 3; ++j)
            {
               for (unsigned int k = 0; k < 3; ++k)
               {
                  result[i][j] += base_[i][k]*M[k][j];
               }
            }
         }
         return result;
      }

      Matrix33 & operator*= (const float & v)
      {
         for (unsigned int i = 0; i < 3; ++i)
         {
            base_[i] *= v;
         }
         return *this;
      }

      Matrix33 & operator*= (const Matrix33 & M)
      {
         Matrix33 result(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
         for (unsigned int i = 0; i < 3; ++i)
         {
            for (unsigned int j = 0; j < 3; ++j)
            {
               for (unsigned int k = 0; k < 3; ++k)
               {
                  result[i][j] += base_[i][k]*M[k][j];
               }
            }
         }
         for (unsigned int i = 0; i < 3; ++i)
         {
            for (unsigned int j = 0; j < 3; ++j)
            {
               base_[i][j] = result[i][j];
            }
         }
         return *this;
      }

      Matrix33 operator/ (const float & v) const
      {
         Matrix33 A;
         for (unsigned int i = 0; i < 3; ++i)
         {
            for (unsigned int j = 0; j < 3; ++j)
            {
               A[i][j] = base_[i][j]/v;
            }
         }
         return A;
      }

      Matrix33 & operator/= (const float & v)
      {
         for (unsigned int i = 0; i < 3; ++i)
         {
            for (unsigned int j = 0; j < 3; ++j)
            {
               base_[i][j] /= v;
            }
         }
         return *this;
      }

      Matrix33 operator+ (const Matrix33 & M) const
      {
         Matrix33 result;
         const Matrix33 copy(*this);
         for (unsigned int i = 0; i < 3; ++i)
         {
            result[i] = copy[i] + M[i];
         }
         return result;
      }

      Matrix33 & operator+= (const Matrix33 & M)
      {
         for (unsigned int i = 0; i < 3; ++i)
         {
            base_[i] += M[i];
         }
         return *this;
      }

      Matrix33 operator- (const Matrix33 & M) const
      {
         Matrix33 result;
         for (unsigned int i = 0; i < 3; ++i)
         {
            for (unsigned int j = 0; j < 3; ++j)
            {
               result[i][j] = base_[i][j] - M[i][j];
            }
         }
         return result;
      }

      Matrix33 & operator-= (const Matrix33 & M)
      {
         for (unsigned int i = 0; i < 3; ++i)
         {
            base_[i] -= M[i];
         }
         return *this;
      }

      Matrix33 operator~ (void) const
      {
         Matrix33 A(*this);
         float det = determinant();
         // Can't invert a singular matrix, but this can probably be handled
         // better.
         assert (det != 0.0);
         // |00 01 02|
         // |10 11 12|
         // |20 21 22|
         A[0][0] =       base_[1][1]*base_[2][2] - base_[1][2]*base_[2][1];
         A[0][1] = -1.0*(base_[1][0]*base_[2][2] - base_[1][2]*base_[2][0]);
         A[0][2] =       base_[1][0]*base_[2][1] - base_[1][1]*base_[2][0];
         A[1][0] = -1.0*(base_[0][1]*base_[2][2] - base_[0][2]*base_[2][1]);
         A[1][1] =       base_[0][0]*base_[2][2] - base_[0][2]*base_[2][0];
         A[1][2] = -1.0*(base_[0][0]*base_[2][1] - base_[0][1]*base_[2][0]);
         A[2][0] =       base_[0][1]*base_[1][2] - base_[0][2]*base_[1][1];
         A[2][1] = -1.0*(base_[0][0]*base_[1][2] - base_[0][2]*base_[1][0]);
         A[2][2] =       base_[0][0]*base_[1][1] - base_[0][1]*base_[1][0];
         A = A.transpose();
         
         A /= det;
         return A;
      }

/*
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
*/

   private:

      Vector3 base_[3];

//      std::string name_;

      float determinant_;

#ifdef MATHDEBUGGING
      Logger logger_;
#endif

      void CalculateDeterminant (void)
      {
         determinant_ = base_[0][0]*base_[1][1]*base_[2][2] +
                        base_[0][1]*base_[1][2]*base_[2][0] +
                        base_[0][2]*base_[1][0]*base_[2][1] -
                        base_[2][0]*base_[1][1]*base_[0][2] -
                        base_[2][1]*base_[1][2]*base_[0][0] -
                        base_[2][2]*base_[1][0]*base_[0][1];
      }

};

/*
std::ostream & operator << (std::ostream & out, const Matrix33 & M)
{
   for (unsigned int i = 0; i < 3; ++i)
   {
      for (unsigned int j = 0; j < 3; ++j)
      {
         out << std::setw(10) << std::right << M[i][j];
         if (j < 2)
         {
            out << " ";
         }
         else
         {
            out << "\n";
         }
      }
   }
   return out;
}
*/

Matrix33 crossProductMatrix(const Vector3 & a)
{
   Matrix33 A(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
   A[0][1] = -1*a[2];
   A[0][2] = a[1];
   A[1][2] = -1*a[0];
   A[1][0] = a[2];
   A[2][0] = -1*a[1];
   A[2][1] = a[0];
   return A;
}

Matrix33 identityMatrix(void)
{
   Matrix33 I(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
   return I;
}

Matrix33 outerProduct(const Vector3 & a, const Vector3 & b)
{
   Matrix33 A(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
   for (unsigned int i = 0; i < 3; ++i)
   {
      for (unsigned int j = 0; j < 3; ++j)
      {
         A[i][j] = a[i]*b[j];
      }
   }
   return A;
}

Matrix33 operator* (const float & v, const Matrix33 & M)
{
   return M*v;
}


#endif