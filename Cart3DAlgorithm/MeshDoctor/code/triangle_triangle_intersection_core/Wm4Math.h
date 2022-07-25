#ifndef WM4MATH_H
#define WM4MATH_H
#include <iostream>
#include <cmath>
namespace Sn3DAlgorithm
{

	template <class Real>
	class Math
	{
	public:
		static Real ACos(Real fValue);
		static Real ASin(Real fValue);
		static Real ATan(Real fValue);
		static Real ATan2(Real fY, Real fX);
		static Real Ceil(Real fValue);
		static Real Cos(Real fValue);
		static Real Exp(Real fValue);
		static Real FAbs(Real fValue);
		static Real Floor(Real fValue);
		static Real FMod(Real fX, Real fY);
		static Real InvSqrt(Real fValue);
		static Real Log(Real fValue);
		static Real Log2(Real fValue);
		static Real Log10(Real fValue);
		static Real Pow(Real fBase, Real fExponent);
		static Real Sin(Real fValue);
		static Real Sqr(Real fValue);
		static Real Sqrt(Real fValue);
		static Real Tan(Real fValue);

		// Return -1 if the input is negative, 0 if the input is zero, and +1
		// if the input is positive.
		static int Sign(int iValue);
		static Real Sign(Real fValue);

		// Generate a random number in [0,1].  The random number generator may
		// be seeded by a first call to UnitRandom with a positive seed.
		static Real UnitRandom(unsigned int uiSeed = 0);

		// Generate a random number in [-1,1].  The random number generator may
		// be seeded by a first call to SymmetricRandom with a positive seed.
		static Real SymmetricRandom(unsigned int uiSeed = 0);

		// Generate a random number in [min,max].  The random number generator may
		// be seeded by a first call to IntervalRandom with a positive seed.
		static Real IntervalRandom(Real fMin, Real fMax,
			unsigned int uiSeed = 0);

		// Fast evaluation of trigonometric and inverse trigonometric functions
		// using polynomial approximations.  The speed ups were measured on an
		// AMD 2800 (2.08 GHz) processor using Visual Studion .NET 2003 with a
		// release build.

		// The input must be in [0,pi/2].
		// max error sin0 = 1.7e-04, speed up = 4.0
		// max error sin1 = 1.9e-08, speed up = 2.8
		static Real FastSin0(Real fAngle);
		static Real FastSin1(Real fAngle);

		// The input must be in [0,pi/2]
		// max error cos0 = 1.2e-03, speed up = 4.5
		// max error cos1 = 6.5e-09, speed up = 2.8
		static Real FastCos0(Real fAngle);
		static Real FastCos1(Real fAngle);

		// The input must be in [0,pi/4].
		// max error tan0 = 8.1e-04, speed up = 5.6
		// max error tan1 = 1.9e-08, speed up = 3.4
		static Real FastTan0(Real fAngle);
		static Real FastTan1(Real fAngle);

		// The input must be in [0,1].
		// max error invsin0 = 6.8e-05, speed up = 7.5
		// max error invsin1 = 1.4e-07, speed up = 5.5
		static Real FastInvSin0(Real fValue);
		static Real FastInvSin1(Real fValue);

		// The input must be in [0,1].
		// max error invcos0 = 6.8e-05, speed up = 7.5
		// max error invcos1 = 1.4e-07, speed up = 5.7
		static Real FastInvCos0(Real fValue);
		static Real FastInvCos1(Real fValue);

		// The input must be in [-1,1]. 
		// max error invtan0 = 1.2e-05, speed up = 2.8
		// max error invtan1 = 2.3e-08, speed up = 1.8
		static Real FastInvTan0(Real fValue);
		static Real FastInvTan1(Real fValue);

		// A fast approximation to 1/sqrt.
		static Real FastInvSqrt(Real fValue);

		// Fast approximations to exp(-x).  The input x must be in [0,infinity).
		// max error negexp0 = 0.00024, speed up = 25.4
		// max error negexp1 = 0.000024, speed up = 25.4
		// max error negexp2 = 0.0000024, speed up = 20.5
		// max error negexp3 = 0.00000025, speed up = 17.3
		static Real FastNegExp0(Real fValue);
		static Real FastNegExp1(Real fValue);
		static Real FastNegExp2(Real fValue);
		static Real FastNegExp3(Real fValue);

		// common constants
		static const Real EPSILON;
		static const Real ZERO_TOLERANCE;
		static const Real MAX_REAL;
		static const Real PI;
		static const Real TWO_PI;
		static const Real HALF_PI;
		static const Real INV_PI;
		static const Real INV_TWO_PI;
		static const Real DEG_TO_RAD;
		static const Real RAD_TO_DEG;
		static const Real LN_2;
		static const Real LN_10;
		static const Real INV_LN_2;
		static const Real INV_LN_10;
	};

	template <class Real>
	class Vector2
	{
	public:
		// construction
		Vector2();  // uninitialized
		Vector2(Real fX, Real fY);
		Vector2(const Real* afTuple);
		Vector2(const Vector2& rkV);

		// coordinate access
		inline operator const Real* () const;
		inline operator Real* ();
		inline Real operator[] (int i) const;
		inline Real& operator[] (int i);
		inline Real X() const;
		inline Real& X();
		inline Real Y() const;
		inline Real& Y();

		// assignment
		inline Vector2& operator= (const Vector2& rkV);

		// comparison
		bool operator== (const Vector2& rkV) const;
		bool operator!= (const Vector2& rkV) const;
		bool operator<  (const Vector2& rkV) const;
		bool operator<= (const Vector2& rkV) const;
		bool operator>  (const Vector2& rkV) const;
		bool operator>= (const Vector2& rkV) const;

		// arithmetic operations
		inline Vector2 operator+ (const Vector2& rkV) const;
		inline Vector2 operator- (const Vector2& rkV) const;
		inline Vector2 operator* (Real fScalar) const;
		inline Vector2 operator/ (Real fScalar) const;
		inline Vector2 operator- () const;

		// arithmetic updates
		inline Vector2& operator+= (const Vector2& rkV);
		inline Vector2& operator-= (const Vector2& rkV);
		inline Vector2& operator*= (Real fScalar);
		inline Vector2& operator/= (Real fScalar);

		// vector operations
		inline Real Length() const;
		inline Real SquaredLength() const;
		inline Real Dot(const Vector2& rkV) const;
		inline Real Normalize();

		// returns (y,-x)
		inline Vector2 Perp() const;

		// returns (y,-x)/sqrt(x*x+y*y)
		inline Vector2 UnitPerp() const;

		// returns DotPerp((x,y),(V.x,V.y)) = x*V.y - y*V.x
		inline Real DotPerp(const Vector2& rkV) const;

		// Compute the barycentric coordinates of the point with respect to the
		// triangle <V0,V1,V2>, P = b0*V0 + b1*V1 + b2*V2, where b0 + b1 + b2 = 1.
		void GetBarycentrics(const Vector2& rkV0, const Vector2& rkV1,
			const Vector2& rkV2, Real afBary[3]) const;

		// Gram-Schmidt orthonormalization.  Take linearly independent vectors U
		// and V and compute an orthonormal set (unit length, mutually
		// perpendicular).
		static void Orthonormalize(Vector2& rkU, Vector2& rkV);

		// Input V must be a nonzero vector.  The output is an orthonormal basis
		// {U,V}.  The input V is normalized by this function.  If you know V is
		// already unit length, use U = V.Perp().
		static void GenerateOrthonormalBasis(Vector2& rkU, Vector2& rkV);

		// Compute the extreme values.
		static void ComputeExtremes(int iVQuantity, const Vector2* akPoint,
			Vector2& rkMin, Vector2& rkMax);

		// special vectors
		static const Vector2 ZERO;    // (0,0)
		static const Vector2 UNIT_X;  // (1,0)
		static const Vector2 UNIT_Y;  // (0,1)
		static const Vector2 ONE;     // (1,1)

	private:
		// support for comparisons
		int CompareArrays(const Vector2& rkV) const;

		Real m_afTuple[2];
	};


	template <class Real>
	class Vector3
	{
	public:
		// construction
		Vector3();  // uninitialized
		Vector3(Real fX, Real fY, Real fZ);
		Vector3(const Real* afTuple);
		Vector3(const Vector3& rkV);

		// coordinate access
		inline operator const Real* () const;
		inline operator Real* ();
		inline Real operator[] (int i) const;
		inline Real& operator[] (int i);
		inline Real X() const;
		inline Real& X();
		inline Real Y() const;
		inline Real& Y();
		inline Real Z() const;
		inline Real& Z();

		// assignment
		inline Vector3& operator= (const Vector3& rkV);

		// comparison
		bool operator== (const Vector3& rkV) const;
		bool operator!= (const Vector3& rkV) const;
		bool operator<  (const Vector3& rkV) const;
		bool operator<= (const Vector3& rkV) const;
		bool operator>  (const Vector3& rkV) const;
		bool operator>= (const Vector3& rkV) const;

		// arithmetic operations
		inline Vector3 operator+ (const Vector3& rkV) const;
		inline Vector3 operator- (const Vector3& rkV) const;
		inline Vector3 operator* (Real fScalar) const;
		inline Vector3 operator/ (Real fScalar) const;
		inline Vector3 operator- () const;

		// arithmetic updates
		inline Vector3& operator+= (const Vector3& rkV);
		inline Vector3& operator-= (const Vector3& rkV);
		inline Vector3& operator*= (Real fScalar);
		inline Vector3& operator/= (Real fScalar);

		// vector operations
		inline Real Length() const;
		inline Real SquaredLength() const;
		inline Real Dot(const Vector3& rkV) const;
		inline Real Normalize();

		// The cross products are computed using the right-handed rule.  Be aware
		// that some graphics APIs use a left-handed rule.  If you have to compute
		// a cross product with these functions and send the result to the API
		// that expects left-handed, you will need to change sign on the vector
		// (replace each component value c by -c).
		inline Vector3 Cross(const Vector3& rkV) const;
		inline Vector3 UnitCross(const Vector3& rkV) const;

		// Compute the barycentric coordinates of the point with respect to the
		// tetrahedron <V0,V1,V2,V3>, P = b0*V0 + b1*V1 + b2*V2 + b3*V3, where
		// b0 + b1 + b2 + b3 = 1.
		void GetBarycentrics(const Vector3& rkV0, const Vector3& rkV1,
			const Vector3& rkV2, const Vector3& rkV3, Real afBary[4]) const;

		// Gram-Schmidt orthonormalization.  Take linearly independent vectors
		// U, V, and W and compute an orthonormal set (unit length, mutually
		// perpendicular).
		static void Orthonormalize(Vector3& rkU, Vector3& rkV, Vector3& rkW);
		static void Orthonormalize(Vector3* akV);

		// Input W must be a nonzero vector. The output is an orthonormal basis
		// {U,V,W}.  The input W is normalized by this function.  If you know
		// W is already unit length, use GenerateComplementBasis to compute U
		// and V.
		static void GenerateOrthonormalBasis(Vector3& rkU, Vector3& rkV,
			Vector3& rkW);

		// Input W must be a unit-length vector.  The output vectors {U,V} are
		// unit length and mutually perpendicular, and {U,V,W} is an orthonormal
		// basis.
		static void GenerateComplementBasis(Vector3& rkU, Vector3& rkV,
			const Vector3& rkW);

		// Compute the extreme values.
		static void ComputeExtremes(int iVQuantity, const Vector3* akPoint,
			Vector3& rkMin, Vector3& rkMax);

		// special vectors
		static const Vector3 ZERO;    // (0,0,0)
		static const Vector3 UNIT_X;  // (1,0,0)
		static const Vector3 UNIT_Y;  // (0,1,0)
		static const Vector3 UNIT_Z;  // (0,0,1)
		static const Vector3 ONE;     // (1,1,1)

	private:
		// support for comparisons
		int CompareArrays(const Vector3& rkV) const;

		Real m_afTuple[3];
	};
	


	// arithmetic operations
	template <class Real>
	inline Vector2<Real> operator* (Real fScalar, const Vector2<Real>& rkV)
	{
		return Vector2<Real>(
			fScalar*rkV[0],
			fScalar*rkV[1]);
	}

	template <class Real>
	inline std::ostream& operator<< (std::ostream& rkOStr, const Vector2<Real>& rkV)
	{
		return rkOStr << rkV.X() << ' ' << rkV.Y();
	}

	template <class Real>
	std::ostream& operator<< (std::ostream& rkOStr, const Vector3<Real>& rkV)
	{
		return rkOStr << rkV.X() << ' ' << rkV.Y() << ' ' << rkV.Z();
	}

	template <class Real>
    Vector3<Real> operator* (Real fScalar, const Vector3<Real>& rkV)
	{
		return Vector3<Real>(
			fScalar*rkV[0],
			fScalar*rkV[1],
			fScalar*rkV[2]);
	}

	typedef Math<float> Mathf;
	typedef Math<double> Mathd;
	typedef Vector2<float> Vector2f;
	typedef Vector2<double> Vector2d;
	typedef Vector3<float> Vector3f;
	typedef Vector3<double> Vector3d;
}

#endif
