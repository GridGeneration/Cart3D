#include "Wm4Math.h"
#include <iostream>
#include <cmath>
#include <assert.h>
namespace Sn3DAlgorithm
{
	//----------------------------------------------------------------------------
	template <class Real>
	Real Math<Real>::ACos(Real fValue)
	{
		if (-(Real)1.0 < fValue)
		{
			if (fValue < (Real)1.0)
			{
				return (Real)acos((double)fValue);
			}
			else
			{
				return (Real)0.0;
			}
		}
		else
		{
			return PI;
		}
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Real Math<Real>::ASin(Real fValue)
	{
		if (-(Real)1.0 < fValue)
		{
			if (fValue < (Real)1.0)
			{
				return (Real)asin((double)fValue);
			}
			else
			{
				return HALF_PI;
			}
		}
		else
		{
			return -HALF_PI;
		}
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Real Math<Real>::ATan(Real fValue)
	{
		return (Real)atan((double)fValue);
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Real Math<Real>::ATan2(Real fY, Real fX)
	{
		return (Real)atan2((double)fY, (double)fX);
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Real Math<Real>::Ceil(Real fValue)
	{
		return (Real)ceil((double)fValue);
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Real Math<Real>::Cos(Real fValue)
	{
		return (Real)cos((double)fValue);
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Real Math<Real>::Exp(Real fValue)
	{
		return (Real)exp((double)fValue);
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Real Math<Real>::FAbs(Real fValue)
	{
		return (Real)fabs((double)fValue);
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Real Math<Real>::Floor(Real fValue)
	{
		return (Real)floor((double)fValue);
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Real Math<Real>::FMod(Real fX, Real fY)
	{
		return (Real)fmod((double)fX, (double)fY);
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Real Math<Real>::InvSqrt(Real fValue)
	{
		return (Real)(1.0 / sqrt((double)fValue));
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Real Math<Real>::Log(Real fValue)
	{
		return (Real)log((double)fValue);
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Real Math<Real>::Log2(Real fValue)
	{
		return Math<Real>::INV_LN_2 * (Real)log((double)fValue);
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Real Math<Real>::Log10(Real fValue)
	{
		return Math<Real>::INV_LN_10 * (Real)log((double)fValue);
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Real Math<Real>::Pow(Real fBase, Real fExponent)
	{
		return (Real)pow((double)fBase, (double)fExponent);
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Real Math<Real>::Sin(Real fValue)
	{
		return (Real)sin((double)fValue);
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Real Math<Real>::Sqr(Real fValue)
	{
		return fValue * fValue;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Real Math<Real>::Sqrt(Real fValue)
	{
		return (Real)sqrt((double)fValue);
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Real Math<Real>::Tan(Real fValue)
	{
		return (Real)tan((double)fValue);
	}
	//----------------------------------------------------------------------------
	template <class Real>
	int Math<Real>::Sign(int iValue)
	{
		if (iValue > 0)
		{
			return 1;
		}

		if (iValue < 0)
		{
			return -1;
		}

		return 0;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Real Math<Real>::Sign(Real fValue)
	{
		if (fValue > (Real)0.0)
		{
			return (Real)1.0;
		}

		if (fValue < (Real)0.0)
		{
			return -(Real)1.0;
		}

		return (Real)0.0;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Real Math<Real>::UnitRandom(unsigned int uiSeed)
	{
		if (uiSeed > 0)
		{
			srand(uiSeed);
		}

		double dRatio = ((double)rand()) / ((double)(RAND_MAX));
		return (Real)dRatio;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Real Math<Real>::SymmetricRandom(unsigned int uiSeed)
	{
		if (uiSeed > 0.0)
		{
			srand(uiSeed);
		}

		double dRatio = ((double)rand()) / ((double)(RAND_MAX));
		return (Real)(2.0*dRatio - 1.0);
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Real Math<Real>::IntervalRandom(Real fMin, Real fMax, unsigned int uiSeed)
	{
		if (uiSeed > 0)
		{
			srand(uiSeed);
		}

		double dRatio = ((double)rand()) / ((double)(RAND_MAX));
		return fMin + (fMax - fMin)*((Real)dRatio);
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Real Math<Real>::FastSin0(Real fAngle)
	{
		Real fASqr = fAngle * fAngle;
		Real fResult = (Real)7.61e-03;
		fResult *= fASqr;
		fResult -= (Real)1.6605e-01;
		fResult *= fASqr;
		fResult += (Real)1.0;
		fResult *= fAngle;
		return fResult;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Real Math<Real>::FastSin1(Real fAngle)
	{
		Real fASqr = fAngle * fAngle;
		Real fResult = -(Real)2.39e-08;
		fResult *= fASqr;
		fResult += (Real)2.7526e-06;
		fResult *= fASqr;
		fResult -= (Real)1.98409e-04;
		fResult *= fASqr;
		fResult += (Real)8.3333315e-03;
		fResult *= fASqr;
		fResult -= (Real)1.666666664e-01;
		fResult *= fASqr;
		fResult += (Real)1.0;
		fResult *= fAngle;
		return fResult;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Real Math<Real>::FastCos0(Real fAngle)
	{
		Real fASqr = fAngle * fAngle;
		Real fResult = (Real)3.705e-02;
		fResult *= fASqr;
		fResult -= (Real)4.967e-01;
		fResult *= fASqr;
		fResult += (Real)1.0;
		return fResult;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Real Math<Real>::FastCos1(Real fAngle)
	{
		Real fASqr = fAngle * fAngle;
		Real fResult = -(Real)2.605e-07;
		fResult *= fASqr;
		fResult += (Real)2.47609e-05;
		fResult *= fASqr;
		fResult -= (Real)1.3888397e-03;
		fResult *= fASqr;
		fResult += (Real)4.16666418e-02;
		fResult *= fASqr;
		fResult -= (Real)4.999999963e-01;
		fResult *= fASqr;
		fResult += (Real)1.0;
		return fResult;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Real Math<Real>::FastTan0(Real fAngle)
	{
		Real fASqr = fAngle * fAngle;
		Real fResult = (Real)2.033e-01;
		fResult *= fASqr;
		fResult += (Real)3.1755e-01;
		fResult *= fASqr;
		fResult += (Real)1.0;
		fResult *= fAngle;
		return fResult;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Real Math<Real>::FastTan1(Real fAngle)
	{
		Real fASqr = fAngle * fAngle;
		Real fResult = (Real)9.5168091e-03;
		fResult *= fASqr;
		fResult += (Real)2.900525e-03;
		fResult *= fASqr;
		fResult += (Real)2.45650893e-02;
		fResult *= fASqr;
		fResult += (Real)5.33740603e-02;
		fResult *= fASqr;
		fResult += (Real)1.333923995e-01;
		fResult *= fASqr;
		fResult += (Real)3.333314036e-01;
		fResult *= fASqr;
		fResult += (Real)1.0;
		fResult *= fAngle;
		return fResult;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Real Math<Real>::FastInvSin0(Real fValue)
	{
		Real fRoot = Math<Real>::Sqrt(((Real)1.0) - fValue);
		Real fResult = -(Real)0.0187293;
		fResult *= fValue;
		fResult += (Real)0.0742610;
		fResult *= fValue;
		fResult -= (Real)0.2121144;
		fResult *= fValue;
		fResult += (Real)1.5707288;
		fResult = HALF_PI - fRoot * fResult;
		return fResult;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Real Math<Real>::FastInvSin1(Real fValue)
	{
		Real fRoot = Math<Real>::Sqrt(FAbs(((Real)1.0) - fValue));
		Real fResult = -(Real)0.0012624911;
		fResult *= fValue;
		fResult += (Real)0.0066700901;
		fResult *= fValue;
		fResult -= (Real)0.0170881256;
		fResult *= fValue;
		fResult += (Real)0.0308918810;
		fResult *= fValue;
		fResult -= (Real)0.0501743046;
		fResult *= fValue;
		fResult += (Real)0.0889789874;
		fResult *= fValue;
		fResult -= (Real)0.2145988016;
		fResult *= fValue;
		fResult += (Real)1.5707963050;
		fResult = HALF_PI - fRoot * fResult;
		return fResult;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Real Math<Real>::FastInvCos0(Real fValue)
	{
		Real fRoot = Math<Real>::Sqrt(((Real)1.0) - fValue);
		Real fResult = -(Real)0.0187293;
		fResult *= fValue;
		fResult += (Real)0.0742610;
		fResult *= fValue;
		fResult -= (Real)0.2121144;
		fResult *= fValue;
		fResult += (Real)1.5707288;
		fResult *= fRoot;
		return fResult;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Real Math<Real>::FastInvCos1(Real fValue)
	{
		Real fRoot = Math<Real>::Sqrt(FAbs(((Real)1.0) - fValue));
		Real fResult = -(Real)0.0012624911;
		fResult *= fValue;
		fResult += (Real)0.0066700901;
		fResult *= fValue;
		fResult -= (Real)0.0170881256;
		fResult *= fValue;
		fResult += (Real)0.0308918810;
		fResult *= fValue;
		fResult -= (Real)0.0501743046;
		fResult *= fValue;
		fResult += (Real)0.0889789874;
		fResult *= fValue;
		fResult -= (Real)0.2145988016;
		fResult *= fValue;
		fResult += (Real)1.5707963050;
		fResult *= fRoot;
		return fResult;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Real Math<Real>::FastInvTan0(Real fValue)
	{
		Real fVSqr = fValue * fValue;
		Real fResult = (Real)0.0208351;
		fResult *= fVSqr;
		fResult -= (Real)0.085133;
		fResult *= fVSqr;
		fResult += (Real)0.180141;
		fResult *= fVSqr;
		fResult -= (Real)0.3302995;
		fResult *= fVSqr;
		fResult += (Real)0.999866;
		fResult *= fValue;
		return fResult;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Real Math<Real>::FastInvTan1(Real fValue)
	{
		Real fVSqr = fValue * fValue;
		Real fResult = (Real)0.0028662257;
		fResult *= fVSqr;
		fResult -= (Real)0.0161657367;
		fResult *= fVSqr;
		fResult += (Real)0.0429096138;
		fResult *= fVSqr;
		fResult -= (Real)0.0752896400;
		fResult *= fVSqr;
		fResult += (Real)0.1065626393;
		fResult *= fVSqr;
		fResult -= (Real)0.1420889944;
		fResult *= fVSqr;
		fResult += (Real)0.1999355085;
		fResult *= fVSqr;
		fResult -= (Real)0.3333314528;
		fResult *= fVSqr;
		fResult += (Real)1.0;
		fResult *= fValue;
		return fResult;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Real Math<Real>::FastNegExp0(Real fValue)
	{
		Real fResult = (Real)0.0038278;
		fResult *= fValue;
		fResult += (Real)0.0292732;
		fResult *= fValue;
		fResult += (Real)0.2507213;
		fResult *= fValue;
		fResult += (Real)1.0;
		fResult *= fResult;
		fResult *= fResult;
		fResult = ((Real)1.0) / fResult;
		return fResult;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Real Math<Real>::FastNegExp1(Real fValue)
	{
		Real fResult = (Real)0.00026695;
		fResult *= fValue;
		fResult += (Real)0.00227723;
		fResult *= fValue;
		fResult += (Real)0.03158565;
		fResult *= fValue;
		fResult += (Real)0.24991035;
		fResult *= fValue;
		fResult += (Real)1.0;
		fResult *= fResult;
		fResult *= fResult;
		fResult = ((Real)1.0) / fResult;
		return fResult;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Real Math<Real>::FastNegExp2(Real fValue)
	{
		Real fResult = (Real)0.000014876;
		fResult *= fValue;
		fResult += (Real)0.000127992;
		fResult *= fValue;
		fResult += (Real)0.002673255;
		fResult *= fValue;
		fResult += (Real)0.031198056;
		fResult *= fValue;
		fResult += (Real)0.250010936;
		fResult *= fValue;
		fResult += (Real)1.0;
		fResult *= fResult;
		fResult *= fResult;
		fResult = ((Real)1.0) / fResult;
		return fResult;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Real Math<Real>::FastNegExp3(Real fValue)
	{
		Real fResult = (Real)0.0000006906;
		fResult *= fValue;
		fResult += (Real)0.0000054302;
		fResult *= fValue;
		fResult += (Real)0.0001715620;
		fResult *= fValue;
		fResult += (Real)0.0025913712;
		fResult *= fValue;
		fResult += (Real)0.0312575832;
		fResult *= fValue;
		fResult += (Real)0.2499986842;
		fResult *= fValue;
		fResult += (Real)1.0;
		fResult *= fResult;
		fResult *= fResult;
		fResult = ((Real)1.0) / fResult;
		return fResult;
	}
	//----------------------------------------------------------------------------
	template<> const float Math<float>::EPSILON = FLT_EPSILON;
	template<> const float Math<float>::ZERO_TOLERANCE = 1e-04f;
	template<> const float Math<float>::MAX_REAL = FLT_MAX;
	template<> const float Math<float>::PI = (float)(4.0*atan(1.0));
	template<> const float Math<float>::TWO_PI = 2.0f*Math<float>::PI;
	template<> const float Math<float>::HALF_PI = 0.5f*Math<float>::PI;
	template<> const float Math<float>::INV_PI = 1.0f / Math<float>::PI;
	template<> const float Math<float>::INV_TWO_PI = 1.0f / Math<float>::TWO_PI;
	template<> const float Math<float>::DEG_TO_RAD = Math<float>::PI / 180.0f;
	template<> const float Math<float>::RAD_TO_DEG = 180.0f / Math<float>::PI;
	template<> const float Math<float>::LN_2 = Math<float>::Log(2.0f);
	template<> const float Math<float>::LN_10 = Math<float>::Log(10.0f);
	template<> const float Math<float>::INV_LN_2 = 1.0f / Math<float>::LN_2;
	template<> const float Math<float>::INV_LN_10 = 1.0f / Math<float>::LN_10;

	template<> const double Math<double>::EPSILON = DBL_EPSILON;
	template<> const double Math<double>::ZERO_TOLERANCE = 1e-05;
	template<> const double Math<double>::MAX_REAL = DBL_MAX;
	template<> const double Math<double>::PI = 4.0*atan(1.0);
	template<> const double Math<double>::TWO_PI = 2.0*Math<double>::PI;
	template<> const double Math<double>::HALF_PI = 0.5*Math<double>::PI;
	template<> const double Math<double>::INV_PI = 1.0 / Math<double>::PI;
	template<> const double Math<double>::INV_TWO_PI = 1.0 / Math<double>::TWO_PI;
	template<> const double Math<double>::DEG_TO_RAD = Math<double>::PI / 180.0;
	template<> const double Math<double>::RAD_TO_DEG = 180.0 / Math<double>::PI;
	template<> const double Math<double>::LN_2 = Math<double>::Log(2.0);
	template<> const double Math<double>::LN_10 = Math<double>::Log(10.0);
	template<> const double Math<double>::INV_LN_2 = 1.0 / Math<double>::LN_2;
	template<> const double Math<double>::INV_LN_10 = 1.0 / Math<double>::LN_10;

	//----------------------------------------------------------------------------
	template <>
	inline float Math<float>::FastInvSqrt(float fValue)
	{
		float fHalf = 0.5f*fValue;
		int i = *(int*)&fValue;
		i = 0x5f3759df - (i >> 1);
		fValue = *(float*)&i;
		fValue = fValue * (1.5f - fHalf * fValue*fValue);
		return fValue;
	}
	//----------------------------------------------------------------------------
	template <>
	inline double Math<double>::FastInvSqrt(double dValue)
	{
		double dHalf = 0.5*dValue;
		long long i = *(long long*)&dValue;
#if defined(WM4_USING_VC70) || defined(WM4_USING_VC6)
		i = 0x5fe6ec85e7de30da - (i >> 1);
#else
		i = 0x5fe6ec85e7de30daLL - (i >> 1);
#endif
		dValue = *(double*)&i;
		dValue = dValue * (1.5 - dHalf * dValue*dValue);
		return dValue;
	}
	//----------------------------------------------------------------------------


	//----------------------------------------------------------------------------
	template <class Real>
	Vector2<Real>::Vector2()
	{
		// uninitialized for performance in array construction
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Vector2<Real>::Vector2(Real fX, Real fY)
	{
		m_afTuple[0] = fX;
		m_afTuple[1] = fY;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Vector2<Real>::Vector2(const Real* afTuple)
	{
		m_afTuple[0] = afTuple[0];
		m_afTuple[1] = afTuple[1];
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Vector2<Real>::Vector2(const Vector2& rkV)
	{
		m_afTuple[0] = rkV.m_afTuple[0];
		m_afTuple[1] = rkV.m_afTuple[1];
	}
	//----------------------------------------------------------------------------
	template <class Real>
	inline Vector2<Real>::operator const Real* () const
	{
		return m_afTuple;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	inline Vector2<Real>::operator Real* ()
	{
		return m_afTuple;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	inline Real Vector2<Real>::operator[] (int i) const
	{
		return m_afTuple[i];
	}
	//----------------------------------------------------------------------------
	template <class Real>
	inline Real& Vector2<Real>::operator[] (int i)
	{
		return m_afTuple[i];
	}
	//----------------------------------------------------------------------------
	template <class Real>
	inline Real Vector2<Real>::X() const
	{
		return m_afTuple[0];
	}
	//----------------------------------------------------------------------------
	template <class Real>
	inline Real& Vector2<Real>::X()
	{
		return m_afTuple[0];
	}
	//----------------------------------------------------------------------------
	template <class Real>
	inline Real Vector2<Real>::Y() const
	{
		return m_afTuple[1];
	}
	//----------------------------------------------------------------------------
	template <class Real>
	inline Real& Vector2<Real>::Y()
	{
		return m_afTuple[1];
	}
	//----------------------------------------------------------------------------
	template <class Real>
	inline Vector2<Real>& Vector2<Real>::operator= (const Vector2& rkV)
	{
		m_afTuple[0] = rkV.m_afTuple[0];
		m_afTuple[1] = rkV.m_afTuple[1];
		return *this;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	int Vector2<Real>::CompareArrays(const Vector2& rkV) const
	{
		return memcmp(m_afTuple, rkV.m_afTuple, 2 * sizeof(Real));
	}
	//----------------------------------------------------------------------------
	template <class Real>
	bool Vector2<Real>::operator== (const Vector2& rkV) const
	{
		return CompareArrays(rkV) == 0;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	bool Vector2<Real>::operator!= (const Vector2& rkV) const
	{
		return CompareArrays(rkV) != 0;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	bool Vector2<Real>::operator< (const Vector2& rkV) const
	{
		return CompareArrays(rkV) < 0;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	bool Vector2<Real>::operator<= (const Vector2& rkV) const
	{
		return CompareArrays(rkV) <= 0;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	bool Vector2<Real>::operator> (const Vector2& rkV) const
	{
		return CompareArrays(rkV) > 0;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	bool Vector2<Real>::operator>= (const Vector2& rkV) const
	{
		return CompareArrays(rkV) >= 0;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	inline Vector2<Real> Vector2<Real>::operator+ (const Vector2& rkV) const
	{
		return Vector2(
			m_afTuple[0] + rkV.m_afTuple[0],
			m_afTuple[1] + rkV.m_afTuple[1]);
	}
	//----------------------------------------------------------------------------
	template <class Real>
	inline Vector2<Real> Vector2<Real>::operator- (const Vector2& rkV) const
	{
		return Vector2(
			m_afTuple[0] - rkV.m_afTuple[0],
			m_afTuple[1] - rkV.m_afTuple[1]);
	}
	//----------------------------------------------------------------------------
	template <class Real>
	inline Vector2<Real> Vector2<Real>::operator* (Real fScalar) const
	{
		return Vector2(
			fScalar*m_afTuple[0],
			fScalar*m_afTuple[1]);
	}
	//----------------------------------------------------------------------------
	template <class Real>
	inline Vector2<Real> Vector2<Real>::operator/ (Real fScalar) const
	{
		Vector2 kQuot;

		if (fScalar != (Real)0.0)
		{
			Real fInvScalar = ((Real)1.0) / fScalar;
			kQuot.m_afTuple[0] = fInvScalar * m_afTuple[0];
			kQuot.m_afTuple[1] = fInvScalar * m_afTuple[1];
		}
		else
		{
			kQuot.m_afTuple[0] = Math<Real>::MAX_REAL;
			kQuot.m_afTuple[1] = Math<Real>::MAX_REAL;
		}

		return kQuot;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	inline Vector2<Real> Vector2<Real>::operator- () const
	{
		return Vector2(
			-m_afTuple[0],
			-m_afTuple[1]);
	}
	//----------------------------------------------------------------------------

	//----------------------------------------------------------------------------
	template <class Real>
	inline Vector2<Real>& Vector2<Real>::operator+= (const Vector2& rkV)
	{
		m_afTuple[0] += rkV.m_afTuple[0];
		m_afTuple[1] += rkV.m_afTuple[1];
		return *this;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	inline Vector2<Real>& Vector2<Real>::operator-= (const Vector2& rkV)
	{
		m_afTuple[0] -= rkV.m_afTuple[0];
		m_afTuple[1] -= rkV.m_afTuple[1];
		return *this;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	inline Vector2<Real>& Vector2<Real>::operator*= (Real fScalar)
	{
		m_afTuple[0] *= fScalar;
		m_afTuple[1] *= fScalar;
		return *this;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	inline Vector2<Real>& Vector2<Real>::operator/= (Real fScalar)
	{
		if (fScalar != (Real)0.0)
		{
			Real fInvScalar = ((Real)1.0) / fScalar;
			m_afTuple[0] *= fInvScalar;
			m_afTuple[1] *= fInvScalar;
		}
		else
		{
			m_afTuple[0] = Math<Real>::MAX_REAL;
			m_afTuple[1] = Math<Real>::MAX_REAL;
		}

		return *this;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	inline Real Vector2<Real>::Length() const
	{
		return Math<Real>::Sqrt(
			m_afTuple[0] * m_afTuple[0] +
			m_afTuple[1] * m_afTuple[1]);
	}
	//----------------------------------------------------------------------------
	template <class Real>
	inline Real Vector2<Real>::SquaredLength() const
	{
		return
			m_afTuple[0] * m_afTuple[0] +
			m_afTuple[1] * m_afTuple[1];
	}
	//----------------------------------------------------------------------------
	template <class Real>
	inline Real Vector2<Real>::Dot(const Vector2& rkV) const
	{
		return
			m_afTuple[0] * rkV.m_afTuple[0] +
			m_afTuple[1] * rkV.m_afTuple[1];
	}
	//----------------------------------------------------------------------------
	template <class Real>
	inline Real Vector2<Real>::Normalize()
	{
		Real fLength = Length();

		if (fLength > Math<Real>::ZERO_TOLERANCE)
		{
			Real fInvLength = ((Real)1.0) / fLength;
			m_afTuple[0] *= fInvLength;
			m_afTuple[1] *= fInvLength;
		}
		else
		{
			fLength = (Real)0.0;
			m_afTuple[0] = (Real)0.0;
			m_afTuple[1] = (Real)0.0;
		}

		return fLength;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	inline Vector2<Real> Vector2<Real>::Perp() const
	{
		return Vector2(m_afTuple[1], -m_afTuple[0]);
	}
	//----------------------------------------------------------------------------
	template <class Real>
	inline Vector2<Real> Vector2<Real>::UnitPerp() const
	{
		Vector2 kPerp(m_afTuple[1], -m_afTuple[0]);
		kPerp.Normalize();
		return kPerp;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	inline Real Vector2<Real>::DotPerp(const Vector2& rkV) const
	{
		return m_afTuple[0] * rkV.m_afTuple[1] - m_afTuple[1] * rkV.m_afTuple[0];
	}
	//----------------------------------------------------------------------------
	template <class Real>
	void Vector2<Real>::GetBarycentrics(const Vector2& rkV0, const Vector2& rkV1,
		const Vector2& rkV2, Real afBary[3]) const
	{
		// Compute the vectors relative to V2 of the triangle.
		Vector2 akDiff[3] =
		{
			rkV0 - rkV2,
			rkV1 - rkV2,
			*this - rkV2
		};

		// If the vertices have large magnitude, the linear system of equations
		// for computing barycentric coordinates can be ill-conditioned.  To avoid
		// this, uniformly scale the triangle edges to be of order 1.  The scaling
		// of all differences does not change the barycentric coordinates.
		Real fMax = (Real)0.0;
		int i;
		for (i = 0; i < 2; i++)
		{
			for (int j = 0; j < 2; j++)
			{
				Real fValue = Math<Real>::FAbs(akDiff[i][j]);
				if (fValue > fMax)
				{
					fMax = fValue;
				}
			}
		}

		// Scale down only large data.
		Real fInvMax = (Real)0.0;
		if (fMax > (Real)1.0)
		{
			fInvMax = ((Real)1.0) / fMax;
			for (i = 0; i < 3; i++)
			{
				akDiff[i] *= fInvMax;
			}
		}

		Real fDet = akDiff[0].DotPerp(akDiff[1]);
		if (Math<Real>::FAbs(fDet) > Math<Real>::ZERO_TOLERANCE)
		{
			Real fInvDet = ((Real)1.0) / fDet;
			afBary[0] = akDiff[2].DotPerp(akDiff[1])*fInvDet;
			afBary[1] = akDiff[0].DotPerp(akDiff[2])*fInvDet;
			afBary[2] = (Real)1.0 - afBary[0] - afBary[1];
		}
		else
		{
			// The triangle is a sliver.  Determine the longest edge and
			// compute barycentric coordinates with respect to that edge.
			Vector2 kE2 = rkV0 - rkV1;
			if (fInvMax != (Real)0.0)
			{
				kE2 *= fInvMax;
			}

			Real fMaxSqrLength = kE2.SquaredLength();
			int iMaxIndex = 2;
			Real fSqrLength = akDiff[1].SquaredLength();
			if (fSqrLength > fMaxSqrLength)
			{
				iMaxIndex = 1;
				fMaxSqrLength = fSqrLength;
			}
			fSqrLength = akDiff[0].SquaredLength();
			if (fSqrLength > fMaxSqrLength)
			{
				iMaxIndex = 0;
				fMaxSqrLength = fSqrLength;
			}

			if (fMaxSqrLength > Math<Real>::ZERO_TOLERANCE)
			{
				Real fInvSqrLength = ((Real)1.0) / fMaxSqrLength;
				if (iMaxIndex == 0)
				{
					// P-V2 = t(V0-V2)
					afBary[0] = akDiff[2].Dot(akDiff[0])*fInvSqrLength;
					afBary[1] = (Real)0.0;
					afBary[2] = (Real)1.0 - afBary[0];
				}
				else if (iMaxIndex == 1)
				{
					// P-V2 = t(V1-V2)
					afBary[0] = (Real)0.0;
					afBary[1] = akDiff[2].Dot(akDiff[1])*fInvSqrLength;
					afBary[2] = (Real)1.0 - afBary[1];
				}
				else
				{
					// P-V1 = t(V0-V1)
					akDiff[2] = *this - rkV1;
					if (fInvMax != (Real)0.0)
					{
						akDiff[2] *= fInvMax;
					}

					afBary[0] = akDiff[2].Dot(kE2)*fInvSqrLength;
					afBary[1] = (Real)1.0 - afBary[0];
					afBary[2] = (Real)0.0;
				}
			}
			else
			{
				// The triangle is a nearly a point, just return equal weights.
				afBary[0] = ((Real)1.0) / (Real)3.0;
				afBary[1] = afBary[0];
				afBary[2] = afBary[0];
			}
		}
	}
	//----------------------------------------------------------------------------
	template <class Real>
	void Vector2<Real>::Orthonormalize(Vector2& rkU, Vector2& rkV)
	{
		// If the input vectors are v0 and v1, then the Gram-Schmidt
		// orthonormalization produces vectors u0 and u1 as follows,
		//
		//   u0 = v0/|v0|
		//   u1 = (v1-(u0*v1)u0)/|v1-(u0*v1)u0|
		//
		// where |A| indicates length of vector A and A*B indicates dot
		// product of vectors A and B.

		// compute u0
		rkU.Normalize();

		// compute u1
		Real fDot0 = rkU.Dot(rkV);
		rkV -= rkU * fDot0;
		rkV.Normalize();
	}
	//----------------------------------------------------------------------------
	template <class Real>
	void Vector2<Real>::GenerateOrthonormalBasis(Vector2& rkU, Vector2& rkV)
	{
		rkV.Normalize();
		rkU = rkV.Perp();
	}
	//----------------------------------------------------------------------------
	template <class Real>
	void Vector2<Real>::ComputeExtremes(int iVQuantity, const Vector2* akPoint,
		Vector2& rkMin, Vector2& rkMax)
	{
		assert(iVQuantity > 0 && akPoint);

		rkMin = akPoint[0];
		rkMax = rkMin;
		for (int i = 1; i < iVQuantity; i++)
		{
			const Vector2<Real>& rkPoint = akPoint[i];
			for (int j = 0; j < 2; j++)
			{
				if (rkPoint[j] < rkMin[j])
				{
					rkMin[j] = rkPoint[j];
				}
				else if (rkPoint[j] > rkMax[j])
				{
					rkMax[j] = rkPoint[j];
				}
			}
		}
	}
	//----------------------------------------------------------------------------
	
	//----------------------------------------------------------------------------
	template<> const Vector2<float> Vector2<float>::ZERO(0.0f, 0.0f);
	template<> const Vector2<float> Vector2<float>::UNIT_X(1.0f, 0.0f);
	template<> const Vector2<float> Vector2<float>::UNIT_Y(0.0f, 1.0f);
	template<> const Vector2<float> Vector2<float>::ONE(1.0f, 1.0f);

	template<> const Vector2<double> Vector2<double>::ZERO(0.0, 0.0);
	template<> const Vector2<double> Vector2<double>::UNIT_X(1.0, 0.0);
	template<> const Vector2<double> Vector2<double>::UNIT_Y(0.0, 1.0);
	template<> const Vector2<double> Vector2<double>::ONE(1.0, 1.0);
	//----------------------------------------------------------------------------


	//----------------------------------------------------------------------------
	template <class Real>
	Vector3<Real>::Vector3()
	{
		// uninitialized for performance in array construction
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Vector3<Real>::Vector3(Real fX, Real fY, Real fZ)
	{
		m_afTuple[0] = fX;
		m_afTuple[1] = fY;
		m_afTuple[2] = fZ;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Vector3<Real>::Vector3(const Real* afTuple)
	{
		m_afTuple[0] = afTuple[0];
		m_afTuple[1] = afTuple[1];
		m_afTuple[2] = afTuple[2];
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Vector3<Real>::Vector3(const Vector3& rkV)
	{
		m_afTuple[0] = rkV.m_afTuple[0];
		m_afTuple[1] = rkV.m_afTuple[1];
		m_afTuple[2] = rkV.m_afTuple[2];
	}
	//----------------------------------------------------------------------------
	template <class Real>
	inline Vector3<Real>::operator const Real* () const
	{
		return m_afTuple;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	inline Vector3<Real>::operator Real* ()
	{
		return m_afTuple;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	inline Real Vector3<Real>::operator[] (int i) const
	{
		return m_afTuple[i];
	}
	//----------------------------------------------------------------------------
	template <class Real>
	inline Real& Vector3<Real>::operator[] (int i)
	{
		return m_afTuple[i];
	}
	//----------------------------------------------------------------------------
	template <class Real>
	inline Real Vector3<Real>::X() const
	{
		return m_afTuple[0];
	}
	//----------------------------------------------------------------------------
	template <class Real>
	inline Real& Vector3<Real>::X()
	{
		return m_afTuple[0];
	}
	//----------------------------------------------------------------------------
	template <class Real>
	inline Real Vector3<Real>::Y() const
	{
		return m_afTuple[1];
	}
	//----------------------------------------------------------------------------
	template <class Real>
	inline Real& Vector3<Real>::Y()
	{
		return m_afTuple[1];
	}
	//----------------------------------------------------------------------------
	template <class Real>
	inline Real Vector3<Real>::Z() const
	{
		return m_afTuple[2];
	}
	//----------------------------------------------------------------------------
	template <class Real>
	inline Real& Vector3<Real>::Z()
	{
		return m_afTuple[2];
	}
	//----------------------------------------------------------------------------
	template <class Real>
	inline Vector3<Real>& Vector3<Real>::operator= (const Vector3& rkV)
	{
		m_afTuple[0] = rkV.m_afTuple[0];
		m_afTuple[1] = rkV.m_afTuple[1];
		m_afTuple[2] = rkV.m_afTuple[2];
		return *this;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	int Vector3<Real>::CompareArrays(const Vector3& rkV) const
	{
		return memcmp(m_afTuple, rkV.m_afTuple, 3 * sizeof(Real));
	}
	//----------------------------------------------------------------------------
	template <class Real>
	bool Vector3<Real>::operator== (const Vector3& rkV) const
	{
		return CompareArrays(rkV) == 0;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	bool Vector3<Real>::operator!= (const Vector3& rkV) const
	{
		return CompareArrays(rkV) != 0;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	bool Vector3<Real>::operator< (const Vector3& rkV) const
	{
		return CompareArrays(rkV) < 0;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	bool Vector3<Real>::operator<= (const Vector3& rkV) const
	{
		return CompareArrays(rkV) <= 0;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	bool Vector3<Real>::operator> (const Vector3& rkV) const
	{
		return CompareArrays(rkV) > 0;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	bool Vector3<Real>::operator>= (const Vector3& rkV) const
	{
		return CompareArrays(rkV) >= 0;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	inline Vector3<Real> Vector3<Real>::operator+ (const Vector3& rkV) const
	{
		return Vector3(
			m_afTuple[0] + rkV.m_afTuple[0],
			m_afTuple[1] + rkV.m_afTuple[1],
			m_afTuple[2] + rkV.m_afTuple[2]);
	}
	//----------------------------------------------------------------------------
	template <class Real>
	inline Vector3<Real> Vector3<Real>::operator- (const Vector3& rkV) const
	{
		return Vector3(
			m_afTuple[0] - rkV.m_afTuple[0],
			m_afTuple[1] - rkV.m_afTuple[1],
			m_afTuple[2] - rkV.m_afTuple[2]);
	}
	//----------------------------------------------------------------------------
	template <class Real>
	inline Vector3<Real> Vector3<Real>::operator* (Real fScalar) const
	{
		return Vector3(
			fScalar*m_afTuple[0],
			fScalar*m_afTuple[1],
			fScalar*m_afTuple[2]);
	}
	//----------------------------------------------------------------------------
	template <class Real>
	inline Vector3<Real> Vector3<Real>::operator/ (Real fScalar) const
	{
		Vector3 kQuot;

		if (fScalar != (Real)0.0)
		{
			Real fInvScalar = ((Real)1.0) / fScalar;
			kQuot.m_afTuple[0] = fInvScalar * m_afTuple[0];
			kQuot.m_afTuple[1] = fInvScalar * m_afTuple[1];
			kQuot.m_afTuple[2] = fInvScalar * m_afTuple[2];
		}
		else
		{
			kQuot.m_afTuple[0] = Math<Real>::MAX_REAL;
			kQuot.m_afTuple[1] = Math<Real>::MAX_REAL;
			kQuot.m_afTuple[2] = Math<Real>::MAX_REAL;
		}

		return kQuot;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	inline Vector3<Real> Vector3<Real>::operator- () const
	{
		return Vector3(
			-m_afTuple[0],
			-m_afTuple[1],
			-m_afTuple[2]);
	}
	//----------------------------------------------------------------------------
	
	//----------------------------------------------------------------------------
	template <class Real>
	inline Vector3<Real>& Vector3<Real>::operator+= (const Vector3& rkV)
	{
		m_afTuple[0] += rkV.m_afTuple[0];
		m_afTuple[1] += rkV.m_afTuple[1];
		m_afTuple[2] += rkV.m_afTuple[2];
		return *this;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	inline Vector3<Real>& Vector3<Real>::operator-= (const Vector3& rkV)
	{
		m_afTuple[0] -= rkV.m_afTuple[0];
		m_afTuple[1] -= rkV.m_afTuple[1];
		m_afTuple[2] -= rkV.m_afTuple[2];
		return *this;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	inline Vector3<Real>& Vector3<Real>::operator*= (Real fScalar)
	{
		m_afTuple[0] *= fScalar;
		m_afTuple[1] *= fScalar;
		m_afTuple[2] *= fScalar;
		return *this;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	inline Vector3<Real>& Vector3<Real>::operator/= (Real fScalar)
	{
		if (fScalar != (Real)0.0)
		{
			Real fInvScalar = ((Real)1.0) / fScalar;
			m_afTuple[0] *= fInvScalar;
			m_afTuple[1] *= fInvScalar;
			m_afTuple[2] *= fInvScalar;
		}
		else
		{
			m_afTuple[0] = Math<Real>::MAX_REAL;
			m_afTuple[1] = Math<Real>::MAX_REAL;
			m_afTuple[2] = Math<Real>::MAX_REAL;
		}

		return *this;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	inline Real Vector3<Real>::Length() const
	{
		return Math<Real>::Sqrt(
			m_afTuple[0] * m_afTuple[0] +
			m_afTuple[1] * m_afTuple[1] +
			m_afTuple[2] * m_afTuple[2]);
	}
	//----------------------------------------------------------------------------
	template <class Real>
	inline Real Vector3<Real>::SquaredLength() const
	{
		return
			m_afTuple[0] * m_afTuple[0] +
			m_afTuple[1] * m_afTuple[1] +
			m_afTuple[2] * m_afTuple[2];
	}
	//----------------------------------------------------------------------------
	template <class Real>
	inline Real Vector3<Real>::Dot(const Vector3& rkV) const
	{
		return
			m_afTuple[0] * rkV.m_afTuple[0] +
			m_afTuple[1] * rkV.m_afTuple[1] +
			m_afTuple[2] * rkV.m_afTuple[2];
	}
	//----------------------------------------------------------------------------
	template <class Real>
	inline Real Vector3<Real>::Normalize()
	{
		Real fLength = Length();

		if (fLength > Math<Real>::ZERO_TOLERANCE)
		{
			Real fInvLength = ((Real)1.0) / fLength;
			m_afTuple[0] *= fInvLength;
			m_afTuple[1] *= fInvLength;
			m_afTuple[2] *= fInvLength;
		}
		else
		{
			fLength = (Real)0.0;
			m_afTuple[0] = (Real)0.0;
			m_afTuple[1] = (Real)0.0;
			m_afTuple[2] = (Real)0.0;
		}

		return fLength;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	inline Vector3<Real> Vector3<Real>::Cross(const Vector3& rkV) const
	{
		return Vector3(
			m_afTuple[1] * rkV.m_afTuple[2] - m_afTuple[2] * rkV.m_afTuple[1],
			m_afTuple[2] * rkV.m_afTuple[0] - m_afTuple[0] * rkV.m_afTuple[2],
			m_afTuple[0] * rkV.m_afTuple[1] - m_afTuple[1] * rkV.m_afTuple[0]);
	}
	//----------------------------------------------------------------------------
	template <class Real>
	inline Vector3<Real> Vector3<Real>::UnitCross(const Vector3& rkV) const
	{
		Vector3 kCross(
			m_afTuple[1] * rkV.m_afTuple[2] - m_afTuple[2] * rkV.m_afTuple[1],
			m_afTuple[2] * rkV.m_afTuple[0] - m_afTuple[0] * rkV.m_afTuple[2],
			m_afTuple[0] * rkV.m_afTuple[1] - m_afTuple[1] * rkV.m_afTuple[0]);
		kCross.Normalize();
		return kCross;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	void Vector3<Real>::GetBarycentrics(const Vector3<Real>& rkV0,
		const Vector3<Real>& rkV1, const Vector3<Real>& rkV2,
		const Vector3<Real>& rkV3, Real afBary[4]) const
	{
		// Compute the vectors relative to V3 of the tetrahedron.
		Vector3<Real> akDiff[4] =
		{
			rkV0 - rkV3,
			rkV1 - rkV3,
			rkV2 - rkV3,
			*this - rkV3
		};

		// If the vertices have large magnitude, the linear system of
		// equations for computing barycentric coordinates can be
		// ill-conditioned.  To avoid this, uniformly scale the tetrahedron
		// edges to be of order 1.  The scaling of all differences does not
		// change the barycentric coordinates.
		Real fMax = (Real)0.0;
		int i;
		for (i = 0; i < 4; i++)
		{
			for (int j = 0; j < 3; j++)
			{
				Real fValue = Math<Real>::FAbs(akDiff[i][j]);
				if (fValue > fMax)
				{
					fMax = fValue;
				}
			}
		}

		// Scale down only large data.
		Real fInvMax = (Real)0.0;
		if (fMax > (Real)1.0)
		{
			fInvMax = ((Real)1.0) / fMax;
			for (i = 0; i < 4; i++)
			{
				akDiff[i] *= fInvMax;
			}
		}

		Real fDet = akDiff[0].Dot(akDiff[1].Cross(akDiff[2]));
		Vector3<Real> kE1cE2 = akDiff[1].Cross(akDiff[2]);
		Vector3<Real> kE2cE0 = akDiff[2].Cross(akDiff[0]);
		Vector3<Real> kE0cE1 = akDiff[0].Cross(akDiff[1]);
		if (Math<Real>::FAbs(fDet) > Math<Real>::ZERO_TOLERANCE)
		{
			Real fInvDet = ((Real)1.0) / fDet;
			afBary[0] = akDiff[3].Dot(kE1cE2)*fInvDet;
			afBary[1] = akDiff[3].Dot(kE2cE0)*fInvDet;
			afBary[2] = akDiff[3].Dot(kE0cE1)*fInvDet;
			afBary[3] = (Real)1.0 - afBary[0] - afBary[1] - afBary[2];
		}
		else
		{
			// The tetrahedron is potentially flat.  Determine the face of
			// maximum area and compute barycentric coordinates with respect
			// to that face.
			Vector3<Real> kE02 = rkV0 - rkV2;
			Vector3<Real> kE12 = rkV1 - rkV2;
			if (fInvMax != (Real)0.0)
			{
				kE02 *= fInvMax;
				kE12 *= fInvMax;
			}

			Vector3<Real> kE02cE12 = kE02.Cross(kE12);
			Real fMaxSqrArea = kE02cE12.SquaredLength();
			int iMaxIndex = 3;
			Real fSqrArea = kE0cE1.SquaredLength();
			if (fSqrArea > fMaxSqrArea)
			{
				iMaxIndex = 0;
				fMaxSqrArea = fSqrArea;
			}
			fSqrArea = kE1cE2.SquaredLength();
			if (fSqrArea > fMaxSqrArea)
			{
				iMaxIndex = 1;
				fMaxSqrArea = fSqrArea;
			}
			fSqrArea = kE2cE0.SquaredLength();
			if (fSqrArea > fMaxSqrArea)
			{
				iMaxIndex = 2;
				fMaxSqrArea = fSqrArea;
			}

			if (fMaxSqrArea > Math<Real>::ZERO_TOLERANCE)
			{
				Real fInvSqrArea = ((Real)1.0) / fMaxSqrArea;
				Vector3<Real> kTmp;
				if (iMaxIndex == 0)
				{
					kTmp = akDiff[3].Cross(akDiff[1]);
					afBary[0] = kE0cE1.Dot(kTmp)*fInvSqrArea;
					kTmp = akDiff[0].Cross(akDiff[3]);
					afBary[1] = kE0cE1.Dot(kTmp)*fInvSqrArea;
					afBary[2] = (Real)0.0;
					afBary[3] = (Real)1.0 - afBary[0] - afBary[1];
				}
				else if (iMaxIndex == 1)
				{
					afBary[0] = (Real)0.0;
					kTmp = akDiff[3].Cross(akDiff[2]);
					afBary[1] = kE1cE2.Dot(kTmp)*fInvSqrArea;
					kTmp = akDiff[1].Cross(akDiff[3]);
					afBary[2] = kE1cE2.Dot(kTmp)*fInvSqrArea;
					afBary[3] = (Real)1.0 - afBary[1] - afBary[2];
				}
				else if (iMaxIndex == 2)
				{
					kTmp = akDiff[2].Cross(akDiff[3]);
					afBary[0] = kE2cE0.Dot(kTmp)*fInvSqrArea;
					afBary[1] = (Real)0.0;
					kTmp = akDiff[3].Cross(akDiff[0]);
					afBary[2] = kE2cE0.Dot(kTmp)*fInvSqrArea;
					afBary[3] = (Real)1.0 - afBary[0] - afBary[2];
				}
				else
				{
					akDiff[3] = *this - rkV2;
					if (fInvMax != (Real)0.0)
					{
						akDiff[3] *= fInvMax;
					}

					kTmp = akDiff[3].Cross(kE12);
					afBary[0] = kE02cE12.Dot(kTmp)*fInvSqrArea;
					kTmp = kE02.Cross(akDiff[3]);
					afBary[1] = kE02cE12.Dot(kTmp)*fInvSqrArea;
					afBary[2] = (Real)1.0 - afBary[0] - afBary[1];
					afBary[3] = (Real)0.0;
				}
			}
			else
			{
				// The tetrahedron is potentially a sliver.  Determine the edge of
				// maximum length and compute barycentric coordinates with respect
				// to that edge.
				Real fMaxSqrLength = akDiff[0].SquaredLength();
				iMaxIndex = 0;  // <V0,V3>
				Real fSqrLength = akDiff[1].SquaredLength();
				if (fSqrLength > fMaxSqrLength)
				{
					iMaxIndex = 1;  // <V1,V3>
					fMaxSqrLength = fSqrLength;
				}
				fSqrLength = akDiff[2].SquaredLength();
				if (fSqrLength > fMaxSqrLength)
				{
					iMaxIndex = 2;  // <V2,V3>
					fMaxSqrLength = fSqrLength;
				}
				fSqrLength = kE02.SquaredLength();
				if (fSqrLength > fMaxSqrLength)
				{
					iMaxIndex = 3;  // <V0,V2>
					fMaxSqrLength = fSqrLength;
				}
				fSqrLength = kE12.SquaredLength();
				if (fSqrLength > fMaxSqrLength)
				{
					iMaxIndex = 4;  // <V1,V2>
					fMaxSqrLength = fSqrLength;
				}
				Vector3<Real> kE01 = rkV0 - rkV1;
				fSqrLength = kE01.SquaredLength();
				if (fSqrLength > fMaxSqrLength)
				{
					iMaxIndex = 5;  // <V0,V1>
					fMaxSqrLength = fSqrLength;
				}

				if (fMaxSqrLength > Math<Real>::ZERO_TOLERANCE)
				{
					Real fInvSqrLength = ((Real)1.0) / fMaxSqrLength;
					if (iMaxIndex == 0)
					{
						// P-V3 = t*(V0-V3)
						afBary[0] = akDiff[3].Dot(akDiff[0])*fInvSqrLength;
						afBary[1] = (Real)0.0;
						afBary[2] = (Real)0.0;
						afBary[3] = (Real)1.0 - afBary[0];
					}
					else if (iMaxIndex == 1)
					{
						// P-V3 = t*(V1-V3)
						afBary[0] = (Real)0.0;
						afBary[1] = akDiff[3].Dot(akDiff[1])*fInvSqrLength;
						afBary[2] = (Real)0.0;
						afBary[3] = (Real)1.0 - afBary[1];
					}
					else if (iMaxIndex == 2)
					{
						// P-V3 = t*(V2-V3)
						afBary[0] = (Real)0.0;
						afBary[1] = (Real)0.0;
						afBary[2] = akDiff[3].Dot(akDiff[2])*fInvSqrLength;
						afBary[3] = (Real)1.0 - afBary[2];
					}
					else if (iMaxIndex == 3)
					{
						// P-V2 = t*(V0-V2)
						akDiff[3] = *this - rkV2;
						if (fInvMax != (Real)0.0)
						{
							akDiff[3] *= fInvMax;
						}

						afBary[0] = akDiff[3].Dot(kE02)*fInvSqrLength;
						afBary[1] = (Real)0.0;
						afBary[2] = (Real)1.0 - afBary[0];
						afBary[3] = (Real)0.0;
					}
					else if (iMaxIndex == 4)
					{
						// P-V2 = t*(V1-V2)
						akDiff[3] = *this - rkV2;
						if (fInvMax != (Real)0.0)
						{
							akDiff[3] *= fInvMax;
						}

						afBary[0] = (Real)0.0;
						afBary[1] = akDiff[3].Dot(kE12)*fInvSqrLength;
						afBary[2] = (Real)1.0 - afBary[1];
						afBary[3] = (Real)0.0;
					}
					else
					{
						// P-V1 = t*(V0-V1)
						akDiff[3] = *this - rkV1;
						if (fInvMax != (Real)0.0)
						{
							akDiff[3] *= fInvMax;
						}

						afBary[0] = akDiff[3].Dot(kE01)*fInvSqrLength;
						afBary[1] = (Real)1.0 - afBary[0];
						afBary[2] = (Real)0.0;
						afBary[3] = (Real)0.0;
					}
				}
				else
				{
					// The tetrahedron is a nearly a point, just return equal
					// weights.
					afBary[0] = (Real)0.25;
					afBary[1] = afBary[0];
					afBary[2] = afBary[0];
					afBary[3] = afBary[0];
				}
			}
		}
	}
	//----------------------------------------------------------------------------
	template <class Real>
	void Vector3<Real>::Orthonormalize(Vector3& rkU, Vector3& rkV, Vector3& rkW)
	{
		// If the input vectors are v0, v1, and v2, then the Gram-Schmidt
		// orthonormalization produces vectors u0, u1, and u2 as follows,
		//
		//   u0 = v0/|v0|
		//   u1 = (v1-(u0*v1)u0)/|v1-(u0*v1)u0|
		//   u2 = (v2-(u0*v2)u0-(u1*v2)u1)/|v2-(u0*v2)u0-(u1*v2)u1|
		//
		// where |A| indicates length of vector A and A*B indicates dot
		// product of vectors A and B.

		// compute u0
		rkU.Normalize();

		// compute u1
		Real fDot0 = rkU.Dot(rkV);
		rkV -= fDot0 * rkU;
		rkV.Normalize();

		// compute u2
		Real fDot1 = rkV.Dot(rkW);
		fDot0 = rkU.Dot(rkW);
		rkW -= fDot0 * rkU + fDot1 * rkV;
		rkW.Normalize();
	}
	//----------------------------------------------------------------------------
	template <class Real>
	void Vector3<Real>::Orthonormalize(Vector3* akV)
	{
		Orthonormalize(akV[0], akV[1], akV[2]);
	}
	//----------------------------------------------------------------------------
	template <class Real>
	void Vector3<Real>::GenerateOrthonormalBasis(Vector3& rkU, Vector3& rkV,
		Vector3& rkW)
	{
		rkW.Normalize();
		GenerateComplementBasis(rkU, rkV, rkW);
	}
	//----------------------------------------------------------------------------
	template <class Real>
	void Vector3<Real>::GenerateComplementBasis(Vector3& rkU, Vector3& rkV,
		const Vector3& rkW)
	{
		Real fInvLength;

		if (Math<Real>::FAbs(rkW.m_afTuple[0]) >=
			Math<Real>::FAbs(rkW.m_afTuple[1]))
		{
			// W.x or W.z is the largest magnitude component, swap them
			fInvLength = Math<Real>::InvSqrt(rkW.m_afTuple[0] * rkW.m_afTuple[0] +
				rkW.m_afTuple[2] * rkW.m_afTuple[2]);
			rkU.m_afTuple[0] = -rkW.m_afTuple[2] * fInvLength;
			rkU.m_afTuple[1] = (Real)0.0;
			rkU.m_afTuple[2] = +rkW.m_afTuple[0] * fInvLength;
			rkV.m_afTuple[0] = rkW.m_afTuple[1] * rkU.m_afTuple[2];
			rkV.m_afTuple[1] = rkW.m_afTuple[2] * rkU.m_afTuple[0] -
				rkW.m_afTuple[0] * rkU.m_afTuple[2];
			rkV.m_afTuple[2] = -rkW.m_afTuple[1] * rkU.m_afTuple[0];
		}
		else
		{
			// W.y or W.z is the largest magnitude component, swap them
			fInvLength = Math<Real>::InvSqrt(rkW.m_afTuple[1] * rkW.m_afTuple[1] +
				rkW.m_afTuple[2] * rkW.m_afTuple[2]);
			rkU.m_afTuple[0] = (Real)0.0;
			rkU.m_afTuple[1] = +rkW.m_afTuple[2] * fInvLength;
			rkU.m_afTuple[2] = -rkW.m_afTuple[1] * fInvLength;
			rkV.m_afTuple[0] = rkW.m_afTuple[1] * rkU.m_afTuple[2] -
				rkW.m_afTuple[2] * rkU.m_afTuple[1];
			rkV.m_afTuple[1] = -rkW.m_afTuple[0] * rkU.m_afTuple[2];
			rkV.m_afTuple[2] = rkW.m_afTuple[0] * rkU.m_afTuple[1];
		}
	}
	//----------------------------------------------------------------------------
	template <class Real>
	void Vector3<Real>::ComputeExtremes(int iVQuantity, const Vector3* akPoint,
		Vector3& rkMin, Vector3& rkMax)
	{
		assert(iVQuantity > 0 && akPoint);

		rkMin = akPoint[0];
		rkMax = rkMin;
		for (int i = 1; i < iVQuantity; i++)
		{
			const Vector3<Real>& rkPoint = akPoint[i];
			for (int j = 0; j < 3; j++)
			{
				if (rkPoint[j] < rkMin[j])
				{
					rkMin[j] = rkPoint[j];
				}
				else if (rkPoint[j] > rkMax[j])
				{
					rkMax[j] = rkPoint[j];
				}
			}
		}
	}
	//----------------------------------------------------------------------------

	//----------------------------------------------------------------------------

	template<> const Vector3<float> Vector3<float>::ZERO(0.0f, 0.0f, 0.0f);
	template<> const Vector3<float> Vector3<float>::UNIT_X(1.0f, 0.0f, 0.0f);
	template<> const Vector3<float> Vector3<float>::UNIT_Y(0.0f, 1.0f, 0.0f);
	template<> const Vector3<float> Vector3<float>::UNIT_Z(0.0f, 0.0f, 1.0f);
	template<> const Vector3<float> Vector3<float>::ONE(1.0f, 1.0f, 1.0f);

	template<> const Vector3<double> Vector3<double>::ZERO(0.0, 0.0, 0.0);
	template<> const Vector3<double> Vector3<double>::UNIT_X(1.0, 0.0, 0.0);
	template<> const Vector3<double> Vector3<double>::UNIT_Y(0.0, 1.0, 0.0);
	template<> const Vector3<double> Vector3<double>::UNIT_Z(0.0, 0.0, 1.0);
	template<> const Vector3<double> Vector3<double>::ONE(1.0, 1.0, 1.0);
	//----------------------------------------------------------------------------
	template class Math<float>;
	template class Math<double>;
	template class Vector2<float>;
	template class Vector2<double>;
	template class Vector3<float>;
	template class Vector3<double>;
	//----------------------------------------------------------------------------
}


