#include <assert.h>
#include "Wm4Intersector.h"
#include "Wm4Query.h"
namespace Sn3DAlgorithm
{
	//----------------------------------------------------------------------------
	template <class Real, class TVector>
	Intersector<Real, TVector>::Intersector()
	{
		m_fContactTime = (Real)0.0;
		m_iIntersectionType = IT_EMPTY;
	}
	//----------------------------------------------------------------------------
	template <class Real, class TVector>
	Intersector<Real, TVector>::~Intersector()
	{
	}
	//----------------------------------------------------------------------------
	template <class Real, class TVector>
	Real Intersector<Real, TVector>::GetContactTime() const
	{
		return m_fContactTime;
	}
	//----------------------------------------------------------------------------
	template <class Real, class TVector>
	int Intersector<Real, TVector>::GetIntersectionType() const
	{
		return m_iIntersectionType;
	}
	//----------------------------------------------------------------------------
	template <class Real, class TVector>
	bool Intersector<Real, TVector>::Test()
	{
		// stub for derived class
		assert(false);
		return false;
	}
	//----------------------------------------------------------------------------
	template <class Real, class TVector>
	bool Intersector<Real, TVector>::Find()
	{
		// stub for derived class
		assert(false);
		return false;
	}
	//----------------------------------------------------------------------------
	template <class Real, class TVector>
	bool Intersector<Real, TVector>::Test(Real, const TVector&, const TVector&)
	{
		// stub for derived class
		assert(false);
		return false;
	}
	//----------------------------------------------------------------------------
	template <class Real, class TVector>
	bool Intersector<Real, TVector>::Find(Real, const TVector&, const TVector&)
	{
		// stub for derived class
		assert(false);
		return false;
	}
	//----------------------------------------------------------------------------
	//----------------------------------------------------------------------------
	template <class Real>
	Intersector1<Real>::Intersector1(Real fU0, Real fU1, Real fV0, Real fV1)
	{
		assert(fU0 <= fU1 && fV0 <= fV1);
		m_afU[0] = fU0;
		m_afU[1] = fU1;
		m_afV[0] = fV0;
		m_afV[1] = fV1;
		m_fFirstTime = (Real)0.0;
		m_fLastTime = (Real)0.0;
		m_iQuantity = 0;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Intersector1<Real>::Intersector1(Real afU[2], Real afV[2])
	{
		assert(afU[0] <= afU[1] && afV[0] <= afV[1]);
		for (int i = 0; i < 2; i++)
		{
			m_afU[i] = afU[i];
			m_afV[i] = afV[i];
		}
		m_fFirstTime = (Real)0.0;
		m_fLastTime = (Real)0.0;
		m_iQuantity = 0;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Intersector1<Real>::~Intersector1()
	{
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Real Intersector1<Real>::GetU(int i) const
	{
		assert(0 <= i && i < 2);
		return m_afU[i];
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Real Intersector1<Real>::GetV(int i) const
	{
		assert(0 <= i && i < 2);
		return m_afV[i];
	}
	//----------------------------------------------------------------------------
	template <class Real>
	bool Intersector1<Real>::Test()
	{
		return m_afU[0] <= m_afV[1] && m_afU[1] >= m_afV[0];
	}
	//----------------------------------------------------------------------------
	template <class Real>
	bool Intersector1<Real>::Find()
	{
		if (m_afU[1] < m_afV[0] || m_afU[0] > m_afV[1])
		{
			m_iQuantity = 0;
		}
		else if (m_afU[1] > m_afV[0])
		{
			if (m_afU[0] < m_afV[1])
			{
				m_iQuantity = 2;
				m_afOverlap[0] = (m_afU[0] < m_afV[0] ? m_afV[0] : m_afU[0]);
				m_afOverlap[1] = (m_afU[1] > m_afV[1] ? m_afV[1] : m_afU[1]);
				if (m_afOverlap[0] == m_afOverlap[1])
				{
					m_iQuantity = 1;
				}
			}
			else  // m_afU[0] == m_afV[1]
			{
				m_iQuantity = 1;
				m_afOverlap[0] = m_afU[0];
			}
		}
		else  // m_afU[1] == m_afV[0]
		{
			m_iQuantity = 1;
			m_afOverlap[0] = m_afU[1];
		}

		return m_iQuantity > 0;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	bool Intersector1<Real>::Test(Real fTMax, Real fSpeedU, Real fSpeedV)
	{
		Real fDiffSpeed, fInvDiffSpeed, fDiffPos;

		if (m_afU[1] < m_afV[0])
		{
			// [u0,u1] initially to the left of [v0,v1]
			fDiffSpeed = fSpeedU - fSpeedV;
			if (fDiffSpeed > (Real)0.0)
			{
				// the intervals must move towards each other
				fDiffPos = m_afV[0] - m_afU[1];
				if (fDiffPos <= fTMax * fDiffSpeed)
				{
					// the intervals intersect within the specified time
					fInvDiffSpeed = ((Real)1.0) / fDiffSpeed;
					m_fFirstTime = fDiffPos * fInvDiffSpeed;
					m_fLastTime = (m_afV[1] - m_afU[0])*fInvDiffSpeed;
					return true;
				}
			}
		}
		else if (m_afU[0] > m_afV[1])
		{
			// [u0,u1] initially to the right of [v0,v1]
			fDiffSpeed = fSpeedV - fSpeedU;
			if (fDiffSpeed > (Real)0.0)
			{
				// the intervals must move towards each other
				fDiffPos = m_afU[0] - m_afV[1];
				if (fDiffPos <= fTMax * fDiffSpeed)
				{
					// the intervals intersect within the specified time
					fInvDiffSpeed = ((Real)1.0) / fDiffSpeed;
					m_fFirstTime = fDiffPos * fInvDiffSpeed;
					m_fLastTime = (m_afU[1] - m_afV[0])*fInvDiffSpeed;
					return true;
				}
			}
		}
		else
		{
			// the intervals are initially intersecting
			m_fFirstTime = 0.0f;
			if (fSpeedV > fSpeedU)
			{
				m_fLastTime = (m_afU[1] - m_afV[0]) / (fSpeedV - fSpeedU);
			}
			else if (fSpeedV < fSpeedU)
			{
				m_fLastTime = (m_afV[1] - m_afU[0]) / (fSpeedU - fSpeedV);
			}
			else
			{
				m_fLastTime = Math<Real>::MAX_REAL;
			}

			return true;
		}

		return false;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	bool Intersector1<Real>::Find(Real fTMax, Real fSpeedU, Real fSpeedV)
	{
		Real fDiffSpeed, fInvDiffSpeed, fDiffPos;

		if (m_afU[1] < m_afV[0])
		{
			// [u0,u1] initially to the left of [v0,v1]
			fDiffSpeed = fSpeedU - fSpeedV;
			if (fDiffSpeed > (Real)0.0)
			{
				// the intervals must move towards each other
				fDiffPos = m_afV[0] - m_afU[1];
				if (fDiffPos <= fTMax * fDiffSpeed)
				{
					// the intervals intersect within the specified time
					fInvDiffSpeed = ((Real)1.0) / fDiffSpeed;
					m_fFirstTime = fDiffPos * fInvDiffSpeed;
					m_fLastTime = (m_afV[1] - m_afU[0])*fInvDiffSpeed;
					m_iQuantity = 1;
					m_afOverlap[0] = m_afU[0] + m_fFirstTime * fSpeedU;
					return true;
				}
			}
		}
		else if (m_afU[0] > m_afV[1])
		{
			// [u0,u1] initially to the right of [v0,v1]
			fDiffSpeed = fSpeedV - fSpeedU;
			if (fDiffSpeed > (Real)0.0)
			{
				// the intervals must move towards each other
				fDiffPos = m_afU[0] - m_afV[1];
				if (fDiffPos <= fTMax * fDiffSpeed)
				{
					// the intervals intersect within the specified time
					fInvDiffSpeed = ((Real)1.0) / fDiffSpeed;
					m_fFirstTime = fDiffPos * fInvDiffSpeed;
					m_fLastTime = (m_afU[1] - m_afV[0])*fInvDiffSpeed;
					m_iQuantity = 1;
					m_afOverlap[0] = m_afV[1] + m_fFirstTime * fSpeedV;
					return true;
				}
			}
		}
		else
		{
			// the intervals are initially intersecting
			m_fFirstTime = 0.0f;
			if (fSpeedV > fSpeedU)
			{
				m_fLastTime = (m_afU[1] - m_afV[0]) / (fSpeedV - fSpeedU);
			}
			else if (fSpeedV < fSpeedU)
			{
				m_fLastTime = (m_afV[1] - m_afU[0]) / (fSpeedU - fSpeedV);
			}
			else
			{
				m_fLastTime = Math<Real>::MAX_REAL;
			}

			if (m_afU[1] > m_afV[0])
			{
				if (m_afU[0] < m_afV[1])
				{
					m_iQuantity = 2;
					m_afOverlap[0] = (m_afU[0] < m_afV[0] ? m_afV[0] : m_afU[0]);
					m_afOverlap[1] = (m_afU[1] > m_afV[1] ? m_afV[1] : m_afU[1]);
				}
				else  // m_afU[0] == m_afV[1]
				{
					m_iQuantity = 1;
					m_afOverlap[0] = m_afU[0];
				}
			}
			else  // m_afU[1] == m_afV[0]
			{
				m_iQuantity = 1;
				m_afOverlap[0] = m_afU[1];
			}
			return true;
		}

		m_iQuantity = 0;
		return false;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Real Intersector1<Real>::GetFirstTime() const
	{
		return m_fFirstTime;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Real Intersector1<Real>::GetLastTime() const
	{
		return m_fLastTime;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	int Intersector1<Real>::GetQuantity() const
	{
		return m_iQuantity;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Real Intersector1<Real>::GetOverlap(int i) const
	{
		assert(0 <= i && i < m_iQuantity);
		return m_afOverlap[i];
	}
	//----------------------------------------------------------------------------
	//----------------------------------------------------------------------------
	template <class Real>
	IntrLine2Triangle2<Real>::IntrLine2Triangle2(const Line2<Real>& rkLine,
		const Triangle2<Real>& rkTriangle)
		:
		m_pkLine(&rkLine),
		m_pkTriangle(&rkTriangle)
	{
	}
	//----------------------------------------------------------------------------
	template <class Real>
	const Line2<Real>& IntrLine2Triangle2<Real>::GetLine() const
	{
		return *m_pkLine;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	const Triangle2<Real>& IntrLine2Triangle2<Real>::GetTriangle() const
	{
		return *m_pkTriangle;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	bool IntrLine2Triangle2<Real>::Test()
	{
		Real afDist[3];
		int aiSign[3], iPositive, iNegative, iZero;
		TriangleLineRelations(m_pkLine->Origin, m_pkLine->Direction, *m_pkTriangle,
			afDist, aiSign, iPositive, iNegative, iZero);

		if (iPositive == 3 || iNegative == 3)
		{
			m_iIntersectionType = IT_EMPTY;
		}
		else
		{
			Real afParam[2];
			GetInterval(m_pkLine->Origin, m_pkLine->Direction, *m_pkTriangle, afDist,
				aiSign, afParam);

			Intersector1<Real> kIntr(afParam[0], afParam[1],
				-Math<Real>::MAX_REAL, +Math<Real>::MAX_REAL);

			kIntr.Find();

			m_iQuantity = kIntr.GetQuantity();
			if (m_iQuantity == 2)
			{
				m_iIntersectionType = IT_SEGMENT;
			}
			else if (m_iQuantity == 1)
			{
				m_iIntersectionType = IT_POINT;
			}
			else
			{
				m_iIntersectionType = IT_EMPTY;
			}
		}

		return m_iIntersectionType != IT_EMPTY;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	bool IntrLine2Triangle2<Real>::Find()
	{
		Real afDist[3];
		int aiSign[3], iPositive, iNegative, iZero;
		TriangleLineRelations(m_pkLine->Origin, m_pkLine->Direction, *m_pkTriangle,
			afDist, aiSign, iPositive, iNegative, iZero);

		if (iPositive == 3 || iNegative == 3)
		{
			// No intersections.
			m_iQuantity = 0;
			m_iIntersectionType = IT_EMPTY;
		}
		else
		{
			Real afParam[2];
			GetInterval(m_pkLine->Origin, m_pkLine->Direction, *m_pkTriangle, afDist,
				aiSign, afParam);

			Intersector1<Real> kIntr(afParam[0], afParam[1],
				-Math<Real>::MAX_REAL, +Math<Real>::MAX_REAL);

			kIntr.Find();

			m_iQuantity = kIntr.GetQuantity();
			if (m_iQuantity == 2)
			{
				// Segment intersection.
				m_iIntersectionType = IT_SEGMENT;
				m_akPoint[0] = m_pkLine->Origin + kIntr.GetOverlap(0)*
					m_pkLine->Direction;
				m_akPoint[1] = m_pkLine->Origin + kIntr.GetOverlap(1)*
					m_pkLine->Direction;
			}
			else if (m_iQuantity == 1)
			{
				// Point intersection.
				m_iIntersectionType = IT_POINT;
				m_akPoint[0] = m_pkLine->Origin + kIntr.GetOverlap(0)*
					m_pkLine->Direction;
			}
			else
			{
				// No intersections.
				m_iIntersectionType = IT_EMPTY;
			}
		}

		return m_iIntersectionType != IT_EMPTY;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	int IntrLine2Triangle2<Real>::GetQuantity() const
	{
		return m_iQuantity;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	const Vector2<Real>& IntrLine2Triangle2<Real>::GetPoint(int i) const
	{
		return m_akPoint[i];
	}
	//----------------------------------------------------------------------------
	template <class Real>
	void IntrLine2Triangle2<Real>::TriangleLineRelations(
		const Vector2<Real>& rkOrigin, const Vector2<Real>& rkDirection,
		const Triangle2<Real>& rkTriangle, Real afDist[3], int aiSign[3],
		int& riPositive, int& riNegative, int& riZero)
	{
		riPositive = 0;
		riNegative = 0;
		riZero = 0;
		for (int i = 0; i < 3; i++)
		{
			Vector2<Real> kDiff = rkTriangle.V[i] - rkOrigin;
			afDist[i] = kDiff.DotPerp(rkDirection);
			if (afDist[i] > Math<Real>::ZERO_TOLERANCE)
			{
				aiSign[i] = 1;
				riPositive++;
			}
			else if (afDist[i] < -Math<Real>::ZERO_TOLERANCE)
			{
				aiSign[i] = -1;
				riNegative++;
			}
			else
			{
				afDist[i] = (Real)0.0;
				aiSign[i] = 0;
				riZero++;
			}
		}
	}
	//----------------------------------------------------------------------------
	template <class Real>
	void IntrLine2Triangle2<Real>::GetInterval(const Vector2<Real>& rkOrigin,
		const Vector2<Real>& rkDirection, const Triangle2<Real>& rkTriangle,
		const Real afDist[3], const int aiSign[3], Real afParam[2])
	{
		// Project triangle onto line.
		Real afProj[3];
		int i;
		for (i = 0; i < 3; i++)
		{
			Vector2<Real> kDiff = rkTriangle.V[i] - rkOrigin;
			afProj[i] = rkDirection.Dot(kDiff);
		}

		// Compute transverse intersections of triangle edges with line.
		Real fNumer, fDenom;
		int i0, i1, i2;
		int iQuantity = 0;
		for (i0 = 2, i1 = 0; i1 < 3; i0 = i1++)
		{
			if (aiSign[i0] * aiSign[i1] < 0)
			{
				assert(iQuantity < 2);
				fNumer = afDist[i0] * afProj[i1] - afDist[i1] * afProj[i0];
				fDenom = afDist[i0] - afDist[i1];
				afParam[iQuantity++] = fNumer / fDenom;
			}
		}

		// Check for grazing contact.
		if (iQuantity < 2)
		{
			for (i0 = 1, i1 = 2, i2 = 0; i2 < 3; i0 = i1, i1 = i2++)
			{
				if (aiSign[i2] == 0)
				{
					assert(iQuantity < 2);
					afParam[iQuantity++] = afProj[i2];
				}
			}
		}

		// Sort.
		assert(iQuantity >= 1);
		if (iQuantity == 2)
		{
			if (afParam[0] > afParam[1])
			{
				Real fSave = afParam[0];
				afParam[0] = afParam[1];
				afParam[1] = fSave;
			}
		}
		else
		{
			afParam[1] = afParam[0];
		}
	}
	//-----------------------------------------------------------------------------
	template <class Real>
	IntrSegment2Triangle2<Real>::IntrSegment2Triangle2(
		const Segment2<Real>& rkSegment, const Triangle2<Real>& rkTriangle)
		:
		m_pkSegment(&rkSegment),
		m_pkTriangle(&rkTriangle)
	{
	}
	//----------------------------------------------------------------------------
	template <class Real>
	const Segment2<Real>& IntrSegment2Triangle2<Real>::GetSegment() const
	{
		return *m_pkSegment;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	const Triangle2<Real>& IntrSegment2Triangle2<Real>::GetTriangle() const
	{
		return *m_pkTriangle;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	bool IntrSegment2Triangle2<Real>::Test()
	{
		Real afDist[3];
		int aiSign[3], iPositive, iNegative, iZero;
		IntrLine2Triangle2<Real>::TriangleLineRelations(m_pkSegment->Origin,
			m_pkSegment->Direction, *m_pkTriangle, afDist, aiSign, iPositive,
			iNegative, iZero);

		if (iPositive == 3 || iNegative == 3)
		{
			m_iIntersectionType = IT_EMPTY;
		}
		else
		{
			Real afParam[2];
			IntrLine2Triangle2<Real>::GetInterval(m_pkSegment->Origin,
				m_pkSegment->Direction, *m_pkTriangle, afDist, aiSign, afParam);

			Intersector1<Real> kIntr(afParam[0], afParam[1],
				-m_pkSegment->Extent, +m_pkSegment->Extent);

			kIntr.Find();

			m_iQuantity = kIntr.GetQuantity();
			if (m_iQuantity == 2)
			{
				m_iIntersectionType = IT_SEGMENT;
			}
			else if (m_iQuantity == 1)
			{
				m_iIntersectionType = IT_POINT;
			}
			else
			{
				m_iIntersectionType = IT_EMPTY;
			}
		}

		return m_iIntersectionType != IT_EMPTY;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	bool IntrSegment2Triangle2<Real>::Find()
	{
		Real afDist[3];
		int aiSign[3], iPositive, iNegative, iZero;
		IntrLine2Triangle2<Real>::TriangleLineRelations(m_pkSegment->Origin,
			m_pkSegment->Direction, *m_pkTriangle, afDist, aiSign, iPositive,
			iNegative, iZero);

		if (iPositive == 3 || iNegative == 3)
		{
			// No intersections.
			m_iQuantity = 0;
			m_iIntersectionType = IT_EMPTY;
		}
		else
		{
			Real afParam[2];
			IntrLine2Triangle2<Real>::GetInterval(m_pkSegment->Origin,
				m_pkSegment->Direction, *m_pkTriangle, afDist, aiSign, afParam);

			Intersector1<Real> kIntr(afParam[0], afParam[1],
				-m_pkSegment->Extent, +m_pkSegment->Extent);

			kIntr.Find();

			m_iQuantity = kIntr.GetQuantity();
			if (m_iQuantity == 2)
			{
				// Segment intersection.
				m_iIntersectionType = IT_SEGMENT;
				m_akPoint[0] = m_pkSegment->Origin + kIntr.GetOverlap(0)*
					m_pkSegment->Direction;
				m_akPoint[1] = m_pkSegment->Origin + kIntr.GetOverlap(1)*
					m_pkSegment->Direction;
			}
			else if (m_iQuantity == 1)
			{
				// Point intersection.
				m_iIntersectionType = IT_POINT;
				m_akPoint[0] = m_pkSegment->Origin + kIntr.GetOverlap(0)*
					m_pkSegment->Direction;
			}
			else
			{
				// No intersections.
				m_iIntersectionType = IT_EMPTY;
			}
		}

		return m_iIntersectionType != IT_EMPTY;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	int IntrSegment2Triangle2<Real>::GetQuantity() const
	{
		return m_iQuantity;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	const Vector2<Real>& IntrSegment2Triangle2<Real>::GetPoint(int i) const
	{
		return m_akPoint[i];
	}
	//----------------------------------------------------------------------------

	//----------------------------------------------------------------------------
	template <class Real>
	IntrTriangle2Triangle2<Real>::IntrTriangle2Triangle2(
		const Triangle2<Real>& rkTriangle0, const Triangle2<Real>& rkTriangle1)
		:
		m_pkTriangle0(&rkTriangle0),
		m_pkTriangle1(&rkTriangle1)
	{
		m_iQuantity = 0;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	const Triangle2<Real>& IntrTriangle2Triangle2<Real>::GetTriangle0() const
	{
		return *m_pkTriangle0;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	const Triangle2<Real>& IntrTriangle2Triangle2<Real>::GetTriangle1() const
	{
		return *m_pkTriangle1;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	bool IntrTriangle2Triangle2<Real>::Test()
	{
		int i0, i1;
		Vector2<Real> kDir;

		// test edges of triangle0 for separation
		for (i0 = 0, i1 = 2; i0 < 3; i1 = i0, i0++)
		{
			// test axis V0[i1] + t*perp(V0[i0]-V0[i1]), perp(x,y) = (y,-x)
			kDir.X() = m_pkTriangle0->V[i0].Y() - m_pkTriangle0->V[i1].Y();
			kDir.Y() = m_pkTriangle0->V[i1].X() - m_pkTriangle0->V[i0].X();
			if (WhichSide(m_pkTriangle1->V, m_pkTriangle0->V[i1], kDir) > 0)
			{
				// triangle1 is entirely on positive side of triangle0 edge
				return false;
			}
		}

		// test edges of triangle1 for separation
		for (i0 = 0, i1 = 2; i0 < 3; i1 = i0, i0++)
		{
			// test axis V1[i1] + t*perp(V1[i0]-V1[i1]), perp(x,y) = (y,-x)
			kDir.X() = m_pkTriangle1->V[i0].Y() - m_pkTriangle1->V[i1].Y();
			kDir.Y() = m_pkTriangle1->V[i1].X() - m_pkTriangle1->V[i0].X();
			if (WhichSide(m_pkTriangle0->V, m_pkTriangle1->V[i1], kDir) > 0)
			{
				// triangle0 is entirely on positive side of triangle1 edge
				return false;
			}
		}

		return true;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	bool IntrTriangle2Triangle2<Real>::Find()
	{
		// The potential intersection is initialized to triangle1.  The set of
		// vertices is refined based on clipping against each edge of triangle0.
		m_iQuantity = 3;
		for (int i = 0; i < 3; i++)
		{
			m_akPoint[i] = m_pkTriangle1->V[i];
		}

		for (int i1 = 2, i0 = 0; i0 < 3; i1 = i0, i0++)
		{
			// clip against edge <V0[i1],V0[i0]>
			Vector2<Real> kN(
				m_pkTriangle0->V[i1].Y() - m_pkTriangle0->V[i0].Y(),
				m_pkTriangle0->V[i0].X() - m_pkTriangle0->V[i1].X());
			Real fC = kN.Dot(m_pkTriangle0->V[i1]);
			ClipConvexPolygonAgainstLine(kN, fC, m_iQuantity, m_akPoint);
			if (m_iQuantity == 0)
			{
				// triangle completely clipped, no intersection occurs
				return false;
			}
		}

		return true;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	bool IntrTriangle2Triangle2<Real>::Test(Real fTMax,
		const Vector2<Real>& rkVelocity0, const Vector2<Real>& rkVelocity1)
	{
		// process as if V0-triangle is stationary and V1-triangle is moving
		Vector2<Real> kW = rkVelocity1 - rkVelocity0;
		int iSide = 0;  // 0 = NONE, -1 = LEFT, +1 = RIGHT
		Real fTFirst = (Real)0.0;
		Real fTLast = Math<Real>::MAX_REAL;

		Configuration kCfg0, kCfg1, kTCfg0, kTCfg1;
		int i0, i1, i2;
		Vector2<Real> kD;
		Real fSpeed;

		// process edges of V0-triangle
		for (i0 = 1, i1 = 2, i2 = 0; i2 < 3; i0 = i1, i1 = i2, i2++)
		{
			// test axis V0[i1] + t*perp(V0[i2]-V0[i1]), perp(x,y) = (y,-x)
			kD.X() = m_pkTriangle0->V[i2].Y() - m_pkTriangle0->V[i1].Y();
			kD.Y() = m_pkTriangle0->V[i1].X() - m_pkTriangle0->V[i2].X();
			fSpeed = kD.Dot(kW);

			ComputeTwo(kCfg0, m_pkTriangle0->V, kD, i0, i1, i2);
			ComputeThree(kCfg1, m_pkTriangle1->V, kD, m_pkTriangle0->V[i1]);

			if (NoIntersect(kCfg0, kCfg1, fTMax, fSpeed, iSide, kTCfg0, kTCfg1,
				fTFirst, fTLast))
			{
				return false;
			}
		}

		// process edges of V1-triangle
		for (i0 = 1, i1 = 2, i2 = 0; i2 < 3; i0 = i1, i1 = i2, i2++)
		{
			// test axis V1[i1] + t*perp(V1[i2]-V1[i1]), perp(x,y) = (y,-x)
			kD.X() = m_pkTriangle1->V[i2].Y() - m_pkTriangle1->V[i1].Y();
			kD.Y() = m_pkTriangle1->V[i1].X() - m_pkTriangle1->V[i2].X();
			fSpeed = kD.Dot(kW);

			ComputeTwo(kCfg1, m_pkTriangle1->V, kD, i0, i1, i2);
			ComputeThree(kCfg0, m_pkTriangle0->V, kD, m_pkTriangle1->V[i1]);

			if (NoIntersect(kCfg0, kCfg1, fTMax, fSpeed, iSide, kTCfg0, kTCfg1,
				fTFirst, fTLast))
			{
				return false;
			}
		}

		m_fContactTime = fTFirst;
		return true;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	bool IntrTriangle2Triangle2<Real>::Find(Real fTMax,
		const Vector2<Real>& rkVelocity0, const Vector2<Real>& rkVelocity1)
	{
		// process as if V0-triangle is stationary and V1-triangle is moving
		Vector2<Real> kW = rkVelocity1 - rkVelocity0;
		int iSide = 0;  // 0 = NONE, -1 = LEFT, +1 = RIGHT
		Real fTFirst = (Real)0.0;
		Real fTLast = Math<Real>::MAX_REAL;

		Configuration kCfg0, kCfg1, kTCfg0, kTCfg1;
		int i0, i1, i2;
		Vector2<Real> kD;
		Real fSpeed;

		// process edges of V0-triangle
		for (i0 = 1, i1 = 2, i2 = 0; i2 < 3; i0 = i1, i1 = i2, i2++)
		{
			// test axis V0[i1] + t*perp(V0[i2]-V0[i1]), perp(x,y) = (y,-x)
			kD.X() = m_pkTriangle0->V[i2].Y() - m_pkTriangle0->V[i1].Y();
			kD.Y() = m_pkTriangle0->V[i1].X() - m_pkTriangle0->V[i2].X();
			fSpeed = kD.Dot(kW);

			ComputeTwo(kCfg0, m_pkTriangle0->V, kD, i0, i1, i2);
			ComputeThree(kCfg1, m_pkTriangle1->V, kD, m_pkTriangle0->V[i1]);

			if (NoIntersect(kCfg0, kCfg1, fTMax, fSpeed, iSide, kTCfg0, kTCfg1,
				fTFirst, fTLast))
			{
				return false;
			}
		}

		// process edges of V1-triangle
		for (i0 = 1, i1 = 2, i2 = 0; i2 < 3; i0 = i1, i1 = i2, i2++)
		{
			// test axis V1[i1] + t*perp(V1[i2]-V1[i1]), perp(x,y) = (y,-x)
			kD.X() = m_pkTriangle1->V[i2].Y() - m_pkTriangle1->V[i1].Y();
			kD.Y() = m_pkTriangle1->V[i1].X() - m_pkTriangle1->V[i2].X();
			fSpeed = kD.Dot(kW);

			ComputeTwo(kCfg1, m_pkTriangle1->V, kD, i0, i1, i2);
			ComputeThree(kCfg0, m_pkTriangle0->V, kD, m_pkTriangle1->V[i1]);

			if (NoIntersect(kCfg0, kCfg1, fTMax, fSpeed, iSide, kTCfg0, kTCfg1,
				fTFirst, fTLast))
			{
				return false;
			}
		}

		// move triangles to first contact
		Vector2<Real> akMoveV0[3], akMoveV1[3];
		for (int i = 0; i < 3; i++)
		{
			akMoveV0[i] = m_pkTriangle0->V[i] + fTFirst * rkVelocity0;
			akMoveV1[i] = m_pkTriangle1->V[i] + fTFirst * rkVelocity1;
		};

		GetIntersection(kTCfg0, kTCfg1, iSide, akMoveV0, akMoveV1, m_iQuantity,
			m_akPoint);

		m_fContactTime = fTFirst;
		return m_iQuantity > 0;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	int IntrTriangle2Triangle2<Real>::GetQuantity() const
	{
		return m_iQuantity;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	const Vector2<Real>& IntrTriangle2Triangle2<Real>::GetPoint(int i) const
	{
		assert(0 <= i && i < m_iQuantity);
		return m_akPoint[i];
	}
	//----------------------------------------------------------------------------
	template <class Real>
	int IntrTriangle2Triangle2<Real>::WhichSide(const Vector2<Real> akV[3],
		const Vector2<Real>& rkP, const Vector2<Real>& rkD)
	{
		// Vertices are projected to the form P+t*D.  Return value is +1 if all
		// t > 0, -1 if all t < 0, 0 otherwise, in which case the line splits the
		// triangle.

		int iPositive = 0, iNegative = 0, iZero = 0;
		for (int i = 0; i < 3; i++)
		{
			Real fT = rkD.Dot(akV[i] - rkP);
			if (fT > (Real)0.0)
			{
				iPositive++;
			}
			else if (fT < (Real)0.0)
			{
				iNegative++;
			}
			else
			{
				iZero++;
			}

			if (iPositive > 0 && iNegative > 0)
			{
				return 0;
			}
		}
		return (iZero == 0 ? (iPositive > 0 ? 1 : -1) : 0);
	}
	//----------------------------------------------------------------------------
	template <class Real>
	void IntrTriangle2Triangle2<Real>::ClipConvexPolygonAgainstLine(
		const Vector2<Real>& rkN, Real fC, int& riQuantity,
		Vector2<Real> akV[6])
	{
		// The input vertices are assumed to be in counterclockwise order.  The
		// ordering is an invariant of this function.

		// test on which side of line the vertices are
		int iPositive = 0, iNegative = 0, iPIndex = -1;
		Real afTest[6];
		int i;
		for (i = 0; i < riQuantity; i++)
		{
			afTest[i] = rkN.Dot(akV[i]) - fC;
			if (afTest[i] > (Real)0.0)
			{
				iPositive++;
				if (iPIndex < 0)
				{
					iPIndex = i;
				}
			}
			else if (afTest[i] < (Real)0.0)
			{
				iNegative++;
			}
		}

		if (iPositive > 0)
		{
			if (iNegative > 0)
			{
				// line transversely intersects polygon
				Vector2<Real> akCV[6];
				int iCQuantity = 0, iCur, iPrv;
				Real fT;

				if (iPIndex > 0)
				{
					// first clip vertex on line
					iCur = iPIndex;
					iPrv = iCur - 1;
					fT = afTest[iCur] / (afTest[iCur] - afTest[iPrv]);
					akCV[iCQuantity++] = akV[iCur] + fT * (akV[iPrv] - akV[iCur]);

					// vertices on positive side of line
					while (iCur < riQuantity && afTest[iCur] >(Real)0.0)
					{
						akCV[iCQuantity++] = akV[iCur++];
					}

					// last clip vertex on line
					if (iCur < riQuantity)
					{
						iPrv = iCur - 1;
					}
					else
					{
						iCur = 0;
						iPrv = riQuantity - 1;
					}
					fT = afTest[iCur] / (afTest[iCur] - afTest[iPrv]);
					akCV[iCQuantity++] = akV[iCur] + fT * (akV[iPrv] - akV[iCur]);
				}
				else  // iPIndex is 0
				{
					// vertices on positive side of line
					iCur = 0;
					while (iCur < riQuantity && afTest[iCur] >(Real)0.0)
					{
						akCV[iCQuantity++] = akV[iCur++];
					}

					// last clip vertex on line
					iPrv = iCur - 1;
					fT = afTest[iCur] / (afTest[iCur] - afTest[iPrv]);
					akCV[iCQuantity++] = akV[iCur] + fT * (akV[iPrv] - akV[iCur]);

					// skip vertices on negative side
					while (iCur < riQuantity && afTest[iCur] <= (Real)0.0)
					{
						iCur++;
					}

					// first clip vertex on line
					if (iCur < riQuantity)
					{
						iPrv = iCur - 1;
						fT = afTest[iCur] / (afTest[iCur] - afTest[iPrv]);
						akCV[iCQuantity++] = akV[iCur] + fT * (akV[iPrv] - akV[iCur]);

						// vertices on positive side of line
						while (iCur < riQuantity && afTest[iCur] >(Real)0.0)
						{
							akCV[iCQuantity++] = akV[iCur++];
						}
					}
					else
					{
						// iCur = 0
						iPrv = riQuantity - 1;
						fT = afTest[0] / (afTest[0] - afTest[iPrv]);
						akCV[iCQuantity++] = akV[0] + fT * (akV[iPrv] - akV[0]);
					}
				}

				riQuantity = iCQuantity;
				size_t uiSize = iCQuantity * sizeof(Vector2<Real>);
				std::memcpy(akV, akCV, uiSize);

			}
		}
		else
		{
			riQuantity = 0;
		}
	}
	//----------------------------------------------------------------------------
	template <class Real>
	void IntrTriangle2Triangle2<Real>::ComputeTwo(Configuration& rkCfg,
		const Vector2<Real> akV[3], const Vector2<Real>& rkD, int i0, int i1,
		int i2)
	{
		rkCfg.Map = M12;
		rkCfg.Index[0] = i0;
		rkCfg.Index[1] = i1;
		rkCfg.Index[2] = i2;
		rkCfg.Min = rkD.Dot(akV[i0] - akV[i1]);
		rkCfg.Max = (Real)0.0;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	void IntrTriangle2Triangle2<Real>::ComputeThree(Configuration& rkCfg,
		const Vector2<Real> akV[3], const Vector2<Real>& rkD,
		const Vector2<Real>& rkP)
	{
		Real fD0 = rkD.Dot(akV[0] - rkP);
		Real fD1 = rkD.Dot(akV[1] - rkP);
		Real fD2 = rkD.Dot(akV[2] - rkP);

		// Make sure that m_aiIndex[...] is an even permutation of (0,1,2)
		// whenever the map value is M12 or M21.  This is needed to guarantee
		// the intersection of overlapping edges is properly computed.

		if (fD0 <= fD1)
		{
			if (fD1 <= fD2)  // d0 <= d1 <= d2
			{
				if (fD0 != fD1)
				{
					rkCfg.Map = (fD1 != fD2 ? M11 : M12);
				}
				else
				{
					rkCfg.Map = M21;
				}

				rkCfg.Index[0] = 0;
				rkCfg.Index[1] = 1;
				rkCfg.Index[2] = 2;
				rkCfg.Min = fD0;
				rkCfg.Max = fD2;
			}
			else if (fD0 <= fD2)  // d0 <= d2 < d1
			{
				if (fD0 != fD2)
				{
					rkCfg.Map = M11;
					rkCfg.Index[0] = 0;
					rkCfg.Index[1] = 2;
					rkCfg.Index[2] = 1;
				}
				else
				{
					rkCfg.Map = M21;
					rkCfg.Index[0] = 2;
					rkCfg.Index[1] = 0;
					rkCfg.Index[2] = 1;
				}

				rkCfg.Min = fD0;
				rkCfg.Max = fD1;
			}
			else  // d2 < d0 <= d1
			{
				rkCfg.Map = (fD0 != fD1 ? M12 : M11);
				rkCfg.Index[0] = 2;
				rkCfg.Index[1] = 0;
				rkCfg.Index[2] = 1;
				rkCfg.Min = fD2;
				rkCfg.Max = fD1;
			}
		}
		else
		{
			if (fD2 <= fD1)  // d2 <= d1 < d0
			{
				if (fD2 != fD1)
				{
					rkCfg.Map = M11;
					rkCfg.Index[0] = 2;
					rkCfg.Index[1] = 1;
					rkCfg.Index[2] = 0;
				}
				else
				{
					rkCfg.Map = M21;
					rkCfg.Index[0] = 1;
					rkCfg.Index[1] = 2;
					rkCfg.Index[2] = 0;
				}

				rkCfg.Min = fD2;
				rkCfg.Max = fD0;
			}
			else if (fD2 <= fD0)  // d1 < d2 <= d0
			{
				rkCfg.Map = (fD2 != fD0 ? M11 : M12);
				rkCfg.Index[0] = 1;
				rkCfg.Index[1] = 2;
				rkCfg.Index[2] = 0;
				rkCfg.Min = fD1;
				rkCfg.Max = fD0;
			}
			else  // d1 < d0 < d2
			{
				rkCfg.Map = M11;
				rkCfg.Index[0] = 1;
				rkCfg.Index[1] = 0;
				rkCfg.Index[2] = 2;
				rkCfg.Min = fD1;
				rkCfg.Max = fD2;
			}
		}
	}
	//----------------------------------------------------------------------------
	template <class Real>
	bool IntrTriangle2Triangle2<Real>::NoIntersect(
		const Configuration& rkCfg0, const Configuration& rkCfg1, Real fTMax,
		Real fSpeed, int& riSide, Configuration& rkTCfg0, Configuration& rkTCfg1,
		Real& rfTFirst, Real& rfTLast)
	{
		Real fInvSpeed, fT;

		if (rkCfg1.Max < rkCfg0.Min)
		{
			// V1-interval initially on left of V0-interval
			if (fSpeed <= (Real)0.0)
			{
				return true;  // intervals moving apart
			}

			// update first time
			fInvSpeed = ((Real)1.0) / fSpeed;
			fT = (rkCfg0.Min - rkCfg1.Max)*fInvSpeed;
			if (fT > rfTFirst)
			{
				rfTFirst = fT;
				riSide = -1;
				rkTCfg0 = rkCfg0;
				rkTCfg1 = rkCfg1;
			}

			// test for exceedance of time interval
			if (rfTFirst > fTMax)
			{
				return true;
			}

			// update last time
			fT = (rkCfg0.Max - rkCfg1.Min)*fInvSpeed;
			if (fT < rfTLast)
			{
				rfTLast = fT;
			}

			// test for separation
			if (rfTFirst > rfTLast)
			{
				return true;
			}
		}
		else if (rkCfg0.Max < rkCfg1.Min)
		{
			// V1-interval initially on right of V0-interval
			if (fSpeed >= (Real)0.0)
			{
				return true;  // intervals moving apart
			}

			// update first time
			fInvSpeed = ((Real)1.0) / fSpeed;
			fT = (rkCfg0.Max - rkCfg1.Min)*fInvSpeed;
			if (fT > rfTFirst)
			{
				rfTFirst = fT;
				riSide = 1;
				rkTCfg0 = rkCfg0;
				rkTCfg1 = rkCfg1;
			}

			// test for exceedance of time interval
			if (rfTFirst > fTMax)
			{
				return true;
			}

			// update last time
			fT = (rkCfg0.Min - rkCfg1.Max)*fInvSpeed;
			if (fT < rfTLast)
			{
				rfTLast = fT;
			}

			// test for separation
			if (rfTFirst > rfTLast)
			{
				return true;
			}
		}
		else
		{
			// V0-interval and V1-interval initially overlap
			if (fSpeed > (Real)0.0)
			{
				// update last time
				fInvSpeed = ((Real)1.0) / fSpeed;
				fT = (rkCfg0.Max - rkCfg1.Min)*fInvSpeed;
				if (fT < rfTLast)
				{
					rfTLast = fT;
				}

				// test for separation
				if (rfTFirst > rfTLast)
				{
					return true;
				}
			}
			else if (fSpeed < (Real)0.0)
			{
				// update last time
				fInvSpeed = ((Real)1.0) / fSpeed;
				fT = (rkCfg0.Min - rkCfg1.Max)*fInvSpeed;
				if (fT < rfTLast)
				{
					rfTLast = fT;
				}

				// test for separation
				if (rfTFirst > rfTLast)
				{
					return true;
				}
			}
		}

		return false;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	void IntrTriangle2Triangle2<Real>::GetIntersection(
		const Configuration& rkCfg0, const Configuration& rkCfg1, int iSide,
		const Vector2<Real> akV0[3], const Vector2<Real> akV1[3], int& riQuantity,
		Vector2<Real> akVertex[6])
	{
		Vector2<Real> kEdge, kDiff;
		const Vector2<Real>* pkOrigin;
		Real fInvEdE, fMin, fMax;
		int i;

		if (iSide == 1)  // V1-interval contacts V0-interval on right
		{
			if (rkCfg0.Map == M21 || rkCfg0.Map == M11)
			{
				riQuantity = 1;
				akVertex[0] = akV0[rkCfg0.Index[2]];
			}
			else if (rkCfg1.Map == M12 || rkCfg1.Map == M11)
			{
				riQuantity = 1;
				akVertex[0] = akV1[rkCfg1.Index[0]];
			}
			else  // rkCfg0.Map == M12 && rkCfg1.Map == M21 (edge overlap)
			{
				pkOrigin = &akV0[rkCfg0.Index[1]];
				kEdge = akV0[rkCfg0.Index[2]] - *pkOrigin;
				fInvEdE = ((Real)1.0) / kEdge.Dot(kEdge);
				kDiff = akV1[rkCfg1.Index[1]] - *pkOrigin;
				fMin = kEdge.Dot(kDiff)*fInvEdE;
				kDiff = akV1[rkCfg1.Index[0]] - *pkOrigin;
				fMax = kEdge.Dot(kDiff)*fInvEdE;
				assert(fMin <= fMax);
				Intersector1<Real> kIntr((Real)0.0, (Real)1.0, fMin, fMax);
				riQuantity = kIntr.GetQuantity();
				assert(riQuantity > 0);
				for (i = 0; i < riQuantity; i++)
				{
					akVertex[i] = *pkOrigin + kIntr.GetOverlap(i)*kEdge;
				}
			}
		}
		else if (iSide == -1)  // V1-interval contacts V0-interval on left
		{
			if (rkCfg1.Map == M21 || rkCfg1.Map == M11)
			{
				riQuantity = 1;
				akVertex[0] = akV1[rkCfg1.Index[2]];
			}
			else if (rkCfg0.Map == M12 || rkCfg0.Map == M11)
			{
				riQuantity = 1;
				akVertex[0] = akV0[rkCfg0.Index[0]];
			}
			else  // rkCfg1.Map == M12 && rkCfg0.Map == M21 (edge overlap)
			{
				pkOrigin = &akV1[rkCfg1.Index[1]];
				kEdge = akV1[rkCfg1.Index[2]] - *pkOrigin;
				fInvEdE = 1.0f / kEdge.Dot(kEdge);
				kDiff = akV0[rkCfg0.Index[1]] - *pkOrigin;
				fMin = kEdge.Dot(kDiff)*fInvEdE;
				kDiff = akV0[rkCfg0.Index[0]] - *pkOrigin;
				fMax = kEdge.Dot(kDiff)*fInvEdE;
				assert(fMin <= fMax);
				Intersector1<Real> kIntr((Real)0.0, (Real)1.0, fMin, fMax);
				riQuantity = kIntr.GetQuantity();
				assert(riQuantity > 0);
				for (i = 0; i < riQuantity; i++)
				{
					akVertex[i] = *pkOrigin + kIntr.GetOverlap(i)*kEdge;
				}
			}
		}
		else  // triangles were initially intersecting
		{
			Triangle2<Real> kTri0(akV0), kTri1(akV1);
			IntrTriangle2Triangle2 kIntr(kTri0, kTri1);
			kIntr.Find();
			riQuantity = kIntr.GetQuantity();
			for (i = 0; i < riQuantity; i++)
			{
				akVertex[i] = kIntr.GetPoint(i);
			}
		}
	}
	//----------------------------------------------------------------------------
	//----------------------------------------------------------------------------
	template <class Real>
	IntrTriangle3Triangle3<Real>::IntrTriangle3Triangle3(
		const Triangle3<Real>& rkTriangle0, const Triangle3<Real>& rkTriangle1)
		:
		m_pkTriangle0(&rkTriangle0),
		m_pkTriangle1(&rkTriangle1)
	{
		ReportCoplanarIntersections = true;
		m_iQuantity = 0;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	const Triangle3<Real>& IntrTriangle3Triangle3<Real>::GetTriangle0() const
	{
		return *m_pkTriangle0;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	const Triangle3<Real>& IntrTriangle3Triangle3<Real>::GetTriangle1() const
	{
		return *m_pkTriangle1;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	bool IntrTriangle3Triangle3<Real>::Test()
	{
		// get edge vectors for triangle0
		Vector3<Real> akE0[3] =
		{
			m_pkTriangle0->V[1] - m_pkTriangle0->V[0],
			m_pkTriangle0->V[2] - m_pkTriangle0->V[1],
			m_pkTriangle0->V[0] - m_pkTriangle0->V[2]
		};

		// get normal vector of triangle0
		Vector3<Real> kN0 = akE0[0].UnitCross(akE0[1]);

		// project triangle1 onto normal line of triangle0, test for separation
		Real fN0dT0V0 = kN0.Dot(m_pkTriangle0->V[0]);
		Real fMin1, fMax1;
		ProjectOntoAxis(*m_pkTriangle1, kN0, fMin1, fMax1);
		if (fN0dT0V0 < fMin1 || fN0dT0V0 > fMax1)
		{
			return false;
		}

		// get edge vectors for triangle1
		Vector3<Real> akE1[3] =
		{
			m_pkTriangle1->V[1] - m_pkTriangle1->V[0],
			m_pkTriangle1->V[2] - m_pkTriangle1->V[1],
			m_pkTriangle1->V[0] - m_pkTriangle1->V[2]
		};

		// get normal vector of triangle1
		Vector3<Real> kN1 = akE1[0].UnitCross(akE1[1]);

		Vector3<Real> kDir;
		Real fMin0, fMax0;
		int i0, i1;

		Vector3<Real> kN0xN1 = kN0.UnitCross(kN1);
		if (kN0xN1.Dot(kN0xN1) >= Math<Real>::ZERO_TOLERANCE)
		{
			// triangles are not parallel

			// Project triangle0 onto normal line of triangle1, test for
			// separation.
			Real fN1dT1V0 = kN1.Dot(m_pkTriangle1->V[0]);
			ProjectOntoAxis(*m_pkTriangle0, kN1, fMin0, fMax0);
			if (fN1dT1V0 < fMin0 || fN1dT1V0 > fMax0)
			{
				return false;
			}

			// directions E0[i0]xE1[i1]
			for (i1 = 0; i1 < 3; i1++)
			{
				for (i0 = 0; i0 < 3; i0++)
				{
					kDir = akE0[i0].UnitCross(akE1[i1]);
					ProjectOntoAxis(*m_pkTriangle0, kDir, fMin0, fMax0);
					ProjectOntoAxis(*m_pkTriangle1, kDir, fMin1, fMax1);
					if (fMax0 < fMin1 || fMax1 < fMin0)
					{
						return false;
					}
				}
			}
		}
		else  // triangles are parallel (and, in fact, coplanar)
		{
			// directions N0xE0[i0]
			for (i0 = 0; i0 < 3; i0++)
			{
				kDir = kN0.UnitCross(akE0[i0]);
				ProjectOntoAxis(*m_pkTriangle0, kDir, fMin0, fMax0);
				ProjectOntoAxis(*m_pkTriangle1, kDir, fMin1, fMax1);
				if (fMax0 < fMin1 || fMax1 < fMin0)
				{
					return false;
				}
			}

			// directions N1xE1[i1]
			for (i1 = 0; i1 < 3; i1++)
			{
				kDir = kN1.UnitCross(akE1[i1]);
				ProjectOntoAxis(*m_pkTriangle0, kDir, fMin0, fMax0);
				ProjectOntoAxis(*m_pkTriangle1, kDir, fMin1, fMax1);
				if (fMax0 < fMin1 || fMax1 < fMin0)
				{
					return false;
				}
			}
		}

		return true;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	bool IntrTriangle3Triangle3<Real>::Find()
	{
		int i, iM, iP;

		// Get the plane of triangle0.
		Plane3<Real> kPlane0(m_pkTriangle0->V[0], m_pkTriangle0->V[1],
			m_pkTriangle0->V[2]);

		// Compute the signed distances of triangle1 vertices to plane0.  Use
		// an epsilon-thick plane test.
		int iPos1, iNeg1, iZero1, aiSign1[3];
		Real afDist1[3];
		TrianglePlaneRelations(*m_pkTriangle1, kPlane0, afDist1, aiSign1, iPos1, iNeg1,
			iZero1);

		if (iPos1 == 3 || iNeg1 == 3)
		{
			// Triangle1 is fully on one side of plane0.
			return false;
		}

		if (iZero1 == 3)
		{
			// Triangle1 is contained by plane0.
			if (ReportCoplanarIntersections)
			{
				//此时相交的区域面积不为0,如果相交的区域为0 
				if (!GetCoplanarIntersection(kPlane0, *m_pkTriangle0, *m_pkTriangle1))
				{
					m_iQuantity = 0;
					if (PointIsInSegment(m_pkTriangle0->V[0], m_pkTriangle1->V[0], m_pkTriangle1->V[1]) ||
						PointIsInSegment(m_pkTriangle0->V[0], m_pkTriangle1->V[1], m_pkTriangle1->V[2]) ||
						PointIsInSegment(m_pkTriangle0->V[0], m_pkTriangle1->V[2], m_pkTriangle1->V[0]))
					{
						m_akPoint[m_iQuantity] = m_pkTriangle0->V[0];
						++m_iQuantity;
					}

					if (PointIsInSegment(m_pkTriangle0->V[1], m_pkTriangle1->V[0], m_pkTriangle1->V[1]) ||
						PointIsInSegment(m_pkTriangle0->V[1], m_pkTriangle1->V[1], m_pkTriangle1->V[2]) ||
						PointIsInSegment(m_pkTriangle0->V[1], m_pkTriangle1->V[2], m_pkTriangle1->V[0]))
					{
						m_akPoint[m_iQuantity] = m_pkTriangle0->V[1];
						++m_iQuantity;
					}

					if (PointIsInSegment(m_pkTriangle0->V[2], m_pkTriangle1->V[0], m_pkTriangle1->V[1]) ||
						PointIsInSegment(m_pkTriangle0->V[2], m_pkTriangle1->V[1], m_pkTriangle1->V[2]) ||
						PointIsInSegment(m_pkTriangle0->V[2], m_pkTriangle1->V[2], m_pkTriangle1->V[0]))
					{
						m_akPoint[m_iQuantity] = m_pkTriangle0->V[2];
						++m_iQuantity;
					}

					if (PointIsInSegment(m_pkTriangle1->V[0], m_pkTriangle0->V[0], m_pkTriangle0->V[1]) ||
						PointIsInSegment(m_pkTriangle1->V[0], m_pkTriangle0->V[1], m_pkTriangle0->V[2]) ||
						PointIsInSegment(m_pkTriangle1->V[0], m_pkTriangle0->V[2], m_pkTriangle0->V[0]))
					{
						m_akPoint[m_iQuantity] = m_pkTriangle1->V[0];
						++m_iQuantity;
					}

					if (PointIsInSegment(m_pkTriangle1->V[1], m_pkTriangle0->V[0], m_pkTriangle0->V[1]) ||
						PointIsInSegment(m_pkTriangle1->V[1], m_pkTriangle0->V[1], m_pkTriangle0->V[2]) ||
						PointIsInSegment(m_pkTriangle1->V[1], m_pkTriangle0->V[2], m_pkTriangle0->V[0]))
					{
						m_akPoint[m_iQuantity] = m_pkTriangle1->V[1];
						++m_iQuantity;
					}

					if (PointIsInSegment(m_pkTriangle1->V[2], m_pkTriangle0->V[0], m_pkTriangle0->V[1]) ||
						PointIsInSegment(m_pkTriangle1->V[2], m_pkTriangle0->V[1], m_pkTriangle0->V[2]) ||
						PointIsInSegment(m_pkTriangle1->V[2], m_pkTriangle0->V[2], m_pkTriangle0->V[0]))
					{
						m_akPoint[m_iQuantity] = m_pkTriangle1->V[2];
						++m_iQuantity;
					}

					for (int i = m_iQuantity - 1; i > 0; --i)
					{
						for (int j = i - 1; j >= 0; --j)
						{
							if ((m_akPoint[i] - m_akPoint[j]).SquaredLength() <= Math<Real>::ZERO_TOLERANCE)
							{
								--m_iQuantity;
								break;
							}
						}
					}

					m_iIntersectionType = IT_SEGMENT;
					return m_iQuantity != 0;
				}
				else
				{
					return true;
				}
			}
		}

		// Check for grazing contact between triangle1 and plane0.
		if (iPos1 == 0 || iNeg1 == 0)
		{
			if (iZero1 == 2)
			{
				// An edge of triangle1 is in plane0.
				for (i = 0; i < 3; i++)
				{
					if (aiSign1[i] != 0)
					{
						iM = (i + 2) % 3;
						iP = (i + 1) % 3;
						if (IntersectsSegment(kPlane0, *m_pkTriangle0,
							m_pkTriangle1->V[iM], m_pkTriangle1->V[iP]))
						{
							for (int i = m_iQuantity - 1; i > 0; --i)
							{
								for (int j = i - 1; j >= 0; --j)
								{
									if ((m_akPoint[i] - m_akPoint[j]).SquaredLength() <= Math<Real>::ZERO_TOLERANCE)
									{
										--m_iQuantity;
										break;
									}
								}
							}
							return true;
						}
						else
							return false;
					}
				}
			}
			else // iZero1 == 1
			{
				// A vertex of triangle1 is in plane0.
				for (i = 0; i < 3; i++)
				{
					if (aiSign1[i] == 0)
					{
						if (ContainsPoint(*m_pkTriangle0, kPlane0,
							m_pkTriangle1->V[i]))
						{
							for (int i = m_iQuantity - 1; i > 0; --i)
							{
								for (int j = i - 1; j >= 0; --j)
								{
									if ((m_akPoint[i] - m_akPoint[j]).SquaredLength() <= Math<Real>::ZERO_TOLERANCE)
									{
										--m_iQuantity;
										break;
									}
								}
							}
							return true;
						}
						else
							return false;
					}
				}
			}
		}

		// At this point, triangle1 tranversely intersects plane 0.  Compute the
		// line segment of intersection.  Then test for intersection between this
		// segment and triangle 0.
		Real fT;
		Vector3<Real> kIntr0, kIntr1;
		if (iZero1 == 0)
		{
			int iSign = (iPos1 == 1 ? +1 : -1);
			for (i = 0; i < 3; i++)
			{
				if (aiSign1[i] == iSign)
				{
					iM = (i + 2) % 3;
					iP = (i + 1) % 3;
					fT = afDist1[i] / (afDist1[i] - afDist1[iM]);
					kIntr0 = m_pkTriangle1->V[i] + fT * (m_pkTriangle1->V[iM] -
						m_pkTriangle1->V[i]);
					fT = afDist1[i] / (afDist1[i] - afDist1[iP]);
					kIntr1 = m_pkTriangle1->V[i] + fT * (m_pkTriangle1->V[iP] -
						m_pkTriangle1->V[i]);
					if (IntersectsSegment(kPlane0, *m_pkTriangle0, kIntr0, kIntr1))
					{
						for (int i = m_iQuantity - 1; i > 0; --i)
						{
							for (int j = i - 1; j >= 0; --j)
							{
								if ((m_akPoint[i] - m_akPoint[j]).SquaredLength() <= Math<Real>::ZERO_TOLERANCE)
								{
									--m_iQuantity;
									break;
								}
							}
						}
						return true;
					}
					else
						return false;
				}
			}
		}

		// iZero1 == 1
		for (i = 0; i < 3; i++)
		{
			if (aiSign1[i] == 0)
			{
				iM = (i + 2) % 3;
				iP = (i + 1) % 3;
				fT = afDist1[iM] / (afDist1[iM] - afDist1[iP]);
				kIntr0 = m_pkTriangle1->V[iM] + fT * (m_pkTriangle1->V[iP] -
					m_pkTriangle1->V[iM]);
				if (IntersectsSegment(kPlane0, *m_pkTriangle0,
					m_pkTriangle1->V[i], kIntr0))
				{
					for (int i = m_iQuantity - 1; i > 0; --i)
					{
						for (int j = i - 1; j >= 0; --j)
						{
							if ((m_akPoint[i] - m_akPoint[j]).SquaredLength() <= Math<Real>::ZERO_TOLERANCE)
							{
								--m_iQuantity;
								break;
							}
						}
					}
					return true;
				}
				else
					return false;
			}
		}

		return false;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	bool IntrTriangle3Triangle3<Real>::Test(Real fTMax,
		const Vector3<Real>& rkVelocity0, const Vector3<Real>& rkVelocity1)
	{
		Real fTFirst = (Real)0.0;
		Real fTLast = Math<Real>::MAX_REAL;

		// velocity relative to triangle0
		Vector3<Real> kVel = rkVelocity1 - rkVelocity0;

		// compute edge and normal directions for triangle0
		Vector3<Real> akE[3] =
		{
			m_pkTriangle0->V[1] - m_pkTriangle0->V[0],
			m_pkTriangle0->V[2] - m_pkTriangle0->V[1],
			m_pkTriangle0->V[0] - m_pkTriangle0->V[2]
		};
		Vector3<Real> kN = akE[0].UnitCross(akE[1]);
		if (!TestOverlap(kN, fTMax, kVel, fTFirst, fTLast))
		{
			return false;
		}

		// compute edge and normal directions for triangle1
		Vector3<Real> akF[3] =
		{
			m_pkTriangle1->V[1] - m_pkTriangle1->V[0],
			m_pkTriangle1->V[2] - m_pkTriangle1->V[1],
			m_pkTriangle1->V[0] - m_pkTriangle1->V[2]
		};
		Vector3<Real> kM = akF[0].UnitCross(akF[1]);

		Vector3<Real> kDir;
		int i0, i1;

		if (Math<Real>::FAbs(kN.Dot(kM)) < 1.0f - Math<Real>::ZERO_TOLERANCE)
		{
			// triangles are not parallel

			// direction M
			if (!TestOverlap(kM, fTMax, kVel, fTFirst, fTLast))
			{
				return false;
			}

			// directions E[i0]xF[i1]
			for (i1 = 0; i1 < 3; i1++)
			{
				for (i0 = 0; i0 < 3; i0++)
				{
					kDir = akE[i0].UnitCross(akF[i1]);
					if (!TestOverlap(kDir, fTMax, kVel, fTFirst, fTLast))
					{
						return false;
					}
				}
			}
		}
		else  // triangles are parallel (and, in fact, coplanar)
		{
			// directions NxE[i0]
			for (i0 = 0; i0 < 3; i0++)
			{
				kDir = kN.UnitCross(akE[i0]);
				if (!TestOverlap(kDir, fTMax, kVel, fTFirst, fTLast))
				{
					return false;
				}
			}

			// directions NxF[i1]
			for (i1 = 0; i1 < 3; i1++)
			{
				kDir = kM.UnitCross(akF[i1]);
				if (!TestOverlap(kDir, fTMax, kVel, fTFirst, fTLast))
				{
					return false;
				}
			}
		}

		m_fContactTime = fTFirst;
		return true;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	bool IntrTriangle3Triangle3<Real>::Find(Real fTMax,
		const Vector3<Real>& rkVelocity0, const Vector3<Real>& rkVelocity1)
	{
		Real fTFirst = (Real)0.0;
		Real fTLast = Math<Real>::MAX_REAL;

		// velocity relative to triangle0
		Vector3<Real> kVel = rkVelocity1 - rkVelocity0;

		ContactSide eSide = CS_NONE;
		Configuration kTCfg0, kTCfg1;

		// compute edge and normal directions for triangle0
		Vector3<Real> akE[3] =
		{
			m_pkTriangle0->V[1] - m_pkTriangle0->V[0],
			m_pkTriangle0->V[2] - m_pkTriangle0->V[1],
			m_pkTriangle0->V[0] - m_pkTriangle0->V[2]
		};
		Vector3<Real> kN = akE[0].UnitCross(akE[1]);
		if (!FindOverlap(kN, fTMax, kVel, eSide, kTCfg0, kTCfg1, fTFirst, fTLast))
		{
			return false;
		}

		// compute edge and normal directions for triangle1
		Vector3<Real> akF[3] =
		{
			m_pkTriangle1->V[1] - m_pkTriangle1->V[0],
			m_pkTriangle1->V[2] - m_pkTriangle1->V[1],
			m_pkTriangle1->V[0] - m_pkTriangle1->V[2]
		};
		Vector3<Real> kM = akF[0].UnitCross(akF[1]);

		Vector3<Real> kDir;
		int i0, i1;

		if (Math<Real>::FAbs(kN.Dot(kM)) < 1.0f - Math<Real>::ZERO_TOLERANCE)
		{
			// triangles are not parallel

			// direction M
			if (!FindOverlap(kM, fTMax, kVel, eSide, kTCfg0, kTCfg1, fTFirst, fTLast))
			{
				return false;
			}

			// directions E[i0]xF[i1]
			for (i1 = 0; i1 < 3; i1++)
			{
				for (i0 = 0; i0 < 3; i0++)
				{
					kDir = akE[i0].UnitCross(akF[i1]);
					if (!FindOverlap(kDir, fTMax, kVel, eSide, kTCfg0, kTCfg1, fTFirst,
						fTLast))
					{
						return false;
					}
				}
			}
		}
		else  // triangles are parallel (and, in fact, coplanar)
		{
			// directions NxE[i0]
			for (i0 = 0; i0 < 3; i0++)
			{
				kDir = kN.UnitCross(akE[i0]);
				if (!FindOverlap(kDir, fTMax, kVel, eSide, kTCfg0, kTCfg1, fTFirst,
					fTLast))
				{
					return false;
				}
			}

			// directions NxF[i1]
			for (i1 = 0; i1 < 3; i1++)
			{
				kDir = kM.UnitCross(akF[i1]);
				if (!FindOverlap(kDir, fTMax, kVel, eSide, kTCfg0, kTCfg1, fTFirst,
					fTLast))
				{
					return false;
				}
			}
		}

		if (fTFirst <= (Real)0.0)
		{
			return false;
		}

		m_fContactTime = fTFirst;

		// adjust U and V for first time of contact before finding contact set
		Triangle3<Real> akMTri0, akMTri1;
		for (i0 = 0; i0 < 3; i0++)
		{
			akMTri0.V[i0] = m_pkTriangle0->V[i0] + fTFirst * rkVelocity0;
			akMTri1.V[i0] = m_pkTriangle1->V[i0] + fTFirst * rkVelocity1;
		}

		FindContactSet(akMTri0, akMTri1, eSide, kTCfg0, kTCfg1);
		return true;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	int IntrTriangle3Triangle3<Real>::GetQuantity() const
	{
		return m_iQuantity;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	const Vector3<Real>& IntrTriangle3Triangle3<Real>::GetPoint(int i) const
	{
		assert(0 <= i && i < m_iQuantity);
		return m_akPoint[i];
	}
	//----------------------------------------------------------------------------
	template <class Real>
	void IntrTriangle3Triangle3<Real>::ProjectOntoAxis(
		const Triangle3<Real>& rkTri, const Vector3<Real>& rkAxis, Real& rfMin,
		Real& rfMax)
	{
		Real fDot0 = rkAxis.Dot(rkTri.V[0]);
		Real fDot1 = rkAxis.Dot(rkTri.V[1]);
		Real fDot2 = rkAxis.Dot(rkTri.V[2]);

		rfMin = fDot0;
		rfMax = rfMin;

		if (fDot1 < rfMin)
		{
			rfMin = fDot1;
		}
		else if (fDot1 > rfMax)
		{
			rfMax = fDot1;
		}

		if (fDot2 < rfMin)
		{
			rfMin = fDot2;
		}
		else if (fDot2 > rfMax)
		{
			rfMax = fDot2;
		}
	}
	//----------------------------------------------------------------------------
	template <class Real>
	void IntrTriangle3Triangle3<Real>::TrianglePlaneRelations(
		const Triangle3<Real>& rkTriangle, const Plane3<Real>& rkPlane,
		Real afDistance[3], int aiSign[3], int& riPositive, int& riNegative,
		int& riZero)
	{
		// Compute the signed distances of triangle vertices to the plane.  Use
		// an epsilon-thick plane test.
		riPositive = 0;
		riNegative = 0;
		riZero = 0;
		for (int i = 0; i < 3; i++)
		{
			afDistance[i] = rkPlane.DistanceTo(rkTriangle.V[i]);
			if (afDistance[i] > Math<Real>::ZERO_TOLERANCE)
			{
				aiSign[i] = 1;
				riPositive++;
			}
			else if (afDistance[i] < -Math<Real>::ZERO_TOLERANCE)
			{
				aiSign[i] = -1;
				riNegative++;
			}
			else
			{
				afDistance[i] = (Real)0.0;
				aiSign[i] = 0;
				riZero++;
			}
		}
	}
	//----------------------------------------------------------------------------
	template <class Real>
	void IntrTriangle3Triangle3<Real>::GetInterval(
		const Triangle3<Real>& rkTriangle, const Line3<Real>& rkLine,
		const Real afDistance[3], const int aiSign[3], Real afParam[2])
	{
		// project triangle onto line
		Real afProj[3];
		int i;
		for (i = 0; i < 3; i++)
		{
			Vector3<Real> kDiff = rkTriangle.V[i] - rkLine.Origin;
			afProj[i] = rkLine.Direction.Dot(kDiff);
		}

		// compute transverse intersections of triangle edges with line
		Real fNumer, fDenom;
		int i0, i1, i2;
		int iQuantity = 0;
		for (i0 = 2, i1 = 0; i1 < 3; i0 = i1++)
		{
			if (aiSign[i0] * aiSign[i1] < 0)
			{
				assert(iQuantity < 2);
				fNumer = afDistance[i0] * afProj[i1] - afDistance[i1] * afProj[i0];
				fDenom = afDistance[i0] - afDistance[i1];
				afParam[iQuantity++] = fNumer / fDenom;
			}
		}

		// check for grazing contact
		if (iQuantity < 2)
		{
			for (i0 = 1, i1 = 2, i2 = 0; i2 < 3; i0 = i1, i1 = i2, i2++)
			{
				if (aiSign[i2] == 0)
				{
					assert(iQuantity < 2);
					afParam[iQuantity++] = afProj[i2];
				}
			}
		}

		// sort
		assert(iQuantity == 1 || iQuantity == 2);
		if (iQuantity == 2)
		{
			if (afParam[0] > afParam[1])
			{
				Real fSave = afParam[0];
				afParam[0] = afParam[1];
				afParam[1] = fSave;
			}
		}
		else
		{
			afParam[1] = afParam[0];
		}
	}
	//----------------------------------------------------------------------------
	template <class Real>
	bool IntrTriangle3Triangle3<Real>::ContainsPoint(
		const Triangle3<Real>& rkTriangle, const Plane3<Real>& rkPlane,
		const Vector3<Real>& rkPoint)
	{
		// Generate a coordinate system for the plane.  The incoming triangle has
		// vertices <V0,V1,V2>.  The incoming plane has unit-length normal N.
		// The incoming point is P.  V0 is chosen as the origin for the plane. The
		// coordinate axis directions are two unit-length vectors, U0 and U1,
		// constructed so that {U0,U1,N} is an orthonormal set.  Any point Q
		// in the plane may be written as Q = V0 + x0*U0 + x1*U1.  The coordinates
		// are computed as x0 = Dot(U0,Q-V0) and x1 = Dot(U1,Q-V0).
		Vector3<Real> kU0, kU1;
		Vector3<Real>::GenerateComplementBasis(kU0, kU1, rkPlane.Normal);

		// Compute the planar coordinates for the points P, V1, and V2.  To
		// simplify matters, the origin is subtracted from the points, in which
		// case the planar coordinates are for P-V0, V1-V0, and V2-V0.
		Vector3<Real> kPmV0 = rkPoint - rkTriangle.V[0];
		Vector3<Real> kV1mV0 = rkTriangle.V[1] - rkTriangle.V[0];
		Vector3<Real> kV2mV0 = rkTriangle.V[2] - rkTriangle.V[0];

		// The planar representation of P-V0.
		Vector2<Real> kProjP(kU0.Dot(kPmV0), kU1.Dot(kPmV0));

		// The planar representation of the triangle <V0-V0,V1-V0,V2-V0>.
		Vector2<Real> akProjV[3] =
		{
			Vector2<Real>::ZERO,
			Vector2<Real>(kU0.Dot(kV1mV0),kU1.Dot(kV1mV0)),
			Vector2<Real>(kU0.Dot(kV2mV0),kU1.Dot(kV2mV0))
		};

		// Test whether P-V0 is in the triangle <0,V1-V0,V2-V0>.
		if (Query2<Real>(3, akProjV).ToTriangle(kProjP, 0, 1, 2) <= 0)
		{
			// Report the point of intersection to the caller.
			m_iQuantity = 1;
			m_akPoint[0] = rkPoint;
			return true;
		}

		return false;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	bool IntrTriangle3Triangle3<Real>::IntersectsSegment(
		const Plane3<Real>& rkPlane, const Triangle3<Real>& rkTriangle,
		const Vector3<Real>& rkEnd0, const Vector3<Real>& rkEnd1)
	{
		// Compute the 2D representations of the triangle vertices and the
		// segment endpoints relative to the plane of the triangle.  Then
		// compute the intersection in the 2D space.

		// Project the triangle and segment onto the coordinate plane most
		// aligned with the plane normal.
		int iMaxNormal = 0;
		Real fMax = Math<Real>::FAbs(rkPlane.Normal.X());
		Real fAbs = Math<Real>::FAbs(rkPlane.Normal.Y());
		if (fAbs > fMax)
		{
			iMaxNormal = 1;
			fMax = fAbs;
		}
		fAbs = Math<Real>::FAbs(rkPlane.Normal.Z());
		if (fAbs > fMax)
		{
			iMaxNormal = 2;
		}

		Triangle2<Real> kProjTri;
		Vector2<Real> kProjEnd0, kProjEnd1;
		int i;

		if (iMaxNormal == 0)
		{
			// project onto yz-plane
			for (i = 0; i < 3; i++)
			{
				kProjTri.V[i].X() = rkTriangle.V[i].Y();
				kProjTri.V[i].Y() = rkTriangle.V[i].Z();
				kProjEnd0.X() = rkEnd0.Y();
				kProjEnd0.Y() = rkEnd0.Z();
				kProjEnd1.X() = rkEnd1.Y();
				kProjEnd1.Y() = rkEnd1.Z();
			}
		}
		else if (iMaxNormal == 1)
		{
			// project onto xz-plane
			for (i = 0; i < 3; i++)
			{
				kProjTri.V[i].X() = rkTriangle.V[i].X();
				kProjTri.V[i].Y() = rkTriangle.V[i].Z();
				kProjEnd0.X() = rkEnd0.X();
				kProjEnd0.Y() = rkEnd0.Z();
				kProjEnd1.X() = rkEnd1.X();
				kProjEnd1.Y() = rkEnd1.Z();
			}
		}
		else
		{
			// project onto xy-plane
			for (i = 0; i < 3; i++)
			{
				kProjTri.V[i].X() = rkTriangle.V[i].X();
				kProjTri.V[i].Y() = rkTriangle.V[i].Y();
				kProjEnd0.X() = rkEnd0.X();
				kProjEnd0.Y() = rkEnd0.Y();
				kProjEnd1.X() = rkEnd1.X();
				kProjEnd1.Y() = rkEnd1.Y();
			}
		}

		Vector2<Real> kPSCenter = ((Real)0.5)*(kProjEnd0 + kProjEnd1);
		Vector2<Real> kPSDirection = kProjEnd1 - kProjEnd0;
		Real fPSExtent = ((Real)0.5)*kPSDirection.Normalize();
		Segment2<Real> kProjSeg(kPSCenter, kPSDirection, fPSExtent);
		IntrSegment2Triangle2<Real> kCalc(kProjSeg, kProjTri);
		if (!kCalc.Find())
		{
			return false;
		}

		Vector2<Real> akIntr[2];
		if (kCalc.GetIntersectionType() == IT_SEGMENT)
		{
			m_iIntersectionType = IT_SEGMENT;
			m_iQuantity = 2;
			akIntr[0] = kCalc.GetPoint(0);
			akIntr[1] = kCalc.GetPoint(1);
		}
		else
		{
			assert(kCalc.GetIntersectionType() == IT_POINT);
			m_iIntersectionType = IT_POINT;
			m_iQuantity = 1;
			akIntr[0] = kCalc.GetPoint(0);
		}

		// Unproject the segment of intersection.
		if (iMaxNormal == 0)
		{
			Real fInvNX = ((Real)1.0) / rkPlane.Normal.X();
			for (i = 0; i < m_iQuantity; i++)
			{
				m_akPoint[i].Y() = akIntr[i].X();
				m_akPoint[i].Z() = akIntr[i].Y();
				m_akPoint[i].X() = fInvNX * (rkPlane.Constant -
					rkPlane.Normal.Y()*m_akPoint[i].Y() -
					rkPlane.Normal.Z()*m_akPoint[i].Z());
			}
		}
		else if (iMaxNormal == 1)
		{
			Real fInvNY = ((Real)1.0) / rkPlane.Normal.Y();
			for (i = 0; i < m_iQuantity; i++)
			{
				m_akPoint[i].X() = akIntr[i].X();
				m_akPoint[i].Z() = akIntr[i].Y();
				m_akPoint[i].Y() = fInvNY * (rkPlane.Constant -
					rkPlane.Normal.X()*m_akPoint[i].X() -
					rkPlane.Normal.Z()*m_akPoint[i].Z());
			}
		}
		else
		{
			Real fInvNZ = ((Real)1.0) / rkPlane.Normal.Z();
			for (i = 0; i < m_iQuantity; i++)
			{
				m_akPoint[i].X() = akIntr[i].X();
				m_akPoint[i].Y() = akIntr[i].Y();
				m_akPoint[i].Z() = fInvNZ * (rkPlane.Constant -
					rkPlane.Normal.X()*m_akPoint[i].X() -
					rkPlane.Normal.Y()*m_akPoint[i].Y());
			}
		}

		return true;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	bool IntrTriangle3Triangle3<Real>::GetCoplanarIntersection(
		const Plane3<Real>& rkPlane, const Triangle3<Real>& rkTri0,
		const Triangle3<Real>& rkTri1)
	{
		// Project triangles onto coordinate plane most aligned with plane
		// normal.
		int iMaxNormal = 0;
		Real fMax = Math<Real>::FAbs(rkPlane.Normal.X());
		Real fAbs = Math<Real>::FAbs(rkPlane.Normal.Y());
		if (fAbs > fMax)
		{
			iMaxNormal = 1;
			fMax = fAbs;
		}
		fAbs = Math<Real>::FAbs(rkPlane.Normal.Z());
		if (fAbs > fMax)
		{
			iMaxNormal = 2;
		}

		Triangle2<Real> kProjTri0, kProjTri1;
		int i;

		if (iMaxNormal == 0)
		{
			// project onto yz-plane
			for (i = 0; i < 3; i++)
			{
				kProjTri0.V[i].X() = rkTri0.V[i].Y();
				kProjTri0.V[i].Y() = rkTri0.V[i].Z();
				kProjTri1.V[i].X() = rkTri1.V[i].Y();
				kProjTri1.V[i].Y() = rkTri1.V[i].Z();
			}
		}
		else if (iMaxNormal == 1)
		{
			// project onto xz-plane
			for (i = 0; i < 3; i++)
			{
				kProjTri0.V[i].X() = rkTri0.V[i].X();
				kProjTri0.V[i].Y() = rkTri0.V[i].Z();
				kProjTri1.V[i].X() = rkTri1.V[i].X();
				kProjTri1.V[i].Y() = rkTri1.V[i].Z();
			}
		}
		else
		{
			// project onto xy-plane
			for (i = 0; i < 3; i++)
			{
				kProjTri0.V[i].X() = rkTri0.V[i].X();
				kProjTri0.V[i].Y() = rkTri0.V[i].Y();
				kProjTri1.V[i].X() = rkTri1.V[i].X();
				kProjTri1.V[i].Y() = rkTri1.V[i].Y();
			}
		}

		// 2D triangle intersection routines require counterclockwise ordering
		Vector2<Real> kSave;
		Vector2<Real> kEdge0 = kProjTri0.V[1] - kProjTri0.V[0];
		Vector2<Real> kEdge1 = kProjTri0.V[2] - kProjTri0.V[0];
		if (kEdge0.DotPerp(kEdge1) < (Real)0.0)
		{
			// triangle is clockwise, reorder it
			kSave = kProjTri0.V[1];
			kProjTri0.V[1] = kProjTri0.V[2];
			kProjTri0.V[2] = kSave;
		}

		kEdge0 = kProjTri1.V[1] - kProjTri1.V[0];
		kEdge1 = kProjTri1.V[2] - kProjTri1.V[0];
		if (kEdge0.DotPerp(kEdge1) < (Real)0.0)
		{
			// triangle is clockwise, reorder it
			kSave = kProjTri1.V[1];
			kProjTri1.V[1] = kProjTri1.V[2];
			kProjTri1.V[2] = kSave;
		}

		IntrTriangle2Triangle2<Real> kIntr(kProjTri0, kProjTri1);
		if (!kIntr.Find())
		{
			return false;
		}

		// map 2D intersections back to the 3D triangle space
		m_iQuantity = kIntr.GetQuantity();
		if (iMaxNormal == 0)
		{
			Real fInvNX = ((Real)1.0) / rkPlane.Normal.X();
			for (i = 0; i < m_iQuantity; i++)
			{
				m_akPoint[i].Y() = kIntr.GetPoint(i).X();
				m_akPoint[i].Z() = kIntr.GetPoint(i).Y();
				m_akPoint[i].X() = fInvNX * (rkPlane.Constant -
					rkPlane.Normal.Y()*m_akPoint[i].Y() -
					rkPlane.Normal.Z()*m_akPoint[i].Z());
			}
		}
		else if (iMaxNormal == 1)
		{
			Real fInvNY = ((Real)1.0) / rkPlane.Normal.Y();
			for (i = 0; i < m_iQuantity; i++)
			{
				m_akPoint[i].X() = kIntr.GetPoint(i).X();
				m_akPoint[i].Z() = kIntr.GetPoint(i).Y();
				m_akPoint[i].Y() = fInvNY * (rkPlane.Constant -
					rkPlane.Normal.X()*m_akPoint[i].X() -
					rkPlane.Normal.Z()*m_akPoint[i].Z());
			}
		}
		else
		{
			Real fInvNZ = ((Real)1.0) / rkPlane.Normal.Z();
			for (i = 0; i < m_iQuantity; i++)
			{
				m_akPoint[i].X() = kIntr.GetPoint(i).X();
				m_akPoint[i].Y() = kIntr.GetPoint(i).Y();
				m_akPoint[i].Z() = fInvNZ * (rkPlane.Constant -
					rkPlane.Normal.X()*m_akPoint[i].X() -
					rkPlane.Normal.Y()*m_akPoint[i].Y());
			}
		}

		return true;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	bool IntrTriangle3Triangle3<Real>::TestOverlap(Real fTMax, Real fSpeed,
		Real fUMin, Real fUMax, Real fVMin, Real fVMax, Real& rfTFirst,
		Real& rfTLast)
	{
		// Constant velocity separating axis test.

		Real fT;

		if (fVMax < fUMin) // V on left of U
		{
			if (fSpeed <= (Real)0.0) // V moving away from U
			{
				return false;
			}

			// find first time of contact on this axis
			fT = (fUMin - fVMax) / fSpeed;
			if (fT > rfTFirst)
			{
				rfTFirst = fT;
			}

			// quick out: intersection after desired time interval
			if (rfTFirst > fTMax)
			{
				return false;
			}

			// find last time of contact on this axis
			fT = (fUMax - fVMin) / fSpeed;
			if (fT < rfTLast)
			{
				rfTLast = fT;
			}

			// quick out: intersection before desired time interval
			if (rfTFirst > rfTLast)
			{
				return false;
			}
		}
		else if (fUMax < fVMin)   // V on right of U
		{
			if (fSpeed >= (Real)0.0) // V moving away from U
			{
				return false;
			}

			// find first time of contact on this axis
			fT = (fUMax - fVMin) / fSpeed;
			if (fT > rfTFirst)
			{
				rfTFirst = fT;
			}

			// quick out: intersection after desired time interval
			if (rfTFirst > fTMax)
			{
				return false;
			}

			// find last time of contact on this axis
			fT = (fUMin - fVMax) / fSpeed;
			if (fT < rfTLast)
			{
				rfTLast = fT;
			}

			// quick out: intersection before desired time interval
			if (rfTFirst > rfTLast)
			{
				return false;
			}

		}
		else // V and U on overlapping interval
		{
			if (fSpeed > (Real)0.0)
			{
				// find last time of contact on this axis
				fT = (fUMax - fVMin) / fSpeed;
				if (fT < rfTLast)
				{
					rfTLast = fT;
				}

				// quick out: intersection before desired interval
				if (rfTFirst > rfTLast)
				{
					return false;
				}
			}
			else if (fSpeed < (Real)0.0)
			{
				// find last time of contact on this axis
				fT = (fUMin - fVMax) / fSpeed;
				if (fT < rfTLast)
				{
					rfTLast = fT;
				}

				// quick out: intersection before desired interval
				if (rfTFirst > rfTLast)
				{
					return false;
				}
			}
		}
		return true;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	bool IntrTriangle3Triangle3<Real>::FindOverlap(Real fTMax, Real fSpeed,
		const Configuration& rkUC, const Configuration& rkVC, ContactSide& rkSide,
		Configuration& rkTUC, Configuration& rkTVC, Real& rfTFirst, Real& rfTLast)
	{
		// Constant velocity separating axis test.  UC and VC are the new
		// potential configurations, and TUC and TVC are the best known
		// configurations.

		Real fT;

		if (rkVC.Max < rkUC.Min) // V on left of U
		{
			if (fSpeed <= (Real)0.0) // V moving away from U
			{
				return false;
			}

			// find first time of contact on this axis
			fT = (rkUC.Min - rkVC.Max) / fSpeed;

			// If this is the new maximum first time of contact, set side and
			// configuration.
			if (fT > rfTFirst)
			{
				rfTFirst = fT;
				rkSide = CS_LEFT;
				rkTUC = rkUC;
				rkTVC = rkVC;
			}

			// quick out: intersection after desired interval
			if (rfTFirst > fTMax)
			{
				return false;
			}

			// find last time of contact on this axis
			fT = (rkUC.Max - rkVC.Min) / fSpeed;
			if (fT < rfTLast)
			{
				rfTLast = fT;
			}

			// quick out: intersection before desired interval
			if (rfTFirst > rfTLast)
			{
				return false;
			}
		}
		else if (rkUC.Max < rkVC.Min)   // V on right of U
		{
			if (fSpeed >= (Real)0.0) // V moving away from U
			{
				return false;
			}

			// find first time of contact on this axis
			fT = (rkUC.Max - rkVC.Min) / fSpeed;

			// If this is the new maximum first time of contact, set side and
			// configuration.
			if (fT > rfTFirst)
			{
				rfTFirst = fT;
				rkSide = CS_RIGHT;
				rkTUC = rkUC;
				rkTVC = rkVC;
			}

			// quick out: intersection after desired interval
			if (rfTFirst > fTMax)
			{
				return false;
			}

			// find last time of contact on this axis
			fT = (rkUC.Min - rkVC.Max) / fSpeed;
			if (fT < rfTLast)
			{
				rfTLast = fT;
			}

			// quick out: intersection before desired interval
			if (rfTFirst > rfTLast)
			{
				return false;
			}
		}
		else // V and U on overlapping interval
		{
			if (fSpeed > (Real)0.0)
			{
				// find last time of contact on this axis
				fT = (rkUC.Max - rkVC.Min) / fSpeed;
				if (fT < rfTLast)
				{
					rfTLast = fT;
				}

				// quick out: intersection before desired interval
				if (rfTFirst > rfTLast)
				{
					return false;
				}
			}
			else if (fSpeed < (Real)0.0)
			{
				// find last time of contact on this axis
				fT = (rkUC.Min - rkVC.Max) / fSpeed;
				if (fT < rfTLast)
				{
					rfTLast = fT;
				}

				// quick out: intersection before desired interval
				if (rfTFirst > rfTLast)
				{
					return false;
				}
			}
		}
		return true;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	bool IntrTriangle3Triangle3<Real>::TestOverlap(const Vector3<Real>& rkAxis,
		Real fTMax, const Vector3<Real>& rkVelocity, Real& rfTFirst,
		Real& rfTLast)
	{
		Real fMin0, fMax0, fMin1, fMax1;
		ProjectOntoAxis(*m_pkTriangle0, rkAxis, fMin0, fMax0);
		ProjectOntoAxis(*m_pkTriangle1, rkAxis, fMin1, fMax1);
		Real fSpeed = rkVelocity.Dot(rkAxis);
		return TestOverlap(fTMax, fSpeed, fMin0, fMax0, fMin1, fMax1, rfTFirst,
			rfTLast);
	}
	//----------------------------------------------------------------------------
	template <class Real>
	bool IntrTriangle3Triangle3<Real>::PointIsInSegment(const Vector3<Real>& pt, const Vector3<Real>& segs, const Vector3<Real>& sege)
	{
		Vector3<Real> ps = segs - pt;
		Vector3<Real> pe = sege - pt;
		Real rsecl = ps.Cross(pe).SquaredLength();
		if (rsecl > Math<Real>::ZERO_TOLERANCE)
		{
			return false;
		}

		Real dr = ps.Dot(pe);
		if (dr > Math<Real>::ZERO_TOLERANCE)
			return false;
		return true;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	bool IntrTriangle3Triangle3<Real>::FindOverlap(const Vector3<Real>& rkAxis,
		Real fTMax, const Vector3<Real>& rkVelocity, ContactSide& reSide,
		Configuration& rkTCfg0, Configuration& rkTCfg1, Real& rfTFirst,
		Real& rfTLast)
	{
		Configuration kCfg0, kCfg1;
		ProjectOntoAxis(*m_pkTriangle0, rkAxis, kCfg0);
		ProjectOntoAxis(*m_pkTriangle1, rkAxis, kCfg1);
		Real fSpeed = rkVelocity.Dot(rkAxis);
		return FindOverlap(fTMax, fSpeed, kCfg0, kCfg1, reSide, rkTCfg0, rkTCfg1,
			rfTFirst, rfTLast);
	}
	//----------------------------------------------------------------------------
	template <class Real>
	void IntrTriangle3Triangle3<Real>::ProjectOntoAxis(
		const Triangle3<Real>& rkTri, const Vector3<Real>& rkAxis,
		Configuration& rkCfg)
	{
		// find projections of vertices onto potential separating axis
		Real fD0 = rkAxis.Dot(rkTri.V[0]);
		Real fD1 = rkAxis.Dot(rkTri.V[1]);
		Real fD2 = rkAxis.Dot(rkTri.V[2]);

		// explicit sort of vertices to construct a Configuration object
		if (fD0 <= fD1)
		{
			if (fD1 <= fD2) // D0 <= D1 <= D2
			{
				if (fD0 != fD1)
				{
					if (fD1 != fD2)
					{
						rkCfg.Map = M111;
					}
					else
					{
						rkCfg.Map = M12;
					}
				}
				else // ( D0 == D1 )
				{
					if (fD1 != fD2)
					{
						rkCfg.Map = M21;
					}
					else
					{
						rkCfg.Map = M3;
					}
				}
				rkCfg.Index[0] = 0;
				rkCfg.Index[1] = 1;
				rkCfg.Index[2] = 2;
				rkCfg.Min = fD0;
				rkCfg.Max = fD2;
			}
			else if (fD0 <= fD2) // D0 <= D2 < D1
			{
				if (fD0 != fD2)
				{
					rkCfg.Map = M111;
					rkCfg.Index[0] = 0;
					rkCfg.Index[1] = 2;
					rkCfg.Index[2] = 1;
				}
				else
				{
					rkCfg.Map = M21;
					rkCfg.Index[0] = 2;
					rkCfg.Index[1] = 0;
					rkCfg.Index[2] = 1;
				}
				rkCfg.Min = fD0;
				rkCfg.Max = fD1;
			}
			else // D2 < D0 <= D1
			{
				if (fD0 != fD1)
				{
					rkCfg.Map = M111;
				}
				else
				{
					rkCfg.Map = M12;
				}

				rkCfg.Index[0] = 2;
				rkCfg.Index[1] = 0;
				rkCfg.Index[2] = 1;
				rkCfg.Min = fD2;
				rkCfg.Max = fD1;
			}
		}
		else if (fD2 <= fD1) // D2 <= D1 < D0
		{
			if (fD2 != fD1)
			{
				rkCfg.Map = M111;
				rkCfg.Index[0] = 2;
				rkCfg.Index[1] = 1;
				rkCfg.Index[2] = 0;
			}
			else
			{
				rkCfg.Map = M21;
				rkCfg.Index[0] = 1;
				rkCfg.Index[1] = 2;
				rkCfg.Index[2] = 0;

			}
			rkCfg.Min = fD2;
			rkCfg.Max = fD0;
		}
		else if (fD2 <= fD0) // D1 < D2 <= D0
		{
			if (fD2 != fD0)
			{
				rkCfg.Map = M111;
			}
			else
			{
				rkCfg.Map = M12;
			}

			rkCfg.Index[0] = 1;
			rkCfg.Index[1] = 2;
			rkCfg.Index[2] = 0;
			rkCfg.Min = fD1;
			rkCfg.Max = fD0;
		}
		else // D1 < D0 < D2
		{
			rkCfg.Map = M111;
			rkCfg.Index[0] = 1;
			rkCfg.Index[1] = 0;
			rkCfg.Index[2] = 2;
			rkCfg.Min = fD1;
			rkCfg.Max = fD2;
		}
	}
	//----------------------------------------------------------------------------
	template <class Real>
	void IntrTriangle3Triangle3<Real>::FindContactSet(
		const Triangle3<Real>& rkTri0, const Triangle3<Real>& rkTri1,
		ContactSide& reSide, Configuration& rkCfg0, Configuration& rkCfg1)
	{
		if (reSide == CS_RIGHT) // tri1 to the right of tri0
		{
			if (rkCfg0.Map == M21 || rkCfg0.Map == M111)
			{
				// tri0 touching tri1 at a single point
				m_iIntersectionType = IT_POINT;
				m_iQuantity = 1;
				m_akPoint[0] = rkTri0.V[2];
			}
			else if (rkCfg1.Map == M12 || rkCfg1.Map == M111)
			{
				// tri1 touching tri0 at a single point
				m_iIntersectionType = IT_POINT;
				m_iQuantity = 1;
				m_akPoint[0] = rkTri1.V[0];
			}
			else if (rkCfg0.Map == M12)
			{
				if (rkCfg1.Map == M21)
				{
					// edge0-edge1 intersection
					GetEdgeEdgeIntersection(rkTri0.V[1], rkTri0.V[2], rkTri1.V[0],
						rkTri1.V[1]);
				}
				else // rkCfg1.Map == m3
				{
					// uedge-vface intersection
					GetEdgeFaceIntersection(rkTri0.V[1], rkTri0.V[2], rkTri1);
				}
			}
			else // rkCfg0.Map == M3
			{
				if (rkCfg1.Map == M21)
				{
					// face0-edge1 intersection
					GetEdgeFaceIntersection(rkTri1.V[0], rkTri1.V[1], rkTri0);
				}
				else // rkCfg1.Map == M3
				{
					// face0-face1 intersection
					Plane3<Real> kPlane0(rkTri0.V[0], rkTri0.V[1], rkTri0.V[2]);
					GetCoplanarIntersection(kPlane0, rkTri0, rkTri1);
				}
			}
		}
		else if (reSide == CS_LEFT) // tri1 to the left of tri0
		{
			if (rkCfg1.Map == M21 || rkCfg1.Map == M111)
			{
				// tri1 touching tri0 at a single point
				m_iIntersectionType = IT_POINT;
				m_iQuantity = 1;
				m_akPoint[0] = rkTri1.V[2];
			}
			else if (rkCfg0.Map == M12 || rkCfg0.Map == M111)
			{
				// tri0 touching tri1 at a single point
				m_iIntersectionType = IT_POINT;
				m_iQuantity = 1;
				m_akPoint[0] = rkTri0.V[0];
			}
			else if (rkCfg1.Map == M12)
			{
				if (rkCfg0.Map == M21)
				{
					// edge0-edge1 intersection
					GetEdgeEdgeIntersection(rkTri0.V[0], rkTri0.V[1], rkTri1.V[1],
						rkTri1.V[2]);
				}
				else // rkCfg0.Map == M3
				{
					// edge1-face0 intersection
					GetEdgeFaceIntersection(rkTri1.V[1], rkTri1.V[2], rkTri0);
				}
			}
			else // rkCfg1.Map == M3
			{
				if (rkCfg0.Map == M21)
				{
					// edge0-face1 intersection
					GetEdgeFaceIntersection(rkTri0.V[0], rkTri0.V[1], rkTri1);
				}
				else // rkCfg0.Map == M
				{
					// face0-face1 intersection
					Plane3<Real> kPlane0(rkTri0.V[0], rkTri0.V[1], rkTri0.V[2]);
					GetCoplanarIntersection(kPlane0, rkTri0, rkTri1);
				}
			}
		}
		else // reSide == CS_NONE
		{
			// triangles are already intersecting tranversely
			IntrTriangle3Triangle3<Real> kCalc(rkTri0, rkTri1);
			bool bResult = kCalc.Find();
			assert(bResult);
			(void)bResult;
			m_iIntersectionType = kCalc.GetIntersectionType();
			m_iQuantity = kCalc.GetQuantity();
			for (int i = 0; i < m_iQuantity; i++)
			{
				m_akPoint[i] = kCalc.GetPoint(i);
			}
		}
	}
	//----------------------------------------------------------------------------
	template <class Real>
	void IntrTriangle3Triangle3<Real>::GetEdgeEdgeIntersection(
		const Vector3<Real>& rkU0, const Vector3<Real>& rkU1,
		const Vector3<Real>& rkV0, const Vector3<Real>& rkV1)
	{
		// Compute a normal to the plane of the two edges.
		Vector3<Real> kEdge0 = rkU1 - rkU0;
		Vector3<Real> kEdge1 = rkV1 - rkV0;
		Vector3<Real> kNormal = kEdge0.Cross(kEdge1);

		// Solve U0 + s*(U1 - U0) = V0 + t*(V1 - V0).  We know the edges
		// intersect, so s in [0,1] and t in [0,1].  Thus, just solve for s.
		// Note that s*E0 = D + t*E1, where D = V0 - U0. So s*N = s*E0xE1 = DxE1
		// and s = N*DxE1/N*N.
		Vector3<Real> kDelta = rkV0 - rkU0;
		Real fS = kNormal.Dot(kDelta.Cross(kEdge1)) / kNormal.SquaredLength();
		assert((Real)0 <= fS && fS <= (Real)1);

		m_iIntersectionType = IT_POINT;
		m_iQuantity = 1;
		m_akPoint[0] = rkU0 + fS * kEdge0;

		// TODO:  What if the edges are parallel?
	}
	//----------------------------------------------------------------------------
	template <typename Real>
	void IntrTriangle3Triangle3<Real>::GetEdgeFaceIntersection(
		const Vector3<Real>& rkU0, const Vector3<Real>& rkU1,
		const Triangle3<Real>& rkTri)
	{
		// Compute a plane of the triangle.
		Vector3<Real> kPoint = rkTri.V[0];
		Vector3<Real> kEdge0 = rkTri.V[1] - kPoint;
		Vector3<Real> kEdge1 = rkTri.V[2] - kPoint;
		Vector3<Real> kNormal = kEdge0.UnitCross(kEdge1);
		Vector3<Real> kDir0, kDir1;
		Vector3<Real>::GenerateComplementBasis(kDir0, kDir1, kNormal);

		// Project the edge endpoints onto the plane.
		Vector2<Real> kProjU0, kProjU1;
		Vector3<Real> kDiff;
		kDiff = rkU0 - kPoint;
		kProjU0[0] = kDir0.Dot(kDiff);
		kProjU0[1] = kDir1.Dot(kDiff);
		kDiff = rkU1 - kPoint;
		kProjU1[0] = kDir0.Dot(kDiff);
		kProjU1[1] = kDir1.Dot(kDiff);

		Vector2<Real> kPSCenter = ((Real)0.5)*(kProjU0 + kProjU1);
		Vector2<Real> kPSDirection = kProjU1 - kProjU0;
		Real fPSExtent = ((Real)0.5)*kPSDirection.Normalize();
		Segment2<Real> kProjSeg(kPSCenter, kPSDirection, fPSExtent);

		// Compute the plane coordinates of the triangle.
		Triangle2<Real> kProjTri;
		kProjTri.V[0] = Vector2<Real>::ZERO;
		kProjTri.V[1] = Vector2<Real>(kDir0.Dot(kEdge0), kDir1.Dot(kEdge0));
		kProjTri.V[2] = Vector2<Real>(kDir0.Dot(kEdge1), kDir1.Dot(kEdge1));

		// Compute the intersection.
		IntrSegment2Triangle2<Real> kCalc(kProjSeg, kProjTri);
		bool bResult = kCalc.Find();
		assert(bResult);
		(void)bResult;
		m_iQuantity = kCalc.GetQuantity();
		for (int i = 0; i < m_iQuantity; i++)
		{
			Vector2<Real> kProj = kCalc.GetPoint(i);
			m_akPoint[i] = kPoint + kProj[0] * kDir0 + kProj[1] * kDir1;
		}

		m_iIntersectionType = (m_iQuantity == 2 ? IT_SEGMENT : IT_POINT);
	}
	//----------------------------------------------------------------------------

	//----------------------------------------------------------------------------
	// explicit instantiation
	//----------------------------------------------------------------------------
	template class Intersector1<float>;
	template class Intersector1<double>;
	//----------------------------------------------------------------------------

	//----------------------------------------------------------------------------
	// explicit instantiation
	//----------------------------------------------------------------------------
	template class Intersector<float, Vector2f>;
	template class Intersector<float, Vector3f>;
	template class Intersector<double, Vector2d>;
	template class Intersector<double, Vector3d>;
	//----------------------------------------------------------------------------
	//----------------------------------------------------------------------------
	// explicit instantiation
	//----------------------------------------------------------------------------
	template class IntrLine2Triangle2<float>;
	template class IntrLine2Triangle2<double>;
	//----------------------------------------------------------------------------
	//----------------------------------------------------------------------------
	// explicit instantiation
	//----------------------------------------------------------------------------
	template class IntrSegment2Triangle2<float>;
	template class IntrSegment2Triangle2<double>;
	//----------------------------------------------------------------------------
	//----------------------------------------------------------------------------
	// explicit instantiation
	//----------------------------------------------------------------------------
	template class IntrTriangle2Triangle2<float>;
	template class IntrTriangle2Triangle2<double>;
	//----------------------------------------------------------------------------
	//----------------------------------------------------------------------------
	// explicit instantiation
	//----------------------------------------------------------------------------
	template class IntrTriangle3Triangle3<float>;
	template class IntrTriangle3Triangle3<double>;
	//----------------------------------------------------------------------------
}
