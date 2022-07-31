#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable:4273)
#pragma warning(disable:4251)
#pragma warning(disable:4286)
#pragma warning(disable:4267)
#pragma warning(disable:4244)
#pragma warning(disable:4018)
#pragma warning(disable:26495)
#pragma warning(disable:26817)
#endif
#include <MeshDoctor/RobustTriTriIntersection.h>
namespace Cart3DAlgorithm
{
	//----------------------------------------------------------------------------
	Intersector::Intersector()
	{
		m_fContactTime = (cfloat)0.0;
		m_iIntersectionType = IT_EMPTY;
	}
	//----------------------------------------------------------------------------

	Intersector::~Intersector()
	{
	}
	//----------------------------------------------------------------------------

	cfloat Intersector::GetContactTime() const
	{
		return m_fContactTime;
	}
	//----------------------------------------------------------------------------

	int Intersector::GetIntersectionType() const
	{
		return m_iIntersectionType;
	}
	//----------------------------------------------------------------------------

	bool Intersector::Test()
	{
		// stub for derived class
		assert(false);
		return false;
	}
	//----------------------------------------------------------------------------

	bool Intersector::Find()
	{
		// stub for derived class
		assert(false);
		return false;
	}
	//----------------------------------------------------------------------------

	bool Intersector::Test(cfloat, const cvector3d&, const cvector3d&)
	{
		// stub for derived class
		assert(false);
		return false;
	}
	//----------------------------------------------------------------------------

	bool Intersector::Find(cfloat, const cvector3d&, const cvector3d&)
	{
		// stub for derived class
		assert(false);
		return false;
	}
	//----------------------------------------------------------------------------
	//----------------------------------------------------------------------------
	Intersector1::Intersector1(cfloat fU0, cfloat fU1, cfloat fV0, cfloat fV1)
	{
		assert(fU0 <= fU1 && fV0 <= fV1);
		m_afU[0] = fU0;
		m_afU[1] = fU1;
		m_afV[0] = fV0;
		m_afV[1] = fV1;
		m_fFirstTime = (cfloat)0.0;
		m_fLastTime = (cfloat)0.0;
		m_iQuantity = 0;
	}
	//----------------------------------------------------------------------------
	Intersector1::Intersector1(cfloat afU[2], cfloat afV[2])
	{
		assert(afU[0] <= afU[1] && afV[0] <= afV[1]);
		for (int i = 0; i < 2; i++)
		{
			m_afU[i] = afU[i];
			m_afV[i] = afV[i];
		}
		m_fFirstTime = (cfloat)0.0;
		m_fLastTime = (cfloat)0.0;
		m_iQuantity = 0;
	}
	//----------------------------------------------------------------------------
	Intersector1::~Intersector1()
	{
	}
	//----------------------------------------------------------------------------
	cfloat Intersector1::GetU(int i) const
	{
		assert(0 <= i && i < 2);
		return m_afU[i];
	}
	//----------------------------------------------------------------------------
	cfloat Intersector1::GetV(int i) const
	{
		assert(0 <= i && i < 2);
		return m_afV[i];
	}
	//----------------------------------------------------------------------------
	bool Intersector1::Test()
	{
		return m_afU[0] <= m_afV[1] && m_afU[1] >= m_afV[0];
	}
	//----------------------------------------------------------------------------
	bool Intersector1::Find()
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
	bool Intersector1::Test(cfloat fTMax, cfloat fSpeedU, cfloat fSpeedV)
	{
		cfloat fDiffSpeed, fInvDiffSpeed, fDiffPos;

		if (m_afU[1] < m_afV[0])
		{
			// [u0,u1] initially to the left of [v0,v1]
			fDiffSpeed = fSpeedU - fSpeedV;
			if (fDiffSpeed > (cfloat)0.0)
			{
				// the intervals must move towards each other
				fDiffPos = m_afV[0] - m_afU[1];
				if (fDiffPos <= fTMax * fDiffSpeed)
				{
					// the intervals intersect within the specified time
					fInvDiffSpeed = ((cfloat)1.0) / fDiffSpeed;
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
			if (fDiffSpeed > (cfloat)0.0)
			{
				// the intervals must move towards each other
				fDiffPos = m_afU[0] - m_afV[1];
				if (fDiffPos <= fTMax * fDiffSpeed)
				{
					// the intervals intersect within the specified time
					fInvDiffSpeed = ((cfloat)1.0) / fDiffSpeed;
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
				m_fLastTime = maxcfloat;
			}

			return true;
		}

		return false;
	}
	//----------------------------------------------------------------------------

	bool Intersector1::Find(cfloat fTMax, cfloat fSpeedU, cfloat fSpeedV)
	{
		cfloat fDiffSpeed, fInvDiffSpeed, fDiffPos;

		if (m_afU[1] < m_afV[0])
		{
			// [u0,u1] initially to the left of [v0,v1]
			fDiffSpeed = fSpeedU - fSpeedV;
			if (fDiffSpeed > (cfloat)0.0)
			{
				// the intervals must move towards each other
				fDiffPos = m_afV[0] - m_afU[1];
				if (fDiffPos <= fTMax * fDiffSpeed)
				{
					// the intervals intersect within the specified time
					fInvDiffSpeed = ((cfloat)1.0) / fDiffSpeed;
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
			if (fDiffSpeed > (cfloat)0.0)
			{
				// the intervals must move towards each other
				fDiffPos = m_afU[0] - m_afV[1];
				if (fDiffPos <= fTMax * fDiffSpeed)
				{
					// the intervals intersect within the specified time
					fInvDiffSpeed = ((cfloat)1.0) / fDiffSpeed;
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
				m_fLastTime = maxcfloat;
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

	cfloat Intersector1::GetFirstTime() const
	{
		return m_fFirstTime;
	}
	//----------------------------------------------------------------------------

	cfloat Intersector1::GetLastTime() const
	{
		return m_fLastTime;
	}
	//----------------------------------------------------------------------------

	int Intersector1::GetQuantity() const
	{
		return m_iQuantity;
	}
	//----------------------------------------------------------------------------

	cfloat Intersector1::GetOverlap(int i) const
	{
		assert(0 <= i && i < m_iQuantity);
		return m_afOverlap[i];
	}
	//----------------------------------------------------------------------------
	//----------------------------------------------------------------------------

	IntrLine2Triangle2::IntrLine2Triangle2(const Line2d& rkLine,
		const Triangle2d& rkTriangle):
		m_pkLine(&rkLine),
		m_pkTriangle(&rkTriangle)
	{
	}
	//----------------------------------------------------------------------------

	const Line2d& IntrLine2Triangle2::GetLine() const
	{
		return *m_pkLine;
	}
	//----------------------------------------------------------------------------

	const Triangle2d& IntrLine2Triangle2::GetTriangle() const
	{
		return *m_pkTriangle;
	}
	//----------------------------------------------------------------------------

	bool IntrLine2Triangle2::Test()
	{
		cfloat afDist[3];
		int aiSign[3], iPositive, iNegative, iZero;
		TriangleLineRelations(m_pkLine->origin, m_pkLine->direction, *m_pkTriangle,
			afDist, aiSign, iPositive, iNegative, iZero);

		if (iPositive == 3 || iNegative == 3)
		{
			m_iIntersectionType = IT_EMPTY;
		}
		else
		{
			cfloat afParam[2];
			GetInterval(m_pkLine->origin, m_pkLine->direction, *m_pkTriangle, afDist,
				aiSign, afParam);

			Intersector1 kIntr(afParam[0], afParam[1],
				-maxcfloat, +maxcfloat);

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

	bool IntrLine2Triangle2::Find()
	{
		cfloat afDist[3];
		int aiSign[3], iPositive, iNegative, iZero;
		TriangleLineRelations(m_pkLine->origin, m_pkLine->direction, *m_pkTriangle,
			afDist, aiSign, iPositive, iNegative, iZero);

		if (iPositive == 3 || iNegative == 3)
		{
			// No intersections.
			m_iQuantity = 0;
			m_iIntersectionType = IT_EMPTY;
		}
		else
		{
			cfloat afParam[2];
			GetInterval(m_pkLine->origin, m_pkLine->direction, *m_pkTriangle, afDist,
				aiSign, afParam);

			Intersector1 kIntr(afParam[0], afParam[1],
				-maxcfloat, +maxcfloat);

			kIntr.Find();

			m_iQuantity = kIntr.GetQuantity();
			if (m_iQuantity == 2)
			{
				// Segment intersection.
				m_iIntersectionType = IT_SEGMENT;
				m_akPoint[0] = m_pkLine->origin + kIntr.GetOverlap(0)*
					m_pkLine->direction;
				m_akPoint[1] = m_pkLine->origin + kIntr.GetOverlap(1)*
					m_pkLine->direction;
			}
			else if (m_iQuantity == 1)
			{
				// Point intersection.
				m_iIntersectionType = IT_POINT;
				m_akPoint[0] = m_pkLine->origin + kIntr.GetOverlap(0)*
					m_pkLine->direction;
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

	int IntrLine2Triangle2::GetQuantity() const
	{
		return m_iQuantity;
	}
	//----------------------------------------------------------------------------

	const cvector2d& IntrLine2Triangle2::GetPoint(int i) const
	{
		return m_akPoint[i];
	}
	//----------------------------------------------------------------------------
	static inline cfloat DotPerp(const cvector2d& mIpt,const cvector2d&mIqt)
	{
		return mIpt[0] * mIqt[1] - mIpt[1] * mIqt[0];
	}

	void IntrLine2Triangle2::TriangleLineRelations(
		const cvector2d& rkorigin, const cvector2d& rkdirection,
		const Triangle2d& rkTriangle, cfloat afDist[3], int aiSign[3],
		int& riPositive, int& riNegative, int& riZero)
	{
		riPositive = 0;
		riNegative = 0;
		riZero = 0;
		for (int i = 0; i < 3; ++i)
		{
			cvector2d kDiff = rkTriangle.v[i] - rkorigin;
			afDist[i] = DotPerp(kDiff,rkdirection);
			if (afDist[i] > inv_trunc_val)
			{
				aiSign[i] = 1;
				riPositive++;
			}
			else if (afDist[i] < -inv_trunc_val)
			{
				aiSign[i] = -1;
				riNegative++;
			}
			else
			{
				afDist[i] = (cfloat)0.0;
				aiSign[i] = 0;
				riZero++;
			}
		}
	}
	//----------------------------------------------------------------------------

	void IntrLine2Triangle2::GetInterval(const cvector2d& rkorigin,
		const cvector2d& rkdirection, const Triangle2d& rkTriangle,
		const cfloat afDist[3], const int aiSign[3], cfloat afParam[2])
	{
		// Project triangle onto line.
		cfloat afProj[3];
		int i;
		for (i = 0; i < 3; i++)
		{
			cvector2d kDiff = rkTriangle.v[i] - rkorigin;
			afProj[i] = rkdirection.dot(kDiff);
		}

		// Compute transverse intersections of triangle edges with line.
		cfloat fNumer, fDenom;
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
				cfloat fSave = afParam[0];
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

	IntrSegment2Triangle2::IntrSegment2Triangle2(
		const Segment2d& rkSegment, const Triangle2d& rkTriangle)
		:
		m_pkSegment(&rkSegment),
		m_pkTriangle(&rkTriangle)
	{
	}
	//----------------------------------------------------------------------------

	const Segment2d& IntrSegment2Triangle2::GetSegment() const
	{
		return *m_pkSegment;
	}
	//----------------------------------------------------------------------------

	const Triangle2d& IntrSegment2Triangle2::GetTriangle() const
	{
		return *m_pkTriangle;
	}
	//----------------------------------------------------------------------------

	bool IntrSegment2Triangle2::Test()
	{
		cfloat afDist[3];
		int aiSign[3], iPositive, iNegative, iZero;
		IntrLine2Triangle2::TriangleLineRelations(m_pkSegment->origin,
			m_pkSegment->direction, *m_pkTriangle, afDist, aiSign, iPositive,
			iNegative, iZero);

		if (iPositive == 3 || iNegative == 3)
		{
			m_iIntersectionType = IT_EMPTY;
		}
		else
		{
			cfloat afParam[2];
			IntrLine2Triangle2::GetInterval(m_pkSegment->origin,
				m_pkSegment->direction, *m_pkTriangle, afDist, aiSign, afParam);

			Intersector1 kIntr(afParam[0], afParam[1],
				-m_pkSegment->extent, +m_pkSegment->extent);

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

	bool IntrSegment2Triangle2::Find()
	{
		cfloat afDist[3];
		int aiSign[3], iPositive, iNegative, iZero;
		IntrLine2Triangle2::TriangleLineRelations(m_pkSegment->origin,
			m_pkSegment->direction, *m_pkTriangle, afDist, aiSign, iPositive,
			iNegative, iZero);

		if (iPositive == 3 || iNegative == 3)
		{
			// No intersections.
			m_iQuantity = 0;
			m_iIntersectionType = IT_EMPTY;
		}
		else
		{
			cfloat afParam[2];
			IntrLine2Triangle2::GetInterval(m_pkSegment->origin,
				m_pkSegment->direction, *m_pkTriangle, afDist, aiSign, afParam);

			Intersector1 kIntr(afParam[0], afParam[1],
				-m_pkSegment->extent, +m_pkSegment->extent);

			kIntr.Find();

			m_iQuantity = kIntr.GetQuantity();
			if (m_iQuantity == 2)
			{
				// Segment intersection.
				m_iIntersectionType = IT_SEGMENT;
				m_akPoint[0] = m_pkSegment->origin + kIntr.GetOverlap(0)*
					m_pkSegment->direction;
				m_akPoint[1] = m_pkSegment->origin + kIntr.GetOverlap(1)*
					m_pkSegment->direction;
			}
			else if (m_iQuantity == 1)
			{
				// Point intersection.
				m_iIntersectionType = IT_POINT;
				m_akPoint[0] = m_pkSegment->origin + kIntr.GetOverlap(0)*
					m_pkSegment->direction;
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

	int IntrSegment2Triangle2::GetQuantity() const
	{
		return m_iQuantity;
	}
	//----------------------------------------------------------------------------

	const cvector2d& IntrSegment2Triangle2::GetPoint(int i) const
	{
		return m_akPoint[i];
	}
	//----------------------------------------------------------------------------

	//----------------------------------------------------------------------------

	IntrTriangle2Triangle2::IntrTriangle2Triangle2(
		const Triangle2d& rkTriangle0, const Triangle2d& rkTriangle1)
		:
		m_pkTriangle0(&rkTriangle0),
		m_pkTriangle1(&rkTriangle1)
	{
		m_iQuantity = 0;
	}
	//----------------------------------------------------------------------------

	const Triangle2d& IntrTriangle2Triangle2::GetTriangle0() const
	{
		return *m_pkTriangle0;
	}
	//----------------------------------------------------------------------------

	const Triangle2d& IntrTriangle2Triangle2::GetTriangle1() const
	{
		return *m_pkTriangle1;
	}
	//----------------------------------------------------------------------------

	bool IntrTriangle2Triangle2::Test()
	{
		int i0, i1;
		cvector2d kDir;

		// test edges of triangle0 for separation
		for (i0 = 0, i1 = 2; i0 < 3; i1 = i0, i0++)
		{
			// test axis V0[i1] + t*perp(V0[i0]-V0[i1]), perp(x,y) = (y,-x)
			kDir.x() = m_pkTriangle0->v[i0].y() - m_pkTriangle0->v[i1].y();
			kDir.y() = m_pkTriangle0->v[i1].x() - m_pkTriangle0->v[i0].x();
			if (WhichSide(m_pkTriangle1->v, m_pkTriangle0->v[i1], kDir) > 0)
			{
				// triangle1 is entirely on positive side of triangle0 edge
				return false;
			}
		}

		// test edges of triangle1 for separation
		for (i0 = 0, i1 = 2; i0 < 3; i1 = i0, i0++)
		{
			// test axis V1[i1] + t*perp(V1[i0]-V1[i1]), perp(x,y) = (y,-x)
			kDir.x() = m_pkTriangle1->v[i0].y() - m_pkTriangle1->v[i1].y();
			kDir.y() = m_pkTriangle1->v[i1].x() - m_pkTriangle1->v[i0].x();
			if (WhichSide(m_pkTriangle0->v, m_pkTriangle1->v[i1], kDir) > 0)
			{
				// triangle0 is entirely on positive side of triangle1 edge
				return false;
			}
		}

		return true;
	}
	//----------------------------------------------------------------------------

	bool IntrTriangle2Triangle2::Find()
	{
		// The potential intersection is initialized to triangle1.  The set of
		// vertices is refined based on clipping against each edge of triangle0.
		m_iQuantity = 3;
		for (int i = 0; i < 3; i++)
		{
			m_akPoint[i] = m_pkTriangle1->v[i];
		}

		for (int i1 = 2, i0 = 0; i0 < 3; i1 = i0, i0++)
		{
			// clip against edge <V0[i1],V0[i0]>
			cvector2d kN(
				m_pkTriangle0->v[i1].y() - m_pkTriangle0->v[i0].y(),
				m_pkTriangle0->v[i0].x() - m_pkTriangle0->v[i1].x());
			cfloat fC = kN.dot(m_pkTriangle0->v[i1]);
			ClipConvexPolygonAgainstLine(kN, fC, m_iQuantity, m_akPoint);
			if (m_iQuantity == 0)
			{
				return false;
			}
		}

		return true;
	}
	//----------------------------------------------------------------------------

	bool IntrTriangle2Triangle2::Test(cfloat fTMax,
		const cvector2d& rkVelocity0, const cvector2d& rkVelocity1)
	{
		// process as if V0-triangle is stationary and V1-triangle is moving
		cvector2d kW = rkVelocity1 - rkVelocity0;
		int iSide = 0;  // 0 = NONE, -1 = LEFT, +1 = RIGHT
		cfloat fTFirst = (cfloat)0.0;
		cfloat fTLast = maxcfloat;

		Configuration kCfg0, kCfg1, kTCfg0, kTCfg1;
		int i0, i1, i2;
		cvector2d kD;
		cfloat fSpeed;

		// process edges of V0-triangle
		for (i0 = 1, i1 = 2, i2 = 0; i2 < 3; i0 = i1, i1 = i2, i2++)
		{
			// test axis V0[i1] + t*perp(V0[i2]-V0[i1]), perp(x,y) = (y,-x)
			kD.x() = m_pkTriangle0->v[i2].y() - m_pkTriangle0->v[i1].y();
			kD.y() = m_pkTriangle0->v[i1].x() - m_pkTriangle0->v[i2].x();
			fSpeed = kD.dot(kW);

			ComputeTwo(kCfg0, m_pkTriangle0->v, kD, i0, i1, i2);
			ComputeThree(kCfg1, m_pkTriangle1->v, kD, m_pkTriangle0->v[i1]);

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
			kD.x() = m_pkTriangle1->v[i2].y() - m_pkTriangle1->v[i1].y();
			kD.y() = m_pkTriangle1->v[i1].x() - m_pkTriangle1->v[i2].x();
			fSpeed = kD.dot(kW);

			ComputeTwo(kCfg1, m_pkTriangle1->v, kD, i0, i1, i2);
			ComputeThree(kCfg0, m_pkTriangle0->v, kD, m_pkTriangle1->v[i1]);

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

	bool IntrTriangle2Triangle2::Find(cfloat fTMax,
		const cvector2d& rkVelocity0, const cvector2d& rkVelocity1)
	{
		// process as if V0-triangle is stationary and V1-triangle is moving
		cvector2d kW = rkVelocity1 - rkVelocity0;
		int iSide = 0;  // 0 = NONE, -1 = LEFT, +1 = RIGHT
		cfloat fTFirst = (cfloat)0.0;
		cfloat fTLast = maxcfloat;

		Configuration kCfg0, kCfg1, kTCfg0, kTCfg1;
		int i0, i1, i2;
		cvector2d kD;
		cfloat fSpeed;

		// process edges of V0-triangle
		for (i0 = 1, i1 = 2, i2 = 0; i2 < 3; i0 = i1, i1 = i2, i2++)
		{
			// test axis V0[i1] + t*perp(V0[i2]-V0[i1]), perp(x,y) = (y,-x)
			kD.x() = m_pkTriangle0->v[i2].y() - m_pkTriangle0->v[i1].y();
			kD.y() = m_pkTriangle0->v[i1].x() - m_pkTriangle0->v[i2].x();
			fSpeed = kD.dot(kW);

			ComputeTwo(kCfg0, m_pkTriangle0->v, kD, i0, i1, i2);
			ComputeThree(kCfg1, m_pkTriangle1->v, kD, m_pkTriangle0->v[i1]);

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
			kD.x() = m_pkTriangle1->v[i2].y() - m_pkTriangle1->v[i1].y();
			kD.y() = m_pkTriangle1->v[i1].x() - m_pkTriangle1->v[i2].x();
			fSpeed = kD.dot(kW);

			ComputeTwo(kCfg1, m_pkTriangle1->v, kD, i0, i1, i2);
			ComputeThree(kCfg0, m_pkTriangle0->v, kD, m_pkTriangle1->v[i1]);

			if (NoIntersect(kCfg0, kCfg1, fTMax, fSpeed, iSide, kTCfg0, kTCfg1,
				fTFirst, fTLast))
			{
				return false;
			}
		}

		// move triangles to first contact
		cvector2d akMoveV0[3], akMoveV1[3];
		for (int i = 0; i < 3; i++)
		{
			akMoveV0[i] = m_pkTriangle0->v[i] + fTFirst * rkVelocity0;
			akMoveV1[i] = m_pkTriangle1->v[i] + fTFirst * rkVelocity1;
		};

		GetIntersection(kTCfg0, kTCfg1, iSide, akMoveV0, akMoveV1, m_iQuantity,
			m_akPoint);

		m_fContactTime = fTFirst;
		return m_iQuantity > 0;
	}
	//----------------------------------------------------------------------------

	int IntrTriangle2Triangle2::GetQuantity() const
	{
		return m_iQuantity;
	}
	//----------------------------------------------------------------------------

	const cvector2d& IntrTriangle2Triangle2::GetPoint(int i) const
	{
		assert(0 <= i && i < m_iQuantity);
		return m_akPoint[i];
	}
	//----------------------------------------------------------------------------

	int IntrTriangle2Triangle2::WhichSide(const cvector2d akV[3],
		const cvector2d& rkP, const cvector2d& rkD)
	{
		// Vertices are projected to the form P+t*D.  Return value is +1 if all
		// t > 0, -1 if all t < 0, 0 otherwise, in which case the line splits the
		// triangle.

		int iPositive = 0, iNegative = 0, iZero = 0;
		for (int i = 0; i < 3; i++)
		{
			cfloat fT = rkD.dot(akV[i] - rkP);
			if (fT > (cfloat)0.0)
			{
				iPositive++;
			}
			else if (fT < (cfloat)0.0)
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

	void IntrTriangle2Triangle2::ClipConvexPolygonAgainstLine(
		const cvector2d& rkN, cfloat fC, int& riQuantity,
		cvector2d akV[6])
	{
		// The input vertices are assumed to be in counterclockwise order.  The
		// ordering is an invariant of this function.

		// test on which side of line the vertices are
		int iPositive = 0, iNegative = 0, iPIndex = -1;
		cfloat afTest[6];
		int i;
		for (i = 0; i < riQuantity; i++)
		{
			afTest[i] = rkN.dot(akV[i]) - fC;
			if (afTest[i] > (cfloat)0.0)
			{
				iPositive++;
				if (iPIndex < 0)
				{
					iPIndex = i;
				}
			}
			else if (afTest[i] < (cfloat)0.0)
			{
				iNegative++;
			}
		}

		if (iPositive > 0)
		{
			if (iNegative > 0)
			{
				// line transversely intersects polygon
				cvector2d akCV[6];
				int iCQuantity = 0, iCur, iPrv;
				cfloat fT;

				if (iPIndex > 0)
				{
					// first clip vertex on line
					iCur = iPIndex;
					iPrv = iCur - 1;
					fT = afTest[iCur] / (afTest[iCur] - afTest[iPrv]);
					akCV[iCQuantity++] = akV[iCur] + fT * (akV[iPrv] - akV[iCur]);

					// vertices on positive side of line
					while (iCur < riQuantity && afTest[iCur] >(cfloat)0.0)
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
					while (iCur < riQuantity && afTest[iCur] >(cfloat)0.0)
					{
						akCV[iCQuantity++] = akV[iCur++];
					}

					// last clip vertex on line
					iPrv = iCur - 1;
					fT = afTest[iCur] / (afTest[iCur] - afTest[iPrv]);
					akCV[iCQuantity++] = akV[iCur] + fT * (akV[iPrv] - akV[iCur]);

					// skip vertices on negative side
					while (iCur < riQuantity && afTest[iCur] <= (cfloat)0.0)
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
						while (iCur < riQuantity && afTest[iCur] >(cfloat)0.0)
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
				for (int i = 0; i < iCQuantity; ++i)
					akV[i] = akCV[i];
			}
		}
		else
		{
			riQuantity = 0;
		}
	}
	//----------------------------------------------------------------------------

	void IntrTriangle2Triangle2::ComputeTwo(Configuration& rkCfg,
		const cvector2d akV[3], const cvector2d& rkD, int i0, int i1,
		int i2)
	{
		rkCfg.Map = M12;
		rkCfg.Index[0] = i0;
		rkCfg.Index[1] = i1;
		rkCfg.Index[2] = i2;
		rkCfg.Min = rkD.dot(akV[i0] - akV[i1]);
		rkCfg.Max = (cfloat)0.0;
	}
	//----------------------------------------------------------------------------

	void IntrTriangle2Triangle2::ComputeThree(Configuration& rkCfg,
		const cvector2d akV[3], const cvector2d& rkD,
		const cvector2d& rkP)
	{
		cfloat fD0 = rkD.dot(akV[0] - rkP);
		cfloat fD1 = rkD.dot(akV[1] - rkP);
		cfloat fD2 = rkD.dot(akV[2] - rkP);

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

	bool IntrTriangle2Triangle2::NoIntersect(
		const Configuration& rkCfg0, const Configuration& rkCfg1, cfloat fTMax,
		cfloat fSpeed, int& riSide, Configuration& rkTCfg0, Configuration& rkTCfg1,
		cfloat& rfTFirst, cfloat& rfTLast)
	{
		cfloat fInvSpeed, fT;

		if (rkCfg1.Max < rkCfg0.Min)
		{
			// V1-interval initially on left of V0-interval
			if (fSpeed <= (cfloat)0.0)
			{
				return true;  // intervals moving apart
			}

			// update first time
			fInvSpeed = ((cfloat)1.0) / fSpeed;
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
			if (fSpeed >= (cfloat)0.0)
			{
				return true;  // intervals moving apart
			}

			// update first time
			fInvSpeed = ((cfloat)1.0) / fSpeed;
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
			if (fSpeed > (cfloat)0.0)
			{
				// update last time
				fInvSpeed = ((cfloat)1.0) / fSpeed;
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
			else if (fSpeed < (cfloat)0.0)
			{
				// update last time
				fInvSpeed = ((cfloat)1.0) / fSpeed;
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

	void IntrTriangle2Triangle2::GetIntersection(
		const Configuration& rkCfg0, const Configuration& rkCfg1, int iSide,
		const cvector2d akV0[3], const cvector2d akV1[3], int& riQuantity,
		cvector2d akVertex[6])
	{
		cvector2d kEdge, kDiff;
		const cvector2d* pkorigin;
		cfloat fInvEdE, fMin, fMax;
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
				pkorigin = &akV0[rkCfg0.Index[1]];
				kEdge = akV0[rkCfg0.Index[2]] - *pkorigin;
				fInvEdE = ((cfloat)1.0) / kEdge.dot(kEdge);
				kDiff = akV1[rkCfg1.Index[1]] - *pkorigin;
				fMin = kEdge.dot(kDiff)*fInvEdE;
				kDiff = akV1[rkCfg1.Index[0]] - *pkorigin;
				fMax = kEdge.dot(kDiff)*fInvEdE;
				assert(fMin <= fMax);
				Intersector1 kIntr((cfloat)0.0, (cfloat)1.0, fMin, fMax);
				riQuantity = kIntr.GetQuantity();
				assert(riQuantity > 0);
				for (i = 0; i < riQuantity; i++)
				{
					akVertex[i] = *pkorigin + kIntr.GetOverlap(i)*kEdge;
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
				pkorigin = &akV1[rkCfg1.Index[1]];
				kEdge = akV1[rkCfg1.Index[2]] - *pkorigin;
				fInvEdE = 1.0f / kEdge.dot(kEdge);
				kDiff = akV0[rkCfg0.Index[1]] - *pkorigin;
				fMin = kEdge.dot(kDiff)*fInvEdE;
				kDiff = akV0[rkCfg0.Index[0]] - *pkorigin;
				fMax = kEdge.dot(kDiff)*fInvEdE;
				assert(fMin <= fMax);
				Intersector1 kIntr((cfloat)0.0, (cfloat)1.0, fMin, fMax);
				riQuantity = kIntr.GetQuantity();
				assert(riQuantity > 0);
				for (i = 0; i < riQuantity; i++)
				{
					akVertex[i] = *pkorigin + kIntr.GetOverlap(i)*kEdge;
				}
			}
		}
		else  // triangles were initially intersecting
		{
			Triangle2d kTri0(akV0), kTri1(akV1);
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

	IntrTriangle3Triangle3::IntrTriangle3Triangle3(
		const Triangle3d& rkTriangle0, const Triangle3d& rkTriangle1)
		:
		m_pkTriangle0(&rkTriangle0),
		m_pkTriangle1(&rkTriangle1)
	{
		ReportCoplanarIntersections = true;
		m_iQuantity = 0;
	}
	//----------------------------------------------------------------------------

	const Triangle3d& IntrTriangle3Triangle3::GetTriangle0() const
	{
		return *m_pkTriangle0;
	}
	//----------------------------------------------------------------------------

	const Triangle3d& IntrTriangle3Triangle3::GetTriangle1() const
	{
		return *m_pkTriangle1;
	}
	//----------------------------------------------------------------------------

	bool IntrTriangle3Triangle3::Test()
	{
		// get edge vectors for triangle0
		cvector3d akE0[3] =
		{
			m_pkTriangle0->v[1] - m_pkTriangle0->v[0],
			m_pkTriangle0->v[2] - m_pkTriangle0->v[1],
			m_pkTriangle0->v[0] - m_pkTriangle0->v[2]
		};

		// get normal vector of triangle0
		cvector3d kN0 = akE0[0].cross(akE0[1]);
		kN0.normalize();

		// project triangle1 onto normal line of triangle0, test for separation
		cfloat fN0dT0V0 = kN0.dot(m_pkTriangle0->v[0]);
		cfloat fMin1, fMax1;
		ProjectOntoAxis(*m_pkTriangle1, kN0, fMin1, fMax1);
		if (fN0dT0V0 < fMin1 || fN0dT0V0 > fMax1)
		{
			return false;
		}

		// get edge vectors for triangle1
		cvector3d akE1[3] =
		{
			m_pkTriangle1->v[1] - m_pkTriangle1->v[0],
			m_pkTriangle1->v[2] - m_pkTriangle1->v[1],
			m_pkTriangle1->v[0] - m_pkTriangle1->v[2]
		};

		// get normal vector of triangle1
		cvector3d kN1 = akE1[0].cross(akE1[1]);
		kN1.normalize();

		cvector3d kDir;
		cfloat fMin0, fMax0;
		int i0, i1;

		cvector3d kN0xN1 = kN0.cross(kN1);
		kN0xN1.normalize();


		if (kN0xN1.dot(kN0xN1) >= inv_trunc_val)
		{
			// triangles are not parallel

			// Project triangle0 onto normal line of triangle1, test for
			// separation.
			cfloat fN1dT1V0 = kN1.dot(m_pkTriangle1->v[0]);
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
					kDir = akE0[i0].cross(akE1[i1]);
					kDir.normalize();

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
				kDir = kN0.cross(akE0[i0]);
				kDir.normalize();

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
				kDir = kN1.cross(akE1[i1]);
				kDir.normalize();

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

	bool IntrTriangle3Triangle3::Find()
	{
		int i, iM, iP;

		// Get the plane of triangle0.
		Plane3d kPlane0(m_pkTriangle0->v[0], m_pkTriangle0->v[1],
			m_pkTriangle0->v[2]);

		// Compute the signed distances of triangle1 vertices to plane0.  Use
		// an epsilon-thick plane test.
		int iPos1, iNeg1, iZero1, aiSign1[3];
		cfloat afDist1[3];
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
				if (!GetCoplanarIntersection(kPlane0, *m_pkTriangle0, *m_pkTriangle1))
				{
					m_iQuantity = 0;
					if (PointIsInSegment(m_pkTriangle0->v[0], m_pkTriangle1->v[0], m_pkTriangle1->v[1]) ||
						PointIsInSegment(m_pkTriangle0->v[0], m_pkTriangle1->v[1], m_pkTriangle1->v[2]) ||
						PointIsInSegment(m_pkTriangle0->v[0], m_pkTriangle1->v[2], m_pkTriangle1->v[0]))
					{
						m_akPoint[m_iQuantity] = m_pkTriangle0->v[0];
						++m_iQuantity;
					}

					if (PointIsInSegment(m_pkTriangle0->v[1], m_pkTriangle1->v[0], m_pkTriangle1->v[1]) ||
						PointIsInSegment(m_pkTriangle0->v[1], m_pkTriangle1->v[1], m_pkTriangle1->v[2]) ||
						PointIsInSegment(m_pkTriangle0->v[1], m_pkTriangle1->v[2], m_pkTriangle1->v[0]))
					{
						m_akPoint[m_iQuantity] = m_pkTriangle0->v[1];
						++m_iQuantity;
					}

					if (PointIsInSegment(m_pkTriangle0->v[2], m_pkTriangle1->v[0], m_pkTriangle1->v[1]) ||
						PointIsInSegment(m_pkTriangle0->v[2], m_pkTriangle1->v[1], m_pkTriangle1->v[2]) ||
						PointIsInSegment(m_pkTriangle0->v[2], m_pkTriangle1->v[2], m_pkTriangle1->v[0]))
					{
						m_akPoint[m_iQuantity] = m_pkTriangle0->v[2];
						++m_iQuantity;
					}

					if (PointIsInSegment(m_pkTriangle1->v[0], m_pkTriangle0->v[0], m_pkTriangle0->v[1]) ||
						PointIsInSegment(m_pkTriangle1->v[0], m_pkTriangle0->v[1], m_pkTriangle0->v[2]) ||
						PointIsInSegment(m_pkTriangle1->v[0], m_pkTriangle0->v[2], m_pkTriangle0->v[0]))
					{
						m_akPoint[m_iQuantity] = m_pkTriangle1->v[0];
						++m_iQuantity;
					}

					if (PointIsInSegment(m_pkTriangle1->v[1], m_pkTriangle0->v[0], m_pkTriangle0->v[1]) ||
						PointIsInSegment(m_pkTriangle1->v[1], m_pkTriangle0->v[1], m_pkTriangle0->v[2]) ||
						PointIsInSegment(m_pkTriangle1->v[1], m_pkTriangle0->v[2], m_pkTriangle0->v[0]))
					{
						m_akPoint[m_iQuantity] = m_pkTriangle1->v[1];
						++m_iQuantity;
					}

					if (PointIsInSegment(m_pkTriangle1->v[2], m_pkTriangle0->v[0], m_pkTriangle0->v[1]) ||
						PointIsInSegment(m_pkTriangle1->v[2], m_pkTriangle0->v[1], m_pkTriangle0->v[2]) ||
						PointIsInSegment(m_pkTriangle1->v[2], m_pkTriangle0->v[2], m_pkTriangle0->v[0]))
					{
						m_akPoint[m_iQuantity] = m_pkTriangle1->v[2];
						++m_iQuantity;
					}

					for (int i = m_iQuantity - 1; i > 0; --i)
					{
						for (int j = i - 1; j >= 0; --j)
						{
							if ((m_akPoint[i] - m_akPoint[j]).squaredNorm() <= inv_trunc_val)
							{
								--m_iQuantity;
								break;
							}
						}
					}
					switch(m_iQuantity)
					{
					case 0:
						m_iIntersectionType = IT_EMPTY;
						break;
					case 1:
						m_iIntersectionType = IT_POINT;
						break;
					case 2:
						m_iIntersectionType = IT_SEGMENT;
						break;
					default:
						m_iIntersectionType = IT_POLYGON;
						break;
					}
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
							m_pkTriangle1->v[iM], m_pkTriangle1->v[iP]))
						{
							for (int i = m_iQuantity - 1; i > 0; --i)
							{
								for (int j = i - 1; j >= 0; --j)
								{
									if ((m_akPoint[i] - m_akPoint[j]).squaredNorm() <= inv_trunc_val)
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
							m_pkTriangle1->v[i]))
						{
							for (int i = m_iQuantity - 1; i > 0; --i)
							{
								for (int j = i - 1; j >= 0; --j)
								{
									if ((m_akPoint[i] - m_akPoint[j]).squaredNorm() <= inv_trunc_val)
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
		cfloat fT;
		cvector3d kIntr0, kIntr1;
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
					kIntr0 = m_pkTriangle1->v[i] + fT * (m_pkTriangle1->v[iM] -
						m_pkTriangle1->v[i]);
					fT = afDist1[i] / (afDist1[i] - afDist1[iP]);
					kIntr1 = m_pkTriangle1->v[i] + fT * (m_pkTriangle1->v[iP] -
						m_pkTriangle1->v[i]);
					if (IntersectsSegment(kPlane0, *m_pkTriangle0, kIntr0, kIntr1))
					{
						for (int i = m_iQuantity - 1; i > 0; --i)
						{
							for (int j = i - 1; j >= 0; --j)
							{
								if ((m_akPoint[i] - m_akPoint[j]).squaredNorm() <= inv_trunc_val)
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
				kIntr0 = m_pkTriangle1->v[iM] + fT * (m_pkTriangle1->v[iP] -
					m_pkTriangle1->v[iM]);
				if (IntersectsSegment(kPlane0, *m_pkTriangle0,
					m_pkTriangle1->v[i], kIntr0))
				{
					for (int i = m_iQuantity - 1; i > 0; --i)
					{
						for (int j = i - 1; j >= 0; --j)
						{
							if ((m_akPoint[i] - m_akPoint[j]).squaredNorm() <= inv_trunc_val)
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

	bool IntrTriangle3Triangle3::Test(cfloat fTMax,
		const cvector3d& rkVelocity0, const cvector3d& rkVelocity1)
	{
		cfloat fTFirst = (cfloat)0.0;
		cfloat fTLast = maxcfloat;

		// velocity relative to triangle0
		cvector3d kVel = rkVelocity1 - rkVelocity0;

		// compute edge and normal directions for triangle0
		cvector3d akE[3] =
		{
			m_pkTriangle0->v[1] - m_pkTriangle0->v[0],
			m_pkTriangle0->v[2] - m_pkTriangle0->v[1],
			m_pkTriangle0->v[0] - m_pkTriangle0->v[2]
		};
		cvector3d kN = akE[0].cross(akE[1]);
		kN.normalize();

		if (!TestOverlap(kN, fTMax, kVel, fTFirst, fTLast))
		{
			return false;
		}

		// compute edge and normal directions for triangle1
		cvector3d akF[3] =
		{
			m_pkTriangle1->v[1] - m_pkTriangle1->v[0],
			m_pkTriangle1->v[2] - m_pkTriangle1->v[1],
			m_pkTriangle1->v[0] - m_pkTriangle1->v[2]
		};
		cvector3d kM = akF[0].cross(akF[1]);
		kM.normalize();

		cvector3d kDir;
		int i0, i1;

		if (std::abs(kN.dot(kM)) < 1.0f - inv_trunc_val)
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
					kDir = akE[i0].cross(akF[i1]);
					kDir.normalize();

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
			for (i0 = 0; i0 < 3; ++i0)
			{
				kDir = kN.cross(akE[i0]);
				kDir.normalize();
				if (!TestOverlap(kDir, fTMax, kVel, fTFirst, fTLast))
				{
					return false;
				}
			}

			// directions NxF[i1]
			for (i1 = 0; i1 < 3; ++i1)
			{
				kDir = kM.cross(akF[i1]);
				kDir.normalize();

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

	bool IntrTriangle3Triangle3::Find(cfloat fTMax,
		const cvector3d& rkVelocity0, const cvector3d& rkVelocity1)
	{
		cfloat fTFirst = (cfloat)0.0;
		cfloat fTLast = maxcfloat;

		// velocity relative to triangle0
		cvector3d kVel = rkVelocity1 - rkVelocity0;

		ContactSide eSide = CS_NONE;
		Configuration kTCfg0, kTCfg1;

		// compute edge and normal directions for triangle0
		cvector3d akE[3] =
		{
			m_pkTriangle0->v[1] - m_pkTriangle0->v[0],
			m_pkTriangle0->v[2] - m_pkTriangle0->v[1],
			m_pkTriangle0->v[0] - m_pkTriangle0->v[2]
		};
		cvector3d kN = akE[0].cross(akE[1]);
		kN.normalize();

		if (!FindOverlap(kN, fTMax, kVel, eSide, kTCfg0, kTCfg1, fTFirst, fTLast))
		{
			return false;
		}

		// compute edge and normal directions for triangle1
		cvector3d akF[3] =
		{
			m_pkTriangle1->v[1] - m_pkTriangle1->v[0],
			m_pkTriangle1->v[2] - m_pkTriangle1->v[1],
			m_pkTriangle1->v[0] - m_pkTriangle1->v[2]
		};
		cvector3d kM = akF[0].cross(akF[1]);
		kM.normalize();

		cvector3d kDir;
		int i0, i1;

		if (std::abs(kN.dot(kM)) < 1.0f - inv_trunc_val)
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
					kDir = akE[i0].cross(akF[i1]);
					kDir.normalize();

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
				kDir = kN.cross(akE[i0]);
				kDir.normalize();
				if (!FindOverlap(kDir, fTMax, kVel, eSide, kTCfg0, kTCfg1, fTFirst,
					fTLast))
				{
					return false;
				}
			}

			// directions NxF[i1]
			for (i1 = 0; i1 < 3; i1++)
			{
				kDir = kM.cross(akF[i1]);
				kDir.normalize();
				if (!FindOverlap(kDir, fTMax, kVel, eSide, kTCfg0, kTCfg1, fTFirst,
					fTLast))
				{
					return false;
				}
			}
		}

		if (fTFirst <= (cfloat)0.0)
		{
			return false;
		}

		m_fContactTime = fTFirst;

		// adjust U and v for first time of contact before finding contact set
		Triangle3d akMTri0, akMTri1;
		for (i0 = 0; i0 < 3; i0++)
		{
			akMTri0.v[i0] = m_pkTriangle0->v[i0] + fTFirst * rkVelocity0;
			akMTri1.v[i0] = m_pkTriangle1->v[i0] + fTFirst * rkVelocity1;
		}

		FindContactSet(akMTri0, akMTri1, eSide, kTCfg0, kTCfg1);
		return true;
	}
	//----------------------------------------------------------------------------

	int IntrTriangle3Triangle3::GetQuantity() const
	{
		return m_iQuantity;
	}
	//----------------------------------------------------------------------------

	const cvector3d& IntrTriangle3Triangle3::GetPoint(int i) const
	{
		assert(0 <= i && i < m_iQuantity);
		return m_akPoint[i];
	}
	//----------------------------------------------------------------------------

	void IntrTriangle3Triangle3::ProjectOntoAxis(
		const Triangle3d& rkTri, const cvector3d& rkAxis, cfloat& rfMin,
		cfloat& rfMax)
	{
		cfloat fDot0 = rkAxis.dot(rkTri.v[0]);
		cfloat fDot1 = rkAxis.dot(rkTri.v[1]);
		cfloat fDot2 = rkAxis.dot(rkTri.v[2]);

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

	void IntrTriangle3Triangle3::TrianglePlaneRelations(
		const Triangle3d& rkTriangle, const Plane3d& rkPlane,
		cfloat afDistance[3], int aiSign[3], int& riPositive, int& riNegative,
		int& riZero)
	{
		// Compute the signed distances of triangle vertices to the plane.  Use
		// an epsilon-thick plane test.
		riPositive = 0;
		riNegative = 0;
		riZero = 0;
		for (int i = 0; i < 3; i++)
		{
			afDistance[i] = rkPlane.DistanceTo(rkTriangle.v[i]);
			if (afDistance[i] > inv_trunc_val)
			{
				aiSign[i] = 1;
				riPositive++;
			}
			else if (afDistance[i] < -inv_trunc_val)
			{
				aiSign[i] = -1;
				riNegative++;
			}
			else
			{
				afDistance[i] = (cfloat)0.0;
				aiSign[i] = 0;
				riZero++;
			}
		}
	}
	//----------------------------------------------------------------------------

	void IntrTriangle3Triangle3::GetInterval(
		const Triangle3d& rkTriangle, const Line3d& rkLine,
		const cfloat afDistance[3], const int aiSign[3], cfloat afParam[2])
	{
		// project triangle onto line
		cfloat afProj[3];
		int i;
		for (i = 0; i < 3; i++)
		{
			cvector3d kDiff = rkTriangle.v[i] - rkLine.origin;
			afProj[i] = rkLine.direction.dot(kDiff);
		}

		// compute transverse intersections of triangle edges with line
		cfloat fNumer, fDenom;
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
				cfloat fSave = afParam[0];
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
	inline cfloat InvSqrt(cfloat fValue)
	{
		return (cfloat)(1.0 / sqrt((double)fValue));
	}
	inline static void GenerateComplementBasis(cvector3d& rkU, cvector3d& rkV,const cvector3d& rkW)
	{
		cfloat fInvLength;

		if (std::abs(rkW[0]) >=
			std::abs(rkW[1]))
		{
			fInvLength = InvSqrt(rkW[0] * rkW[0] +rkW[2] * rkW[2]);
			rkU[0] = -rkW[2] * fInvLength;
			rkU[1] = (cfloat)0.0;
			rkU[2] = +rkW[0] * fInvLength;
			rkV[0] = rkW[1] * rkU[2];
			rkV[1] = rkW[2] * rkU[0] -
				rkW[0] * rkU[2];
			rkV[2] = -rkW[1] * rkU[0];
		}
		else
		{
			// W.y or W.z is the largest magnitude component, swap them
			fInvLength = InvSqrt(rkW[1] * rkW[1] + rkW[2] * rkW[2]);
			rkU[0] = (cfloat)0.0;
			rkU[1] = +rkW[2] * fInvLength;
			rkU[2] = -rkW[1] * fInvLength;
			rkV[0] = rkW[1] * rkU[2] -
				rkW[2] * rkU[1];
			rkV[1] = -rkW[0] * rkU[2];
			rkV[2] = rkW[0] * rkU[1];
		}
	}

	static inline cfloat Det2(cfloat fX0, cfloat fY0, cfloat fX1, cfloat fY1)
	{
		return fX0 * fY1 - fX1 * fY0;
	}

	static inline bool Sort(int& iV0, int& iV1)
	{
		int j0, j1;
		bool bPositive;

		if (iV0 < iV1)
		{
			j0 = 0; j1 = 1; bPositive = true;
		}
		else
		{
			j0 = 1; j1 = 0; bPositive = false;
		}

		int aiValue[2] = { iV0, iV1 };
		iV0 = aiValue[j0];
		iV1 = aiValue[j1];
		return bPositive;
	}

	static inline int ToLine(cvector2d m_akVertex[3], const cvector2d& rkP, int iV0, int iV1)
	{
		bool bPositive = Sort(iV0, iV1);
		const cvector2d& rkV0 = m_akVertex[iV0];
		const cvector2d& rkV1 = m_akVertex[iV1];
		cfloat fX0 = rkP[0] - rkV0[0];
		cfloat fY0 = rkP[1] - rkV0[1];
		cfloat fX1 = rkV1[0] - rkV0[0];
		cfloat fY1 = rkV1[1] - rkV0[1];
		cfloat fDet2 = Det2(fX0, fY0, fX1, fY1);
		if (!bPositive)
		{
			fDet2 = -fDet2;
		}
		return (fDet2 > (cfloat)0 ? +1 : (fDet2 < (cfloat)0 ? -1 : 0));
	}

	static inline int ToTriangle(cvector2d akProjV[3],const cvector2d& rkP, int iV0, int iV1,int iV2)
	{
		int iSign0 = ToLine(akProjV,rkP, iV1, iV2);
		if (iSign0 > 0)
		{
			return +1;
		}
		int iSign1 = ToLine(akProjV, rkP, iV0, iV2);
		if (iSign1 < 0)
		{
			return +1;
		}
		int iSign2 = ToLine(akProjV, rkP, iV0, iV1);
		if (iSign2 > 0)
		{
			return +1;
		}
		return ((iSign0 && iSign1 && iSign2) ? -1 : 0);
	}

	bool IntrTriangle3Triangle3::ContainsPoint(
		const Triangle3d& rkTriangle, const Plane3d& rkPlane,
		const cvector3d& rkPoint)
	{
		// Generate a coordinate system for the plane.  The incoming triangle has
		// vertices <V0,V1,V2>.  The incoming plane has unit-length normal N.
		// The incoming point is P.  V0 is chosen as the origin for the plane. The
		// coordinate axis directions are two unit-length vectors, U0 and U1,
		// constructed so that {U0,U1,N} is an orthonormal set.  Any point Q
		// in the plane may be written as Q = V0 + x0*U0 + x1*U1.  The coordinates
		// are computed as x0 = Dot(U0,Q-V0) and x1 = Dot(U1,Q-V0).
		cvector3d kU0, kU1;
		GenerateComplementBasis(kU0, kU1, rkPlane.normal);

		// Compute the planar coordinates for the points P, V1, and V2.  To
		// simplify matters, the origin is subtracted from the points, in which
		// case the planar coordinates are for P-V0, V1-V0, and V2-V0.
		cvector3d kPmV0 = rkPoint - rkTriangle.v[0];
		cvector3d kV1mV0 = rkTriangle.v[1] - rkTriangle.v[0];
		cvector3d kV2mV0 = rkTriangle.v[2] - rkTriangle.v[0];

		// The planar representation of P-V0.
		cvector2d kProjP(kU0.dot(kPmV0), kU1.dot(kPmV0));

		// The planar representation of the triangle <V0-V0,V1-V0,V2-V0>.
		cvector2d akProjV[3] =
		{
			cvector2d(0,0),
			cvector2d(kU0.dot(kV1mV0),kU1.dot(kV1mV0)),
			cvector2d(kU0.dot(kV2mV0),kU1.dot(kV2mV0))
		};

		// Test whether P-V0 is in the triangle <0,V1-V0,V2-V0>.
		if (ToTriangle(akProjV,kProjP, 0, 1, 2) <= 0)
		{
			// Report the point of intersection to the caller.
			m_iQuantity = 1;
			m_akPoint[0] = rkPoint;
			return true;
		}

		return false;
	}
	//----------------------------------------------------------------------------

	bool IntrTriangle3Triangle3::IntersectsSegment(
		const Plane3d& rkPlane, const Triangle3d& rkTriangle,
		const cvector3d& rkEnd0, const cvector3d& rkEnd1)
	{
		// Compute the 2D representations of the triangle vertices and the
		// segment endpoints relative to the plane of the triangle.  Then
		// compute the intersection in the 2D space.

		// Project the triangle and segment onto the coordinate plane most
		// aligned with the plane normal.
		int iMaxNormal = 0;
		cfloat fMax = std::abs(rkPlane.normal.x());
		cfloat fAbs = std::abs(rkPlane.normal.y());
		if (fAbs > fMax)
		{
			iMaxNormal = 1;
			fMax = fAbs;
		}
		fAbs = std::abs(rkPlane.normal.z());
		if (fAbs > fMax)
		{
			iMaxNormal = 2;
		}

		Triangle2d kProjTri;
		cvector2d kProjEnd0, kProjEnd1;
		int i;

		if (iMaxNormal == 0)
		{
			// project onto yz-plane
			for (i = 0; i < 3; i++)
			{
				kProjTri.v[i].x() = rkTriangle.v[i].y();
				kProjTri.v[i].y() = rkTriangle.v[i].z();
				kProjEnd0.x() = rkEnd0.y();
				kProjEnd0.y() = rkEnd0.z();
				kProjEnd1.x() = rkEnd1.y();
				kProjEnd1.y() = rkEnd1.z();
			}
		}
		else if (iMaxNormal == 1)
		{
			// project onto xz-plane
			for (i = 0; i < 3; i++)
			{
				kProjTri.v[i].x() = rkTriangle.v[i].x();
				kProjTri.v[i].y() = rkTriangle.v[i].z();
				kProjEnd0.x() = rkEnd0.x();
				kProjEnd0.y() = rkEnd0.z();
				kProjEnd1.x() = rkEnd1.x();
				kProjEnd1.y() = rkEnd1.z();
			}
		}
		else
		{
			// project onto xy-plane
			for (i = 0; i < 3; i++)
			{
				kProjTri.v[i].x() = rkTriangle.v[i].x();
				kProjTri.v[i].y() = rkTriangle.v[i].y();
				kProjEnd0.x() = rkEnd0.x();
				kProjEnd0.y() = rkEnd0.y();
				kProjEnd1.x() = rkEnd1.x();
				kProjEnd1.y() = rkEnd1.y();
			}
		}

		cvector2d kPSCenter = ((cfloat)0.5)*(kProjEnd0 + kProjEnd1);
		cvector2d kPSdirection = kProjEnd1 - kProjEnd0;
		cfloat fPSExtent = ((cfloat)0.5)*kPSdirection.norm();
		kPSdirection.normalize();
		Segment2d kProjSeg(kPSCenter, kPSdirection, fPSExtent);
		IntrSegment2Triangle2 kCalc(kProjSeg, kProjTri);
		if (!kCalc.Find())
		{
			return false;
		}

		cvector2d akIntr[2];
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
			cfloat fInvNx = ((cfloat)1.0) / rkPlane.normal.x();
			for (i = 0; i < m_iQuantity; i++)
			{
				m_akPoint[i].y() = akIntr[i].x();
				m_akPoint[i].z() = akIntr[i].y();
				m_akPoint[i].x() = fInvNx * (rkPlane.constant -
					rkPlane.normal.y()*m_akPoint[i].y() -
					rkPlane.normal.z()*m_akPoint[i].z());
			}
		}
		else if (iMaxNormal == 1)
		{
			cfloat fInvNY = ((cfloat)1.0) / rkPlane.normal.y();
			for (i = 0; i < m_iQuantity; i++)
			{
				m_akPoint[i].x() = akIntr[i].x();
				m_akPoint[i].z() = akIntr[i].y();
				m_akPoint[i].y() = fInvNY * (rkPlane.constant -
					rkPlane.normal.x()*m_akPoint[i].x() -
					rkPlane.normal.z()*m_akPoint[i].z());
			}
		}
		else
		{
			cfloat fInvNZ = ((cfloat)1.0) / rkPlane.normal.z();
			for (i = 0; i < m_iQuantity; ++i)
			{
				m_akPoint[i].x() = akIntr[i].x();
				m_akPoint[i].y() = akIntr[i].y();
				m_akPoint[i].z() = fInvNZ * (rkPlane.constant -
					rkPlane.normal.x()*m_akPoint[i].x() -
					rkPlane.normal.y()*m_akPoint[i].y());
			}
		}

		return true;
	}
	//----------------------------------------------------------------------------

	bool IntrTriangle3Triangle3::GetCoplanarIntersection(
		const Plane3d& rkPlane, const Triangle3d& rkTri0,
		const Triangle3d& rkTri1)
	{
		// Project triangles onto coordinate plane most aligned with plane
		// normal.
		int iMaxNormal = 0;
		cfloat fMax = std::abs(rkPlane.normal.x());
		cfloat fAbs = std::abs(rkPlane.normal.y());
		if (fAbs > fMax)
		{
			iMaxNormal = 1;
			fMax = fAbs;
		}
		fAbs = std::abs(rkPlane.normal.z());
		if (fAbs > fMax)
		{
			iMaxNormal = 2;
		}

		Triangle2d kProjTri0, kProjTri1;
		int i;

		if (iMaxNormal == 0)
		{
			// project onto yz-plane
			for (i = 0; i < 3; i++)
			{
				kProjTri0.v[i].x() = rkTri0.v[i].y();
				kProjTri0.v[i].y() = rkTri0.v[i].z();
				kProjTri1.v[i].x() = rkTri1.v[i].y();
				kProjTri1.v[i].y() = rkTri1.v[i].z();
			}
		}
		else if (iMaxNormal == 1)
		{
			// project onto xz-plane
			for (i = 0; i < 3; i++)
			{
				kProjTri0.v[i].x() = rkTri0.v[i].x();
				kProjTri0.v[i].y() = rkTri0.v[i].z();
				kProjTri1.v[i].x() = rkTri1.v[i].x();
				kProjTri1.v[i].y() = rkTri1.v[i].z();
			}
		}
		else
		{
			// project onto xy-plane
			for (i = 0; i < 3; i++)
			{
				kProjTri0.v[i].x() = rkTri0.v[i].x();
				kProjTri0.v[i].y() = rkTri0.v[i].y();
				kProjTri1.v[i].x() = rkTri1.v[i].x();
				kProjTri1.v[i].y() = rkTri1.v[i].y();
			}
		}

		// 2D triangle intersection routines require counterclockwise ordering
		cvector2d kSave;
		cvector2d kEdge0 = kProjTri0.v[1] - kProjTri0.v[0];
		cvector2d kEdge1 = kProjTri0.v[2] - kProjTri0.v[0];
		if (DotPerp(kEdge0,kEdge1) < (cfloat)0.0)
		{
			// triangle is clockwise, reorder it
			kSave = kProjTri0.v[1];
			kProjTri0.v[1] = kProjTri0.v[2];
			kProjTri0.v[2] = kSave;
		}

		kEdge0 = kProjTri1.v[1] - kProjTri1.v[0];
		kEdge1 = kProjTri1.v[2] - kProjTri1.v[0];
		if (DotPerp(kEdge0,kEdge1) < (cfloat)0.0)
		{
			// triangle is clockwise, reorder it
			kSave = kProjTri1.v[1];
			kProjTri1.v[1] = kProjTri1.v[2];
			kProjTri1.v[2] = kSave;
		}

		IntrTriangle2Triangle2 kIntr(kProjTri0, kProjTri1);
		if (!kIntr.Find())
		{
			return false;
		}

		// map 2D intersections back to the 3D triangle space
		m_iQuantity = kIntr.GetQuantity();
		if (iMaxNormal == 0)
		{
			cfloat fInvNx = ((cfloat)1.0) / rkPlane.normal.x();
			for (i = 0; i < m_iQuantity; i++)
			{
				m_akPoint[i].y() = kIntr.GetPoint(i).x();
				m_akPoint[i].z() = kIntr.GetPoint(i).y();
				m_akPoint[i].x() = fInvNx * (rkPlane.constant -
					rkPlane.normal.y()*m_akPoint[i].y() -
					rkPlane.normal.z()*m_akPoint[i].z());
			}
		}
		else if (iMaxNormal == 1)
		{
			cfloat fInvNY = ((cfloat)1.0) / rkPlane.normal.y();
			for (i = 0; i < m_iQuantity; i++)
			{
				m_akPoint[i].x() = kIntr.GetPoint(i).x();
				m_akPoint[i].z() = kIntr.GetPoint(i).y();
				m_akPoint[i].y() = fInvNY * (rkPlane.constant -
					rkPlane.normal.x()*m_akPoint[i].x() -
					rkPlane.normal.z()*m_akPoint[i].z());
			}
		}
		else
		{
			cfloat fInvNZ = ((cfloat)1.0) / rkPlane.normal.z();
			for (i = 0; i < m_iQuantity; i++)
			{
				m_akPoint[i].x() = kIntr.GetPoint(i).x();
				m_akPoint[i].y() = kIntr.GetPoint(i).y();
				m_akPoint[i].z() = fInvNZ * (rkPlane.constant -
					rkPlane.normal.x()*m_akPoint[i].x() -
					rkPlane.normal.y()*m_akPoint[i].y());
			}
		}

		return true;
	}
	//----------------------------------------------------------------------------

	bool IntrTriangle3Triangle3::TestOverlap(cfloat fTMax, cfloat fSpeed,
		cfloat fUMin, cfloat fUMax, cfloat fVMin, cfloat fVMax, cfloat& rfTFirst,
		cfloat& rfTLast)
	{
		// constant velocity separating axis test.

		cfloat fT;

		if (fVMax < fUMin) // v on left of U
		{
			if (fSpeed <= (cfloat)0.0) // v moving away from U
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
		else if (fUMax < fVMin)   // v on right of U
		{
			if (fSpeed >= (cfloat)0.0) // v moving away from U
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
		else // v and U on overlapping interval
		{
			if (fSpeed > (cfloat)0.0)
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
			else if (fSpeed < (cfloat)0.0)
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

	bool IntrTriangle3Triangle3::FindOverlap(cfloat fTMax, cfloat fSpeed,
		const Configuration& rkUC, const Configuration& rkVC, ContactSide& rkSide,
		Configuration& rkTUC, Configuration& rkTVC, cfloat& rfTFirst, cfloat& rfTLast)
	{
		// constant velocity separating axis test.  UC and VC are the new
		// potential configurations, and TUC and TVC are the best known
		// configurations.

		cfloat fT;

		if (rkVC.Max < rkUC.Min) // v on left of U
		{
			if (fSpeed <= (cfloat)0.0) // v moving away from U
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
		else if (rkUC.Max < rkVC.Min)   // v on right of U
		{
			if (fSpeed >= (cfloat)0.0) // v moving away from U
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
		else // v and U on overlapping interval
		{
			if (fSpeed > (cfloat)0.0)
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
			else if (fSpeed < (cfloat)0.0)
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

	bool IntrTriangle3Triangle3::TestOverlap(const cvector3d& rkAxis,
		cfloat fTMax, const cvector3d& rkVelocity, cfloat& rfTFirst,
		cfloat& rfTLast)
	{
		cfloat fMin0, fMax0, fMin1, fMax1;
		ProjectOntoAxis(*m_pkTriangle0, rkAxis, fMin0, fMax0);
		ProjectOntoAxis(*m_pkTriangle1, rkAxis, fMin1, fMax1);
		cfloat fSpeed = rkVelocity.dot(rkAxis);
		return TestOverlap(fTMax, fSpeed, fMin0, fMax0, fMin1, fMax1, rfTFirst,
			rfTLast);
	}
	//----------------------------------------------------------------------------

	bool IntrTriangle3Triangle3::PointIsInSegment(const cvector3d& pt, const cvector3d& segs, const cvector3d& sege)
	{
		cvector3d ps = segs - pt;
		cvector3d pe = sege - pt;
		cfloat rsecl = ps.cross(pe).squaredNorm();
		if (rsecl > inv_trunc_val)
		{
			return false;
		}

		cfloat dr = ps.dot(pe);
		if (dr > inv_trunc_val)
			return false;
		return true;
	}
	//----------------------------------------------------------------------------

	bool IntrTriangle3Triangle3::FindOverlap(const cvector3d& rkAxis,
		cfloat fTMax, const cvector3d& rkVelocity, ContactSide& reSide,
		Configuration& rkTCfg0, Configuration& rkTCfg1, cfloat& rfTFirst,
		cfloat& rfTLast)
	{
		Configuration kCfg0, kCfg1;
		ProjectOntoAxis(*m_pkTriangle0, rkAxis, kCfg0);
		ProjectOntoAxis(*m_pkTriangle1, rkAxis, kCfg1);
		cfloat fSpeed = rkVelocity.dot(rkAxis);
		return FindOverlap(fTMax, fSpeed, kCfg0, kCfg1, reSide, rkTCfg0, rkTCfg1,
			rfTFirst, rfTLast);
	}
	//----------------------------------------------------------------------------

	void IntrTriangle3Triangle3::ProjectOntoAxis(
		const Triangle3d& rkTri, const cvector3d& rkAxis,
		Configuration& rkCfg)
	{
		// find projections of vertices onto potential separating axis
		cfloat fD0 = rkAxis.dot(rkTri.v[0]);
		cfloat fD1 = rkAxis.dot(rkTri.v[1]);
		cfloat fD2 = rkAxis.dot(rkTri.v[2]);

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

	void IntrTriangle3Triangle3::FindContactSet(
		const Triangle3d& rkTri0, const Triangle3d& rkTri1,
		ContactSide& reSide, Configuration& rkCfg0, Configuration& rkCfg1)
	{
		if (reSide == CS_RIGHT) // tri1 to the right of tri0
		{
			if (rkCfg0.Map == M21 || rkCfg0.Map == M111)
			{
				// tri0 touching tri1 at a single point
				m_iIntersectionType = IT_POINT;
				m_iQuantity = 1;
				m_akPoint[0] = rkTri0.v[2];
			}
			else if (rkCfg1.Map == M12 || rkCfg1.Map == M111)
			{
				// tri1 touching tri0 at a single point
				m_iIntersectionType = IT_POINT;
				m_iQuantity = 1;
				m_akPoint[0] = rkTri1.v[0];
			}
			else if (rkCfg0.Map == M12)
			{
				if (rkCfg1.Map == M21)
				{
					// edge0-edge1 intersection
					GetEdgeEdgeIntersection(rkTri0.v[1], rkTri0.v[2], rkTri1.v[0],
						rkTri1.v[1]);
				}
				else // rkCfg1.Map == m3
				{
					// uedge-vface intersection
					GetEdgeFaceIntersection(rkTri0.v[1], rkTri0.v[2], rkTri1);
				}
			}
			else // rkCfg0.Map == M3
			{
				if (rkCfg1.Map == M21)
				{
					// face0-edge1 intersection
					GetEdgeFaceIntersection(rkTri1.v[0], rkTri1.v[1], rkTri0);
				}
				else // rkCfg1.Map == M3
				{
					// face0-face1 intersection
					Plane3d kPlane0(rkTri0.v[0], rkTri0.v[1], rkTri0.v[2]);
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
				m_akPoint[0] = rkTri1.v[2];
			}
			else if (rkCfg0.Map == M12 || rkCfg0.Map == M111)
			{
				// tri0 touching tri1 at a single point
				m_iIntersectionType = IT_POINT;
				m_iQuantity = 1;
				m_akPoint[0] = rkTri0.v[0];
			}
			else if (rkCfg1.Map == M12)
			{
				if (rkCfg0.Map == M21)
				{
					// edge0-edge1 intersection
					GetEdgeEdgeIntersection(rkTri0.v[0], rkTri0.v[1], rkTri1.v[1],
						rkTri1.v[2]);
				}
				else // rkCfg0.Map == M3
				{
					// edge1-face0 intersection
					GetEdgeFaceIntersection(rkTri1.v[1], rkTri1.v[2], rkTri0);
				}
			}
			else // rkCfg1.Map == M3
			{
				if (rkCfg0.Map == M21)
				{
					// edge0-face1 intersection
					GetEdgeFaceIntersection(rkTri0.v[0], rkTri0.v[1], rkTri1);
				}
				else // rkCfg0.Map == M
				{
					// face0-face1 intersection
					Plane3d kPlane0(rkTri0.v[0], rkTri0.v[1], rkTri0.v[2]);
					GetCoplanarIntersection(kPlane0, rkTri0, rkTri1);
				}
			}
		}
		else // reSide == CS_NONE
		{
			// triangles are already intersecting tranversely
			IntrTriangle3Triangle3 kCalc(rkTri0, rkTri1);
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

	void IntrTriangle3Triangle3::GetEdgeEdgeIntersection(
		const cvector3d& rkU0, const cvector3d& rkU1,
		const cvector3d& rkV0, const cvector3d& rkV1)
	{
		// Compute a normal to the plane of the two edges.
		cvector3d kEdge0 = rkU1 - rkU0;
		cvector3d kEdge1 = rkV1 - rkV0;
		cvector3d kNormal = kEdge0.cross(kEdge1);

		// Solve U0 + s*(U1 - U0) = V0 + t*(V1 - V0).  We know the edges
		// intersect, so s in [0,1] and t in [0,1].  Thus, just solve for s.
		// Note that s*E0 = D + t*E1, where D = V0 - U0. So s*N = s*E0xE1 = DxE1
		// and s = N*DxE1/N*N.
		cvector3d kDelta = rkV0 - rkU0;
		cfloat fS = kNormal.dot(kDelta.cross(kEdge1)) / kNormal.squaredNorm();
		assert((cfloat)0 <= fS && fS <= (cfloat)1);

		m_iIntersectionType = IT_POINT;
		m_iQuantity = 1;
		m_akPoint[0] = rkU0 + fS * kEdge0;

		// TODO:  What if the edges are parallel?
	}
	//----------------------------------------------------------------------------
	void IntrTriangle3Triangle3::GetEdgeFaceIntersection(
		const cvector3d& rkU0, const cvector3d& rkU1,
		const Triangle3d& rkTri)
	{
		// Compute a plane of the triangle.
		cvector3d kPoint = rkTri.v[0];
		cvector3d kEdge0 = rkTri.v[1] - kPoint;
		cvector3d kEdge1 = rkTri.v[2] - kPoint;
		cvector3d kNormal = kEdge0.cross(kEdge1);
		kNormal.normalize();
		cvector3d kDir0, kDir1;
		GenerateComplementBasis(kDir0, kDir1, kNormal);

		// Project the edge endpoints onto the plane.
		cvector2d kProjU0, kProjU1;
		cvector3d kDiff;
		kDiff = rkU0 - kPoint;
		kProjU0[0] = kDir0.dot(kDiff);
		kProjU0[1] = kDir1.dot(kDiff);
		kDiff = rkU1 - kPoint;
		kProjU1[0] = kDir0.dot(kDiff);
		kProjU1[1] = kDir1.dot(kDiff);

		cvector2d kPSCenter = ((cfloat)0.5)*(kProjU0 + kProjU1);
		cvector2d kPSdirection = kProjU1 - kProjU0;
		cfloat fPSExtent = ((cfloat)0.5)*kPSdirection.norm();
		kPSdirection.normalize();
		Segment2d kProjSeg(kPSCenter, kPSdirection, fPSExtent);

		// Compute the plane coordinates of the triangle.
		Triangle2d kProjTri;
		kProjTri.v[0] = cvector2d(0,0);
		kProjTri.v[1] = cvector2d(kDir0.dot(kEdge0), kDir1.dot(kEdge0));
		kProjTri.v[2] = cvector2d(kDir0.dot(kEdge1), kDir1.dot(kEdge1));

		// Compute the intersection.
		IntrSegment2Triangle2 kCalc(kProjSeg, kProjTri);
		bool bResult = kCalc.Find();
		assert(bResult);
		(void)bResult;
		m_iQuantity = kCalc.GetQuantity();
		for (int i = 0; i < m_iQuantity; i++)
		{
			cvector2d kProj = kCalc.GetPoint(i);
			m_akPoint[i] = kPoint + kProj[0] * kDir0 + kProj[1] * kDir1;
		}

		m_iIntersectionType = (m_iQuantity == 2 ? IT_SEGMENT : IT_POINT);
	}
}

#if defined(_MSC_VER)
#pragma warning(pop)
#endif