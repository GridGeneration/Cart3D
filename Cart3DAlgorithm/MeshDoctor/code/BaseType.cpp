#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable:4273)
#pragma warning(disable:4251)
#pragma warning(disable:4286)
#pragma warning(disable:4267)
#endif
#include <MeshDoctor/BaseType.h>
namespace Cart3DAlgorithm 
{
	template <class Vec, class Real>
	Real DistancePointToTriangle(const Vec& rkQ,const Triangle<Vec,Real>& tri)
	{
		Vec kDiff = tri.v[0] - rkQ;
		Vec kE0 = tri.e0, kE1 = tri.e1;
		Real fA00 = kE0.squaredNorm();
		Real fA01 = kE0.dot(kE1);
		Real fA11 = kE1.squaredNorm();
		Real fB0 = kDiff.dot(kE0);
		Real fB1 = kDiff.dot(kE1);
		Real fC = kDiff.squaredNorm();
		Real fDet = std::abs(fA00 * fA11 - fA01 * fA01);
		Real fS = fA01 * fB1 - fA11 * fB0;
		Real fT = fA01 * fB0 - fA00 * fB1;
		Real fSqrDist;

		if (fS + fT <= fDet)
		{
			if (fS < (Real)0.0)
			{
				if (fT < (Real)0.0)  // region 4
				{
					if (fB0 < (Real)0.0)
					{
						if (-fB0 >= fA00)
						{
							fSqrDist = fA00 + ((Real)2.0) * fB0 + fC;
						}
						else
						{
							fSqrDist = fC - fB0 * fB0 / fA00;
						}
					}
					else
					{
						if (fB1 >= (Real)0.0)
						{
							fSqrDist = fC;
						}
						else if (-fB1 >= fA11)
						{
							fSqrDist = fA11 + ((Real)2.0) * fB1 + fC;
						}
						else
						{
							fSqrDist = fC - fB1 * fB1 / fA11;
						}
					}
				}
				else  // region 3
				{
					if (fB1 >= (Real)0.0)
					{
						fSqrDist = fC;
					}
					else if (-fB1 >= fA11)
					{
						fSqrDist = fA11 + ((Real)2.0) * fB1 + fC;
					}
					else
					{
						fSqrDist = fC - fB1 * fB1 / fA11;
					}
				}
			}
			else if (fT < (Real)0.0)  // region 5
			{
				if (fB0 >= (Real)0.0)
				{
					fSqrDist = fC;
				}
				else if (-fB0 >= fA00)
				{
					fSqrDist = fA00 + ((Real)2.0) * fB0 + fC;
				}
				else
				{
					fSqrDist = fB0 * fS + fC - fB0 * fB0 / fA00;
				}
			}
			else  // region 0
			{
				Real fInvDet = ((Real)1.0) / fDet;
				fS *= fInvDet;
				fT *= fInvDet;
				fSqrDist = fS * (fA00 * fS + fA01 * fT + ((Real)2.0) * fB0) +
					fT * (fA01 * fS + fA11 * fT + ((Real)2.0) * fB1) + fC;
			}
		}
		else
		{
			Real fTmp0, fTmp1, fNumer, fDenom;

			if (fS < (Real)0.0)  // region 2
			{
				fTmp0 = fA01 + fB0;
				fTmp1 = fA11 + fB1;
				if (fTmp1 > fTmp0)
				{
					fNumer = fTmp1 - fTmp0;
					fDenom = fA00 - 2.0f * fA01 + fA11;
					if (fNumer >= fDenom)
					{
						fSqrDist = fA00 + ((Real)2.0) * fB0 + fC;
					}
					else
					{
						fS = fNumer / fDenom;
						fT = (Real)1.0 - fS;
						fSqrDist = fS * (fA00 * fS + fA01 * fT + 2.0f * fB0) +
							fT * (fA01 * fS + fA11 * fT + ((Real)2.0) * fB1) + fC;
					}
				}
				else
				{
					if (fTmp1 <= (Real)0.0)
					{
						fSqrDist = fA11 + ((Real)2.0) * fB1 + fC;
					}
					else if (fB1 >= (Real)0.0)
					{
						fSqrDist = fC;
					}
					else
					{
						fSqrDist = fC - fB1 * fB1 / fA11;
					}
				}
			}
			else if (fT < (Real)0.0)  // region 6
			{
				fTmp0 = fA01 + fB1;
				fTmp1 = fA00 + fB0;
				if (fTmp1 > fTmp0)
				{
					fNumer = fTmp1 - fTmp0;
					fDenom = fA00 - ((Real)2.0) * fA01 + fA11;
					if (fNumer >= fDenom)
					{
						fT = (Real)1.0;
						fS = (Real)0.0;
						fSqrDist = fA11 + ((Real)2.0) * fB1 + fC;
					}
					else
					{
						fT = fNumer / fDenom;
						fS = (Real)1.0 - fT;
						fSqrDist = fS * (fA00 * fS + fA01 * fT + ((Real)2.0) * fB0) +
							fT * (fA01 * fS + fA11 * fT + ((Real)2.0) * fB1) + fC;
					}
				}
				else
				{
					if (fTmp1 <= (Real)0.0)
					{
						fSqrDist = fA00 + ((Real)2.0) * fB0 + fC;
					}
					else if (fB0 >= (Real)0.0)
					{
						fSqrDist = fC;
					}
					else
					{
						fSqrDist = fC - fB0 * fB0 / fA00;
					}
				}
			}
			else  // region 1
			{
				fNumer = fA11 + fB1 - fA01 - fB0;
				if (fNumer <= (Real)0.0)
				{
					fSqrDist = fA11 + ((Real)2.0) * fB1 + fC;
				}
				else
				{
					fDenom = fA00 - 2.0f * fA01 + fA11;
					if (fNumer >= fDenom)
					{
						fSqrDist = fA00 + ((Real)2.0) * fB0 + fC;
					}
					else
					{
						fS = fNumer / fDenom;
						fT = (Real)1.0 - fS;
						fSqrDist = fS * (fA00 * fS + fA01 * fT + ((Real)2.0) * fB0) +
							fT * (fA01 * fS + fA11 * fT + ((Real)2.0) * fB1) + fC;
					}
				}
			}
		}
		return (Real)std::sqrt(std::abs(fSqrDist));
	}

	template <class Vec, class Real>
	Real DistancePointToSegment(
		const Vec& p0,
		const Vec& segs,
		const Vec& sege)
	{
		Vec m_kClosestPoint1;
		Vec kDiff = p0 - segs;
		Vec Origin = ((Real)0.5) * (segs + sege);
		Vec Direction = sege - segs;
		const Real Extent = ((Real)0.5) * Direction.norm();
		Real m_fSegmentParameter = Direction.dot(kDiff);
		if (-Extent < m_fSegmentParameter)
		{
			if (m_fSegmentParameter < Extent)
			{
				m_kClosestPoint1 = Origin + m_fSegmentParameter * Direction;
			}
			else
			{
				m_kClosestPoint1 = Origin + Extent * Direction;
				m_fSegmentParameter = Extent;
			}
		}
		else
		{
			m_kClosestPoint1 = Origin - Extent * Direction;
			m_fSegmentParameter = -Extent;
		}
		Vec m_kClosestPoint0 = p0;
		kDiff = m_kClosestPoint1 - m_kClosestPoint0;
		return kDiff.squaredNorm();
	}

	cfloat DistancePointToTriangle(const cvector2d& rkQ, const Triangle2d& tri)
	{
		return DistancePointToTriangle<cvector2d, cfloat>(rkQ,tri);
	}

	cfloat __stdcall DistancePointToTriangle(const cvector3d& rkQ, const Triangle3d& tri)
	{
		return DistancePointToTriangle<cvector3d, cfloat>(rkQ, tri);
	}

	cfloat __stdcall DistancePointToSegment(const cvector2d& p0, const cvector2d& segs, const cvector2d& sege)
	{
		return DistancePointToSegment<cvector2d,cfloat>(p0, segs, sege);
	}

	cfloat __stdcall DistancePointToSegment(const cvector3d& p0, const cvector3d& segs, const cvector3d& sege)
	{
		return DistancePointToSegment<cvector3d, cfloat>(p0, segs, sege);
	}

	template class Triangle<cvector2d,cfloat>;
	template class Triangle<cvector3d,cfloat>;

}

#if defined(_MSC_VER)
#pragma warning(pop)
#endif