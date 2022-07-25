#include "Wm4BaseType.h"

namespace Cart3DAlgorithm
{

	//----------------------------------------------------------------------------
	template <class Real>
	Line2<Real>::Line2()
	{
		// uninitialized
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Line2<Real>::Line2(const Vector2<Real>& rkOrigin,
		const Vector2<Real>& rkDirection)
		:
		Origin(rkOrigin),
		Direction(rkDirection)
	{
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Segment2<Real>::Segment2()
	{
		// uninitialized
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Segment2<Real>::Segment2(const Vector2<Real>& rkOrigin,
		const Vector2<Real>& rkDirection, Real fExtent)
		:
		Origin(rkOrigin),
		Direction(rkDirection),
		Extent(fExtent)
	{
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Vector2<Real> Segment2<Real>::GetPosEnd() const
	{
		return Origin + Direction * Extent;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Vector2<Real> Segment2<Real>::GetNegEnd() const
	{
		return Origin - Direction * Extent;
	}
	//----------------------------------------------------------------------------

	template <class Real>
	Triangle2<Real>::Triangle2()
	{
		// uninitialized
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Triangle2<Real>::Triangle2(const Vector2<Real>& rkV0,
		const Vector2<Real>& rkV1, const Vector2<Real>& rkV2)
	{
		V[0] = rkV0;
		V[1] = rkV1;
		V[2] = rkV2;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Triangle2<Real>::Triangle2(const Vector2<Real> akV[3])
	{
		for (int i = 0; i < 3; i++)
		{
			V[i] = akV[i];
		}
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Real Triangle2<Real>::DistanceTo(const Vector2<Real>& rkQ) const
	{
		Vector2<Real> kDiff = V[0] - rkQ;
		Vector2<Real> kE0 = V[1] - V[0], kE1 = V[2] - V[0];
		Real fA00 = kE0.SquaredLength();
		Real fA01 = kE0.Dot(kE1);
		Real fA11 = kE1.SquaredLength();
		Real fB0 = kDiff.Dot(kE0);
		Real fB1 = kDiff.Dot(kE1);
		Real fC = kDiff.SquaredLength();
		Real fDet = Math<Real>::FAbs(fA00*fA11 - fA01 * fA01);
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
							fSqrDist = fA00 + ((Real)2.0)*fB0 + fC;
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
							fSqrDist = fA11 + ((Real)2.0)*fB1 + fC;
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
						fSqrDist = fA11 + ((Real)2.0)*fB1 + fC;
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
					fSqrDist = fA00 + ((Real)2.0)*fB0 + fC;
				}
				else
				{
					fSqrDist = fB0 * fS + fC - fB0 * fB0 / fA00;
				}
			}
			else  // region 0
			{
				// minimum at interior point
				Real fInvDet = ((Real)1.0) / fDet;
				fS *= fInvDet;
				fT *= fInvDet;
				fSqrDist = fS * (fA00*fS + fA01 * fT + ((Real)2.0)*fB0) +
					fT * (fA01*fS + fA11 * fT + ((Real)2.0)*fB1) + fC;
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
					fDenom = fA00 - 2.0f*fA01 + fA11;
					if (fNumer >= fDenom)
					{
						fSqrDist = fA00 + ((Real)2.0)*fB0 + fC;
					}
					else
					{
						fS = fNumer / fDenom;
						fT = (Real)1.0 - fS;
						fSqrDist = fS * (fA00*fS + fA01 * fT + 2.0f*fB0) +
							fT * (fA01*fS + fA11 * fT + ((Real)2.0)*fB1) + fC;
					}
				}
				else
				{
					if (fTmp1 <= (Real)0.0)
					{
						fSqrDist = fA11 + ((Real)2.0)*fB1 + fC;
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
					fDenom = fA00 - ((Real)2.0)*fA01 + fA11;
					if (fNumer >= fDenom)
					{
						fT = (Real)1.0;
						fS = (Real)0.0;
						fSqrDist = fA11 + ((Real)2.0)*fB1 + fC;
					}
					else
					{
						fT = fNumer / fDenom;
						fS = (Real)1.0 - fT;
						fSqrDist = fS * (fA00*fS + fA01 * fT + ((Real)2.0)*fB0) +
							fT * (fA01*fS + fA11 * fT + ((Real)2.0)*fB1) + fC;
					}
				}
				else
				{
					if (fTmp1 <= (Real)0.0)
					{
						fSqrDist = fA00 + ((Real)2.0)*fB0 + fC;
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
					fSqrDist = fA11 + ((Real)2.0)*fB1 + fC;
				}
				else
				{
					fDenom = fA00 - 2.0f*fA01 + fA11;
					if (fNumer >= fDenom)
					{
						fSqrDist = fA00 + ((Real)2.0)*fB0 + fC;
					}
					else
					{
						fS = fNumer / fDenom;
						fT = (Real)1.0 - fS;
						fSqrDist = fS * (fA00*fS + fA01 * fT + ((Real)2.0)*fB0) +
							fT * (fA01*fS + fA11 * fT + ((Real)2.0)*fB1) + fC;
					}
				}
			}
		}

		return Math<Real>::Sqrt(Math<Real>::FAbs(fSqrDist));
	}

	//----------------------------------------------------------------------------
	template <class Real>
	Line3<Real>::Line3()
	{
		// uninitialized
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Line3<Real>::Line3(const Vector3<Real>& rkOrigin,
		const Vector3<Real>& rkDirection)
		:
		Origin(rkOrigin),
		Direction(rkDirection)
	{
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Plane3<Real>::Plane3()
	{
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Plane3<Real>::Plane3(const Plane3& rkPlane)
		:
		Normal(rkPlane.Normal)
	{
		Constant = rkPlane.Constant;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Plane3<Real>::Plane3(const Vector3<Real>& rkNormal, Real fConstant)
		:
		Normal(rkNormal)
	{
		Constant = fConstant;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Plane3<Real>::Plane3(const Vector3<Real>& rkNormal, const Vector3<Real>& rkP)
		:
		Normal(rkNormal)
	{
		Constant = rkNormal.Dot(rkP);
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Plane3<Real>::Plane3(const Vector3<Real>& rkP0, const Vector3<Real>& rkP1,
		const Vector3<Real>& rkP2)
	{
		Vector3<Real> kEdge1 = rkP1 - rkP0;
		Vector3<Real> kEdge2 = rkP2 - rkP0;
		Normal = kEdge1.UnitCross(kEdge2);
		Constant = Normal.Dot(rkP0);
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Plane3<Real>& Plane3<Real>::operator= (const Plane3& rkPlane)
	{
		Normal = rkPlane.Normal;
		Constant = rkPlane.Constant;
		return *this;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Real Plane3<Real>::DistanceTo(const Vector3<Real>& rkP) const
	{
		return Normal.Dot(rkP) - Constant;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	int Plane3<Real>::WhichSide(const Vector3<Real>& rkQ) const
	{
		Real fDistance = DistanceTo(rkQ);

		if (fDistance < (Real)0.0)
		{
			return -1;
		}

		if (fDistance > (Real)0.0)
		{
			return +1;
		}

		return 0;
	}

	//----------------------------------------------------------------------------
	template <class Real>
	Triangle3<Real>::Triangle3()
	{
		// uninitialized
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Triangle3<Real>::Triangle3(const Vector3<Real>& rkV0,
		const Vector3<Real>& rkV1, const Vector3<Real>& rkV2)
	{
		V[0] = rkV0;
		V[1] = rkV1;
		V[2] = rkV2;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Triangle3<Real>::Triangle3(const Vector3<Real> akV[3])
	{
		for (int i = 0; i < 3; i++)
		{
			V[i] = akV[i];
		}
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Real Triangle3<Real>::DistanceTo(const Vector3<Real>& rkQ) const
	{
		Vector3<Real> kDiff = V[0] - rkQ;
		Vector3<Real> kE0 = V[1] - V[0], kE1 = V[2] - V[0];
		Real fA00 = kE0.SquaredLength();
		Real fA01 = kE0.Dot(kE1);
		Real fA11 = kE1.SquaredLength();
		Real fB0 = kDiff.Dot(kE0);
		Real fB1 = kDiff.Dot(kE1);
		Real fC = kDiff.SquaredLength();
		Real fDet = Math<Real>::FAbs(fA00*fA11 - fA01 * fA01);
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
							fSqrDist = fA00 + ((Real)2.0)*fB0 + fC;
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
							fSqrDist = fA11 + ((Real)2.0)*fB1 + fC;
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
						fSqrDist = fA11 + ((Real)2.0)*fB1 + fC;
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
					fSqrDist = fA00 + ((Real)2.0)*fB0 + fC;
				}
				else
				{
					fSqrDist = fC - fB0 * fB0 / fA00;
				}
			}
			else  // region 0
			{
				// minimum at interior point
				Real fInvDet = ((Real)1.0) / fDet;
				fS *= fInvDet;
				fT *= fInvDet;
				fSqrDist = fS * (fA00*fS + fA01 * fT + ((Real)2.0)*fB0) +
					fT * (fA01*fS + fA11 * fT + ((Real)2.0)*fB1) + fC;
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
					fDenom = fA00 - 2.0f*fA01 + fA11;
					if (fNumer >= fDenom)
					{
						fSqrDist = fA00 + ((Real)2.0)*fB0 + fC;
					}
					else
					{
						fS = fNumer / fDenom;
						fT = (Real)1.0 - fS;
						fSqrDist = fS * (fA00*fS + fA01 * fT + 2.0f*fB0) +
							fT * (fA01*fS + fA11 * fT + ((Real)2.0)*fB1) + fC;
					}
				}
				else
				{
					if (fTmp1 <= (Real)0.0)
					{
						fSqrDist = fA11 + ((Real)2.0)*fB1 + fC;
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
					fDenom = fA00 - ((Real)2.0)*fA01 + fA11;
					if (fNumer >= fDenom)
					{
						fT = (Real)1.0;
						fS = (Real)0.0;
						fSqrDist = fA11 + ((Real)2.0)*fB1 + fC;
					}
					else
					{
						fT = fNumer / fDenom;
						fS = (Real)1.0 - fT;
						fSqrDist = fS * (fA00*fS + fA01 * fT + ((Real)2.0)*fB0) +
							fT * (fA01*fS + fA11 * fT + ((Real)2.0)*fB1) + fC;
					}
				}
				else
				{
					if (fTmp1 <= (Real)0.0)
					{
						fSqrDist = fA00 + ((Real)2.0)*fB0 + fC;
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
					fSqrDist = fA11 + ((Real)2.0)*fB1 + fC;
				}
				else
				{
					fDenom = fA00 - 2.0f*fA01 + fA11;
					if (fNumer >= fDenom)
					{
						fSqrDist = fA00 + ((Real)2.0)*fB0 + fC;
					}
					else
					{
						fS = fNumer / fDenom;
						fT = (Real)1.0 - fS;
						fSqrDist = fS * (fA00*fS + fA01 * fT + ((Real)2.0)*fB0) +
							fT * (fA01*fS + fA11 * fT + ((Real)2.0)*fB1) + fC;
					}
				}
			}
		}

		return Math<Real>::Sqrt(Math<Real>::FAbs(fSqrDist));
	}
	//----------------------------------------------------------------------------

	template <class Real>
	Real Triangle3<Real>::DistancePointToSegment3(
		const Vector3<Real>& m_pkVector,
		const Vector3<Real>& segs,
		const Vector3<Real>& sege)
	{
		Vector3<Real> m_kClosestPoint1;
		Vector3<Real> kDiff = m_pkVector - segs;
		Vector3<Real> Origin = ((Real)0.5)*(segs + sege);
		Vector3<Real> Direction = sege - segs;
		const Real Extent = ((Real)0.5)*Direction.Normalize();
		Real m_fSegmentParameter = Direction.Dot(kDiff);
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
		Vector3<Real> m_kClosestPoint0 = m_pkVector;
		kDiff = m_kClosestPoint1 - m_kClosestPoint0;
		return kDiff.SquaredLength();
	}
	//----------------------------------------------------------------------------
	template class Line2<float>;
	template class Line2<double>;
	template class Segment2<float>;
	template class Segment2<double>;
	template class Triangle2<float>;
	template class Triangle2<double>;
	template class Line3<float>;
	template class Line3<double>;
	template class Plane3<float>;
	template class Plane3<double>;
	template class Triangle3<float>;
	template class Triangle3<double>;
	//----------------------------------------------------------------------------
}