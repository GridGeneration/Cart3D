#include "Wm4Query.h"
#include <assert.h>


namespace Cart3DAlgorithm
{
	//----------------------------------------------------------------------------
    Query::Query()
	{
	}
	//----------------------------------------------------------------------------
    Query::~Query()
	{
	}
	//----------------------------------------------------------------------------
    bool Query::Sort(int& iV0, int& iV1)
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
	//----------------------------------------------------------------------------
    bool Query::Sort(int& iV0, int& iV1, int& iV2)
	{
		int j0, j1, j2;
		bool bPositive;

		if (iV0 < iV1)
		{
			if (iV2 < iV0)
			{
				j0 = 2; j1 = 0; j2 = 1; bPositive = true;
			}
			else if (iV2 < iV1)
			{
				j0 = 0; j1 = 2; j2 = 1; bPositive = false;
			}
			else
			{
				j0 = 0; j1 = 1; j2 = 2; bPositive = true;
			}
		}
		else
		{
			if (iV2 < iV1)
			{
				j0 = 2; j1 = 1; j2 = 0; bPositive = false;
			}
			else if (iV2 < iV0)
			{
				j0 = 1; j1 = 2; j2 = 0; bPositive = true;
			}
			else
			{
				j0 = 1; j1 = 0; j2 = 2; bPositive = false;
			}
		}

		int aiValue[3] = { iV0, iV1, iV2 };
		iV0 = aiValue[j0];
		iV1 = aiValue[j1];
		iV2 = aiValue[j2];
		return bPositive;
	}
	//----------------------------------------------------------------------------
    bool Query::Sort(int& iV0, int& iV1, int& iV2, int& iV3)
	{
		int j0, j1, j2, j3;
		bool bPositive;

		if (iV0 < iV1)
		{
			if (iV2 < iV3)
			{
				if (iV1 < iV2)
				{
					j0 = 0; j1 = 1; j2 = 2; j3 = 3; bPositive = true;
				}
				else if (iV3 < iV0)
				{
					j0 = 2; j1 = 3; j2 = 0; j3 = 1; bPositive = true;
				}
				else if (iV2 < iV0)
				{
					if (iV3 < iV1)
					{
						j0 = 2; j1 = 0; j2 = 3; j3 = 1; bPositive = false;
					}
					else
					{
						j0 = 2; j1 = 0; j2 = 1; j3 = 3; bPositive = true;
					}
				}
				else
				{
					if (iV3 < iV1)
					{
						j0 = 0; j1 = 2; j2 = 3; j3 = 1; bPositive = true;
					}
					else
					{
						j0 = 0; j1 = 2; j2 = 1; j3 = 3; bPositive = false;
					}
				}
			}
			else
			{
				if (iV1 < iV3)
				{
					j0 = 0; j1 = 1; j2 = 3; j3 = 2; bPositive = false;
				}
				else if (iV2 < iV0)
				{
					j0 = 3; j1 = 2; j2 = 0; j3 = 1; bPositive = false;
				}
				else if (iV3 < iV0)
				{
					if (iV2 < iV1)
					{
						j0 = 3; j1 = 0; j2 = 2; j3 = 1; bPositive = true;
					}
					else
					{
						j0 = 3; j1 = 0; j2 = 1; j3 = 2; bPositive = false;
					}
				}
				else
				{
					if (iV2 < iV1)
					{
						j0 = 0; j1 = 3; j2 = 2; j3 = 1; bPositive = false;
					}
					else
					{
						j0 = 0; j1 = 3; j2 = 1; j3 = 2; bPositive = true;
					}
				}
			}
		}
		else
		{
			if (iV2 < iV3)
			{
				if (iV0 < iV2)
				{
					j0 = 1; j1 = 0; j2 = 2; j3 = 3; bPositive = false;
				}
				else if (iV3 < iV1)
				{
					j0 = 2; j1 = 3; j2 = 1; j3 = 0; bPositive = false;
				}
				else if (iV2 < iV1)
				{
					if (iV3 < iV0)
					{
						j0 = 2; j1 = 1; j2 = 3; j3 = 0; bPositive = true;
					}
					else
					{
						j0 = 2; j1 = 1; j2 = 0; j3 = 3; bPositive = false;
					}
				}
				else
				{
					if (iV3 < iV0)
					{
						j0 = 1; j1 = 2; j2 = 3; j3 = 0; bPositive = false;
					}
					else
					{
						j0 = 1; j1 = 2; j2 = 0; j3 = 3; bPositive = true;
					}
				}
			}
			else
			{
				if (iV0 < iV3)
				{
					j0 = 1; j1 = 0; j2 = 3; j3 = 2; bPositive = true;
				}
				else if (iV2 < iV1)
				{
					j0 = 3; j1 = 2; j2 = 1; j3 = 0; bPositive = true;
				}
				else if (iV3 < iV1)
				{
					if (iV2 < iV0)
					{
						j0 = 3; j1 = 1; j2 = 2; j3 = 0; bPositive = false;
					}
					else
					{
						j0 = 3; j1 = 1; j2 = 0; j3 = 2; bPositive = true;
					}
				}
				else
				{
					if (iV2 < iV0)
					{
						j0 = 1; j1 = 3; j2 = 2; j3 = 0; bPositive = true;
					}
					else
					{
						j0 = 1; j1 = 3; j2 = 0; j3 = 2; bPositive = false;
					}
				}
			}
		}

		int aiValue[4] = { iV0, iV1, iV2, iV3 };
		iV0 = aiValue[j0];
		iV1 = aiValue[j1];
		iV2 = aiValue[j2];
		iV3 = aiValue[j3];
		return bPositive;
	}
	//----------------------------------------------------------------------------


//----------------------------------------------------------------------------
	template <class Real>
	Query2<Real>::Query2(int iVQuantity, const Vector2<Real>* akVertex)
	{
		assert(iVQuantity > 0 && akVertex);
		m_iVQuantity = iVQuantity;
		m_akVertex = akVertex;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Query2<Real>::~Query2()
	{
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Query::Type Query2<Real>::GetType() const
	{
		return Query::QT_REAL;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	int Query2<Real>::GetQuantity() const
	{
		return m_iVQuantity;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	const Vector2<Real>* Query2<Real>::GetVertices() const
	{
		return m_akVertex;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	int Query2<Real>::ToLine(int i, int iV0, int iV1) const
	{
		return ToLine(m_akVertex[i], iV0, iV1);
	}
	//----------------------------------------------------------------------------
	template <class Real>
	int Query2<Real>::ToLine(const Vector2<Real>& rkP, int iV0, int iV1) const
	{
		bool bPositive = Sort(iV0, iV1);

		const Vector2<Real>& rkV0 = m_akVertex[iV0];
		const Vector2<Real>& rkV1 = m_akVertex[iV1];

		Real fX0 = rkP[0] - rkV0[0];
		Real fY0 = rkP[1] - rkV0[1];
		Real fX1 = rkV1[0] - rkV0[0];
		Real fY1 = rkV1[1] - rkV0[1];

		Real fDet2 = Det2(fX0, fY0, fX1, fY1);
		if (!bPositive)
		{
			fDet2 = -fDet2;
		}

		return (fDet2 > (Real)0 ? +1 : (fDet2 < (Real)0 ? -1 : 0));
	}
	//----------------------------------------------------------------------------
	template <class Real>
	int Query2<Real>::ToTriangle(int i, int iV0, int iV1, int iV2) const
	{
		return ToTriangle(m_akVertex[i], iV0, iV1, iV2);
	}
	//----------------------------------------------------------------------------
	template <class Real>
	int Query2<Real>::ToTriangle(const Vector2<Real>& rkP, int iV0, int iV1,
		int iV2) const
	{
		int iSign0 = ToLine(rkP, iV1, iV2);
		if (iSign0 > 0)
		{
			return +1;
		}

		int iSign1 = ToLine(rkP, iV0, iV2);
		if (iSign1 < 0)
		{
			return +1;
		}

		int iSign2 = ToLine(rkP, iV0, iV1);
		if (iSign2 > 0)
		{
			return +1;
		}

		return ((iSign0 && iSign1 && iSign2) ? -1 : 0);
	}
	//----------------------------------------------------------------------------
	template <class Real>
	int Query2<Real>::ToCircumcircle(int i, int iV0, int iV1, int iV2) const
	{
		return ToCircumcircle(m_akVertex[i], iV0, iV1, iV2);
	}
	//----------------------------------------------------------------------------
	template <class Real>
	int Query2<Real>::ToCircumcircle(const Vector2<Real>& rkP, int iV0,
		int iV1, int iV2) const
	{
		bool bPositive = Sort(iV0, iV1, iV2);

		const Vector2<Real>& rkV0 = m_akVertex[iV0];
		const Vector2<Real>& rkV1 = m_akVertex[iV1];
		const Vector2<Real>& rkV2 = m_akVertex[iV2];

		Real fS0x = rkV0[0] + rkP[0];
		Real fD0x = rkV0[0] - rkP[0];
		Real fS0y = rkV0[1] + rkP[1];
		Real fD0y = rkV0[1] - rkP[1];
		Real fS1x = rkV1[0] + rkP[0];
		Real fD1x = rkV1[0] - rkP[0];
		Real fS1y = rkV1[1] + rkP[1];
		Real fD1y = rkV1[1] - rkP[1];
		Real fS2x = rkV2[0] + rkP[0];
		Real fD2x = rkV2[0] - rkP[0];
		Real fS2y = rkV2[1] + rkP[1];
		Real fD2y = rkV2[1] - rkP[1];
		Real fZ0 = fS0x * fD0x + fS0y * fD0y;
		Real fZ1 = fS1x * fD1x + fS1y * fD1y;
		Real fZ2 = fS2x * fD2x + fS2y * fD2y;

		Real fDet3 = Det3(fD0x, fD0y, fZ0, fD1x, fD1y, fZ1, fD2x, fD2y, fZ2);
		if (!bPositive)
		{
			fDet3 = -fDet3;
		}

		return (fDet3 < (Real)0 ? 1 : (fDet3 > (Real)0 ? -1 : 0));
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Real Query2<Real>::Dot(Real fX0, Real fY0, Real fX1, Real fY1)
	{
		return fX0 * fX1 + fY0 * fY1;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Real Query2<Real>::Det2(Real fX0, Real fY0, Real fX1, Real fY1)
	{
		return fX0 * fY1 - fX1 * fY0;
	}
	//----------------------------------------------------------------------------
	template <class Real>
	Real Query2<Real>::Det3(Real fX0, Real fY0, Real fZ0, Real fX1, Real fY1,
		Real fZ1, Real fX2, Real fY2, Real fZ2)
	{
		Real fC00 = fY1 * fZ2 - fY2 * fZ1;
		Real fC01 = fY2 * fZ0 - fY0 * fZ2;
		Real fC02 = fY0 * fZ1 - fY1 * fZ0;
		return fX0 * fC00 + fX1 * fC01 + fX2 * fC02;
	}
	//----------------------------------------------------------------------------
	template class Query2<float>;
	template class Query2<double>;
	//----------------------------------------------------------------------------
}

