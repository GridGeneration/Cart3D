#ifndef WM4QUERY_H
#define WM4QUERY_H

#include "Wm4Math.h"

namespace Cart3DAlgorithm
{

	class Query
	{
	public:
		// abstract base class
		virtual ~Query();

		// run-time type information
		enum Type
		{
			QT_INT64,
			QT_INTEGER,
			QT_RATIONAL,
			QT_REAL,
			QT_FILTERED
		};

		virtual Type GetType() const = 0;
		static bool Sort(int& iV0, int& iV1);
		static bool Sort(int& iV0, int& iV1, int& iV2);
		static bool Sort(int& iV0, int& iV1, int& iV2, int& iV3);

	protected:
		Query();
	};

	template <class Real>
	class Query2 : public Query
	{
	public:
		// The base class handles floating-point queries.
		Query2(int iVQuantity, const Vector2<Real>* akVertex);
		virtual ~Query2();

		// run-time type information
		virtual Query::Type GetType() const;

		// member access
		int GetQuantity() const;
		const Vector2<Real>* GetVertices() const;

		// Queries about the relation of a point to various geometric objects.

		// returns
		//   +1, on right of line
		//   -1, on left of line
		//    0, on the line
		virtual int ToLine(int i, int iV0, int iV1) const;
		virtual int ToLine(const Vector2<Real>& rkP, int iV0, int iV1) const;

		// returns
		//   +1, outside triangle
		//   -1, inside triangle
		//    0, on triangle
		virtual int ToTriangle(int i, int iV0, int iV1, int iV2) const;
		virtual int ToTriangle(const Vector2<Real>& rkP, int iV0, int iV1,
			int iV2) const;

		// returns
		//   +1, outside circumcircle of triangle
		//   -1, inside circumcircle of triangle
		//    0, on circumcircle of triangle
		virtual int ToCircumcircle(int i, int iV0, int iV1, int iV2) const;
		virtual int ToCircumcircle(const Vector2<Real>& rkP, int iV0, int iV1,
			int iV2) const;

	protected:
		int m_iVQuantity;
		const Vector2<Real>* m_akVertex;
		static Real Dot(Real fX0, Real fY0, Real fX1, Real fY1);
		static Real Det2(Real fX0, Real fY0, Real fX1, Real fY1);
		static Real Det3(Real iX0, Real iY0, Real iZ0, Real iX1, Real iY1,
			Real iZ1, Real iX2, Real iY2, Real iZ2);
	};


	typedef Query2<float> Query2f;
	typedef Query2<double> Query2d;
}

#endif