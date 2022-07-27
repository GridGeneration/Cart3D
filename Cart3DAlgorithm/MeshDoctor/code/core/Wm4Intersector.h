#ifndef WM4INTERSECTOR_H
#define WM4INTERSECTOR_H

#include "Wm4Math.h"
#include "Wm4BaseType.h"
namespace Cart3DAlgorithm
{

	template <class Real, class TVector>
	class  Intersector
	{
	public:
		virtual ~Intersector();
		virtual bool Test();
		virtual bool Find();
		virtual bool Test(Real fTMax, const TVector& rkVelocity0,
			const TVector& rkVelocity1);
		virtual bool Find(Real fTMax, const TVector& rkVelocity0,
			const TVector& rkVelocity1);
		Real GetContactTime() const;
		enum
		{
			IT_EMPTY,
			IT_POINT,
			IT_SEGMENT,
			IT_RAY,
			IT_LINE,
			IT_POLYGON,
			IT_PLANE,
			IT_POLYHEDRON,
			IT_OTHER
		};
		int GetIntersectionType() const;

	protected:
		Intersector();

		Real m_fContactTime;
		int m_iIntersectionType;
	};

	template <class Real>
	class  Intersector1
	{
	public:
		Intersector1(Real fU0, Real fU1, Real fV0, Real fV1);
		Intersector1(Real afU[2], Real afV[2]);
		virtual ~Intersector1();

		// object access
		Real GetU(int i) const;
		Real GetV(int i) const;

		// static intersection queries
		virtual bool Test();
		virtual bool Find();

		// Dynamic intersection queries.  The Find query produces a set of first
		// contact.
		virtual bool Test(Real fTMax, Real fSpeedU, Real fSpeedV);
		virtual bool Find(Real fTMax, Real fSpeedU, Real fSpeedV);

		// The time at which two intervals are in first/last contact for the
		// dynamic intersection queries.
		Real GetFirstTime() const;
		Real GetLastTime() const;

		// information about the intersection set
		int GetQuantity() const;
		Real GetOverlap(int i) const;

	protected:
		// the intervals to intersect
		Real m_afU[2], m_afV[2];

		// information about the intersection set
		Real m_fFirstTime, m_fLastTime;
		int m_iQuantity;
		Real m_afOverlap[2];
	};


	template <class Real>
	class  IntrLine2Triangle2
		: public Intersector<Real, Vector2<Real> >
	{
	public:
		IntrLine2Triangle2(const Line2<Real>& rkLine,
			const Triangle2<Real>& rkTriangle);

		// Object access.
		const Line2<Real>& GetLine() const;
		const Triangle2<Real>& GetTriangle() const;

		// Static intersection query.
		virtual bool Test();
		virtual bool Find();

		int GetQuantity() const;
		const Vector2<Real>& GetPoint(int i) const;

	private:
		using Intersector<Real, Vector2<Real> >::IT_EMPTY;
		using Intersector<Real, Vector2<Real> >::IT_POINT;
		using Intersector<Real, Vector2<Real> >::IT_SEGMENT;
		using Intersector<Real, Vector2<Real> >::m_iIntersectionType;

		// The objects to intersect.
		const Line2<Real>* m_pkLine;
		const Triangle2<Real>* m_pkTriangle;

		// Information about the intersection set.
		int m_iQuantity;
		Vector2<Real> m_akPoint[2];

		// Internal use, shared by IntrRay2Triangle2 and IntrSegment2Triangle2.
	public:
		// Determine how the triangle and line intersect (if at all).
		static void TriangleLineRelations(const Vector2<Real>& rkOrigin,
			const Vector2<Real>& rkDirection, const Triangle2<Real>& rkTriangle,
			Real afDist[3], int aiSign[3], int& riPositive, int& riNegative,
			int& riZero);

		static void GetInterval(const Vector2<Real>& rkOrigin,
			const Vector2<Real>& rkDirection, const Triangle2<Real>& rkTriangle,
			const Real afDist[3], const int aiSign[3], Real afParam[2]);
	};


	template <class Real>
	class  IntrSegment2Triangle2: public Intersector<Real, Vector2<Real> >
	{
	public:
		IntrSegment2Triangle2(const Segment2<Real>& rkSegment,
			const Triangle2<Real>& rkTriangle);

		// Object access.
		const Segment2<Real>& GetSegment() const;
		const Triangle2<Real>& GetTriangle() const;

		// Static intersection query.
		virtual bool Test();
		virtual bool Find();

		int GetQuantity() const;
		const Vector2<Real>& GetPoint(int i) const;

	private:
		using Intersector<Real, Vector2<Real> >::IT_EMPTY;
		using Intersector<Real, Vector2<Real> >::IT_POINT;
		using Intersector<Real, Vector2<Real> >::IT_SEGMENT;
		using Intersector<Real, Vector2<Real> >::m_iIntersectionType;

		// The objects to intersect.
		const Segment2<Real>* m_pkSegment;
		const Triangle2<Real>* m_pkTriangle;

		// Information about the intersection set.
		int m_iQuantity;
		Vector2<Real> m_akPoint[2];
	};


	template <class Real>
	class IntrTriangle2Triangle2 : public Intersector<Real, Vector2<Real> >
	{
	public:
		IntrTriangle2Triangle2(const Triangle2<Real>& rkTriangle0,
			const Triangle2<Real>& rkTriangle1);

		// object access
		const Triangle2<Real>& GetTriangle0() const;
		const Triangle2<Real>& GetTriangle1() const;

		// static queries
		virtual bool Test();
		virtual bool Find();

		// dynamic queries
		virtual bool Test(Real fTMax, const Vector2<Real>& rkVelocity0,
			const Vector2<Real>& rkVelocity1);
		virtual bool Find(Real fTMax, const Vector2<Real>& rkVelocity0,
			const Vector2<Real>& rkVelocity1);

		// information about the intersection set
		int GetQuantity() const;
		const Vector2<Real>& GetPoint(int i) const;

	private:
		using Intersector<Real, Vector2<Real> >::m_fContactTime;

		static int WhichSide(const Vector2<Real> akV[3],
			const Vector2<Real>& rkP, const Vector2<Real>& rkD);

		static void ClipConvexPolygonAgainstLine(const Vector2<Real>& rkN,
			Real fC, int& riQuantity, Vector2<Real> akV[6]);

		enum ProjectionMap
		{
			M21,  // 2 vertices map to min, 1 vertex maps to max
			M12,  // 1 vertex maps to min, 2 vertices map to max
			M11   // 1 vertex maps to min, 1 vertex maps to max
		};

		class Configuration
		{
		public:
			ProjectionMap Map;  // how vertices map to the projection interval
			int Index[3];       // the sorted indices of the vertices
			Real Min, Max;      // the interval is [min,max]
		};

		void ComputeTwo(Configuration& rkCfg, const Vector2<Real> akV[3],
			const Vector2<Real>& rkD, int iI0, int iI1, int iI2);

		void ComputeThree(Configuration& rkCfg, const Vector2<Real> akV[3],
			const Vector2<Real>& rkD, const Vector2<Real>& rkP);

		static bool NoIntersect(const Configuration& rkCfg0,
			const Configuration& rkCfg1, Real fTMax, Real fSpeed, int& riSide,
			Configuration& rkTCfg0, Configuration& rkTCfg1, Real& rfTFirst,
			Real& rfTLast);

		static void GetIntersection(const Configuration& rkCfg0,
			const Configuration& rkCfg1, int iSide, const Vector2<Real> akV0[3],
			const Vector2<Real> akV1[3], int& riQuantity,
			Vector2<Real> akVertex[6]);
		const Triangle2<Real>* m_pkTriangle0;
		const Triangle2<Real>* m_pkTriangle1;

		int m_iQuantity;
		Vector2<Real> m_akPoint[6];
	};



	template <class Real>
	class  IntrTriangle3Triangle3
		: public Intersector<Real, Vector3<Real> >
	{
	public:
		IntrTriangle3Triangle3(const Triangle3<Real>& rkTriangle0,
			const Triangle3<Real>& rkTriangle1);

		// object access
		const Triangle3<Real>& GetTriangle0() const;
		const Triangle3<Real>& GetTriangle1() const;

		bool ReportCoplanarIntersections;  // default 'true'

		// static queries
		virtual bool Test();
		virtual bool Find();

		// dynamic queries
		virtual bool Test(Real fTMax, const Vector3<Real>& rkVelocity0,
			const Vector3<Real>& rkVelocity1);
		virtual bool Find(Real fTMax, const Vector3<Real>& rkVelocity0,
			const Vector3<Real>& rkVelocity1);

		// information about the intersection set
		int GetQuantity() const;
		const Vector3<Real>& GetPoint(int i) const;

	private:
		using Intersector<Real, Vector3<Real> >::IT_POINT;
		using Intersector<Real, Vector3<Real> >::IT_SEGMENT;
		using Intersector<Real, Vector3<Real> >::m_iIntersectionType;
		using Intersector<Real, Vector3<Real> >::m_fContactTime;

		static void ProjectOntoAxis(const Triangle3<Real>& rkTri,
			const Vector3<Real>& rkAxis, Real& rfMin, Real& rfMax);

		static void TrianglePlaneRelations(const Triangle3<Real>& rkTriangle,
			const Plane3<Real>& rkPlane, Real afDistance[3], int aiSign[3],
			int& riPositive, int& riNegative, int& riZero);

		static void GetInterval(const Triangle3<Real>& rkTriangle,
			const Line3<Real>& rkLine, const Real afDistance[3],
			const int aiSign[3], Real afParam[2]);

		bool ContainsPoint(const Triangle3<Real>& rkTriangle,
			const Plane3<Real>& rkPlane, const Vector3<Real>& rkPoint);

		bool IntersectsSegment(const Plane3<Real>& rkPlane,
			const Triangle3<Real>& rkTriangle, const Vector3<Real>& rkEnd0,
			const Vector3<Real>& rkEnd1);

		bool GetCoplanarIntersection(const Plane3<Real>& rkPlane,
			const Triangle3<Real>& rkTri0, const Triangle3<Real>& rkTri1);

		static bool TestOverlap(Real fTMax, Real fSpeed, Real fUMin,
			Real fUMax, Real fVMin, Real fVMax, Real& rfTFirst, Real& rfTLast);

		bool TestOverlap(const Vector3<Real>& rkAxis, Real fTMax,
			const Vector3<Real>& rkVelocity, Real& rfTFirst, Real& rfTLast);

		bool PointIsInSegment(const Vector3<Real>& pt, const Vector3<Real>& segs, const Vector3<Real>& sege);

		enum ProjectionMap
		{
			M2, M11,                // lines
			M3, M21, M12, M111,     // triangles
			M44, M2_2, M1_1         // boxes
		};

		enum ContactSide
		{
			CS_LEFT,
			CS_RIGHT,
			CS_NONE
		};

		class  Configuration
		{
		public:
			ProjectionMap Map;  // how vertices map to the projection interval
			int Index[8];       // the sorted indices of the vertices
			Real Min, Max;      // the interval is [min,max]
		};

		static void ProjectOntoAxis(const Triangle3<Real>& rkTri,
			const Vector3<Real>& rkAxis, Configuration& rkCfg);

		bool FindOverlap(Real fTMax, Real fSpeed, const Configuration& rkUC,
			const Configuration& rkVC, ContactSide& reSide, Configuration& rkTUC,
			Configuration& rkTVC, Real& rfTFirst, Real& rfTLast);

		bool FindOverlap(const Vector3<Real>& rkAxis, Real fTMax,
			const Vector3<Real>& rkVelocity, ContactSide& reSide,
			Configuration& rkTCfg0, Configuration& rkTCfg1, Real& rfTFirst,
			Real& rfTLast);

		void FindContactSet(const Triangle3<Real>& rkTri0,
			const Triangle3<Real>& rkTri1, ContactSide& reSide,
			Configuration& rkCfg0, Configuration& rkCfg1);

		void GetEdgeEdgeIntersection(const Vector3<Real>& rkU0,
			const Vector3<Real>& rkU1, const Vector3<Real>& rkV0,
			const Vector3<Real>& rkV1);

		void GetEdgeFaceIntersection(const Vector3<Real>& rkU0,
			const Vector3<Real>& rkU1, const Triangle3<Real>& rkTri);

		// the objects to intersect
		const Triangle3<Real>* m_pkTriangle0;
		const Triangle3<Real>* m_pkTriangle1;

		// information about the intersection set
		int m_iQuantity;
		Vector3<Real> m_akPoint[6];
	};



	typedef Intersector1<float> Intersector1f;
	typedef Intersector1<double> Intersector1d;
	typedef Intersector<float, Vector2<float> > Intersector2f;
	typedef Intersector<float, Vector3<float> > Intersector3f;
	typedef Intersector<double, Vector2<double> > Intersector2d;
	typedef Intersector<double, Vector3<double> > Intersector3d;
	typedef IntrLine2Triangle2<float> IntrLine2Triangle2f;
	typedef IntrLine2Triangle2<double> IntrLine2Triangle2d;
	typedef IntrSegment2Triangle2<float> IntrSegment2Triangle2f;
	typedef IntrSegment2Triangle2<double> IntrSegment2Triangle2d;
	typedef IntrTriangle2Triangle2<float> IntrTriangle2Triangle2f;
	typedef IntrTriangle2Triangle2<double> IntrTriangle2Triangle2d;
	typedef IntrTriangle3Triangle3<float> IntrTriangle3Triangle3f;
	typedef IntrTriangle3Triangle3<double> IntrTriangle3Triangle3d;
}

#endif
