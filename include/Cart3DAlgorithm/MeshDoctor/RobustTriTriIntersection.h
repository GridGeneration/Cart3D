#ifndef CART3D_ALGORITHM_ROUST_TRIANGLE_TRIANGLE_INTERSECTION_H
#define CART3D_ALGORITHM_ROUST_TRIANGLE_TRIANGLE_INTERSECTION_H

#include <MeshDoctor/BaseType.h>
#include <Common/util.h>
namespace Cart3DAlgorithm
{

	class MESHDOCTOR_API Intersector
	{
	public:
		virtual ~Intersector();
		virtual bool Test();
		virtual bool Find();
		virtual bool Test(cfloat fTMax, const cvector3d& rkVelocity0,
			const cvector3d& rkVelocity1);
		virtual bool Find(cfloat fTMax, const cvector3d& rkVelocity0,
			const cvector3d& rkVelocity1);
		cfloat GetContactTime() const;
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

		cfloat m_fContactTime;
		int m_iIntersectionType;
	};

	class MESHDOCTOR_API Intersector1
	{
	public:
		Intersector1(cfloat fU0, cfloat fU1, cfloat fV0, cfloat fV1);
		Intersector1(cfloat afU[2], cfloat afV[2]);
		virtual ~Intersector1();

		// object access
		cfloat GetU(int i) const;
		cfloat GetV(int i) const;

		// static intersection queries
		virtual bool Test();
		virtual bool Find();

		// Dynamic intersection queries.  The Find query produces a set of first
		// contact.
		virtual bool Test(cfloat fTMax, cfloat fSpeedU, cfloat fSpeedV);
		virtual bool Find(cfloat fTMax, cfloat fSpeedU, cfloat fSpeedV);

		// The time at which two intervals are in first/last contact for the
		// dynamic intersection queries.
		cfloat GetFirstTime() const;
		cfloat GetLastTime() const;

		// information about the intersection set
		int GetQuantity() const;
		cfloat GetOverlap(int i) const;

	protected:
		// the intervals to intersect
		cfloat m_afU[2], m_afV[2];

		// information about the intersection set
		cfloat m_fFirstTime, m_fLastTime;
		int m_iQuantity;
		cfloat m_afOverlap[2];
	};

	class MESHDOCTOR_API IntrLine2Triangle2 : public Intersector
	{
	public:
		IntrLine2Triangle2(const Line2d& rkLine,
			const Triangle2d& rkTriangle);

		// Object access.
		const Line2d& GetLine() const;
		const Triangle2d& GetTriangle() const;

		// Static intersection query.
		virtual bool Test();
		virtual bool Find();

		int GetQuantity() const;
		const cvector2d& GetPoint(int i) const;

	private:
		using Intersector::IT_EMPTY;
		using Intersector::IT_POINT;
		using Intersector::IT_SEGMENT;
		using Intersector::m_iIntersectionType;

		// The objects to intersect.
		const Line2d* m_pkLine;
		const Triangle2d* m_pkTriangle;

		// Information about the intersection set.
		int m_iQuantity;
		cvector2d m_akPoint[2];

		// Internal use, shared by IntrRay2Triangle2 and IntrSegment2Triangle2.
	public:
		// Determine how the triangle and line intersect (if at all).
		static void TriangleLineRelations(const cvector2d& rkOrigin,
			const cvector2d& rkDirection, const Triangle2d& rkTriangle,
			cfloat afDist[3], int aiSign[3], int& riPositive, int& riNegative,
			int& riZero);

		static void GetInterval(const cvector2d& rkOrigin,
			const cvector2d& rkDirection, const Triangle2d& rkTriangle,
			const cfloat afDist[3], const int aiSign[3], cfloat afParam[2]);
	};


	class MESHDOCTOR_API  IntrSegment2Triangle2 : public Intersector
	{
	public:
		IntrSegment2Triangle2(const Segment2d& rkSegment,
			const Triangle2d& rkTriangle);

		// Object access.
		const Segment2d& GetSegment() const;
		const Triangle2d& GetTriangle() const;

		// Static intersection query.
		virtual bool Test();
		virtual bool Find();

		int GetQuantity() const;
		const cvector2d& GetPoint(int i) const;

	private:
		using Intersector::IT_EMPTY;
		using Intersector::IT_POINT;
		using Intersector::IT_SEGMENT;
		using Intersector::m_iIntersectionType;

		// The objects to intersect.
		const Segment2d* m_pkSegment;
		const Triangle2d* m_pkTriangle;

		// Information about the intersection set.
		int m_iQuantity;
		cvector2d m_akPoint[2];
	};


	class MESHDOCTOR_API IntrTriangle2Triangle2 : public Intersector
	{
	public:
		IntrTriangle2Triangle2(const Triangle2d& rkTriangle0,
			const Triangle2d& rkTriangle1);

		// object access
		const Triangle2d& GetTriangle0() const;
		const Triangle2d& GetTriangle1() const;

		// static queries
		virtual bool Test();
		virtual bool Find();

		// dynamic queries
		virtual bool Test(cfloat fTMax, const cvector2d& rkVelocity0,
			const cvector2d& rkVelocity1);
		virtual bool Find(cfloat fTMax, const cvector2d& rkVelocity0,
			const cvector2d& rkVelocity1);

		// information about the intersection set
		int GetQuantity() const;
		const cvector2d& GetPoint(int i) const;

	private:
		using Intersector::m_fContactTime;

		static int WhichSide(const cvector2d akV[3],
			const cvector2d& rkP, const cvector2d& rkD);

		static void ClipConvexPolygonAgainstLine(const cvector2d& rkN,
			cfloat fC, int& riQuantity, cvector2d akV[6]);

		enum ProjectionMap
		{
			M21,  // 2 vertices map to min, 1 vertex maps to max
			M12,  // 1 vertex maps to min, 2 vertices map to max
			M11   // 1 vertex maps to min, 1 vertex maps to max
		};

		class MESHDOCTOR_API Configuration
		{
		public:
			ProjectionMap Map;  // how vertices map to the projection interval
			int Index[3];       // the sorted indices of the vertices
			cfloat Min, Max;      // the interval is [min,max]
		};

		void ComputeTwo(Configuration& rkCfg, const cvector2d akV[3],
			const cvector2d& rkD, int iI0, int iI1, int iI2);

		void ComputeThree(Configuration& rkCfg, const cvector2d akV[3],
			const cvector2d& rkD, const cvector2d& rkP);

		static bool NoIntersect(const Configuration& rkCfg0,
			const Configuration& rkCfg1, cfloat fTMax, cfloat fSpeed, int& riSide,
			Configuration& rkTCfg0, Configuration& rkTCfg1, cfloat& rfTFirst,
			cfloat& rfTLast);

		static void GetIntersection(const Configuration& rkCfg0,
			const Configuration& rkCfg1, int iSide, const cvector2d akV0[3],
			const cvector2d akV1[3], int& riQuantity,
			cvector2d akVertex[6]);
		const Triangle2d* m_pkTriangle0;
		const Triangle2d* m_pkTriangle1;

		int m_iQuantity;
		cvector2d m_akPoint[6];
	};



	class  MESHDOCTOR_API IntrTriangle3Triangle3 : public Intersector
	{
	public:
		IntrTriangle3Triangle3(const Triangle3d& rkTriangle0,
			const Triangle3d& rkTriangle1);

		// object access
		const Triangle3d& GetTriangle0() const;
		const Triangle3d& GetTriangle1() const;

		bool ReportCoplanarIntersections;  // default 'true'

		// static queries
		virtual bool Test();
		virtual bool Find();

		// dynamic queries
		virtual bool Test(cfloat fTMax, const cvector3d& rkVelocity0,
			const cvector3d& rkVelocity1);
		virtual bool Find(cfloat fTMax, const cvector3d& rkVelocity0,
			const cvector3d& rkVelocity1);

		// information about the intersection set
		int GetQuantity() const;
		const cvector3d& GetPoint(int i) const;

	private:
		using Intersector::IT_POINT;
		using Intersector::IT_SEGMENT;
		using Intersector::m_iIntersectionType;
		using Intersector::m_fContactTime;

		static void ProjectOntoAxis(const Triangle3d& rkTri,
			const cvector3d& rkAxis, cfloat& rfMin, cfloat& rfMax);

		static void TrianglePlaneRelations(const Triangle3d& rkTriangle,
			const Plane3d& rkPlane, cfloat afDistance[3], int aiSign[3],
			int& riPositive, int& riNegative, int& riZero);

		static void GetInterval(const Triangle3d& rkTriangle,
			const Line3d& rkLine, const cfloat afDistance[3],
			const int aiSign[3], cfloat afParam[2]);

		bool ContainsPoint(const Triangle3d& rkTriangle,
			const Plane3d& rkPlane, const cvector3d& rkPoint);

		bool IntersectsSegment(const Plane3d& rkPlane,
			const Triangle3d& rkTriangle, const cvector3d& rkEnd0,
			const cvector3d& rkEnd1);

		bool GetCoplanarIntersection(const Plane3d& rkPlane,
			const Triangle3d& rkTri0, const Triangle3d& rkTri1);

		static bool TestOverlap(cfloat fTMax, cfloat fSpeed, cfloat fUMin,
			cfloat fUMax, cfloat fVMin, cfloat fVMax, cfloat& rfTFirst, cfloat& rfTLast);

		bool TestOverlap(const cvector3d& rkAxis, cfloat fTMax,
			const cvector3d& rkVelocity, cfloat& rfTFirst, cfloat& rfTLast);

		bool PointIsInSegment(const cvector3d& pt, const cvector3d& segs, const cvector3d& sege);

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

		class MESHDOCTOR_API  Configuration
		{
		public:
			ProjectionMap Map;  // how vertices map to the projection interval
			int Index[8];       // the sorted indices of the vertices
			cfloat Min, Max;      // the interval is [min,max]
		};

		static void ProjectOntoAxis(const Triangle3d& rkTri,
			const cvector3d& rkAxis, Configuration& rkCfg);

		bool FindOverlap(cfloat fTMax, cfloat fSpeed, const Configuration& rkUC,
			const Configuration& rkVC, ContactSide& reSide, Configuration& rkTUC,
			Configuration& rkTVC, cfloat& rfTFirst, cfloat& rfTLast);

		bool FindOverlap(const cvector3d& rkAxis, cfloat fTMax,
			const cvector3d& rkVelocity, ContactSide& reSide,
			Configuration& rkTCfg0, Configuration& rkTCfg1, cfloat& rfTFirst,
			cfloat& rfTLast);

		void FindContactSet(const Triangle3d& rkTri0,
			const Triangle3d& rkTri1, ContactSide& reSide,
			Configuration& rkCfg0, Configuration& rkCfg1);

		void GetEdgeEdgeIntersection(const cvector3d& rkU0,
			const cvector3d& rkU1, const cvector3d& rkV0,
			const cvector3d& rkV1);

		void GetEdgeFaceIntersection(const cvector3d& rkU0,
			const cvector3d& rkU1, const Triangle3d& rkTri);

		// the objects to intersect
		const Triangle3d* m_pkTriangle0;
		const Triangle3d* m_pkTriangle1;

		// information about the intersection set
		int m_iQuantity;
		cvector3d m_akPoint[6];
	};


}

#endif
