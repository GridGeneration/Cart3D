#ifndef WM4BASRTYPE_H
#define WM4BASRTYPE_H
#include "Wm4Math.h"
namespace Sn3DAlgorithm
{
	template <class Real>
	class Line2
	{
	public:
		Line2(); 
		Line2(const Vector2<Real>& rkOrigin, const Vector2<Real>& rkDirection);

		Vector2<Real> Origin, Direction;
	};

	template <class Real>
	class Segment2
	{
	public:
		Segment2();  // uninitialized
		Segment2(const Vector2<Real>& rkOrigin, const Vector2<Real>& rkDirection,
			Real fExtent);

		// end points
		Vector2<Real> GetPosEnd() const;  // P+e*D
		Vector2<Real> GetNegEnd() const;  // P-e*D

		Vector2<Real> Origin, Direction;
		Real Extent;
	};

	template <class Real>
	class Triangle2
	{
	public:
		Triangle2();  // uninitialized
		Triangle2(const Vector2<Real>& rkV0, const Vector2<Real>& rkV1,
			const Vector2<Real>& rkV2);
		Triangle2(const Vector2<Real> akV[3]);

		// distance from the point Q to the triangle
		Real DistanceTo(const Vector2<Real>& rkQ) const;

		Vector2<Real> V[3];
	};

	template <class Real>
	class Line3
	{
	public:
		Line3();
		Line3(const Vector3<Real>& rkOrigin, const Vector3<Real>& rkDirection);
		Vector3<Real> Origin, Direction;
	};

	template <class Real>
	class Plane3
	{
	public:
		Plane3();  // uninitialized
		Plane3(const Plane3& rkPlane);

		// specify N and c directly
		Plane3(const Vector3<Real>& rkNormal, Real fConstant);

		// N is specified, c = Dot(N,P) where P is on the plane
		Plane3(const Vector3<Real>& rkNormal, const Vector3<Real>& rkP);
		Plane3(const Vector3<Real>& rkP0, const Vector3<Real>& rkP1,
			const Vector3<Real>& rkP2);
		Plane3& operator= (const Plane3& rkPlane);
		int WhichSide(const Vector3<Real>& rkP) const;
		Real DistanceTo(const Vector3<Real>& rkQ) const;

		Vector3<Real> Normal;
		Real Constant;
	};

	template <class Real>
	class Triangle3
	{
	public:
		// The triangle is represented as an array of three vertices, V0, V1,
		// and V2.

		// construction
		Triangle3();  // uninitialized
		Triangle3(const Vector3<Real>& rkV0, const Vector3<Real>& rkV1,
			const Vector3<Real>& rkV2);
		Triangle3(const Vector3<Real> akV[3]);

		// distance from the point Q to the triangle
		Real DistanceTo(const Vector3<Real>& rkQ) const;

		static Real DistancePointToSegment3(
			const Vector3<Real>& p0,
			const Vector3<Real>& segs,
			const Vector3<Real>& sege);



		Vector3<Real> V[3];
	};
	//----------------------------------------------------------------------------
	typedef Line2<float> Line2f;
	typedef Line2<double> Line2d;
	//----------------------------------------------------------------------------
	typedef Segment2<float> Segment2f;
	typedef Segment2<double> Segment2d;
	//----------------------------------------------------------------------------
	typedef Triangle2<float> Triangle2f;
	typedef Triangle2<double> Triangle2d;
	//----------------------------------------------------------------------------
	typedef Line3<float> Line3f;
	typedef Line3<double> Line3d;
	//----------------------------------------------------------------------------
	typedef Plane3<float> Plane3f;
	typedef Plane3<double> Plane3d;
	//----------------------------------------------------------------------------
	typedef Triangle3<float> Triangle3f;
	typedef Triangle3<double> Triangle3d;
	//----------------------------------------------------------------------------
}


#endif