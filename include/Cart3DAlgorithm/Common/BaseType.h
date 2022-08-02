#pragma once
#ifndef CART3D_ALGORITHM_BASETYPE_H
#define CART3D_ALGORITHM_BASETYPE_H

#include "util.h"
#include "CommonConfig.h"

namespace Cart3DAlgorithm
{
	template<class Vec>
	class Line
	{
	public:
		Line() :origin(Vec::Zero()),direction(Vec::Ones()) {}
		Line(const Vec& rkOrigin, const Vec& rkDirection) :origin(rkOrigin),direction(rkDirection){}
	public:
		Vec origin, direction;
	};

	template <class Vec,class Real>
	class Segment
	{
	public:
		Segment() :origin(Vec::Zero()), direction(Vec::Ones()),extent(0) {}
		Segment(const Vec& rkOrigin, const Vec& rkDirection,Real fExtent) :
			origin(rkOrigin),direction(rkDirection),extent(fExtent){}
	public:
		Vec GetPosEnd() const { return origin + extent * direction; }
		Vec GetNegEnd() const { return origin - extent * direction; }
	public:
		Vec origin, direction;
		Real extent;
	};

	template <class Vec,class Real>
	class Triangle
	{
	public:
		Triangle() :a(Vec::Zero()), b(Vec::Zero()), c(Vec::Zero()),e0(Vec::Zero()),e1(Vec::Zero()) {}
		Triangle(const Vec& rkV0, const Vec& rkV1, const Vec& rkV2) :
			a(rkV0), b(rkV1), c(rkV2),e0(rkV1- rkV0),e1(rkV2- rkV0) {}
		Triangle(const Vec akV[3]):a(akV[0]), b(akV[1]), c(akV[2]) {
			e0 = b - a;
			e1 = c - a;
		}
	public:
		union
		{
			struct { Vec v[3]; };
			struct { Vec a,b,c; };
		};
		Vec e0, e1;
	};
	class Plane
	{
	public:
		Plane():normal(cvector3d::Zero()), constant(0) {}
		Plane(const Plane& rkPlane) :normal(rkPlane.normal), constant(rkPlane.constant) {}
		Plane(const cvector3d& rkNormal, cfloat fConstant) :normal(rkNormal), constant(fConstant) {}
		Plane(const cvector3d& rkNormal, const cvector3d& rkP) :normal(rkNormal){constant = rkNormal.dot(rkP);}
		Plane(const cvector3d& rkP0, const cvector3d& rkP1,const cvector3d& rkP2) {
			cvector3d kEdge1 = rkP1 - rkP0;
			cvector3d kEdge2 = rkP2 - rkP0;
			normal = kEdge1.cross(kEdge2);
			normal.normalize();
			constant = normal.dot(rkP0);
		}
		Plane& operator= (const Plane& rkPlane)
		{
			if (this != &rkPlane) 
			{
				normal = rkPlane.normal;
				constant = rkPlane.constant;
			}
			return *this;
		}
	public:
		int WhichSide(const cvector3d& rkP) const {
			cfloat fDistance = DistanceTo(rkP);
			if (fDistance < -inv_trunc_val)
			{
				return -1;
			}
			if (fDistance > (cfloat)inv_trunc_val)
			{
				return +1;
			}
			return 0;
		}
		cfloat DistanceTo(const cvector3d& rkQ) const {return normal.dot(rkQ) - constant;}
	public:
		cvector3d normal;
		cfloat constant;
	};

	using Line3d = Line<cvector3d>;
	using Line2d = Line<cvector2d>;
	using Segment3d = Segment<cvector3d, cfloat>;
	using Segment2d = Segment<cvector2d, cfloat>;
	using Triangle3d = Triangle<cvector3d, cfloat>;
	using Triangle2d = Triangle<cvector2d, cfloat>;
	using Plane3d = Plane;

	COMMON_API cfloat __stdcall DistancePointToTriangle(const cvector2d& rkQ, const Triangle2d& tri);
	COMMON_API cfloat __stdcall DistancePointToTriangle(const cvector3d& rkQ, const Triangle3d& tri);
	COMMON_API cfloat __stdcall DistancePointToSegment(const cvector2d& p0, const cvector2d& segs, const cvector2d& sege);
	COMMON_API cfloat __stdcall DistancePointToSegment(const cvector3d& p0, const cvector3d& segs, const cvector3d& sege);

	
}




#endif