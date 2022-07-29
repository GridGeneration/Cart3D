#pragma once
#ifndef CART3D_ALGORITHM_ROUBUST_TRITRIANGLE_INTERSECTION_H
#define CART3D_ALGORITHM_ROUBUST_TRITRIANGLE_INTERSECTION_H

#include <Common/util.h>
#include <MeshDoctor/MeshDoctorConfig.h>

namespace Cart3DAlgorithm
{
	class MESHDOCTOR_API RobustTriTriIntersection
	{
	public:
		template<class Vec>
		struct Line
		{
			Vec Origin, Direction;
		};
		template<class Vec>
		struct Segment
		{
			Vec Origin, Direction;
			cfloat Extent;
		};
		struct Plane
		{
			cvector3d Normal;
			cfloat Constant;
		};
		template<class Vec>
		struct Triangle
		{
			union {
				struct { Vec a, b, c; };
				struct { Vec abc[3]; };
			};
		};


	public:
		static int orient3d(
			const cvector3d& a, const cvector3d& b, 
			const cvector3d& c, const cvector3d& d);
		static int orient2d(
			const cvector2d& a,
			const cvector2d& b,
			const cvector2d& c);
	};
}


#endif