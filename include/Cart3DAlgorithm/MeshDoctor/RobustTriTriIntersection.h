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

		static bool TestTriTri(
			const cvector3d& a0, const cvector3d& a1, const cvector3d& a2,
			const cvector3d& b0, const cvector3d& b1, const cvector3d& b2);
		
		static bool FindTriTri(
			const cvector3d& a0, const cvector3d& a1, const cvector3d& a2,
			const cvector3d& b0, const cvector3d& b1, const cvector3d& b2,
			std::vector<cvector3d>& int_pts);

		static bool TestTriTri(
			const cvector2d& a0, const cvector2d& a1, const cvector2d& a2,
			const cvector2d& b0, const cvector2d& b1, const cvector2d& b2);

		static bool FindTriTri(
			const cvector2d& a0, const cvector2d& a1, const cvector2d& a2,
			const cvector2d& b0, const cvector2d& b1, const cvector2d& b2,
			std::vector<cvector2d>& int_pts);


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