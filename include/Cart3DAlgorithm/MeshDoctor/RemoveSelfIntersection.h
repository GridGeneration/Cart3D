#pragma once
#ifndef CART3D_ALGORITHM_REMOVESELFINTERSECTION_MESHDOCTOR_H
#define CART3D_ALGORITHM_REMOVESELFINTERSECTION_MESHDOCTOR_H
#include "OpenMeshUtil.h"
#include "MeshDoctorConfig.h"
#include <Common/util.h>

/**************************************************************************
 *brief:  µœ÷paper 
     ember_exact_mesh_booleans_via_efficient_and_robust_local_arrangements
	 Fast Exact Booleans for Iterated CSG using Octree-Embedded BSPs
 *author: DWM
 *QQ: 1427336408
 *date: 2022.07.26
 *detil: ToDo...
 ***************************************************************************/

namespace Cart3DAlgorithm
{
	class MESHDOCTOR_API RemoveSelfIntersection
	{
	public:
		RemoveSelfIntersection(OpenTriMesh& in_mesh);
	public:

	private:
		static bool IntTriTri(
			const cvector3d& v0, const cvector3d& v1, const cvector3d& v2,
			const cvector3d& u0, const cvector3d& u1, const cvector3d& u2,
			std::vector<cvector3d>& intps);

	private:
		OpenTriMesh& mesh;
		
	};
}



#endif