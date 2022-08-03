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
		struct MESHDOCTOR_API BooleanPoint
		{
			cvector3d data;
			int jface = -1;
			cfloat& operator[](int i);
			const cfloat& operator[](int i)const;
			bool operator == (const BooleanPoint& c) const;
			bool operator != (const BooleanPoint& c) const;
			bool operator < (const BooleanPoint& c) const;
		};

		struct MESHDOCTOR_API BooleanEdge
		{
			BooleanPoint* pts = nullptr;
			BooleanPoint* pte = nullptr;
			bool del = false;
			int jface = -1;
		};
		using vv_inter = std::vector<std::vector<int>>;
		using vv_BPoint= std::vector<std::vector<BooleanPoint>>;
		using vv_BEdge= std::vector<std::vector<BooleanEdge>>;
	public:
		RemoveSelfIntersection(OpenTriMesh& in_mesh);
	public:
		bool run();

	private:
		bool find_int_pairs(vv_inter& maybe_inter);
		bool find_int_pairs(const vv_inter& maybe_inter, vv_BPoint& pts, vv_BEdge& edges);
	public:
		static bool IntTriTri(
			const cvector3d& v0, const cvector3d& v1, const cvector3d& v2,
			const cvector3d& u0, const cvector3d& u1, const cvector3d& u2,
			std::vector<cvector3d>& intps);
	private:
		OpenTriMesh& mesh;
		
	};
}



#endif