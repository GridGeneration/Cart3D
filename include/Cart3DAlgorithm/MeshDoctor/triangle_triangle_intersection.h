#ifndef CART3D_ALGORITHM_TRITRI_INTERSECTION_H
#define CART3D_ALGORITHM_TRITRI_INTERSECTION_H

#include "MeshDoctorConfig.h"
#include "OpenMeshUtil.h"
#include <Cart3DAlgorithm/Common/util.h>

namespace Cart3DAlgorithm
{
	enum TriangleIntType {
		TR_TYPE_NONE = 0,
		TR_TYPE_TOUCH = 1,
		TR_TYPE_INT = 2,
		TR_TYPE_OTHER = 3,
	};

	enum TriangleInt {
		TR_INT_NONE = 0, // no intersection.
		TR_INT_INT = 1,	 // intersection.
		TR_INT_VERT = 2, // intersection due to shared vertex.
		TR_INT_EDGE = 3, // intersection due to shared edge.
		TR_INT_TRI = 4,	 // intersection due to identical triangle.
		TR_OTHER_TRI = 5	 
	};

	class MESHDOCTOR_API TriTriIntersectionTools
	{
	public:
		static TriangleInt triangle_intersection(
			const cvector2d tri_a[3],
			const cvector2d tri_b[3]);
		static TriangleInt triangle_intersection(
			const cvector3d tri_a[3],
			const cvector3d tri_b[3]);
		static bool triangle_intersection_simple(
			const cvector2d tri_a[3],
			const cvector2d tri_b[3]);
		static bool triangle_intersection_simple(
			const cvector3d tri_a[3],
			const cvector3d tri_b[3]);
		static TriangleIntType triangle_intersection_exact(
			const cvector2d tri_a[3],
			const cvector2d tri_b[3]);
		static TriangleIntType triangle_intersection_exact(
			const cvector3d tri_a[3],
			const cvector3d tri_b[3]);
		static TriangleIntType triangle_linesegment_intersection_exact(
			const cvector2d tri_a[3], 
			const cvector2d line_b[2]);
		static TriangleIntType triangle_point_intersection_exact(
			const cvector2d tri_a[3],
			const cvector2d& pt_b);
	public:
		static int triangle_intersection_exact(
			const cvector3d& a1, const cvector3d& a2, const cvector3d& a3,
			const cvector3d& b1, const cvector3d& b2, const cvector3d& b3,
			std::vector<cvector3d>&lines);
	};

}

#endif