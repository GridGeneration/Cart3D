#ifndef CART3D_ALGORITHM_FASTHOLEFILLER_H
#define CART3D_ALGORITHM_FASTHOLEFILLER_H
#include <MeshDoctor/OpenMeshUtil.h>
#include <Common/util.h>
#include <MeshDoctor/MeshDoctorConfig.h>

/// <summary>
/// 最小面积法补洞
/// </summary>
namespace Cart3DAlgorithm
{
	class MESHDOCTOR_API FastHoleFiller
	{
	public:
		static bool fix_hole(OpenTriMesh& mesh,const HalfedgeHandle&halfedge);
		static bool fix_hole(OpenTriMesh& mesh,const HalfedgeHandle&halfedge, std::vector<FaceHandle>& new_faces);
		static bool fix_hole(OpenTriMesh& mesh, const std::vector<VertexHandle>& hole);
		static bool fix_hole(OpenTriMesh& mesh,const std::vector<VertexHandle>& hole,std::vector<FaceHandle>& new_faces);
	private:
		static OpenTriMesh::Normal compute_vertex_norm(const OpenTriMesh& mesh, const VertexHandle& vh);
	};

}


#endif
