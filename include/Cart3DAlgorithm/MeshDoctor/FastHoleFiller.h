#ifndef CART3D_ALGORITHM_FASTHOLEFILLER_H
#define CART3D_ALGORITHM_FASTHOLEFILLER_H
#include <MeshDoctor/OpenMeshUtil.h>
#include <Common/util.h>
#include <MeshDoctor/MeshDoctorConfig.h>

/// <summary>
/// ¸î¶ú·¨²¹¶´
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
		static OpenTriMesh::Scalar compute_weight(
			const OpenTriMesh& mesh,
			const OpenTriMesh::Normal& n1,
			const OpenTriMesh::Normal& n2,
			const OpenTriMesh::Normal& n3,
			const VertexHandle& _v1,
			const VertexHandle& _v2,
			const VertexHandle& _v3,
			OpenTriMesh::Scalar meanedgelen,
			bool hack);
		static OpenTriMesh::Normal compute_vertex_norm(const OpenTriMesh& mesh, const VertexHandle& vh);
	};

}


#endif
