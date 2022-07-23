#ifndef CART3D_ALGORITHM_FASTHOLEFILLER_H
#define CART3D_ALGORITHM_FASTHOLEFILLER_H
#include <MeshDoctor/OpenMeshUtil.h>
#include <Common/util.h>

/// <summary>
/// ¸î¶ú·¨²¹¶´
/// </summary>
namespace Cart3DAlgorithm
{
	
	class FastHoleFiller
	{
	public:
		bool fix_hole(OpenTriMesh& mesh,const std::vector<VertexHandle>& hole);

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
