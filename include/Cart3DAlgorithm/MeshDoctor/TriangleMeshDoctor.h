#ifndef CART3D_ALGORITHM_TRIANGLEMESH_MESHDOCTOR_H
#define CART3D_ALGORITHM_TRIANGLEMESH_MESHDOCTOR_H

#include "MeshDoctorConfig.h"
#include "OpenMeshUtil.h"

namespace Cart3DAlgorithm
{
	struct MESHDOCTOR_API MeshDoctorParam
	{
		union
		{
			struct
			{
				bool is_fix_selfintersection : 1;
				bool is_join_points_eps : 1;
				bool is_join_edges_eps : 1;
				bool is_fill_hole : 1;
				bool is_refine_hole : 1;
				bool is_fair_hole : 1;
			};
			struct {
				std::uint64_t param_mask;
			};
		};
		cfloat join_points_eps;
		cfloat join_edge_eps;
		int fill_max_hole;

		MeshDoctorParam();

	};


	class MESHDOCTOR_API TriangleMeshDoctor
	{
	public:
		//�Զ��޸��㷨
		static bool auto_fix_mesh(const OpenTriMesh& in_mesh, OpenTriMesh& out_mesh);
	public:
		//��ͨ�����
		static int mark_part(const OpenTriMesh& in_mesh, std::vector<int>& parts);
		//��Ͻ��ڵ�
		static bool join_point(OpenTriMesh& in_mesh,cfloat eps);
		//ȥ��С���
		static bool delete_small_part(OpenTriMesh& in_mesh, int max_nvert);
		//���С��
		static bool fill_small_hole(OpenTriMesh& in_mesh, int max_hole);
	private:
		
	};
}


#endif
