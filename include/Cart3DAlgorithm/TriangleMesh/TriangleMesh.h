#ifndef CART3D_ALGORITHM_TRIANGLEMESH_H
#define CART3D_ALGORITHM_TRIANGLEMESH_H
#include  <vector>
#include  <array>
#include  "TriangleMeshConfig.h"
#include  <Common/BBox.h>
#include  <string>

namespace Cart3DAlgorithm
{
	
	class TRIANGLEMESH_API TriangleMesh
	{
	public:
		TriangleMesh(std::vector<cvector3d>& pts,
			std::vector<std::array<int, 3>>& face,
			bool keepori);
		//֧�ֶ�ȡstl����
		TriangleMesh(const std::string& filename);
	public:
		cvector3d& get_point(int id);
		const cvector3d& get_point(int id)const;
		std::array<int, 3>& get_tri(int id);
		const std::array<int, 3>& get_tri(int id)const;
	public:
		int n_vertices()const;
		int n_faces()const;
	public:
		BoundingBox get_tri_box(int id)const;
	private:
		std::vector<cvector3d> pts;
		std::vector<std::array<int, 3>> faces;
	};
}

#endif // !CART3D_ALGORITHM_TRIANGLEMESH_H

