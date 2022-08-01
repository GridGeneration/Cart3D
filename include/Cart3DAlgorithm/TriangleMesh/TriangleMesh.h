#ifndef CART3D_ALGORITHM_TRIANGLEMESH_H
#define CART3D_ALGORITHM_TRIANGLEMESH_H
#include  <vector>
#include  <array>
#include "TriangleMeshConfig.h"
#include  <SearchAlgo/BBox.h>
#include  <string>

namespace Cart3DAlgorithm
{
	
	class TRIANGLEMESH_API TriangleMesh
	{
	public:
		TriangleMesh(std::vector<cvector3d>& pts,
			std::vector<std::array<int, 3>>& face,
			bool keepori);
		//支持读取stl数据
		TriangleMesh(const std::string& filename);
	public:
		cvector3d& get_point(int id);
		const cvector3d& get_point(int id)const;
		std::array<int, 3>& get_tri(int id);
		const std::array<int, 3>& get_tri(int id)const;
	public:
		BoundingBox get_tri_box(int id)const;
	private:
		std::vector<cvector3d> pts;
		std::vector<std::array<int, 3>> faces;
	};
}

#endif // !CART3D_ALGORITHM_TRIANGLEMESH_H

