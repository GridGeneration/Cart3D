#include <SearchAlgo/Collider.h>
#include <SearchAlgo/Bvh.h>
#include <MeshDoctor/OpenMeshUtil.h>
#include <LoadOpenMesh.h>
#include <LoadCart3DAlgorithm.h>
#include <tbb/parallel_for.h>
#include <LoadTbb.h>
#include <TriangleMesh/TriangleMesh.h>
#include <MeshDoctor/RobustTriTriIntersection.h>
using namespace Cart3DAlgorithm;
int main(int argc, char* argv[])
{
	clock_t st = clock();
	TriangleMesh tmesh("h:/E_Data/Flight stl format.stl");
	std::cout << "LoadStlTime:" << clock() - st<<"ms" << std::endl;
	std::cout << "MeshInfoFaces:" << tmesh.n_faces() << std::endl;
	std::cout << "MeshInfoVertices:" << tmesh.n_vertices() << std::endl;
	int nface = tmesh.n_faces();
	std::vector<std::vector<int>> obb(nface);
	std::vector<std::vector<int>> abb(nface);
	clock_t stbvh = 0, stobb = 0;
	st = clock();
	std::vector<GeomBlob> gnode(nface);
	for (int i = 0; i < nface; ++i)
	{
		gnode[i].id = i;
		gnode[i].bbox = tmesh.get_tri_box(i);
		gnode[i].point = gnode[i].bbox.centroid();
	}
	Cart3DBvh bvh(gnode);
	std::cout << "CreatBVHTime:" << (stbvh+=clock() - st) << "ms" << std::endl;

	st = clock();
	for (int i = 0; i < nface; ++i)
	{
		bvh.for_each_in_box(gnode[i].bbox, [&](int id) {
			abb[i].push_back(id);
			});
	}
	std::cout << "QueryBvhTime:" << (stbvh += clock() - st) << "ms" << std::endl;

	st = clock();
	std::vector<std::pair<int, int>> bpairs;
	for (int i = 0; i < nface; ++i)
	{
		auto& ff = tmesh.get_tri(i);
		std::vector<cvector3d> pts;
		pts.push_back(tmesh.get_point(ff[0]));
		pts.push_back(tmesh.get_point(ff[1]));
		pts.push_back(tmesh.get_point(ff[2]));
		Triangle3d t0(pts[0], pts[1], pts[2]);
		for (auto j : abb[i])
		{
			auto& ff = tmesh.get_tri(j);
			std::vector<cvector3d> pts;
			pts.push_back(tmesh.get_point(ff[0]));
			pts.push_back(tmesh.get_point(ff[1]));
			pts.push_back(tmesh.get_point(ff[2]));
			Triangle3d t1(pts[0], pts[1], pts[2]);
			IntrTriangle3Triangle3 in(t0, t1);
			if (in.Find())
			{
				bpairs.push_back(std::make_pair(i, j));
			}
		}
	}
	std::cout << "QueryBvh_TriangleTime:" << (stbvh += clock() - st) << "ms" << std::endl;


	st = clock();
	Collider collider;
	collider.start_build_model(0);
	for (int i = 0; i < nface; ++i)
	{
		auto& ff = tmesh.get_tri(i);
		collider.add_tris(0, i, tmesh.get_point(ff[0]), tmesh.get_point(ff[1]), tmesh.get_point(ff[2]));
	}
	collider.end_build_model(0);
	std::cout << "CreateColliderTime:" << (stobb+=clock() - st) << "ms" << std::endl;


	st = clock();
	for (int i = 0; i < nface; ++i)
	{
		auto& ff = tmesh.get_tri(i);
		std::vector<cvector3d> pts;
		pts.push_back(tmesh.get_point(ff[0]));
		pts.push_back(tmesh.get_point(ff[1]));
		pts.push_back(tmesh.get_point(ff[2]));
		auto bv = Collider::creat_bv_node(cmatrix3d::Identity(), pts);
		collider.query_bv_intersecion(bv, cmatrix4d::Identity(), 0, cmatrix4d::Identity(), obb[i]);
	}
	std::cout << "QueryColliderTime:" << (stobb += clock() - st) <<"ms" << std::endl;

	st = clock();
	std::vector<std::pair<int, int>> opairs;
	for (int i = 0; i < nface; ++i)
	{
		auto& ff = tmesh.get_tri(i);
		std::vector<cvector3d> pts;
		pts.push_back(tmesh.get_point(ff[0]));
		pts.push_back(tmesh.get_point(ff[1]));
		pts.push_back(tmesh.get_point(ff[2]));
		Triangle3d t0(pts[0], pts[1], pts[2]);
		for (auto j : obb[i])
		{
			auto& ff = tmesh.get_tri(j);
			std::vector<cvector3d> pts;
			pts.push_back(tmesh.get_point(ff[0]));
			pts.push_back(tmesh.get_point(ff[1]));
			pts.push_back(tmesh.get_point(ff[2]));
			Triangle3d t1(pts[0], pts[1], pts[2]);
			IntrTriangle3Triangle3 in(t0, t1);
			if (in.Find())
			{
				opairs.push_back(std::make_pair(i, j));
			}
		}
	}
	std::cout << "QueryOBB_TriangleTime:" << (stobb += clock() - st) << "ms" << std::endl;
	std::cout << "TotalBvh:" << stbvh << "ms" << std::endl;
	std::cout << "TotalObb:" << stobb << "ms" << std::endl;
	std::cout << "BVH-Res:" << bpairs.size() << std::endl;
	std::cout << "OBB-Res:" << opairs.size() << std::endl;

	return 0;
}

