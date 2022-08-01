#include <SearchAlgo/Collider.h>
#include <MeshDoctor/OpenMeshUtil.h>
#include <LoadOpenMesh.h>
#include <LoadCart3DAlgorithm.h>
#include <ppl.h>

using namespace Cart3DAlgorithm;
int main(int argc, char* argv[])
{
	Collider collider;
	OpenTriMesh mesh0, mesh1, mesh2;
	OpenMesh::IO::read_mesh(mesh0, "H:/E_Data/FLYWHEEL.stl");
	OpenMesh::IO::read_mesh(mesh1, "H:/E_Data/Flight stl format.stl");
	OpenMesh::IO::read_mesh(mesh2, "H:/E_Data/raspberry Pi vesa mount.stl");
	clock_t st = clock();
	collider.start_build_model(0, mesh0.n_faces());
	collider.start_build_model(1, mesh1.n_faces());
	collider.start_build_model(2, mesh2.n_faces());

	concurrency::parallel_for(0, 3, [&](int i) {
		OpenTriMesh* opm = nullptr;
		switch (i)
		{
		case 0:
			opm = &mesh0;
			break;
		case 1:
			opm = &mesh1;
			break;
		case 2:
			opm = &mesh2;
			break;
		}

		for (auto f : opm->faces())
		{
			auto fv = opm->fv_begin(f);
			auto& p0 = opm->point(*fv); ++fv;
			auto& p1 = opm->point(*fv); ++fv;
			auto& p2 = opm->point(*fv);
			collider.add_tris(i, f.idx(), p0, p1, p2);
		}
		collider.end_build_model(i);
	});
	
	std::vector<OpenTriMesh::Point> pts;
	int id = 6966;
	auto fv = mesh0.fv_begin(FaceHandle(id));
	auto& p0 = mesh0.point(*fv); ++fv;
	auto& p1 = mesh0.point(*fv); ++fv;
	auto& p2 = mesh0.point(*fv);
	pts.push_back(p0);
	pts.push_back(p1);
	pts.push_back(p2);
	std::shared_ptr<Collider::BV_box> pnode = 
		Collider::creat_bv_node(cmatrix3d::Identity(), pts);
	st = clock();
	std::vector<int> tris;
	collider.query_bv_intersecion(pnode, cmatrix4d::Identity(), 0, cmatrix4d::Identity(), tris);
	std::cout << "Time:" << clock() - st << "ms" << std::endl;
	mesh0.request_face_colors();
	for (auto f : mesh0.faces())
	{
		mesh0.set_color(f, { 0,255,0 });
	}
	for (auto f : tris)
	{
		mesh0.set_color(FaceHandle(f), {255,0,0});
	}

	mesh0.set_color(FaceHandle(id), { 0,0,255 });

	OpenMesh::IO::write_mesh(mesh0, "H:/E_Data/mesh0.ply", OpenMesh::IO::Options::FaceColor);

	return 0;
}

