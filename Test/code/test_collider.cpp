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
	OpenMesh::IO::read_mesh(mesh0, "H:/E_Data/CF-18 v2.stl");
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
	
	std::cout << "TimeCreaOBBTree:" << clock() - st << "ms" << std::endl;



	return 0;
}

