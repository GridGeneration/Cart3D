#include <Cart3DAlgorithm/MeshDoctor/TriangleMeshDoctor.h>

using namespace Cart3DAlgorithm;



int main(int argc, char* argv[])
{
	OpenTriMesh otm;

	OpenMesh::IO::read_mesh(otm, "f:/UpperArch.stl");
	clock_t st = clock();
	std::vector<int> parts;
	int num_part=TriangleMeshDoctor::mark_part(otm, parts);
	std::cout << "TimeCock:" << clock() - st << "ms" << std::endl;
	
	otm.request_vertex_colors();
	for (auto iv : otm.vertices())
	{
		OpenTriMesh::Color col;
		ecode_rgb(parts[iv.idx()], col);
		otm.set_color(iv, col);
	}

	OpenMesh::IO::write_mesh(otm, "f:/f16_col.ply", OpenMesh::IO::Options::VertexColor);


	return 0;
}
