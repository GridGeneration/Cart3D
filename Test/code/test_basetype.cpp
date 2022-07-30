#include <MeshDoctor/BaseType.h>
#include <LoadCart3DAlgorithm.h>
#include <LoadOpenMesh.h>
#include <iostream>
int main(int argc, char* argv[])
{
	using namespace Cart3DAlgorithm;



	Triangle3d tri(cvector3d(1, 1, 1), cvector3d(1, 2, 3), cvector3d(3, 2, 1));
	
	cfloat dist=DistancePointToTriangle(cvector3d(0, 0.3, 5),tri);

	std::cout << "DistancePointToTriangle:" << dist << std::endl;


	return 0;
}