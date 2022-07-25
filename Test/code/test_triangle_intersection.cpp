#include <iostream>
#include <fstream>
#include <vector>
#include <Cart3DAlgorithm/Common/util.h>
#include <Cart3DAlgorithm/MeshDoctor/triangle_triangle_intersection.h>
int main(int argc, char* argv[])
{
	using namespace Cart3DAlgorithm;
	cvector3d tri0[3] =
	{
		{0,0,0},
		{0.56666,0.66664,0},
		{0.2123,0.1,0},
	};

	cvector3d tri1[3] =
	{
		{0,0,0},
		{0.2,0.7,0},
		{0.3,0.1,0},
	};

	
	auto res=TriTriIntersectionTools::triangle_intersection_exact(tri0, tri1);
	std::cout << res << std::endl;
	std::vector<cvector3d> lines;
	int num_int = TriTriIntersectionTools::triangle_intersection_exact(
		tri0[0], tri0[1], tri0[2], tri1[0], tri1[1], tri1[2], lines);

	std::cout << "num_int:" << num_int << std::endl;
	return 0;
}