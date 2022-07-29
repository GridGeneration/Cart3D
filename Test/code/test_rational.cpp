#include <iostream>
#include <rationals.h>
#include <MeshDoctor/Rational.h>
#include <LoadOpenMesh.h>
#include <LoadCart3DAlgorithm.h>
int main(int argc, char* argv[])
{
	using namespace Cart3DAlgorithm;
	using frac64_t = rationals::rational<std::int64_t>;
	frac64_t a(1212, 23256647);
	frac64_t b(151552, 26598447);
	clock_t st = clock();
	frac64_t c;
	for(int i=0;i<102456458;++i)
		c = Rational::fromcfloat(-3.1415926537942);
	std::cout << "Time:" << clock() - st << "ms" << std::endl;
	std::cout << c << std::endl;
	std::cout << (a < b) << std::endl;
	return 0;
}