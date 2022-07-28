#include <iostream>
#include <rationals.h>
int main(int argc, char* argv[])
{
	using frac64_t = rationals::rational<std::int64_t>;
	frac64_t a(1212, 23256647);
	frac64_t b(151552, 26598447);
	frac64_t c = a + b;
	std::cout << c << std::endl;
	std::cout << (a < b) << std::endl;
	return 0;
}