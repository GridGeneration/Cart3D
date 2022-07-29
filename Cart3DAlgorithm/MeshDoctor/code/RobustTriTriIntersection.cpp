#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable:4273)
#pragma warning(disable:4251)
#pragma warning(disable:4286)
#endif

#include <MeshDoctor/RobustTriTriIntersection.h>

namespace Cart3DAlgorithm
{


	int RobustTriTriIntersection::orient3d(
		const cvector3d& a, const cvector3d& b,
		const cvector3d& c, const cvector3d& d)
	{
		cmatrix4d mat;
		mat.setConstant(1);
		mat.block<1, 3>(0, 0) = a;
		mat.block<1, 3>(1, 0) = b;
		mat.block<1, 3>(2, 0) = c;
		mat.block<1, 3>(3, 0) = d;
		cfloat det = mat.determinant();
		if (det < -inv_trunc_val)
			return -1;
		else if (det > inv_trunc_val)
			return 1;
		return 0;
	}

	int RobustTriTriIntersection::orient2d(
		const cvector2d& a,
		const cvector2d& b,
		const cvector2d& c)
	{
		cmatrix3d mat;
		mat.setConstant(1);
		mat.block<1, 2>(0, 0) = a;
		mat.block<1, 2>(1, 0) = b;
		mat.block<1, 2>(2, 0) = c;
		cfloat det = mat.determinant();
		if (det < -inv_trunc_val)
			return -1;
		else if (det > inv_trunc_val)
			return 1;
		return 0;
	}

}



#if defined(_MSC_VER)
#pragma warning(pop)
#endif