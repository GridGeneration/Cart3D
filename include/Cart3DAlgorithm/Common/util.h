#ifndef CART3DALGORITHM_UTIL_H
#define CART3DALGORITHM_UTIL_H
#include <Eigen/Eigen>


namespace Cart3DAlgorithm
{
	using cfloat32 = float;
	using cfloat64 = double;

	template<class T>
	using cmatrix = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;
	using cmatrixf32 = cmatrix<cfloat32>;
	using cmatrixf64 = cmatrix<cfloat64>;

	template<class T>
	using cvector = Eigen::Vector<T>;
	using cvectorf32 = cvector<cfloat32>;
	using cvectorf64 = cvector<cfloat64>;


}

#endif
