#ifndef CART3DALGORITHM_UTIL_H
#define CART3DALGORITHM_UTIL_H

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable:26451)
#pragma warning(disable:26495)
#endif
#include <Eigen/Eigen>


namespace Cart3DAlgorithm
{
	using cfloat32 = float;
	using cfloat64 = double;
	using cfloat = cfloat64;

	template<class T>
	using cmatrixt = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;
	using cmatrixf32 = cmatrixt<cfloat32>;
	using cmatrixf64 = cmatrixt<cfloat64>;
	using cmatrix = cmatrixf64;

	template<class T,int size>
	using cvectort = Eigen::Vector<T,size>;
	template<int dim>
	using cvectorf32 = cvectort<cfloat32,dim>;
	template<int dim>
	using cvectorf64 = cvectort<cfloat64,dim>;
	template<int dim>
	using cvector = cvectorf64<dim>;

	using cvector3d = cvector<3>;
	using cvector2d = cvector<2>;

	
}

#if defined(_MSC_VER)
#pragma warning(pop)
#endif

#endif
