#ifndef CART3DALGORITHM_UTIL_H
#define CART3DALGORITHM_UTIL_H

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable:26451)
#pragma warning(disable:26495)
#pragma warning(disable:4244)
#endif
#include <Eigen/Eigen>
#include <numeric>
#include "CommonConfig.h"
#include <rationals.h>
namespace Cart3DAlgorithm
{
	using cfloat32 = float;
	using cfloat64 = double;
	using cfloat = cfloat64;
	using crational = rationals::rational<std::int64_t>;
	template<class T,int udim,int vdim>
	using cmatrixt = Eigen::Matrix<T, udim, vdim>;
	template<int udim, int vdim>
	using cmatrixf32 = cmatrixt<cfloat32,udim,vdim>;
	template<int udim, int vdim>
	using cmatrixf64 = cmatrixt<cfloat64,udim,vdim>;
	template<int udim, int vdim>
	using cmatrix = cmatrixt<cfloat64, udim, vdim>;
	using cmatrix4d = cmatrix<4, 4>;
	using cmatrix3d = cmatrix<3, 3>;
	using cmatrix2d = cmatrix<2, 2>;


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

#define maxint std::numeric_limits<int>::max()
#define mincfloat -std::numeric_limits<cfloat>::max()
#define maxcfloat std::numeric_limits<cfloat>::max()
#define epsilon_cfloat   std::numeric_limits<cfloat>::epsilon()

	COMMON_API void ecode_rgb(int sID, cvector3d& col);

}

#if defined(_MSC_VER)
#pragma warning(pop)
#endif

#endif
