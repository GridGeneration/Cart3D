#ifndef CART3DALGORITHM_UTIL_H
#define CART3DALGORITHM_UTIL_H

#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable:26451)
#pragma warning(disable:26495)
#endif
#include <Eigen/Eigen>
#include <numeric>

namespace Cart3DAlgorithm
{
	using cfloat32 = float;
	using cfloat64 = double;
	using cfloat = cfloat64;

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

	template<class Color>
	void ecode_rgb(int sID,Color& col)
	{
		cfloat h = (cfloat)((72 + 1327 * sID) % 251);
		cfloat l = (cfloat)((18 + 337 * sID) % 37) / 37.0f;
		l = (l * 0.5f) + 0.3f;
		cfloat s = 1;
		cfloat q = (l < 0.5) ? l * (1 + s) : l + s - (l * s);
		cfloat p = 2 * l - q;
		cfloat hk = (h / 360.0f);
		cfloat t[3], rgb[3];
		t[0] = fmodf(hk + 1.0f / 3.0f, 1);
		t[1] = fmodf(hk, 1);
		t[2] = fmodf(hk - 1.0f / 3.0f, 1);
		for (int i = 0; i < 3; ++i) {
			if (t[i] < (1.0f / 6.0f))
				rgb[i] = p + ((q - p) * 6 * t[i]);
			else if (t[i] < 0.5f)
				rgb[i] = q;
			else if (t[i] < 2.0f / 3.0f)
				rgb[i] = p + ((q - p) * 6 * (2.0f / 3.0f - t[i]));
			else
				rgb[i] = p;
		}
		col[0] = (int)(255 * rgb[0]) % 255;
		col[1] = (int)(255 * rgb[1]) % 255;
		col[2] = (int)(255 * rgb[2]) % 255;
	}

}

#if defined(_MSC_VER)
#pragma warning(pop)
#endif

#endif
