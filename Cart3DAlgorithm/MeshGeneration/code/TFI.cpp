#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable:4273)
#pragma warning(disable:4251)
#pragma warning(disable:4286)
#endif
#include <Cart3DAlgorithm/MeshGeration/TFI.h>


namespace Cart3DAlgorithm
{
	namespace
	{
		inline cfloat g_linear_interpolation(cfloat r, cfloat x0, cfloat x1)
		{
			cfloat val = (1 - r) * x0 + (1 + r) * x1;
			val *= 0.5;
			return val;
		}
		inline cfloat g_bilinear_interpolation(
			cfloat r, cfloat s,
			cfloat x00, cfloat x10,
			cfloat x11, cfloat x01)
		{
			cfloat val =
				(1 - r) * (1 - s) * x00 +
				(1 + r) * (1 - s) * x10 +
				(1 + r) * (1 + s) * x11 +
				(1 - r) * (1 + s) * x01;
			val *= 0.25;
			return val;
		}


		inline cfloat g_trilinear_interpolation(
			cfloat r, cfloat s, cfloat t,
			cfloat x000, cfloat x100, cfloat x110, cfloat x010,
			cfloat x001, cfloat x101, cfloat x111, cfloat x011)
		{
			cfloat val = 
				(1 - r) * (1 - s) * (1 - t) * x000 +
				(1 + r) * (1 - s) * (1 - t) * x100 +
				(1 + r) * (1 + s) * (1 - t) * x110 +
				(1 - r) * (1 + s) * (1 - t) * x010 +
				(1 - r) * (1 - s) * (1 + t) * x001 +
				(1 + r) * (1 - s) * (1 + t) * x101 +
				(1 + r) * (1 + s) * (1 + t) * x111 +
				(1 - r) * (1 + s) * (1 + t) * x011;
			val *= 0.125;
			return val;
		}
		/*
		 *  01-----r1-----11
		 *   |      .      | 
		 *   |      .      | 
		 *  0s.....rs.....1s 
		 *   |      .      | 
		 *   |      .      | 
		 *  00-----r0-----10 
		 */
		inline cfloat g_transfinite(
			cfloat r, cfloat s,
			cfloat x00, cfloat x10, cfloat x11, cfloat x01,
			cfloat xr0, cfloat x1s, cfloat xr1, cfloat x0s)
		{
			cfloat u = g_linear_interpolation(r, x0s, x1s);
			cfloat v = g_linear_interpolation(s, xr0, xr1);
			cfloat uv = g_bilinear_interpolation(r, s, x00, x10, x11, x01);
			cfloat val = u + v - uv;
			return val;
		}
		/*
		 *  010-----r10-----110    011-----r11-----111
		 *    |       .       |      |       .       |
		 *    |       .       |      |       .       |
		 *  0s0.....rs0.....1s0    0s1.....rs1.....1s1     S
		 *    |       .       |      |       .       |     |
		 *    |       .       |      |       .       |     |
		 *  000-----r00-----100    001-----r01-----101     +----R
		 *     BACK                     FRONT
		 * 
		 *  001-----0s1-----011    101-----1s1-----111
		 *    |       .       |      |       .       |
		 *    |       .       |      |       .       |
		 *  00t.....0st.....01t    10t.....1st.....11t     T
		 *    |       .       |      |       .       |     |
		 *    |       .       |      |       .       |     |
		 *  000-----0s0-----010    100-----1s0-----110     +----S
		 *     LEFT                       RIGHT
		 * 
		 *  001-----r01-----101    011-----r11-----111
		 *    |       .       |      |       .       |
		 *    |       .       |      |       .       |
		 *  00t.....r0t.....10t    01t.....r1t.....11t     T
		 *    |       .       |      |       .       |     |
		 *    |       .       |      |       .       |     |
		 *  000-----r0t-----100    010-----r10-----110     +----R
		 *     BOTTOM                       TOP
		 */
		inline cfloat g_transfinite(
			cfloat r, cfloat s, cfloat t,
			cfloat x000, cfloat xr00, cfloat x100,
			cfloat x0s0, cfloat xrs0, cfloat x1s0,
			cfloat x010, cfloat xr10, cfloat x110,
			cfloat x00t, cfloat xr0t, cfloat x10t,
			cfloat x0st, cfloat x1st,
			cfloat x01t, cfloat xr1t, cfloat x11t,
			cfloat x001, cfloat xr01, cfloat x101,
			cfloat x0s1, cfloat xrs1, cfloat x1s1,
			cfloat x011, cfloat xr11, cfloat x111)
		{
			cfloat u = g_linear_interpolation(r, x0st, x1st);
			cfloat v = g_linear_interpolation(s, xr0t, xr1t);
			cfloat w = g_linear_interpolation(t, xrs0, xrs1);
			cfloat uv = g_bilinear_interpolation(r, s, x00t, x10t, x11t, x01t);
			cfloat uw = g_bilinear_interpolation(r, t, x0s0, x1s0, x1s1, x0s1);
			cfloat vw = g_bilinear_interpolation(s, t, xr00, xr10, xr11, xr01);
			cfloat uvw = g_trilinear_interpolation(r, s, t,x000, x100, x110, x010,x001, x101, x111, x011);
			cfloat val = u + v + w - uw - uv - vw + uvw;
			return val;
		}

		inline void tfi_from_corners(std::vector<cfloat>& x, const std::vector<cfloat>& glxnodes)
		{
			for (int i = 1,n=(int)glxnodes.size(); i < n - 1; ++i)
			{
				x[i] = g_linear_interpolation(glxnodes[i], x[0], x[n - 1]);
			}
		}

	}

	void TFI::tfi_from_edges(
		const std::vector<cfloat>& glxnodes,
		const std::vector<cfloat>& glynodes,
		std::vector<cfloat>& x)
	{
		int nx = (int)glxnodes.size();
		int ny = (int)glynodes.size();
		int nynx = nx * ny;
		cfloat x0 = x[0];
		cfloat x1 = x[nx - 1];
		cfloat x2 = x[nynx - 1];
		cfloat x3 = x[nynx-nx];
		for (int j = 1; j < ny - 1; ++j)
		{
			cfloat s = glynodes[j];
			for (int i = 1; i < nx - 1; ++i)
			{
				int il = j * nx;
				int ir = il + nx - 1;
				int it = nynx - nx + i;
				int ib = i;
				int offset = il + i;
				x[offset] = g_transfinite(glxnodes[i], s, x0, x1, x2, x3, x[ib], x[ir], x[it], x[il]);
			}
		}
	}

	void TFI::tfi_from_corners(
		const std::vector<cfloat>& glxnodes,
		const std::vector<cfloat>& glynodes,
		std::vector<cfloat>& x)
	{
		int nx = (int)glxnodes.size();
		int ny = (int)glynodes.size();
		int nxy = nx * ny;
		for (int i = 1; i < nx - 1; ++i)
		{
			x[i] = g_linear_interpolation(glxnodes[i], x[0], x[nx - 1]);
			x[nxy-nx+i] = g_linear_interpolation(glxnodes[i], x[nxy - nx], x[nxy - 1]);
		}
		for (int j = 1; j < ny - 1; ++j)
		{
			x[j * nx] = g_linear_interpolation(glynodes[j], x[0], x[nxy - nx]);
			x[j * nx + (nx - 1)] = g_linear_interpolation(glynodes[j], x[nx - 1], x[nxy - 1]);
		}
		tfi_from_edges( glxnodes, glynodes,x);
	}

}

#if defined(_MSC_VER)
#pragma warning(pop)
#endif