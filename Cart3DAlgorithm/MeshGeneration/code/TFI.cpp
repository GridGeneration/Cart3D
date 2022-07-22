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


	void TFI::tfi_from_faces(
		const std::vector<cfloat>& glxnodes,
		const std::vector<cfloat>& glynodes,
		const std::vector<cfloat>& glznodes,
		std::vector<cfloat>& x)
	{
		const int nx = (int)glxnodes.size();
		const int ny = (int)glynodes.size();
		const int nz = (int)glznodes.size();
		const int nxy = nx * ny;

		for (int k = 1; k < nz - 1; k++)
		{
			cfloat t = glznodes[k];
			for (int j = 1; j < ny - 1; j++)
			{
				cfloat s = glynodes[j];
				for (int i = 1; i < nx - 1; i++)
				{
					cfloat r = glxnodes[i];

					cfloat x000 = x[0];
					cfloat xr00 = x[i];
					cfloat x100 = x[(nx - 1)];

					cfloat x0s0 = x[j * nx];
					cfloat xrs0 = x[i + j * nx];
					cfloat x1s0 = x[(nx - 1) + j * nx];

					cfloat x010 = x[(ny - 1) * nx];
					cfloat xr10 = x[i + (ny - 1) * nx];
					cfloat x110 = x[(ny - 1) * nx + (nx - 1)];

					cfloat x00t = x[k * nxy];
					cfloat xr0t = x[i + k * nxy];
					cfloat x10t = x[(nx - 1) + k * nxy];

					cfloat x0st = x[j * nx + k * nxy];
					cfloat x1st = x[(nx - 1) + j * nx + k * nxy];

					cfloat x01t = x[(ny - 1) * nx + k * nxy];
					cfloat xr1t = x[i + (ny - 1) * nx + k * nxy];
					cfloat x11t = x[(nx - 1) + (ny - 1) * nx + k * nxy];

					cfloat x001 = x[(nz - 1) * nxy];
					cfloat xr01 = x[i + (nz - 1) * nxy];
					cfloat x101 = x[(nz - 1) * nxy + (nx - 1)];

					cfloat x0s1 = x[j * nx + (nz - 1) * nxy];
					cfloat xrs1 = x[i + j * nx + (nz - 1) * nxy];
					cfloat x1s1 = x[(nx - 1) + j * nx + (nz - 1) * nxy];

					cfloat x011 = x[(ny - 1) * nx + (nz - 1) * nxy];
					cfloat xr11 = x[i + (ny - 1) * nx + (nz - 1) * nxy];
					cfloat x111 = x[(nz - 1) * nxy + (ny - 1) * nx + (nx - 1)];

					int offset = k * nxy + j * nx + i;
					x[offset] = g_transfinite(r, s, t,
						x000, xr00, x100,
						x0s0, xrs0, x1s0,
						x010, xr10, x110,
						x00t, xr0t, x10t,
						x0st, x1st,
						x01t, xr1t, x11t,
						x001, xr01, x101,
						x0s1, xrs1, x1s1,
						x011, xr11, x111);
				}

			}

		}

	}

	void TFI::tfi_from_edges(
		const std::vector<cfloat>& glxnodes,
		const std::vector<cfloat>& glynodes,
		const std::vector<cfloat>& glznodes,
		std::vector<cfloat>& x)
	{
		int offset;
		int nx = (int)glxnodes.size();
		int ny = (int)glynodes.size();
		int nz = (int)glznodes.size();
		int nxy = nx * ny;

		cfloat r, s, t;
		cfloat x00, x10, x11, x01;
		cfloat xs0, x1t, xs1, x0t;

		for (int k = 1; k < nz - 1; k++)
		{
			t = glznodes[k];
			for (int j = 1; j < ny - 1; j++)
			{
				s = glynodes[j];
				//Left Face ...
				x00 = x[0];
				x10 = x[(ny - 1) * nx];
				x11 = x[(nz - 1) * nxy + (ny - 1) * nx];
				x01 = x[(nz - 1) * nxy];
				xs0 = x[j * nx];
				x1t = x[k * nxy + (ny - 1) * nx];
				xs1 = x[(nz - 1) * nxy + j * nx];
				x0t = x[k * nxy];
				offset = k * nxy + j * nx;
				x[offset] = g_transfinite(s, t, x00, x10, x11, x01, xs0, x1t, xs1, x0t);
				// Right Face ...
				x00 = x[nx - 1];
				x10 = x[(ny - 1) * nx + (nx - 1)];
				x11 = x[(nz - 1) * nxy + (ny - 1) * nx + (nx - 1)];
				x01 = x[(nz - 1) * nxy + (nx - 1)];
				xs0 = x[j * nx + (nx - 1)];
				x1t = x[k * nxy + (ny - 1) * nx + (nx - 1)];
				xs1 = x[(nz - 1) * nxy + j * nx + (nx - 1)];
				x0t = x[k * nxy + (nx - 1)];
				offset = k * nxy + j * nx + nx - 1;
				x[offset] = g_transfinite(s, t, x00, x10, x11, x01, xs0, x1t, xs1, x0t);
			}
		}
		cfloat xr0, xr1;
		for (int k = 1; k < nz - 1; k++)
		{
			t = glznodes[k];
			for (int i = 1; i < nx - 1; i++)
			{
				r = glxnodes[i];
				// Bottom Face ...
				x00 = x[0];
				x10 = x[nx - 1];
				x11 = x[(nz - 1) * nxy + (nx - 1)];
				x01 = x[(nz - 1) * nxy];
				xr0 = x[i];
				x1t = x[k * nxy + (nx - 1)];
				xr1 = x[(nz - 1) * nxy + i];
				x0t = x[k * nxy];
				offset = k * nxy + i;
				x[offset] = g_transfinite(r, t, x00, x10, x11, x01, xr0, x1t, xr1, x0t);
				// Top Face ...
				x00 = x[(ny - 1) * nx];
				x10 = x[(ny - 1) * nx + nx - 1];
				x11 = x[(nz - 1) * nxy + (ny - 1) * nx + nx - 1];
				x01 = x[(nz - 1) * nxy + (ny - 1) * nx];
				xr0 = x[(ny - 1) * nx + i];
				x1t = x[k * nxy + (ny - 1) * nx + nx - 1];
				xr1 = x[(nz - 1) * nxy + (ny - 1) * nx + i];
				x0t = x[k * nxy + (ny - 1) * nx];
				offset = k * nxy + (ny - 1) * nx + i;
				x[offset] = g_transfinite(r, t, x00, x10, x11, x01, xr0, x1t, xr1, x0t);

			}
		}
		cfloat x0s, x1s;
		for (int j = 1; j < ny - 1; j++)
		{
			s = glynodes[j];
			for (int i = 1; i < nx - 1; i++)
			{
				r = glxnodes[i];
				// Back Face ...
				x00 = x[0];
				x10 = x[nx - 1];
				x11 = x[(ny - 1) * nx + (nx - 1)];
				x01 = x[(ny - 1) * nx];
				xr0 = x[i];
				x1s = x[j * nx + nx - 1];
				xr1 = x[(ny - 1) * nx + i];
				x0s = x[j * nx];
				offset = j * nx + i;
				x[offset] = g_transfinite(r, s, x00, x10, x11, x01, xr0, x1s, xr1, x0s);
				// Front Face ...
				x00 = x[(nz - 1) * nxy];
				x10 = x[(nz - 1) * nxy + nx - 1];
				x11 = x[(nz - 1) * nxy + (ny - 1) * nx + (nx - 1)];
				x01 = x[(nz - 1) * nxy + (ny - 1) * nx];
				xr0 = x[(nz - 1) * nxy + i];
				x1s = x[(nz - 1) * nxy + j * nx + nx - 1];
				xr1 = x[(nz - 1) * nxy + (ny - 1) * nx + i];
				x0s = x[(nz - 1) * nxy + j * nx];
				offset = (nz - 1) * nx * ny + j * nx + i;
				x[offset] = g_transfinite(r, s, x00, x10, x11, x01, xr0, x1s, xr1, x0s);
			}
		}
		tfi_from_faces(glxnodes, glynodes, glznodes,x);
	}

	void TFI::tfi_from_corners(
		const std::vector<cfloat>& glxnodes,
		const std::vector<cfloat>& glynodes,
		const std::vector<cfloat>& glznodes,
		std::vector<cfloat>& x)
	{
		const int nx = (int)glxnodes.size();
		const int ny = (int)glynodes.size();
		const int nz = (int)glznodes.size();
		const int nxy = nx * ny;

		for (int i = 1; i < nx - 1; ++i)
		{
			cfloat r = glxnodes[i];

			cfloat x0 = x[0];
			cfloat x1 = x[nx - 1];
			int offset = i;
			x[offset] = g_linear_interpolation(r, x0, x1);
			x0 = x[(ny - 1) * nx];
			x1 = x[(ny - 1) * nx + nx - 1];
			offset = (ny - 1) * nx + +i;
			x[offset] = g_linear_interpolation(r, x0, x1);
			x0 = x[(nz - 1) * nxy];
			x1 = x[(nz - 1) * nxy + nx - 1];
			offset = (nz - 1) * nxy + i;
			x[offset] = g_linear_interpolation(r, x0, x1);
			x0 = x[(nz - 1) * nxy + (ny - 1) * nx];
			x1 = x[(nz - 1) * nxy + (ny - 1) * nx + nx - 1];
			offset = (nz - 1) * nxy + (ny - 1) * nx + i;
			x[offset] = g_linear_interpolation(r, x0, x1);

		}
		for (int j = 1; j < ny - 1; j++)
		{
			cfloat s = glynodes[j];
			cfloat x0 = x[0];
			cfloat x1 = x[(ny - 1) * nx];
			int offset = j * nx;
			x[offset] = g_linear_interpolation(s, x0, x1);
			x0 = x[nx - 1];
			x1 = x[(ny - 1) * nx + (nx - 1)];
			offset = j * nx + nx - 1;
			x[offset] = g_linear_interpolation(s, x0, x1);
			x0 = x[(nz - 1) * nxy];
			x1 = x[(nz - 1) * nxy + (ny - 1) * nx];
			offset = (nz - 1) * nxy + j * nx;
			x[offset] = g_linear_interpolation(s, x0, x1);
			x0 = x[(nz - 1) * nxy + nx - 1];
			x1 = x[(nz - 1) * nxy + (ny - 1) * nx + nx - 1];
			offset = (nz - 1) * nxy + j * nx + nx - 1;
			x[offset] = g_linear_interpolation(s, x0, x1);
		}

		for (int k = 1; k < nz - 1; k++)
		{
			cfloat t = glznodes[k];

			cfloat x0 = x[0];
			cfloat x1 = x[(nz - 1) * nxy];
			int offset = k * nxy;
			x[offset] = g_linear_interpolation(t, x0, x1);

			x0 = x[nx - 1];
			x1 = x[(nz - 1) * nxy + nx - 1];
			offset = k * nxy + nx - 1;
			x[offset] = g_linear_interpolation(t, x0, x1);

			x0 = x[(ny - 1) * nx];
			x1 = x[(nz - 1) * nxy + (ny - 1) * nx];
			offset = k * nxy + (ny - 1) * nx;
			x[offset] = g_linear_interpolation(t, x0, x1);

			x0 = x[(ny - 1) * nx + nx - 1];
			x1 = x[(nz - 1) * nxy + (ny - 1) * nx + nx - 1];
			offset = k * nx * ny + (ny - 1) * nx + nx - 1;
			x[offset] = g_linear_interpolation(t, x0, x1);
		}
		tfi_from_edges(glxnodes, glynodes, glznodes,x);
	}
}

#if defined(_MSC_VER)
#pragma warning(pop)
#endif