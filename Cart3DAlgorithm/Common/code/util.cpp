#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable:4273)
#pragma warning(disable:4251)
#pragma warning(disable:4286)
#pragma warning(disable:4267)
#endif
#include <Common/util.h>

namespace Cart3DAlgorithm
{

	void ecode_rgb(int sID, cvector3d& col)
	{
		cfloat h = (cfloat)((72 + 1327 * sID) % 251);
		cfloat l = (cfloat)((18 + 337 * sID) % 37) / 37.0;
		l = (l * 0.5) + 0.3;
		cfloat s = 1;
		cfloat q = (l < 0.5) ? l * (1 + s) : l + s - (l * s);
		cfloat p = 2 * l - q;
		cfloat hk = (h / 360.0);
		cfloat t[3], rgb[3];
		t[0] = fmod(hk + 1.0 / 3.0, 1);
		t[1] = fmod(hk, 1);
		t[2] = fmod(hk - 1.0 / 3.0, 1);
		for (int i = 0; i < 3; ++i) {
			if (t[i] < (1.0 / 6.0))
				rgb[i] = p + ((q - p) * 6 * t[i]);
			else if (t[i] < 0.5)
				rgb[i] = q;
			else if (t[i] < 2.0 / 3.0)
				rgb[i] = p + ((q - p) * 6 * (2.0 / 3.0 - t[i]));
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