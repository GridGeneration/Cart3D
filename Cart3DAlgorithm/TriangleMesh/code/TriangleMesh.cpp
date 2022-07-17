#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable:4273)
#pragma warning(disable:4251)
#pragma warning(disable:4286)
#endif
#include <Cart3DAlgorithm/TriangleMesh/TriangleMesh.h>
#include <tbb/parallel_for.h>

namespace Cart3DAlgorithm
{
	namespace
	{
		struct CoordWithIndex {
			cvector3d data;
			int index;

			bool operator == (const CoordWithIndex& c) const
			{
				return (c[0] == data[0]) && (c[1] == data[1]) && (c[2] == data[2]);
			}

			bool operator != (const CoordWithIndex& c) const
			{
				return (c[0] != data[0]) || (c[1] != data[1]) || (c[2] != data[2]);
			}

			bool operator < (const CoordWithIndex& c) const
			{
				return (data[0] < c[0])
					|| (data[0] == c[0] && data[1] < c[1])
					|| (data[0] == c[0] && data[1] == c[1] && data[2] < c[2]);
			}

			inline cfloat& operator [] (const size_t i) { return data[i]; }
			inline cfloat operator [] (const size_t i) const { return data[i]; }
		};

		static bool read_mesh_impl(const std::string& filename, std::vector<CoordWithIndex>& meshpts)
		{

			FILE* in = nullptr;
			fopen_s(&in, filename.c_str(), "r");
			if (!in)
				return false;
			char line[100], * c;
			c = fgets(line, 6, in);
			if (c == nullptr)
			{
				return false;
			}
			const bool binary = ((strncmp(line, "SOLID", 5) != 0) && (strncmp(line, "solid", 5) != 0));
			if (binary) {
				fclose(in);
				fopen_s(&in, filename.c_str(), "rb");
				int n_items = fseek(in, 80, 0);
				unsigned int nT = 0;
				fread((char*)&nT, 1, sizeof(nT), in);
				if (nT <= 0) {
					return false;
				}
				unsigned int nT3 = nT + nT + nT;
				meshpts.resize(nT3);
				std::unique_ptr<uint8_t[]> buffer(new uint8_t[nT * 50]);
				fread((char*)buffer.get(), 1, nT * 50, in);
				const size_t float3 = sizeof(float) * 3;
				const size_t float6 = sizeof(float) * 6;
				const size_t float9 = sizeof(float) * 9;
				char* bufferd = (char*)buffer.get();
				int nTT = (int)nT;
				tbb::parallel_for(0, nTT, [&](int i) {
						unsigned iu = (unsigned)i;
						auto b = bufferd + (iu << 5) + (iu << 4) + (iu << 1);
						unsigned int i3 = iu + iu + iu;
						float p0[3], p1[3], p2[3];
						memcpy_s(p0, float3, b + float3, float3);
						memcpy_s(p1, float3, b + float6, float3);
						memcpy_s(p2, float3, b + float9, float3);
						meshpts[i3 + 0].data = cvector3d(p0[0], p0[1], p0[2]);
						meshpts[i3 + 0].index = i3 + 0;
						meshpts[i3 + 1].data = cvector3d(p1[0], p1[1], p1[2]);
						meshpts[i3 + 1].index = i3 + 1;
						meshpts[i3 + 2].data = cvector3d(p2[0], p2[1], p2[2]);
						meshpts[i3 + 2].index = i3 + 2;
					});
				fclose(in);
			}
			else
			{
				if (in) {
					meshpts.clear();
					meshpts.reserve(500000);
					CoordWithIndex p;
					while (!feof(in) && fgets(line, 100, in))
					{
						// skip white-space
						for (c = line; isspace(*c) && *c != '\0'; ++c)
						{
						};
						// face begins
						if ((strncmp(c, "outer", 5) == 0) || (strncmp(c, "OUTER", 5) == 0))
						{

							c = fgets(line, 100, in);
							for (c = line; isspace(*c) && *c != '\0'; ++c)
							{
							};
							sscanf_s(c + 6, "%lf %lf %lf", &p[0], &p[1], &p[2]);
							p.index = (int)meshpts.size();
							meshpts.push_back(p);

							c = fgets(line, 100, in);
							for (c = line; isspace(*c) && *c != '\0'; ++c)
							{
							};
							sscanf_s(c + 6, "%lf %lf %lf", &p[0], &p[1], &p[2]);
							p.index = (int)meshpts.size();
							meshpts.push_back(p);

							c = fgets(line, 100, in);
							for (c = line; isspace(*c) && *c != '\0'; ++c)
							{
							};
							sscanf_s(c + 6, "%lf %lf %lf", &p[0], &p[1], &p[2]);
							p.index = (int)meshpts.size();
							meshpts.push_back(p);
						}
					}
					fclose(in);
				}
				meshpts.shrink_to_fit();
			}
			return meshpts.size() % 3 == 0;
		}

	}


	TriangleMesh::TriangleMesh(std::vector<cvector3d>& _pts,
		std::vector<std::array<int, 3>>& _face,
		bool keepori)
	{
		if (keepori)
		{
			pts = _pts;
			faces = _face;
		}
		else
		{
			pts.swap(_pts);
			faces.swap(_face);
		}
	}

	TriangleMesh::TriangleMesh(const std::string& filename)
	{
		std::vector<CoordWithIndex> rpts;
		if (!read_mesh_impl(filename, rpts)||pts.empty())
			return;
		std::vector<int> trisOut(rpts.size());
		int nrpts = static_cast<int>(rpts.size());
		for (int i = 0; i < nrpts; ++i){
			trisOut[i] = i;
		}
		std::sort(rpts.begin(), rpts.end());
		int numUnique = 1;
		for (size_t i = 1; i < rpts.size(); ++i) {
			if (rpts[i] != rpts[i - 1])
				++numUnique;
		}
		pts.resize(numUnique);
		std::vector<int> newIndex(rpts.size());
		int curInd = 0;
		newIndex[0] = 0;
		pts[0] = rpts[0].data;
		for (size_t i = 1; i < rpts.size(); ++i)
		{
			const auto& c = rpts[i];
			if (c != rpts[i - 1])
			{
				++curInd;
				pts[curInd] = rpts[i].data;
			}
			newIndex[c.index] = curInd;
		}

		int numUniqueTriInds = 0;
		for (int i = 0; i < trisOut.size(); i += 3) {
			int ni[3];
			for (int j = 0; j < 3; ++j)
				ni[j] = newIndex[trisOut[i + j]];

			if ((ni[0] != ni[1]) && (ni[0] != ni[2]) && (ni[1] != ni[2])) {
				for (int j = 0; j < 3; ++j)
					trisOut[numUniqueTriInds + j] = ni[j];
				numUniqueTriInds += 3;
			}
		}
		if (numUniqueTriInds < trisOut.size())
			trisOut.resize(numUniqueTriInds);
		faces.resize(numUniqueTriInds / 3);
		for (int i = 0,j=0; i < numUniqueTriInds; i += 3,++j)
		{
			faces[j][0] = trisOut[i];
			faces[j][1] = trisOut[i + 1];
			faces[j][2] = trisOut[i + 2];
		}
	}

	cvector3d& TriangleMesh::get_point(int id)
	{
		return pts[id];
	}
	const cvector3d& TriangleMesh::get_point(int id)const
	{
		return pts[id];
	}
	std::array<int, 3>& TriangleMesh::get_tri(int id)
	{
		return faces[id];
	}
	const std::array<int, 3>& TriangleMesh::get_tri(int id)const
	{
		return faces[id];
	}
}


#if defined(_MSC_VER)
#pragma warning(pop)
#endif