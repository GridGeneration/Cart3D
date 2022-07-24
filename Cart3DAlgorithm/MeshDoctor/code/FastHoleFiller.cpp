#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable:4273)
#pragma warning(disable:4251)
#pragma warning(disable:4286)
#pragma warning(disable:4267)
#endif
#include <functional>
#include "MeshDoctor/FastHoleFiller.h"
#include <MeshDoctor/MeshBoundaryExtractor.h>
#include <Common/util.h>
#include <queue>
#include <unordered_map>
#include <set>
namespace Cart3DAlgorithm
{
	namespace
	{
		static bool sewUpLoopMinArea(const std::vector<OpenTriMesh::Point>& loops, std::vector<int>& tris)
		{
			int size = static_cast<int>(loops.size());
			if (size < 3)
				return false;
			auto area = [&loops](int i, int j, int k) {
				OpenTriMesh::Point a = loops[i];
				OpenTriMesh::Point b = loops[j];
				OpenTriMesh::Point c = loops[k];
				OpenTriMesh::Point n((b - a) % (c - b));
				return static_cast<float>(0.5 * n.norm());
			};
			std::function<void(std::vector<int>& tris,
				const std::vector<int>& hole_id,
				std::vector<std::vector<int>>& minimum_weight_index,
				int begin, int end)>AddHoleToMesh;
			AddHoleToMesh = [&AddHoleToMesh](
				std::vector<int>& tris,
				const std::vector<int>& hole_id,
				std::vector<std::vector<int>>& minimum_weight_index,
				int begin, int end) {
					if (end - begin > 1) {
						int cu = minimum_weight_index[begin][end];
						tris.push_back(hole_id[begin]);
						tris.push_back(hole_id[cu]);
						tris.push_back(hole_id[end]);
						AddHoleToMesh(tris, hole_id, minimum_weight_index, begin, cu);
						AddHoleToMesh(tris, hole_id, minimum_weight_index, cu, end);
					}
			};
			std::vector<std::vector<float>> minimum_weight(size, std::vector<float>(size, 0));
			std::vector<std::vector<int>> minimum_weight_index(size, std::vector<int>(size, -1));
			std::vector<int> ids;
			ids.reserve(size);
			for (int ic = 0; ic < size; ++ic)
				ids.push_back(ic);
			tris.clear();
			tris.reserve(size * 3);
			for (int j = 2; j < size; ++j) {
				for (int i = 0; i < size - j; ++i) {
					float min = FLT_MAX;
					int index = -1;
					int k = i + j;
					for (int m = i + 1; m < k; m++) {
						float farea = area(i, m, k);
						float val = minimum_weight[i][m] + minimum_weight[m][k] + farea;
						if (val < min) {
							min = val; index = m;
						}
					}
					minimum_weight[i][k] = min;
					minimum_weight_index[i][k] = index;
				}
			}
			AddHoleToMesh(tris, ids, minimum_weight_index, 0, size - 1);
			return true;
		}
	}
	bool FastHoleFiller::fix_hole(OpenTriMesh& mesh, const HalfedgeHandle& halfedge)
	{
		if (!mesh.is_valid_handle(halfedge))
			return false;
		if (!mesh.is_boundary(halfedge))
			return false;
		std::vector<VertexHandle> vhs;
		MeshBoundaryExtractor::ExtraBoundary(mesh, halfedge, vhs);
		return fix_hole(mesh, vhs);
	}

	bool FastHoleFiller::fix_hole(OpenTriMesh& mesh, const HalfedgeHandle& halfedge, std::vector<FaceHandle>& new_faces)
	{
		if (!mesh.is_valid_handle(halfedge))
			return false;
		if (!mesh.is_boundary(halfedge))
			return false;
		std::vector<VertexHandle> vhs;
		MeshBoundaryExtractor::ExtraBoundary(mesh, halfedge, vhs);
		return fix_hole(mesh, vhs,new_faces);
	}

	bool FastHoleFiller::fix_hole(OpenTriMesh& mesh, const std::vector<VertexHandle>& hole)
	{
		std::vector<FaceHandle> new_faces;
		return fix_hole(mesh, hole, new_faces);
	}


	bool FastHoleFiller::fix_hole(OpenTriMesh& mesh, const std::vector<VertexHandle>& hole, std::vector<FaceHandle>& new_faces)
	{
		if (hole.size()<3)
			return false;
		else if (hole.size() == 3)
		{
			auto fh = mesh.add_face(hole);
			if (!mesh.is_valid_handle(fh))
				return false;
			else
			{
				new_faces.push_back(fh);
				return true;
			}
		}
		else
		{
			int nhole = static_cast<int>(hole.size());
			std::vector<OpenTriMesh::Point> vinfos;
			vinfos.reserve(nhole);
			for (auto& iv : hole)
				vinfos.push_back(mesh.point(iv));
			std::vector<int> tris;
			if (!sewUpLoopMinArea(vinfos, tris))
				return false;
			int ntris = static_cast<int>(tris.size());
			for (int i = 0; i < ntris; i += 3)
			{
				new_faces.push_back(mesh.add_face(hole[tris[i]], hole[tris[i+1]], hole[tris[i+2]]));
			}
		}
		return true;
	}
	
	OpenTriMesh::Normal FastHoleFiller::compute_vertex_norm(const OpenTriMesh& mesh, const VertexHandle& vh)
	{
		if (!mesh.is_valid_handle(vh)||mesh.is_isolated(vh))
			return OpenTriMesh::Normal(0, 0, 0);
		OpenTriMesh::Normal vn(0, 0, 0);
		for (auto vf = mesh.cvf_begin(vh); vf != mesh.cvf_end(vh); ++vf)
		{
			if (!mesh.is_valid_handle(*vf))
				continue;
			auto fv = mesh.cfv_ccwiter(*vf);
			auto& p0 = mesh.point(*fv); VertexHandle vh0 = *fv; ++fv;
			auto& p1 = mesh.point(*fv); VertexHandle vh1 = *fv; ++fv;
			auto& p2 = mesh.point(*fv); VertexHandle vh2 = *fv;
			auto vec0 = p2 - p1;
			auto vec1 = p0 - p2;
			auto vec2 = p1 - p0;
			auto sqr0 = vec0.sqrnorm();
			auto sqr1 = vec1.sqrnorm();
			auto sqr2 = vec2.sqrnorm();
			if (sqr0 <= FLT_EPSILON ||
				sqr1 <= FLT_EPSILON ||
				sqr2 <= FLT_EPSILON)
			{
				continue;
			}
			auto fnor = -1.0f * OpenMesh::cross(vec2, vec1);
			if (vh == vh0)
				vn += fnor * (1.0f / (sqr1 * sqr2));
			else if (vh == vh1)
				vn += fnor * (1.0f / (sqr0 * sqr2));
			else
				vn += fnor * (1.0f / (sqr0 * sqr1));
		}
		vn.normalize_cond();
		return vn;
	}
}


#if defined(_MSC_VER)
#pragma warning(pop)
#endif