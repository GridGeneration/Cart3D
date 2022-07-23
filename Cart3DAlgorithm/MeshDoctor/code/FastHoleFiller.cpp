#include "FastHoleFiller.h"
#include <MeshDoctor/MeshBoundaryExtractor.h>
#include <Common/util.h>
#include <queue>
#include <unordered_map>
#include <set>
namespace Cart3DAlgorithm
{
	namespace
	{
		inline static OpenTriMesh::Scalar sqr(OpenTriMesh::Scalar a)
		{
			return a * a;
		}

		struct Weight
		{
			VertexHandle v1, v2, v3;
			OpenTriMesh::Scalar quality;
			Weight(
				const VertexHandle& _v1,
				const VertexHandle& _v2,
				const VertexHandle& _v3, OpenTriMesh::Scalar _q);
			bool operator < (const Weight& rhs) const;
		};

		Weight::Weight(
			const VertexHandle& _v1,
			const VertexHandle& _v2,
			const VertexHandle& _v3, OpenTriMesh::Scalar _q) :
			v1(_v1), v2(_v2), v3(_v3), quality(_q)
		{}
		bool Weight::operator < (const Weight& rhs) const
		{
			return quality < rhs.quality;
		}
	}


	bool FastHoleFiller::fix_hole(OpenTriMesh& mesh, const std::vector<VertexHandle>& hole)
	{
		if (hole.size()<3)
			return false;
		else if (hole.size() == 3)
		{
			auto fh = mesh.add_face(hole);
			if (!mesh.is_valid_handle(fh))
				return false;
			else
				return true;
		}
		else
		{
			std::vector<OpenTriMesh::Normal> vnorms;
			for (auto& iv : hole)
			{
				vnorms.push_back(compute_vertex_norm(mesh, iv));
			}
			int nhole = static_cast<int>(hole.size());
			OpenTriMesh::Scalar sclen = 0;
			std::vector<OpenTriMesh::Scalar> hole_len;
			std::unordered_map<int,std::set<int>> inser_points;
			for (int i = 0; i < nhole; ++i)
			{
				auto& p0 = mesh.point(hole[i]);
				auto& p1 = mesh.point(hole[(i + 1) % nhole]);
				hole_len.push_back((p0 - p1).sqrnorm());
				for (auto vv = mesh.vv_begin(hole[i]); vv != mesh.vv_end(hole[i]); ++vv)
					inser_points[hole[i].idx()].insert(vv->idx());
			}
			std::sort(hole_len.begin(), hole_len.end());
			const OpenTriMesh::Scalar mean_edge = hole_len[nhole / 2];
			std::priority_queue<Weight> tQ;
			for (int i = 0; i < nhole; ++i)
			{
				int i0 = i;
				int i1 = (i + 1) % nhole;
				int i2 = (i + 2) % nhole;
				VertexHandle vh0 = hole[i0];
				VertexHandle vh1 = hole[i1];
				VertexHandle vh2 = hole[i2];
				if (inser_points[vh0.idx()].find(vh2.idx()) == inser_points[i0].end())
				{
					OpenTriMesh::Scalar qual = compute_weight(mesh,
						vnorms[i0], vnorms[i1], vnorms[i2],
						vh0, vh1, vh2, mean_edge, true);
					tQ.push(Weight(vh0, vh1, vh2, qual));
				}
				else
				{
					tQ.push(Weight(vh0, vh1, vh2, -1000));
				}
			}
			
			while (!tQ.empty())
			{
				const auto& node = tQ.top();
				tQ.pop();

			}

		}
		return true;
	}
	


	OpenTriMesh::Scalar FastHoleFiller::compute_weight(
		const OpenTriMesh& mesh,
		const OpenTriMesh::Normal& n1,
		const OpenTriMesh::Normal& n2,
		const OpenTriMesh::Normal& n3,
		const VertexHandle& _v1,
		const VertexHandle& _v2,
		const VertexHandle& _v3,
		OpenTriMesh::Scalar meanedgelen,
		bool hack)
	{
		const auto& p1 = mesh.point(_v1);
		const auto& p2 = mesh.point(_v2);
		const auto& p3 = mesh.point(_v3);
		OpenTriMesh::Point side1 = p1 - p2;
		OpenTriMesh::Point side2 = p2 - p3;
		OpenTriMesh::Point side3 = p3 - p1;
		OpenTriMesh::Point norm = side2.cross(side1);
		norm.normalize_cond();
		OpenTriMesh::Scalar dot1 = norm.dot(n1);
		OpenTriMesh::Scalar dot2 = norm.dot(n2);
		OpenTriMesh::Scalar dot3 = norm.dot(n3);
		if (dot1 < -0.999f || dot2 < -0.999f || dot3 < -0.999f)
			return -1000.0f;
		OpenTriMesh::Scalar  len1 = side1.norm();
		OpenTriMesh::Scalar  len2 = side2.norm();
		OpenTriMesh::Scalar  len3 = side3.norm();
		OpenTriMesh::Scalar maxedgelen = std::max(std::max(len1, len2), len3);
		side1.normalize_cond();
		side2.normalize_cond();
		side3.normalize_cond();
		OpenTriMesh::Scalar f = dot1 / (1.0f + dot1) + dot2 / (1.0f + dot2) + dot3 / (1.0f + dot3);
		OpenTriMesh::Scalar d1 = 1.0f + maxedgelen / meanedgelen;
		OpenTriMesh::Scalar a=0;
		if (hack)
		{
			a = 2.0f + side1.dot(side2);
		}
		else
		{
			a = sqr(1.0f + side1.dot(side2)) +
				sqr(1.0f + side2.dot(side3)) +
				sqr(1.0f + side3.dot(side1));
		}
		return f * sqrt(d1) - a * d1;
	}

	OpenTriMesh::Normal FastHoleFiller::compute_vertex_norm(const OpenTriMesh& mesh, const VertexHandle& vh)
	{
		if (!mesh.is_valid_handle(vh)||mesh.is_isolated(vh))
			return OpenTriMesh::Normal(0, 0, 0);
		OpenTriMesh::Normal vn(0, 0, 0);
		for (auto vf = mesh.cvf_begin(vh); vf != mesh.cvf_end(vh); ++vf)
		{
			if (mesh.is_valid_handle(*vf))
				continue;
			auto fv = mesh.cfv_ccwbegin(*vf);
			auto& p0 = mesh.point(*fv); ++fv;
			auto& p1 = mesh.point(*fv); ++fv;
			auto& p2 = mesh.point(*fv); 
			vn += (p1 - p0).cross(p2 - p0);
		}
		vn.normalize_cond();
		return vn;
	}
}
