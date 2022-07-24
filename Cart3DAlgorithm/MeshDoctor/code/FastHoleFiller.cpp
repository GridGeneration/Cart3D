#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable:4273)
#pragma warning(disable:4251)
#pragma warning(disable:4286)
#pragma warning(disable:4267)
#endif



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
		inline static OpenTriMesh::Scalar sqr(OpenTriMesh::Scalar a)
		{
			return a * a;
		}

		struct Weight
		{
			int v1, v2, v3;
			OpenTriMesh::Scalar quality;
			Weight(
				const int& _v1,
				const int& _v2,
				const int& _v3, OpenTriMesh::Scalar _q);
			bool operator < (const Weight& rhs) const;
		};

		Weight::Weight(
			const int& _v1,
			const int& _v2,
			const int& _v3, OpenTriMesh::Scalar _q) :
			v1(_v1), v2(_v2), v3(_v3), quality(_q)
		{}
		bool Weight::operator < (const Weight& rhs) const
		{
			return quality < rhs.quality;
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
				if (inser_points[vh0.idx()].find(vh2.idx()) == inser_points[vh0.idx()].end())
				{
					OpenTriMesh::Scalar qual = compute_weight(mesh,
						vnorms[i0], vnorms[i1], vnorms[i2],
						vh0, vh1, vh2, mean_edge, true);
					tQ.push(Weight(i0, i1, i2, qual));
				}
				else
				{
					tQ.push(Weight(i0, i1, i2, -1000));
				}
			}
			std::vector<bool> iscall(nhole, false);
			int num_hole = nhole;
			while (!tQ.empty())
			{
				const auto& node = tQ.top();
				tQ.pop();
				if (!iscall[node.v1] && !iscall[node.v3])
				{
					iscall[node.v2] = true;
					auto fh = mesh.add_face(hole[node.v1], hole[node.v2], hole[node.v3]);
					if (mesh.is_valid_handle(fh))
					{
						new_faces.push_back(fh);
					}
					else
					{
						continue;
					}
					//去掉耳点，更新耳点周围的点
					int forw = node.v3;
					do {
						++forw;
						forw %= nhole;
					} while (iscall[forw]);
					if (forw == node.v1)
						break;
					int back = node.v1;
					do {
						--back;
						if (back < 0) 
							back += nhole;
						back %= nhole;
					} while (iscall[back]);

					//更新前沿补洞权重
					OpenTriMesh::Scalar q13f = compute_weight(mesh, 
						vnorms[node.v1], vnorms[node.v3], vnorms[forw],
						hole[node.v1], hole[node.v3], hole[forw],
						mean_edge,true);
					if (inser_points[hole[node.v1].idx()].find(hole[forw].idx()) !=
						inser_points[hole[node.v1].idx()].end())
						q13f = -2000.0f;
					OpenTriMesh::Scalar qb13 = compute_weight(mesh,
						vnorms[back], vnorms[node.v1], vnorms[node.v3],
						hole[back], hole[node.v1], hole[node.v3],
						mean_edge, true);

					if (inser_points[hole[back].idx()].find(hole[node.v3].idx()) !=
						inser_points[hole[back].idx()].end())
						qb13 = -2000.0f;
					tQ.push(Weight(node.v1, node.v3, forw, q13f));
					tQ.push(Weight(back, node.v1, node.v3, qb13));
				}
			}
		    if(num_hole>3)
			{
				return false;
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
			if (!mesh.is_valid_handle(*vf))
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


#if defined(_MSC_VER)
#pragma warning(pop)
#endif