#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable:4273)
#pragma warning(disable:4251)
#pragma warning(disable:4286)
#endif
#include "LoadOpenMesh.h"
#include <Cart3DAlgorithm/MeshDoctor/MeshBoundaryExtractor.h>
#include <unordered_set>

namespace Cart3DAlgorithm
{
	bool MeshBoundaryExtractor::ExtraBoundary(const OpenTriMesh& mesh, const HalfedgeHandle& hh, std::vector<VertexHandle>& boundary)
	{
		std::vector<HalfedgeHandle> hhs;
		if (!ExtraBoundary(mesh, hh, hhs)) {
			return false;
		}
		boundary.clear();
		for (auto& ip : hhs) {
			boundary.emplace_back(mesh.from_vertex_handle(ip));
		}
		return true;
	}

	bool MeshBoundaryExtractor::ExtraBoundary(const OpenTriMesh& mesh, const HalfedgeHandle& hh, std::vector<HalfedgeHandle>& boundary)
	{
		if (!mesh.is_valid_handle(hh))
			return false;
		if (!mesh.is_boundary(hh))
			return false;
		HalfedgeHandle ch = hh;
		boundary.clear();
		do {
			if (!mesh.is_valid_handle(ch)) {
				return false;
			}
			boundary.push_back(ch);
			int c = 0;
			VertexHandle vh = mesh.to_vertex_handle(ch);
			for (OpenTriMesh::VertexOHalfedgeIter voh_it(mesh, vh); voh_it.is_valid(); ++voh_it)
				if (mesh.is_boundary(*voh_it))
					++c;
			if (c >= 2) {
				HalfedgeHandle  op = mesh.opposite_halfedge_handle(ch);
				OpenTriMesh::VertexOHalfedgeIter voh_it(mesh, op);
				ch = *(++voh_it);
			}
			else
				ch = mesh.next_halfedge_handle(ch);
		} while (ch != hh);
		return boundary.size() > 0;
	}


	bool MeshBoundaryExtractor::ExtrAllBoundary(const OpenTriMesh& mesh, std::vector<HalfedgeHandle>& bounary)
	{
		bounary.clear();
		std::unordered_set<int> iscall;
		for (auto hh : mesh.halfedges()) {
			if (!mesh.is_boundary(hh) || iscall.count(hh.idx()) != 0 || !mesh.is_valid_handle(hh)) {
				continue;
			}
			HalfedgeHandle ch = hh;
			do {
				if (!mesh.is_valid_handle(ch))
					break;
				int c = 0;
				VertexHandle vh = mesh.to_vertex_handle(ch);
				for (OpenTriMesh::VertexOHalfedgeIter voh_it(mesh, vh); voh_it.is_valid(); ++voh_it)
					if (mesh.is_boundary(*voh_it))
						++c;
				if (c >= 2) {
					HalfedgeHandle  op = mesh.opposite_halfedge_handle(ch);
					OpenTriMesh::VertexOHalfedgeIter voh_it(mesh, op);
					ch = *(++voh_it);
				}
				else
					ch = mesh.next_halfedge_handle(ch);
				iscall.insert(ch.idx());
			} while (ch != hh);
			bounary.push_back(hh);
		}
		return bounary.size() > 0;
	}


	bool MeshBoundaryExtractor::ExtraMaxBounary(const OpenTriMesh& mesh, HalfedgeHandle& hh)
	{
		std::vector<HalfedgeHandle> bounary;
		ExtrAllBoundary(mesh, bounary);
		if (bounary.size() == 0)
			return false;
		size_t numVhs = 0;
		int nB = static_cast<int>(bounary.size());

		for (int i = 0; i < nB; ++i) {

			std::vector<VertexHandle> vhs;
			ExtraBoundary(mesh, bounary[i], vhs);
			if (numVhs < vhs.size()) {
				numVhs = vhs.size();
				hh = bounary[i];
			}

		}
		return true;
	}
}


#if defined(_MSC_VER)
#pragma warning(pop)
#endif