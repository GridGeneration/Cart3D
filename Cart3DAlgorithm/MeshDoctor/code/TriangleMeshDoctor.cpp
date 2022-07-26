#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable:4273)
#pragma warning(disable:4251)
#pragma warning(disable:4286)
#endif
#include "LoadOpenMesh.h"
#include "MeshDoctor/TriangleMeshDoctor.h"
#include "MeshDoctor/FastHoleFiller.h"
#include "MeshDoctor/MeshBoundaryExtractor.h"
#include <stack>
namespace Cart3DAlgorithm
{
    namespace
    {
        bool collapse_flips_normal(OpenTriMesh& in_mesh, const HalfedgeHandle& _heh)
        {
            if (!in_mesh.is_collapse_ok(_heh))
                return true;
            auto fh0 = in_mesh.face_handle(_heh);
            auto fh1 = in_mesh.face_handle(in_mesh.opposite_halfedge_handle(_heh));
            auto collapsing_vertex = in_mesh.from_vertex_handle(_heh);
            auto point_before = in_mesh.point(collapsing_vertex);
            std::vector<decltype (in_mesh.calc_face_normal(OpenMesh::FaceHandle(0)))> normals_before;
            for (auto fh : in_mesh.vf_range(collapsing_vertex))
                normals_before.push_back(in_mesh.calc_face_normal(fh));
            in_mesh.point(collapsing_vertex) = in_mesh.point(in_mesh.to_vertex_handle(_heh));
            bool collapse_ok = true;
            int i = 0;
            for (auto fh : in_mesh.vf_range(collapsing_vertex))
            {
                if (fh != fh0 && fh != fh1) 
                {
                    auto normal_after = in_mesh.calc_face_normal(fh);
                    if ((normal_after | normals_before[i]) <= 0)
                        collapse_ok = false;
                }
                ++i;
            }
            in_mesh.point(collapsing_vertex) = point_before;
            return !collapse_ok;
        }

        struct AttributeAlocateGuard
        {
            OpenTriMesh& otm;
            AttributeAlocateGuard(OpenTriMesh& _otm) :otm(_otm) {
                start_presolve_mesh(otm);
            }
            ~AttributeAlocateGuard() {
                end_presolve_mesh(otm);
            }

            static void start_presolve_mesh(OpenTriMesh& in_mesh)
            {
                in_mesh.request_face_normals();
                in_mesh.request_vertex_normals();
                in_mesh.update_face_normals();
                in_mesh.update_vertex_normals();
                if (!in_mesh.has_vertex_status())
                {
                    in_mesh.request_vertex_status();
                    for (auto& iv : in_mesh.vertices())
                    {
                        in_mesh.status(iv).set_feature(false);
                        in_mesh.status(iv).set_locked(false);
                    }
                }
                if (!in_mesh.has_face_status())
                {
                    in_mesh.request_face_status();
                }
                if (!in_mesh.has_edge_status())
                {
                    in_mesh.request_edge_status();
                }
            }

            static void end_presolve_mesh(OpenTriMesh& in_mesh)
            {
                in_mesh.release_vertex_status();
                in_mesh.release_edge_status();
                in_mesh.release_face_status();
                in_mesh.release_vertex_normals();
                in_mesh.release_face_normals();
            }
        };

    }


	MeshDoctorParam::MeshDoctorParam():
		is_fix_selfintersection(true),
		is_join_points_eps(true),
		is_join_edges_eps(true),
		is_fill_hole(true),
		is_refine_hole(true),
		is_fair_hole(true),
		join_points_eps(0.02),
		join_edge_eps(0.02),
		fill_max_hole(500)
	{

	}

	bool TriangleMeshDoctor::auto_fix_mesh(const OpenTriMesh& in_mesh, OpenTriMesh& out_mesh)
	{
        out_mesh.assign(in_mesh, true);
        AttributeAlocateGuard guard(out_mesh);




        
		return true;
	}


	int TriangleMeshDoctor::mark_part(const OpenTriMesh& in_mesh, std::vector<int>& parts)
	{
		parts.clear();
		int nvert = static_cast<int>(in_mesh.n_vertices());
		int part_id = -1;
		if (nvert == 0)
			return part_id;
		parts.resize(nvert, part_id);
		for (auto iv:in_mesh.vertices())
		{
			if (!in_mesh.is_valid_handle(iv))
				continue;
			if (-1 != parts[iv.idx()])
				continue;
			++part_id;
			std::stack<VertexHandle> cache_vhs;
			cache_vhs.push(iv);
			while (!cache_vhs.empty())
			{
				VertexHandle tv = cache_vhs.top();
				cache_vhs.pop();
				parts[tv.idx()] = part_id;
				for (auto iter = in_mesh.cvv_begin(tv),
					iter_end = in_mesh.cvv_end(tv);
					iter != iter_end; ++iter)
				{
					if (!in_mesh.is_valid_handle(*iter))
						continue;
					if (-1 != parts[iter->idx()])
						continue;
					cache_vhs.push(*iter);
				}
			} 
		}
		return part_id;
	}

	bool TriangleMeshDoctor::join_point(OpenTriMesh& in_mesh, cfloat eps)
	{
		int nvert = static_cast<int>(in_mesh.n_vertices());
		if (nvert == 0)
			return false;
       
        const cfloat eps2 = eps * eps;
        auto is_too_short = [&](const VertexHandle&vha, const VertexHandle& vhb) {
            return (in_mesh.point(vha) - in_mesh.point(vhb)).sqrnorm() < eps2;
        };
        for (int ok = false, i = 0; !ok && i < 100; ++i) {
            ok = true;
            for (auto e_it = in_mesh.edges_begin(), e_end = in_mesh.edges_end(); e_it != e_end; ++e_it) {
                if (!in_mesh.status(*e_it).deleted() && !in_mesh.status(*e_it).locked()) {
                    auto h10 = in_mesh.halfedge_handle(*e_it, 0);
                    auto h01 = in_mesh.halfedge_handle(*e_it, 1);
                    auto v0 = in_mesh.to_vertex_handle(h10);
                    auto v1 = in_mesh.to_vertex_handle(h01);
                    if (is_too_short(v0, v1)) {
                       auto  b0 = in_mesh.is_boundary(v0);
                       auto  b1 = in_mesh.is_boundary(v1);
                       auto  l0 = in_mesh.status(v0).locked();
                       auto  l1 = in_mesh.status(v1).locked();
                       auto  f0 = in_mesh.status(v0).feature();
                       auto  f1 = in_mesh.status(v1).feature();
                       auto  hcol01 = true;
                       auto hcol10 = true;
                       if (b0 && b1) {
                            if (!in_mesh.is_boundary(*e_it))
                                continue;
                        }
                        else if (b0)
                            hcol01 = false;
                        else if (b1)
                            hcol10 = false;
                        if (l0 && l1)
                            continue;
                        else if (l0)
                            hcol01 = false;
                        else if (l1)
                            hcol10 = false;
                        if (f0 && f1)
                            continue;
                        else if (f0)
                            hcol01 = false;
                        else if (f1)
                            hcol10 = false;
                        {
                            int feature_valence_0 = 0;
                            for (auto eh : in_mesh.ve_range(v0))
                                if (in_mesh.status(eh).feature())
                                    ++feature_valence_0;
                            if (feature_valence_0 == 2)
                            {
                                if (!in_mesh.status(*e_it).feature())
                                    hcol01 = false;
                            }
                            else if (feature_valence_0 != 0)
                                hcol01 = false;
                        }
                        {
                            int feature_valence_1 = 0;
                            for (auto eh : in_mesh.ve_range(v1))
                                if (in_mesh.status(eh).feature())
                                    ++feature_valence_1;
                            if (feature_valence_1 == 2)
                            {
                                if (!in_mesh.status(*e_it).feature())
                                    hcol10 = false;
                            }
                            else if (feature_valence_1 != 0)
                                hcol10 = false;
                        }
                        auto h0 = in_mesh.prev_halfedge_handle(h01);
                        auto h1 = in_mesh.next_halfedge_handle(h10);
                        if (in_mesh.status(in_mesh.edge_handle(h0)).feature() || 
                            in_mesh.status(in_mesh.edge_handle(h1)).feature())
                            hcol01 = false;
                        h0 = in_mesh.prev_halfedge_handle(h10);
                        h1 = in_mesh.next_halfedge_handle(h01);
                        if (in_mesh.status(in_mesh.edge_handle(h0)).feature() || 
                            in_mesh.status(in_mesh.edge_handle(h1)).feature())
                            hcol10 = false;
                        if (hcol01)
                            hcol01 = in_mesh.is_collapse_ok(h01);
                        if (hcol10)
                            hcol10 = in_mesh.is_collapse_ok(h10);
                        if (hcol01 && hcol10) {
                            if (in_mesh.valence(v0) < in_mesh.valence(v1))
                                hcol10 = false;
                            else
                                hcol01 = false;
                        }
                        if (collapse_flips_normal(in_mesh,h01))
                            hcol01 = false;
                        if (collapse_flips_normal(in_mesh,h10))
                            hcol10 = false;
                        if (hcol10) {
                            in_mesh.collapse(h10);
                        }
                        else if (hcol01) {
                            in_mesh.collapse(h01);
                        }
                    }
                }
            }
        }
		in_mesh.garbage_collection();
		
		return true;
	}

    bool TriangleMeshDoctor::trunc_mesh(OpenTriMesh& in_mesh)
    {
        if (in_mesh.n_vertices() < 3)
            return false;
        for (auto& iv : in_mesh.vertices())
        {
            auto& p = in_mesh.point(iv);
            p[0] = std::trunc(p[0] * 100000) / 100000;
            p[1] = std::trunc(p[1] * 100000) / 100000;
            p[2] = std::trunc(p[2] * 100000) / 100000;
        }
        return true;
    }

   

    bool TriangleMeshDoctor::delete_small_part(OpenTriMesh& in_mesh, int max_nvert)
    {
        std::vector<int> parts;
        int ncomp = 0;
        if (-1 == (ncomp = mark_part(in_mesh, parts)))
            return false;
        std::vector<int> count(ncomp + 1, 0);
        for (auto& ip : parts)
        {
            ++count[ip];
        }
        for (auto& iv : in_mesh.vertices())
        {
            if (count[parts.at(iv.idx())] < max_nvert)
                in_mesh.delete_vertex(iv);
        }
        in_mesh.garbage_collection();
        return true;
    }



    bool TriangleMeshDoctor::fill_small_hole(OpenTriMesh& in_mesh, int max_hole)
    {
        std::vector<HalfedgeHandle> hhs;
        MeshBoundaryExtractor::ExtrAllBoundary(in_mesh, hhs);
        for (auto& h : hhs)
        {
            std::vector<VertexHandle> vhs;
            MeshBoundaryExtractor::ExtraBoundary(in_mesh, h, vhs);
            if (vhs.size() < max_hole)
            {
                FastHoleFiller::fix_hole(in_mesh, vhs);
            }
        }
        return true;
    }


}



#if defined(_MSC_VER)
#pragma warning(pop)
#endif