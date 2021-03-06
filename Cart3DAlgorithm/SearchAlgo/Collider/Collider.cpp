
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable:4273)
#pragma warning(disable:4251)
#pragma warning(disable:4286)
#pragma warning(disable:4286)
#pragma warning(disable:26495)
#pragma warning(disable:4101)
#endif
#include <SearchAlgo/Collider.h>
#include "pqpimpl/PQP_Compile.h"
#include "pqpimpl/PQP_Internal.h"
#include "pqpimpl/PQP.h"

namespace Cart3DAlgorithm
{

	namespace
	{
		inline void swap_rt(const cmatrix4d& rt,PQP_REAL rot[3][3], PQP_REAL trans[3])
		{
			for (int i = 0; i < 3; ++i)
			{
				for (int j = 0; j < 3; ++j)
				{
					rot[i][j] = rt(i, j);
				}
				trans[i] = rt(i, 3);
			}
		}
		inline void swap_rt(const cmatrix3d& rt, PQP_REAL rot[3][3])
		{
			for (int i = 0; i < 3; ++i)
			{
				for (int j = 0; j < 3; ++j)
				{
					rot[i][j] = rt(i, j);
				}
			}
		}
	}

	struct Collider::Cache
	{
		std::unique_ptr<PQP_Model> pTree;
		Cache() :
			pTree(std::make_unique<PQP_Model>())
		{}
	};
	struct Collider::BV_box
	{
		std::shared_ptr<BV> pBox;
		BV_box() :pBox(std::make_shared<BV>()) {}
	};

	bool Collider::release_model(int idmodel)
	{
		if (collider.find(idmodel) == collider.end())
			return false;
		collider.erase(idmodel);
		return true;
	}

	bool Collider::start_build_model(int idmodel, int numtris)
	{
		if (collider.find(idmodel) != collider.end())
			return false;
		collider[idmodel] = std::make_shared<Cache>();
		collider[idmodel]->pTree->BeginModel(numtris);
		return true;
	}

	bool Collider::add_tris(int idmodel, int idtris,
		const cvector3d& v0,
		const cvector3d& v1,
		const cvector3d& v2)
	{
		auto iter = collider.find(idmodel);
		if (iter == collider.end()||iter->second==nullptr)
		{
			return false;
		}
		auto& tree = iter->second->pTree;
		if (0!= tree->AddTri(v0.data(), v1.data(), v2.data(), idtris))
		{
			return false;
		}
		return true;
	}

	bool Collider::end_build_model(int idmodel)
	{
		auto iter = collider.find(idmodel);
		if (iter == collider.end()||iter->second==nullptr)
		{
			return false;
		}
		if (0!=iter->second->pTree->EndModel())
		{
			return false;
		}
		return true;
	}

	bool Collider::is_collider(int id0, const cmatrix4d& rt0, int id1, const cmatrix4d& rt1)const
	{
		auto iter0 = collider.find(id0);
		if (iter0 == collider.end())
			return false;
		auto iter1 = collider.find(id1);
		if (iter1 == collider.end())
			return false;
		PQP_REAL rot0[3][3],rot1[3][3];
		PQP_REAL trans0[3], trans1[3];
		swap_rt(rt0, rot0, trans0);
		swap_rt(rt1, rot1, trans1);
	
		PQP_CollideResult res;
		PQP_Collide(&res, 
			rot0, trans0, iter0->second->pTree.get(), 
			rot1, trans1, iter1->second->pTree.get(),
			PQP_FIRST_CONTACT);
		int num_int=res.num_pairs;
		res.FreePairsList();
		return num_int != 0;
	}

	bool Collider::query_collider_pairs(
		int id0, const cmatrix4d& rt0,
		int id1, const cmatrix4d& rt1,
		std::vector<std::pair<int, int>>& int_pairs)const
	{
		int_pairs.clear();
		auto iter0 = collider.find(id0);
		if (iter0 == collider.end())
			return false;
		auto iter1 = collider.find(id1);
		if (iter1 == collider.end())
			return false;
		PQP_REAL rot0[3][3], rot1[3][3];
		PQP_REAL trans0[3], trans1[3];
		swap_rt(rt0, rot0, trans0);
		swap_rt(rt1, rot1, trans1);
		PQP_CollideResult res;
		PQP_Collide(&res,
			rot0, trans0, iter0->second->pTree.get(),
			rot1, trans1, iter1->second->pTree.get(),
			PQP_ALL_CONTACTS);
		int_pairs.reserve(res.num_pairs);
		for (int i = 0; i < res.num_pairs; ++i)
			int_pairs.emplace_back(std::make_pair(res.pairs[i].id1, res.pairs[i].id2));
		res.FreePairsList();
		int_pairs.shrink_to_fit();
		return !int_pairs.empty();
	}

	std::shared_ptr<Collider::BV_box> Collider::creat_bv_node(
		const cmatrix3d& rot,
		const std::vector<cvector3d>& pts)
	{
		std::shared_ptr<Collider::BV_box> pc = std::make_shared<Collider::BV_box>();
		auto& pv = pc->pBox;
		PQP_REAL mat[3][3];
		swap_rt(rot, mat);
		pv->FitToPts(mat, pts);
		return pc;
	}

	bool Collider::query_bv_intersecion(
		std::shared_ptr<BV_box> bv, const cmatrix4d& rt0,
		int id, const cmatrix4d& rt1,std::vector<int>& res)
	{
		if (bv == nullptr)
			return false;
		auto iter = collider.find(id);
		if (iter == collider.end())
			return false;
		if (iter->second == nullptr)
			return false;
	
		PQP_REAL rot0[3][3], rot1[3][3];
		PQP_REAL trans0[3], trans1[3];
		swap_rt(rt0, rot0, trans0);
		swap_rt(rt1, rot1, trans1);
		PQP_CollideResult rest;
		PQP_Collide(&rest,
			rot0, trans0, bv->pBox.get(),
			rot1, trans1, iter->second->pTree.get(), 
			PQP_ALL_CONTACTS);
		res.clear();
		res.reserve(rest.num_pairs);
		for (int i = 0; i < rest.num_pairs; ++i)
		{
			res.push_back(rest.pairs[i].id2);
		}
		rest.FreePairsList();
		return !res.empty();
	}

	bool Collider::query_undist_model(const cvector3d& pt, int id, DistTool& dtl)const
	{
		auto iter = collider.find(id);
		if (iter == collider.end())
			return false;
		if (iter->second == nullptr)
			return false;
		dtl.dist = PQP_Shortest_Dist(
			const_cast<cfloat*>(pt.data()),
			iter->second->pTree.get(),
			dtl.closet_p.data(),
			dtl.findex, 
			dtl.tex.data());
		return true;
	}

	bool Collider::query_unsigned_dist_model(
		int id0, const cmatrix4d& rt0,
		int id1, const cmatrix4d& rt1,
		std::tuple<cfloat, cvector3d, cvector3d>& res)const
	{
		auto iter0 = collider.find(id0);
		if (iter0 == collider.end())
			return false;
		auto iter1 = collider.find(id1);
		if (iter1 == collider.end())
			return false;
		PQP_REAL rot0[3][3], rot1[3][3];
		PQP_REAL trans0[3], trans1[3];
		swap_rt(rt0, rot0, trans0);
		swap_rt(rt1, rot1, trans1);
		PQP_REAL rel_err=0.000001;
		PQP_REAL abs_err=0.000001;
		PQP_DistanceResult pdr;
		PQP_Distance(&pdr,
			rot0, trans0, iter0->second->pTree.get(),
			rot1, trans0, iter1->second->pTree.get(),
			rel_err, abs_err, 2);
		res = std::make_tuple(pdr.distance,
			cvector3d(pdr.p1[0], pdr.p1[1], pdr.p1[2]),
			cvector3d(pdr.p2[0], pdr.p2[1], pdr.p2[2]));
		return true;
	}

}

#if defined(_MSC_VER)
#pragma warning(pop)
#endif