#pragma once
#include <vector>
#include <unordered_map>
#include "SearchAlgoConfig.h"
#include <Cart3DAlgorithm/Common/util.h>
#include <memory>
#include <tuple>
namespace Cart3DAlgorithm
{
	struct SEARCHALGO_API DistTool
	{
		cvector3d closet_p;
		cvector2d tex;
		int findex;
		cfloat dist;
	};

	class SEARCHALGO_API Collider
	{
	public:
		struct Cache;
	public:
		bool release_model(int idmodel);
		bool start_build_model(int idmodel,int numtris=8);
		bool add_tris(int idmodel,int idtris,
			const cvector3d& v0, 
			const cvector3d& v1,
			const cvector3d& v2);
		bool end_build_model(int idmodel);
	public:
		bool is_collider(int id0,const cmatrix4d& rt0,int id1,const cmatrix4d& rt1)const;
		bool query_collider_pairs(
			int id0, const cmatrix4d& rt0,
			int id1, const cmatrix4d& rt1,
			std::vector<std::pair<int, int>>& int_pairs)const;
	public:
		bool query_dist_model(const cvector3d& pt, int id, DistTool& dtl)const;
	public:
		bool query_unsigned_dist_model(
			int id0, const cmatrix4d& rt0,
			int id1, const cmatrix4d& rt1,
			std::tuple<cfloat,cvector3d,cvector3d>& res)const;
	private:
		std::unordered_map<int, std::shared_ptr<Cache>> collider;
	};



}