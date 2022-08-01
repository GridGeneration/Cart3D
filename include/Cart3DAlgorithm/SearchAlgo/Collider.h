#ifndef CART3DALGORITHM_COLLIDER_H
#define CART3DALGORITHM_COLLIDER_H
#include <vector>
#include <unordered_map>
#include "SearchAlgoConfig.h"
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
		DistTool():
			dist(maxcfloat),
			findex(0),
			tex(0,0),
			closet_p(0,0,0)
		{}
	};

	

	class SEARCHALGO_API Collider
	{
	public:
		struct SEARCHALGO_API Cache;
		struct SEARCHALGO_API BV_box;
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
		static std::shared_ptr<BV_box> creat_bv_node(
			const cmatrix3d&rot,
			const std::vector<cvector3d>& pts);
	public:
		bool query_bv_intersecion(
			std::shared_ptr<BV_box> bv, const cmatrix4d& rt0, 
			int id,const cmatrix4d& rt1,std::vector<int>& res);
	public:
		bool query_undist_model(const cvector3d& pt, int id, DistTool& dtl)const;
	public:
		bool query_unsigned_dist_model(
			int id0, const cmatrix4d& rt0,
			int id1, const cmatrix4d& rt1,
			std::tuple<cfloat,cvector3d,cvector3d>& res)const;
	private:
		std::unordered_map<int, std::shared_ptr<Cache>> collider;
	};



}

#endif