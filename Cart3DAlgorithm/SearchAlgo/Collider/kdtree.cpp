
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable:4273)
#pragma warning(disable:4251)
#pragma warning(disable:26495)
#pragma warning(disable:28020)
#pragma warning(disable:4267)
#pragma warning(disable:4286)
#endif
#include <Cart3DAlgorithm/SearchAlgo/kdtree.h>
#include <nanoflann.hpp>
namespace Cart3DAlgorithm
{
	struct kdtree::Cache
	{
		using my_kd_tree_t = nanoflann::KDTreeSingleIndexAdaptor<
			nanoflann::L2_Simple_Adaptor<cfloat, kdtree>,
			kdtree,3,size_t>;
		std::unique_ptr<my_kd_tree_t> ptree;

		Cache(kdtree& kdt):ptree(std::make_unique<my_kd_tree_t>(3,kdt))
		{
			ptree->buildIndex();
		}

	};

	kdtree::kdtree(const std::vector<cvector3d>& _pts):
		pts(_pts),pTree(std::make_shared<Cache>(*this))
	{
		
	}

	void kdtree::update_tree()
	{
		pTree = std::make_shared<Cache>(*this);
	}

	size_t kdtree::kdtree_get_point_count() const { return pts.size(); }

	cfloat kdtree::kdtree_get_pt(const size_t idx, const size_t dim) const
	{
		return pts[idx][dim];
	}

	size_t kdtree::knn_search(
		const cvector3d& point, size_t num_closest,
		std::vector<size_t>& k_indices,
		std::vector<cfloat>& k_squared_distances, bool sort)const
	{
		k_indices.clear();
		k_squared_distances.clear();
		k_indices.resize(num_closest);
		k_squared_distances.resize(num_closest);
		nanoflann::KNNResultSet<cfloat> resultSet(num_closest);
		resultSet.init(k_indices.data(), k_squared_distances.data());
		nanoflann::SearchParams params(32,0,sort);
		pTree->ptree->findNeighbors(resultSet,point.data(), params);
		return resultSet.size();
	}

	size_t kdtree::sqr_radius_search(const cvector3d& point, cfloat radius,std::vector<size_t>& k_indices, bool sort)const
	{
		std::vector<std::pair<size_t, cfloat>> indices_dist;
		indices_dist.reserve(128);
		nanoflann::RadiusResultSet<cfloat> resultSet(radius,indices_dist);
		nanoflann::SearchParams params(32, 0, sort);
		pTree->ptree->findNeighbors(resultSet, point.data(), params);
		const size_t nFound = resultSet.size();
		if (params.sorted) {
			std::sort(indices_dist.begin(), indices_dist.end(),nanoflann::IndexDist_Sorter());
		}
		k_indices.resize(nFound);
		for (size_t i = 0; i < nFound; ++i) {
			k_indices[i] = indices_dist[i].first;
		}
		return nFound;
	}

	std::pair<size_t, cfloat> kdtree::knn_search(const cvector3d& point)const
	{
		std::pair<std::size_t, cfloat> res;
		pTree->ptree->knnSearch(point.data(), 1,&res.first, &res.second);
		return res;
	}





	struct  kdtree2d::Cache
	{
		using my_kd_tree_t = nanoflann::KDTreeSingleIndexAdaptor<
			nanoflann::L2_Simple_Adaptor<cfloat, kdtree2d>,kdtree2d,2,size_t>;
		std::unique_ptr<my_kd_tree_t> ptree;

		Cache(kdtree2d& kdt) :ptree(std::make_unique<my_kd_tree_t>(2, kdt))
		{
			ptree->buildIndex();
		}

	};

	kdtree2d::kdtree2d(const std::vector<cvector2d>& _pts) :
		pts(_pts), pTree(std::make_shared<Cache>(*this))
	{

	}

	void kdtree2d::update_tree()
	{
		pTree = std::make_shared<Cache>(*this);
	}

	size_t kdtree2d::kdtree_get_point_count() const { return pts.size(); }

	cfloat kdtree2d::kdtree_get_pt(const size_t idx, const size_t dim) const
	{
		return pts[idx][dim];
	}

	size_t kdtree2d::knn_search(
		const cvector2d& point, size_t num_closest,
		std::vector<size_t>& k_indices,
		std::vector<cfloat>& k_squared_distances, bool sort)const
	{
		k_indices.clear();
		k_squared_distances.clear();
		k_indices.resize(num_closest);
		k_squared_distances.resize(num_closest);
		nanoflann::KNNResultSet<cfloat> resultSet(num_closest);
		resultSet.init(k_indices.data(), k_squared_distances.data());
		nanoflann::SearchParams params(32, 0, sort);
		pTree->ptree->findNeighbors(resultSet, point.data(), params);
		return resultSet.size();
	}

	size_t kdtree2d::sqr_radius_search(const cvector2d& point, cfloat radius, std::vector<size_t>& k_indices, bool sort)const
	{
		std::vector<std::pair<size_t, cfloat>> indices_dist;
		indices_dist.reserve(128);
		nanoflann::RadiusResultSet<cfloat> resultSet(radius, indices_dist);
		nanoflann::SearchParams params(32, 0, sort);
		pTree->ptree->findNeighbors(resultSet, point.data(), params);
		const size_t nFound = resultSet.size();
		if (params.sorted) {
			std::sort(indices_dist.begin(), indices_dist.end(), nanoflann::IndexDist_Sorter());
		}
		k_indices.resize(nFound);
		for (size_t i = 0; i < nFound; ++i) {
			k_indices[i] = indices_dist[i].first;
		}
		return nFound;
	}

	std::pair<size_t, cfloat> kdtree2d::knn_search(const cvector2d& point)const
	{
		std::pair<size_t, cfloat> res;
		pTree->ptree->knnSearch(point.data(), 1, &res.first, &res.second);
		return res;
	}


}

#if defined(_MSC_VER)
#pragma warning(pop)
#endif

