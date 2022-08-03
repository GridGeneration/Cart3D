#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable:4273)
#pragma warning(disable:4251)
#pragma warning(disable:4286)
#endif
#include "MeshDoctor/RobustTriTriIntersection.h"
#include <MeshDoctor/RemoveSelfIntersection.h>
#include <unordered_map>
#include <set>
#include <SearchAlgo/Bvh.h>
#include <LoadTbb.h>
namespace Cart3DAlgorithm
{
	namespace
	{
		static OpenMesh::VPropHandleT<cvector3d> CTP;
		struct TruncMesh
		{
			TruncMesh(OpenTriMesh& _mesh,int _num=10000):
				num(_num),mesh(_mesh)
			{
				mesh.add_property(CTP);
				for (auto iv : mesh.vertices())
				{
					auto& p = mesh.point(iv);
					cvector3d cp;
					cp[0] = std::trunc(p[0] * num);
					cp[1] = std::trunc(p[1] * num);
					cp[2] = std::trunc(p[2] * num);
					mesh.property(CTP, iv) = cp;
				}
			}
			~TruncMesh()
			{
				for (auto iv : mesh.vertices())
				{
					auto& p = mesh.property(CTP, iv);
					p /= num;
				}
				mesh.remove_property(CTP);
			}
			int num;
			OpenTriMesh& mesh;
			static cfloat eps;
		};

		struct point {
			cfloat x;
			cfloat y;
			int id;
			point(cfloat xIn=0, cfloat yIn=0,int _id=-1) : x(xIn), y(yIn),id(_id) { }
		};
		static cfloat ccw(const point& a, const point& b, const point& c) {
			return (b.x - a.x) * (c.y - a.y) - (b.y - a.y) * (c.x - a.x);
		}
		static bool isLeftOf(const point& a, const point& b) {
			return (a.x < b.x || (a.x == b.x && a.y < b.y));
		}
		struct ccwSorter {
			const point& pivot;
			ccwSorter(const point& inPivot) : pivot(inPivot) { }
			bool operator()(const point& a, const point& b) {
				return ccw(pivot, a, b) < 0;
			}
		};
		std::vector<point> GrahamScan(std::vector<point>& v) {
			std::swap(v[0], *std::min_element(v.begin(), v.end(), isLeftOf));
			std::sort(v.begin() + 1, v.end(), ccwSorter(v[0]));
			std::vector<point> hull;
			auto it = v.begin();
			hull.push_back(*it++);
			hull.push_back(*it++);
			hull.push_back(*it++);
			while (it != v.end()) {
				while (ccw(*(hull.rbegin() + 1), *(hull.rbegin()), *it) >= 0) {
					hull.pop_back();
				}
				hull.push_back(*it++);
			}
			return hull;
		}

		static bool gramScan(
			const cvector3d& pn,
			const std::vector<RemoveSelfIntersection::BooleanPoint*>& pts, 
			std::vector<int>& hull)
		{
			std::vector<point> cpts;
			int ic = -1;
			if (std::abs(pn[0]) < inv_trunc_val)
			{
				for (auto& ip : pts) {
					point p(ip->data.y(), ip->data.z(), ++ic);
					cpts.emplace_back(p);
				}
			}
			else if (std::abs(pn[1]) < inv_trunc_val)
			{
				for (auto& ip : pts) {
					point p(ip->data.x(), ip->data.z(), ++ic);
					cpts.emplace_back(p);
				}
			}
			else if (std::abs(pn[2]) < inv_trunc_val)
			{
				for (auto& ip : pts) {
					point p(ip->data.x(), ip->data.y(), ++ic);
					cpts.emplace_back(p);
				}
			}
			else
				return false;
			std::vector<point> hu = GrahamScan(cpts);
			hull.clear();
			for (auto& ip : hu)
				hull.emplace_back(ip.id);
			return true;
		}

	}

	cfloat& RemoveSelfIntersection::BooleanPoint::operator[](int i)
	{
		return data[i];
	}

	const cfloat& RemoveSelfIntersection::BooleanPoint::
		operator[](int i)const
	{
		return data[i];
	}

	bool RemoveSelfIntersection::BooleanPoint::
		operator == (const BooleanPoint& c) const
	{
		return (c[0] == data[0]) && (c[1] == data[1]) && (c[2] == data[2]);
	}

	bool RemoveSelfIntersection::BooleanPoint::
		 operator != (const BooleanPoint& c) const
	{
		return (c[0] != data[0]) || (c[1] != data[1]) || (c[2] != data[2]);
	}

	bool RemoveSelfIntersection::BooleanPoint::
		operator < (const BooleanPoint& c) const
	{
		return (data[0] < c[0])
			|| (data[0] == c[0] && data[1] < c[1])
			|| (data[0] == c[0] && data[1] == c[1] && data[2] < c[2]);
	}

	RemoveSelfIntersection::RemoveSelfIntersection(OpenTriMesh& in_mesh):mesh(in_mesh){}

	bool RemoveSelfIntersection::run()
	{
		TruncMesh trunc_mesh(mesh);
		std::vector<std::vector<int>> int_pairs;
		if (!find_int_pairs(int_pairs))
			return false;
		vv_BPoint int_pts;
		vv_BEdge int_edges;
		if (!find_int_pairs(int_pairs, int_pts, int_edges))
			return false;


		return true;
	}

	bool RemoveSelfIntersection::find_int_pairs(vv_inter& maybe_inter)
	{
		maybe_inter.clear();
		int nface = static_cast<int>(mesh.n_faces());
		maybe_inter.resize(nface);

		std::vector<GeomBlob> geom(nface);
		tbb::parallel_for(0, nface, [&](int i) {
			geom[i].id = i;
			FaceHandle fh(i);
			auto fv = mesh.fv_begin(fh);
			geom[i].bbox.expand_to_include(mesh.point(*fv)); ++fv;
			geom[i].bbox.expand_to_include(mesh.point(*fv)); ++fv;
			geom[i].bbox.expand_to_include(mesh.point(*fv));
			geom[i].point = geom[i].bbox.centroid();
		});

		Cart3DBvh bvh(geom);
		tbb::parallel_for(0, nface, [&](int i) {
			bvh.for_each_in_box(geom[i].bbox, [&](int j) {
				if (i != j)
					maybe_inter[i].push_back(j);
			});
		});
		return true;
	}

	bool RemoveSelfIntersection::find_int_pairs(
		const vv_inter& maybe_inter, 
		vv_BPoint& pts, 
		vv_BEdge& edges)
	{
		int nface = static_cast<int>(mesh.n_faces());
		pts.clear();
		edges.clear();
		pts.resize(nface);
		edges.resize(nface);
		tbb::parallel_for(0, nface, [&](int i) {
			if (!maybe_inter[i].empty())
			{
				FaceHandle ph(i);
				auto pfv = mesh.fv_begin(ph);
				const auto& p0 = mesh.point(*pfv); ++pfv;
				const auto& p1 = mesh.point(*pfv); ++pfv;
				const auto& p2 = mesh.point(*pfv);
				auto pn = (p1 - p0).cross(p2 - p0);
				pn.normalize();
				for (auto& j : maybe_inter[i])
				{
					if (i < j) //避免重复求交
					{
						FaceHandle qh(j);
						auto qfv = mesh.fv_begin(qh);
						const auto& q0 = mesh.point(*qfv); ++qfv;
						const auto& q1 = mesh.point(*qfv); ++qfv;
						const auto& q2 = mesh.point(*qfv);
						std::vector<cvector3d> intps;
						if (IntTriTri(p0, p1, p2, q0, q1, q2, intps))
						{
							int iStart = (int)pts[i].size();
							for (auto& ip : intps)
							{
								BooleanPoint bp;
								bp.data = ip;
								bp.jface = j;
								pts[i].emplace_back(bp);
							}
							int iEnd = static_cast<int>(pts.size());
							int nintp = static_cast<int>(intps.size());
							if (nintp == 2) //如果相交为线段时
							{
								BooleanEdge be;
								be.pts = &pts[i][iStart];
								be.pte = &pts[i][iStart + 1];
								be.jface = j;
								edges[i].emplace_back(be);
							}
							else if (nintp > 2) //共面的情况时
							{
								std::vector<BooleanPoint*> cpts;
								for (int k = iStart; k < iEnd; ++k)
								{
									cpts.push_back(&pts[i][k]);
								}
								std::vector<int> hull;
								if (gramScan(pn, cpts, hull))
								{
									int nhull = static_cast<int>(hull.size());
									for (int h = 0; h < nhull; ++h)
									{
										BooleanEdge be;
										be.pts = cpts[hull[h]];
										be.pte = cpts[hull[(h + 1) % nhull]];
										be.jface = j;
										edges[i].emplace_back(be);
									}
								}
							}
						}
					}
				}
			}
		});



		return true;
	}

	bool RemoveSelfIntersection::IntTriTri(
		const cvector3d& v0, const cvector3d& v1, const cvector3d& v2,
		const cvector3d& u0, const cvector3d& u1, const cvector3d& u2,
		std::vector<cvector3d>& intps)
	{
		Triangle3d tu(v0, v1, v2);
		Triangle3d tv(u0, u1, u2);
		IntrTriangle3Triangle3 tools(tu, tv);

		if (tools.Find())
		{
			int npts = tools.GetQuantity();
			intps.swap(std::vector<cvector3d>(npts));
			for (int i = 0; i < npts; ++i)
			{
				intps[i] = tools.GetPoint(i);
			}
		}
		else
		{
			intps.clear();
		}
		return !intps.empty();
	}
}


#if defined(_MSC_VER)
#pragma warning(pop)
#endif