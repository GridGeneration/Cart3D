#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable:4273)
#pragma warning(disable:4251)
#pragma warning(disable:4286)
#endif
#include "MeshDoctor/RobustTriTriIntersection.h"
#include <MeshDoctor/RemoveSelfIntersection.h>
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
					auto& p = mesh.point(iv);
					p /= num;
				}
				mesh.remove_property(CTP);
			}
			int num;
			OpenTriMesh& mesh;
			static cfloat eps;
		};
		
	

	}

	RemoveSelfIntersection::RemoveSelfIntersection(OpenTriMesh& in_mesh):mesh(in_mesh){}


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