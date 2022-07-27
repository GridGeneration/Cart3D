#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable:4273)
#pragma warning(disable:4251)
#pragma warning(disable:4286)
#endif

#include<MeshDoctor/RemoveSelfIntersection.h>
#include "core/Wm4Intersector.h"
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
		Triangle3<cfloat> t0(
			Vector3<cfloat>(v0[0], v0[1], v0[2]),
			Vector3<cfloat>(v1[0], v1[1], v1[2]),
			Vector3<cfloat>(v2[0], v2[1], v2[2]));

		Triangle3<cfloat> t1(
			Vector3<cfloat>(u0[0], u0[1], u0[2]),
			Vector3<cfloat>(u1[0], u1[1], u1[2]),
			Vector3<cfloat>(u2[0], u2[1], u2[2]));
		IntrTriangle3Triangle3<cfloat> int_tools(t0, t1);
		if (int_tools.Find())
		{
			int nid = int_tools.GetQuantity();
			intps.clear();
			intps.reserve(nid);
			for (int i = 0; i < nid; ++i)
			{
				const auto& vd = int_tools.GetPoint(i);
				intps.push_back(cvector3d(vd[0], vd[1], vd[2]));
			}
		}
		return true;
	}
}


#if defined(_MSC_VER)
#pragma warning(pop)
#endif