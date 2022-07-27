#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable:4273)
#pragma warning(disable:4251)
#pragma warning(disable:4286)
#endif

#include<MeshDoctor/RemoveSelfIntersection.h>

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
		cfloat TruncMesh::eps = 1e-4;
		struct Plane3d
		{
			cvector3d norm;
			cfloat constant;
			inline cfloat distanceto(const cvector3d& v)const
			{
				return norm.dot(v) - constant;
			}
		};

		struct Triangle3D
		{
			Triangle3D(const cvector3d& u0, const cvector3d& u1, const cvector3d& u2):
				v0(u0),v1(u1),v2(u2)
			{
				e0 = v1 - v0;
				e1 = v2 - v0;
				tn = e0.cross(e1);
			}
			union {
				struct 
				{
					cvector3d v[3];
				};
				struct 
				{
					cvector3d v0, v1, v2;
				};
			};
			cvector3d e0, e1;
			cvector3d tn;
		};

		inline void tri_plane_rel(
			const Triangle3D& triangle,const Plane3d& planev,
			cfloat afDistance[3], int aiSign[3], int& riPositive, int& riNegative,
			int& riZero)
		{
			riPositive = 0;
			riNegative = 0;
			riZero = 0;
			for (int i = 0; i < 3; ++i)
			{
				afDistance[i] = planev.distanceto(triangle.v[i]);
				if (afDistance[i] > TruncMesh::eps)
				{
					aiSign[i] = 1;
					++riPositive;
				}
				else if (afDistance[i] < -TruncMesh::eps)
				{
					aiSign[i] = -1;
					++riNegative;
				}
				else
				{
					afDistance[i] = 0;
					aiSign[i] = 0;
					riZero++;
				}
			}
		}

	

	}

	RemoveSelfIntersection::RemoveSelfIntersection(OpenTriMesh& in_mesh):mesh(in_mesh){}


	bool RemoveSelfIntersection::IntTriTri(
		const cvector3d& v0, const cvector3d& v1, const cvector3d& v2,
		const cvector3d& u0, const cvector3d& u1, const cvector3d& u2,
		std::vector<cvector3d>& intps)
	{
		Triangle3D vtri(v0, v1, v2);
		Plane3d planev;
		planev.norm = vtri.e0.cross(vtri.e1);
		planev.constant = planev.norm.dot(v0);
		Triangle3D utri(u0, u1, u2);

		int iPos1, iNeg1, iZero1, aiSign1[3];
		cfloat afDist1[3];
		tri_plane_rel(utri, planev, afDist1, aiSign1, iPos1, iNeg1,iZero1);

		intps.reserve(6);
		if (iPos1 == 3 || iNeg1 == 3)//三角形在平面的一边的情况下直接返回
		{
			return false;
		}
		//ToDo...
		if (iZero1 == 3)//三角形与三角形共面的情况
		{

		}
		

		return true;
	}
}


#if defined(_MSC_VER)
#pragma warning(pop)
#endif