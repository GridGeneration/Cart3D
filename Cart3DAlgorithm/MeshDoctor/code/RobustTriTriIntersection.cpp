#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable:4273)
#pragma warning(disable:4251)
#pragma warning(disable:4286)
#endif

#include <MeshDoctor/RobustTriTriIntersection.h>
#include <MeshDoctor/BaseType.h>
namespace Cart3DAlgorithm
{
	namespace
	{
		using TT = RobustTriTriIntersection;
		void TrianglePlaneRelations(
			const Triangle3d& rkTriangle, const Plane3d& rkPlane,
			cfloat afDistance[3], int aiSign[3], int& riPositive, int& riNegative,int& riZero)
		{
			riPositive = 0;
			riNegative = 0;
			riZero = 0;
			for (int i = 0; i < 3; ++i)
			{
				afDistance[i] = rkPlane.DistanceTo(rkTriangle.v[i]);
				if (afDistance[i] > inv_trunc_val)
				{
					aiSign[i] = 1;
					++riPositive;
				}
				else if (afDistance[i] < -inv_trunc_val)
				{
					aiSign[i] = -1;
					++riNegative;
				}
				else
				{
					afDistance[i] = (cfloat)0.0;
					aiSign[i] = 0;
					++riZero;
				}
			}
		}

		inline cfloat __stdcall DotPerp(const cvector2d& rkU,const cvector2d& rkV)
		{
			return rkU[0] * rkV[1] - rkU[1] * rkV[0];
		}

		//bool GetCoplanarIntersection(
		//	const Plane3d& rkPlane, 
		//	const Triangle3d& rkTri0,
		//	const Triangle3d& rkTri1,
		//	std::vector<cvector3d>& int_pts)
		//{
		//	int iMaxNormal = 0;
		//	cfloat fMax = std::abs(rkPlane.normal.x());
		//	cfloat fAbs = std::abs(rkPlane.normal.y());
		//	if (fAbs > fMax)
		//	{
		//		iMaxNormal = 1;
		//		fMax = fAbs;
		//	}
		//	fAbs = std::abs(rkPlane.normal.z());
		//	if (fAbs > fMax)
		//	{
		//		iMaxNormal = 2;
		//	}
		//	Triangle2d kProjTri0, kProjTri1;
		//	if (iMaxNormal == 0)
		//	{
		//		// project onto yz-plane
		//		for (int i = 0; i < 3; ++i)
		//		{
		//			kProjTri0.v[i].x() = rkTri0.v[i].y();
		//			kProjTri0.v[i].y() = rkTri0.v[i].z();
		//			kProjTri1.v[i].x() = rkTri1.v[i].y();
		//			kProjTri1.v[i].y() = rkTri1.v[i].z();
		//		}
		//	}
		//	else if (iMaxNormal == 1)
		//	{
		//		// project onto xz-plane
		//		for (int i = 0; i < 3; ++i)
		//		{
		//			kProjTri0.v[i].x() = rkTri0.v[i].x();
		//			kProjTri0.v[i].y() = rkTri0.v[i].z();
		//			kProjTri1.v[i].x() = rkTri1.v[i].x();
		//			kProjTri1.v[i].y() = rkTri1.v[i].z();
		//		}
		//	}
		//	else
		//	{
		//		// project onto xy-plane
		//		for (int i = 0; i < 3; ++i)
		//		{
		//			kProjTri0.v[i].x() = rkTri0.v[i].x();
		//			kProjTri0.v[i].y() = rkTri0.v[i].y();
		//			kProjTri1.v[i].x() = rkTri1.v[i].x();
		//			kProjTri1.v[i].y() = rkTri1.v[i].y();
		//		}
		//	}
		//	cvector3d kSave;
		//	cvector3d kEdge0 = kProjTri0.v[1] - kProjTri0.v[0];
		//	cvector3d kEdge1 = kProjTri0.v[2] - kProjTri0.v[0];
		//	if (DotPerp(kEdge0,kEdge1) < (cfloat)0.0)
		//	{
		//		kSave = kProjTri0.v[1];
		//		kProjTri0.v[1] = kProjTri0.v[2];
		//		kProjTri0.v[2] = kSave;
		//	}

		//	kEdge0 = kProjTri1.v[1] - kProjTri1.v[0];
		//	kEdge1 = kProjTri1.v[2] - kProjTri1.v[0];
		//	if (DotPerp(kEdge0,kEdge1) < (cfloat)0.0)
		//	{
		//		kSave = kProjTri1.v[1];
		//		kProjTri1.v[1] = kProjTri1.v[2];
		//		kProjTri1.v[2] = kSave;
		//	}
		//	std::vector<cvector2d> intps;
		//	if (!TT::FindTriTri(
		//		kProjTri0.v[0], kProjTri0.v[1], kProjTri0.v[2],
		//		kProjTri1.v[0], kProjTri1.v[1], kProjTri1.v[2],intps))
		//	{
		//		return false;
		//	}
		//	if (iMaxNormal == 0)
		//	{
		//		cfloat fInvNX = ((cfloat)1.0) / rkPlane.normal.x();
		//		int npts = (int)intps.size();
		//		int_pts.clear();
		//		int_pts.resize(npts);
		//		for (int i = 0; i < npts; ++i)
		//		{
		//			int_pts[i].y() = intps[i].x();
		//			int_pts[i].z() = intps[i].y();
		//			int_pts[i].x() = fInvNX * (rkPlane.constant -
		//				rkPlane.normal.y() * intps[i].y() -rkPlane.normal.z() * intps[i].z());
		//		}
		//	}
		//	else if (iMaxNormal == 1)
		//	{
		//		cfloat fInvNY = ((cfloat)1.0) / rkPlane.normal.y();
		//		int npts = (int)intps.size();
		//		int_pts.clear();
		//		int_pts.resize(npts);
		//		for (int i = 0; i < npts; ++i)
		//		{
		//			int_pts[i].x() = intps[i].x();
		//			int_pts[i].z() = intps[i].y();
		//			int_pts[i].y() = fInvNY * (rkPlane.constant -
		//				rkPlane.normal.x() * intps[i].x() -
		//				rkPlane.normal.z() * intps[i].z());
		//		}
		//	}
		//	else
		//	{
		//		cfloat fInvNZ = ((cfloat)1.0) / rkPlane.normal.z();
		//		int npts = (int)intps.size();
		//		int_pts.clear();
		//		int_pts.resize(npts);
		//		for (int i = 0; i < npts; ++i)
		//		{
		//			int_pts[i].x() = intps[i].x();
		//			int_pts[i].y() = intps[i].y();
		//			int_pts[i].z() = fInvNZ * (rkPlane.constant -
		//				rkPlane.normal.x() * intps[i].x() -
		//				rkPlane.normal.y() * intps[i].y());
		//		}
		//	}

		//	return true;
		//}


	

	}

	bool RobustTriTriIntersection::TestTriTri(
		const cvector3d& a0, const cvector3d& a1, const cvector3d& a2,
		const cvector3d& b0, const cvector3d& b1, const cvector3d& b2)
	{


		return true;
	}

	bool RobustTriTriIntersection::TestTriTri(
		const cvector2d& a0, const cvector2d& a1, const cvector2d& a2,
		const cvector2d& b0, const cvector2d& b1, const cvector2d& b2)
	{


		return true;
	}

	bool RobustTriTriIntersection::FindTriTri(
		const cvector2d& a0, const cvector2d& a1, const cvector2d& a2,
		const cvector2d& b0, const cvector2d& b1, const cvector2d& b2,
		std::vector<cvector2d>& int_pts)
	{


		return true;
	}

	bool RobustTriTriIntersection::FindTriTri(
		const cvector3d& a0, const cvector3d& a1, const cvector3d& a2,
		const cvector3d& b0, const cvector3d& b1, const cvector3d& b2,
		std::vector<cvector3d>& int_pts)
	{
		Triangle3d tu(a0, a1, a2);
		Triangle3d tv(b0, b1, b2);
		Plane3d kPlane0(a0, a1, a2);
		int iPos1, iNeg1, iZero1, aiSign1[3];
		cfloat afDist1[3];
		TrianglePlaneRelations(tv, kPlane0, afDist1, aiSign1, iPos1, iNeg1,iZero1);

		if (iPos1 == 3 || iNeg1 == 3)
		{ //Triangle0 lie the side of triangle1
			return false;
		}
		if (iZero1 == 3){
			
		}

		

		return !int_pts.empty();
	}




	int RobustTriTriIntersection::orient3d(
		const cvector3d& a, const cvector3d& b,
		const cvector3d& c, const cvector3d& d)
	{
		cmatrix4d mat;
		mat.setConstant(1);
		mat.block<1, 3>(0, 0) = a;
		mat.block<1, 3>(1, 0) = b;
		mat.block<1, 3>(2, 0) = c;
		mat.block<1, 3>(3, 0) = d;
		cfloat det = mat.determinant();
		if (det < -inv_trunc_val)
			return -1;
		else if (det > inv_trunc_val)
			return 1;
		return 0;
	}

	int RobustTriTriIntersection::orient2d(
		const cvector2d& a,
		const cvector2d& b,
		const cvector2d& c)
	{
		cmatrix3d mat;
		mat.setConstant(1);
		mat.block<1, 2>(0, 0) = a;
		mat.block<1, 2>(1, 0) = b;
		mat.block<1, 2>(2, 0) = c;
		cfloat det = mat.determinant();
		if (det < -inv_trunc_val)
			return -1;
		else if (det > inv_trunc_val)
			return 1;
		return 0;
	}

}



#if defined(_MSC_VER)
#pragma warning(pop)
#endif