#pragma once
#ifndef CART3DALGORITHM_MESH_BOUNDARY_EXTRACTOR_H
#define CART3DALGORITHM_MESH_BOUNDARY_EXTRACTOR_H

#include "OpenMeshUtil.h"
#include "MeshDoctorConfig.h"

namespace Cart3DAlgorithm
{
	class MESHDOCTOR_API MeshBoundaryExtractor
	{
	public:
		/*
		 * @brief    根据半边提取边界圈
		 * @detail
		 * @param[in]      mesh 待提取的mesh
		 * @param[in]        hh 半边
		 * @param[out] boundary 边界
		 */
		static bool ExtraBoundary(const OpenTriMesh& mesh, const HalfedgeHandle& hh, std::vector<VertexHandle>& boundary);
		/*
		 * @brief    根据半边提取边界圈
		 * @detail
		 * @param[in]      mesh 待提取的mesh
		 * @param[in]        hh 半边
		 * @param[out] boundary 边界
		 */
		static bool ExtraBoundary(const OpenTriMesh& mesh, const HalfedgeHandle& hh, std::vector<HalfedgeHandle>& boundary);
		/*
		 * @brief    提取所有边界圈
		 * @detail
		 * @param[in]      mesh 待提取的mesh
		 * @param[out] boundary 边界
		 */
		static bool ExtrAllBoundary(const OpenTriMesh& mesh, std::vector<HalfedgeHandle>& bounary);

		/*
		 * @brief    提取最大边界圈
		 * @detail
		 * @param[in]      mesh 待提取的mesh
		 * @param[out]       hh 最大边界圈
		 */
		static bool ExtraMaxBounary(const OpenTriMesh& mesh, HalfedgeHandle& hh);
	};
}



#endif