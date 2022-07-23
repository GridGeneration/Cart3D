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
		 * @brief    ���ݰ����ȡ�߽�Ȧ
		 * @detail
		 * @param[in]      mesh ����ȡ��mesh
		 * @param[in]        hh ���
		 * @param[out] boundary �߽�
		 */
		static bool ExtraBoundary(const OpenTriMesh& mesh, const HalfedgeHandle& hh, std::vector<VertexHandle>& boundary);
		/*
		 * @brief    ���ݰ����ȡ�߽�Ȧ
		 * @detail
		 * @param[in]      mesh ����ȡ��mesh
		 * @param[in]        hh ���
		 * @param[out] boundary �߽�
		 */
		static bool ExtraBoundary(const OpenTriMesh& mesh, const HalfedgeHandle& hh, std::vector<HalfedgeHandle>& boundary);
		/*
		 * @brief    ��ȡ���б߽�Ȧ
		 * @detail
		 * @param[in]      mesh ����ȡ��mesh
		 * @param[out] boundary �߽�
		 */
		static bool ExtrAllBoundary(const OpenTriMesh& mesh, std::vector<HalfedgeHandle>& bounary);

		/*
		 * @brief    ��ȡ���߽�Ȧ
		 * @detail
		 * @param[in]      mesh ����ȡ��mesh
		 * @param[out]       hh ���߽�Ȧ
		 */
		static bool ExtraMaxBounary(const OpenTriMesh& mesh, HalfedgeHandle& hh);
	};
}



#endif