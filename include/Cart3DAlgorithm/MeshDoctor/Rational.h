#pragma once
#ifndef CART3D_ALGORITHM_RATIONAL_MESHDOCTOR_H
#define CART3D_ALGORITHM_RATIONAL_MESHDOCTOR_H
#include "OpenMeshUtil.h"
#include "MeshDoctorConfig.h"
#include <Common/util.h>

namespace Cart3DAlgorithm
{
	class MESHDOCTOR_API Rational
	{
	public:
		/**
         * ��������ת��Ϊ������
         *
         * In case of infinity, the returned value is expressed as `{1, 0}` or
         * `{-1, 0}` depending on the sign.
         *
         * @param d   ��ת���ĸ�����
         * @param max ����������ӷ�ĸ
         * @return `d`ת����������
         * @see av_q2d()
         */
		static crational fromcfloat(cfloat d, int max = INT_MAX);
	};

}


#endif