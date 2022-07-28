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
         * 将浮点数转化为有理数
         *
         * In case of infinity, the returned value is expressed as `{1, 0}` or
         * `{-1, 0}` depending on the sign.
         *
         * @param d   带转化的浮点数
         * @param max 允许的最大分子分母
         * @return `d`转化的有理数
         * @see av_q2d()
         */
		static crational fromcfloat(cfloat d, int max = INT_MAX);
	};

}


#endif