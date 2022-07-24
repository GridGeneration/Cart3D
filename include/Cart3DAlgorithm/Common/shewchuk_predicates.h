#ifndef CART3DALGORITHM_COMMON_PREDICATES_H
#define CART3DALGORITHM_COMMON_PREDICATES_H
#include "util.h"
#include "CommonConfig.h"
namespace Cart3DAlgorithm
{
	namespace shewchuk {
		COMMON_API cfloat orient2dfast(
			const cfloat* pa, const cfloat* pb, const cfloat* pc);
		COMMON_API cfloat orient2dexact(
			const cfloat* pa, const cfloat* pb, const cfloat* pc);
		COMMON_API cfloat orient2dslow(
			const cfloat* pa, const cfloat* pb, const cfloat* pc);
		COMMON_API cfloat orient2dadapt(
			const cfloat* pa, const cfloat* pb, const cfloat* pc, cfloat detsum);
		COMMON_API cfloat orient2d(
			const cfloat* pa, const cfloat* pb, const cfloat* pc);

		COMMON_API cfloat orient3dfast(
			const cfloat* pa, const cfloat* pb, const cfloat* pc, const cfloat* pd);
		COMMON_API cfloat orient3dexact(
			const cfloat* pa, const cfloat* pb, const cfloat* pc, const cfloat* pd);
		COMMON_API cfloat orient3dslow(
			const cfloat* pa, const cfloat* pb, const cfloat* pc, const cfloat* pd);
		COMMON_API cfloat orient3dadapt(
			const cfloat* pa, const cfloat* pb, const cfloat* pc, const cfloat* pd, cfloat permanent);
		COMMON_API cfloat orient3d(
			const cfloat* pa, const cfloat* pb, const cfloat* pc, const cfloat* pd);

		COMMON_API cfloat incirclefast(
			const cfloat* pa, const cfloat* pb, const cfloat* pc, const cfloat* pd);
		COMMON_API cfloat incircleexact(
			const cfloat* pa, const cfloat* pb, const cfloat* pc, const cfloat* pd);
		COMMON_API cfloat incircleslow(
			const cfloat* pa, const cfloat* pb, const cfloat* pc, const cfloat* pd);
		COMMON_API cfloat incircleadapt(
			const cfloat* pa, const cfloat* pb, const cfloat* pc, const cfloat* pd, cfloat permanent);
		COMMON_API cfloat incircle(
			const cfloat* pa, const cfloat* pb, const cfloat* pc, const cfloat* pd);

		COMMON_API cfloat inspherefast(
			const cfloat* pa, const cfloat* pb, const cfloat* pc, const cfloat* pd, const cfloat* pe);
		COMMON_API cfloat insphereexact(
			const cfloat* pa, const cfloat* pb, const cfloat* pc, const cfloat* pd, const cfloat* pe);
		COMMON_API cfloat insphereslow(
			const cfloat* pa, const cfloat* pb, const cfloat* pc, const cfloat* pd, const cfloat* pe);
		COMMON_API cfloat insphereadapt(
			const cfloat* pa, const cfloat* pb, const cfloat* pc, const cfloat* pd, const cfloat* pe, cfloat permanent);
		COMMON_API cfloat insphere(
			const cfloat* pa, const cfloat* pb, const cfloat* pc, const cfloat* pd, const cfloat* pe);
	} // namespace shewchuk

}// namespace Cart3DAlgorithm


#endif
