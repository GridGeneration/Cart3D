#ifndef CART3DALGORITHM_COMMON_PREDICATES_H
#define CART3DALGORITHM_COMMON_PREDICATES_H
#include "util.h"
#include "CommonConfig.h"
namespace Cart3DAlgorithm
{
	class COMMON_API shewchuk {
	public:
		static cfloat orient2dfast(
			const cfloat* pa, const cfloat* pb, const cfloat* pc);
		static cfloat orient2dexact(
			const cfloat* pa, const cfloat* pb, const cfloat* pc);
		static cfloat orient2dslow(
			const cfloat* pa, const cfloat* pb, const cfloat* pc);
		static cfloat orient2dadapt(
			const cfloat* pa, const cfloat* pb, const cfloat* pc, cfloat detsum);
		static cfloat orient2d(
			const cfloat* pa, const cfloat* pb, const cfloat* pc);

		static cfloat orient3dfast(
			const cfloat* pa, const cfloat* pb, const cfloat* pc, const cfloat* pd);
		static cfloat orient3dexact(
			const cfloat* pa, const cfloat* pb, const cfloat* pc, const cfloat* pd);
		static cfloat orient3dslow(
			const cfloat* pa, const cfloat* pb, const cfloat* pc, const cfloat* pd);
		static cfloat orient3dadapt(
			const cfloat* pa, const cfloat* pb, const cfloat* pc, const cfloat* pd, cfloat permanent);
		static cfloat orient3d(
			const cfloat* pa, const cfloat* pb, const cfloat* pc, const cfloat* pd);

		static cfloat incirclefast(
			const cfloat* pa, const cfloat* pb, const cfloat* pc, const cfloat* pd);
		static cfloat incircleexact(
			const cfloat* pa, const cfloat* pb, const cfloat* pc, const cfloat* pd);
		static cfloat incircleslow(
			const cfloat* pa, const cfloat* pb, const cfloat* pc, const cfloat* pd);
		static cfloat incircleadapt(
			const cfloat* pa, const cfloat* pb, const cfloat* pc, const cfloat* pd, cfloat permanent);
		static cfloat incircle(
			const cfloat* pa, const cfloat* pb, const cfloat* pc, const cfloat* pd);

		static cfloat inspherefast(
			const cfloat* pa, const cfloat* pb, const cfloat* pc, const cfloat* pd, const cfloat* pe);
		static cfloat insphereexact(
			const cfloat* pa, const cfloat* pb, const cfloat* pc, const cfloat* pd, const cfloat* pe);
		static cfloat insphereslow(
			const cfloat* pa, const cfloat* pb, const cfloat* pc, const cfloat* pd, const cfloat* pe);
		static cfloat insphereadapt(
			const cfloat* pa, const cfloat* pb, const cfloat* pc, const cfloat* pd, const cfloat* pe, cfloat permanent);
		static cfloat insphere(
			const cfloat* pa, const cfloat* pb, const cfloat* pc, const cfloat* pd, const cfloat* pe);
	};

}// namespace Cart3DAlgorithm


#endif
