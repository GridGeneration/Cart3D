#ifndef CART3DALGORITHM_TFI_H
#define CART3DALGORITHM_TFI_H

#include "MeshGenerationConfig.h"
#include <vector>
#include <Cart3DAlgorithm/Common/util.h>

namespace Cart3DAlgorithm
{
	class MESHGEN_API TFI
	{
	public:
		/*
		 *  ( X00,  X01,  X02,  X03,  X04,  X05,  X06 )
		 *  ( X10,  ...,  ...,  ...,  ...,  ...,  X16 )
		 *  ( X20,  ...,  ...,  ...,  ...,  ...,  X26 )
		 *  ( X30,  ...,  ...,  ...,  ...,  ...,  X36 )
		 *  ( X40,  X41,  X42,  X43,  X44,  X45,  X46 )
		 */
		static void tfi_from_edges(
			const std::vector<cfloat>& glxnodes,
			const std::vector<cfloat>& glynodes,
			std::vector<cfloat>& x);
		/* 
		 * ( X00,  ..., ..., ..., ..., ..., X06 )
		 * ( ...,  ..., ..., ..., ..., ..., ... )
		 * ( ...,  ..., ..., ..., ..., ..., ... )
		 * ( ...,  ..., ..., ..., ..., ..., ... )
		 * ( X40,  ..., ..., ..., ..., ..., X46 )
		 */
		static void tfi_from_corners(
			const std::vector<cfloat>& glxnodes,
			const std::vector<cfloat>& glynodes,
			std::vector<cfloat>& x);

	};



}


#endif
