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
		/*
		* ( X000    X010    X020    X030    X040    X050 )
		* ( X100    X110    X120    X130    X140    X150 )
		* ( X200    X210    X220    X230    X240    X250 )   Layer 1
		* ( X300    X310    X320    X330    X340    X350 )
		* ( X400    X410    X420    X430    X440    X450 )
		* 
		* ( X001    X011    X021    X031    X041    X051 )
		* ( X101    ...     ....    ....    ....    X151 )
		* ( X201    ...     ....    ....    ....    X251 )   Layer K
		* ( X301    ...     ....    ....    ....    X351 )   1 < K < M3
		* ( X401    X411    X421    X431    X441    X451 )
		* 
		* ( X002    X012    X022    X032    X042    X052 )
		* ( X102    X112    X122    X132    X142    X152 )
		* ( X202    X212    X222    X232    X242    X252 )   Layer M3
		* ( X302    X312    X322    X332    X342    X352 )
		* ( X402    X412    X422    X432    X442    X452 )
		* 
		 */
		static void tfi_from_faces(
			const std::vector<cfloat>& glxnodes,
			const std::vector<cfloat>& glynodes,
			const std::vector<cfloat>& glznodes,
			std::vector<cfloat>& x);
		/*
		* ( X000,   X010,   X020,   X030,   X040,   X050 )
		* ( X100,   ...,    ...,    ...,    ...,    X150 )
		* ( X200,   ...,    ...,    ...,    ...,    X250 )   Layer 1
		* ( X300,   ...,    ...,    ...,    ...,    X350 )
		* ( X400,   X410,   X420,   X430,   X440,   X450 )
		*
		* ( X001,   ...,    ...,    ...,    ...,    X051 )
		* ( ....,   ...,    ...,    ...,    ...,    ...  )
		* ( ....,   ...,    ...,    ...,    ...,    ...  )   Layer K
		* ( ....,   ...,    ...,    ...,    ...,    ...  )   1 < K < M3
		* ( X401,   ...,    ...,    ...,    ...,    X451 )
		*
		* ( X002,   X012,   X022,   X032,   X042,   X052 )
		* ( X102,   ...,    ...,    ...,    ...,    X152 )
		* ( X202,   ...,    ...,    ...,    ...,    X252 )   Layer M3
		* ( X302    ...,    ...,    ...,    ...,    X352 )
		* ( X402,   X412,   X422,   X432,   X442,   X452 )
		 */
		static void tfi_from_edges(
			const std::vector<cfloat>& glxnodes,
			const std::vector<cfloat>& glynodes,
			const std::vector<cfloat>& glznodes,
			std::vector<cfloat>& x);
		/*
		* ( X000,   ...,  ...,  ...,  ...,  ...,  X060 )
		* ( ....,   ...,  ...,  ...,  ...,  ...,  ...  )
		* ( ....,   ...,  ...,  ...,  ...,  ...,  ...  )   First "layer"
		* ( ....,   ...,  ...,  ...,  ...,  ...,  ...  )
		* ( X400,   ...,  ...,  ...,  ...,  ...,  X460 )
		* 
		* ( ....,   ...,  ...,  ...,  ...,  ...,  ...  )
		* ( ....,   ...,  ...,  ...,  ...,  ...,  ...  )
		* ( ....,   ...,  ...,  ...,  ...,  ...,  ...  )   Middle "layers"
		* ( ....,   ...,  ...,  ...,  ...,  ...,  ...  )
		* ( ....,   ...,  ...,  ...,  ...,  ...,  ...  )
		* 
		* ( X003,  ...,  ...,  ...,  ...,  ...,  X063  )
		* ( ....,   ...,  ...,  ...,  ...,  ...,  ...  )
		* ( ....,   ...,  ...,  ...,  ...,  ...,  ...  )   Last "layer"
		* ( ....,   ...,  ...,  ...,  ...,  ...,  ...  )
		* ( X403,  ...,  ...,  ...,  ...,  ...,  X463  )
		 */
		static void tfi_from_corners(
			const std::vector<cfloat>& glxnodes,
			const std::vector<cfloat>& glynodes,
			const std::vector<cfloat>& glznodes,
			std::vector<cfloat>& x);
		
	};



}


#endif
