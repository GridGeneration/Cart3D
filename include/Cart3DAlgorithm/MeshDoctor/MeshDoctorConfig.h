#ifndef CART3DALGORITHM_MESH_MESHDOCTOR_H
#define CART3DALGORITHM_MESH_MESHDOCTOR_H

#include <Cart3DAlgorithm/Common/util.h>

#ifdef USING_DLL
	#ifdef MESHDOCTOR_EXPORT_DLL
		#define MESHDOCTOR_API  __declspec(dllimport)
	#else
		#define MESHDOCTOR_API  __declspec(dllexport)
	#endif
#else
	#define MESHDOCTOR_API 
#endif
	
#endif