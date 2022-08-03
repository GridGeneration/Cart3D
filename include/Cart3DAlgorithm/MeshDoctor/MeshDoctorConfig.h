#ifndef CART3DALGORITHM_MESH_MESHDOCTOR_H
#define CART3DALGORITHM_MESH_MESHDOCTOR_H

#include <Cart3DAlgorithm/Common/util.h>

#ifdef USING_DLL
	#ifndef MESHDOCTOR_EXPORT_DLL
		#define MESHDOCTOR_API  __declspec(dllimport)
	#else
		#define MESHDOCTOR_API  __declspec(dllexport)
	#endif
#else
	#define MESHDOCTOR_API 
#endif

#ifdef NDEBUG
#pragma comment(lib,"Commonx64.lib")
#pragma comment(lib,"SearchAlgox64.lib")
#else
#pragma comment(lib,"Commonx64_d.lib")
#pragma comment(lib,"SearchAlgox64_d.lib")
#endif


#endif