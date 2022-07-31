#ifndef CART3DALGORITHM_MESH_GENERATION_H
#define CART3DALGORITHM_MESH_GENERATION_H

#ifdef USING_DLL
	#ifndef MESHGEN_EXPORT_DLL
		#define MESHGEN_API  __declspec(dllimport)
	#else
		#define MESHGEN_API  __declspec(dllexport)
	#endif
#else
	#define SEARCHALGO_API 
#endif

#ifdef NDEBUG
#pragma comment(lib,"Commonx64.lib")
#else
#pragma comment(lib,"Commonx64_d.lib")
#endif
	
#endif