#ifndef CART3DALGORITHM_MESH_GENERATION_H
#define CART3DALGORITHM_MESH_GENERATION_H

#ifdef USING_DLL
	#ifdef MESHGEN_EXPORT_DLL
		#define MESHGEN_API  __declspec(dllimport)
	#else
		#define MESHGEN_API  __declspec(dllexport)
	#endif
#else
	#define SEARCHALGO_API 
#endif
	
#endif