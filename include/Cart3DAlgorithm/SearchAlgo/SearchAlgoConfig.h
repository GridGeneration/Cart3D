#ifndef CART3DALGORITHM_SEARCHALGO_H
#define CART3DALGORITHM_SEARCHALGO_H

#ifdef USING_DLL
	#ifdef SEARCHALGO_EXPORT_DLL
		#define SEARCHALGO_API  __declspec(dllimport)
	#else
		#define SEARCHALGO_API  __declspec(dllexport)
	#endif
#else
	#define SEARCHALGO_API 
#endif
	
#endif