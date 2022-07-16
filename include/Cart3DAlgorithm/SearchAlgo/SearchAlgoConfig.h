#ifndef CART3DALGORITHM_COMMONCONFIG_H
#define CART3DALGORITHM_COMMONCONFIG_H

#ifdef USING_DLL
	#ifdef SEARCHALGO_EXPORT_DLL
		#define SEARCHALGO_API  __declspec(dllimport)
	#else
		#define SEARCHALGO_API  __declspec(dllexport)
	#endif
#else
	#define COMMON_API 
#endif
	
#endif