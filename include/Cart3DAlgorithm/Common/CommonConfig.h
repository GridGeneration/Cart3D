#ifndef CART3DALGORITHM_COMMONCONFIG_H
#define CART3DALGORITHM_COMMONCONFIG_H

#ifdef USING_DLL
	#ifndef COMMON_EXPORT_DLL
		#define COMMON_API  __declspec(dllimport)
	#else
		#define COMMON_API  __declspec(dllexport)
	#endif
#else
	#define COMMON_API 
#endif
	
#endif