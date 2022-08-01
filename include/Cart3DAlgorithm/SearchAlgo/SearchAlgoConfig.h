#ifndef CART3DALGORITHM_SEARCHALGO_H
#define CART3DALGORITHM_SEARCHALGO_H

#include <Common/util.h>
#ifdef USING_DLL
	#ifndef SEARCHALGO_EXPORT_DLL
		#define SEARCHALGO_API  __declspec(dllimport)
	#else
		#define SEARCHALGO_API  __declspec(dllexport)
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