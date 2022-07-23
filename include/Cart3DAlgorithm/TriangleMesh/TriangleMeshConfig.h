#ifndef CART3D_ALGORITHM_TRIANGLEMESH_CONFIG_H
#define CART3D_ALGORITHM_TRIANGLEMESH_CONFIG_H
#include <Cart3DAlgorithm/Common/util.h>
#ifdef USING_DLL
#ifdef TRIANGLEMESH_EXPORT_DLL
#define TRIANGLEMESH_API  __declspec(dllimport)
#else
#define TRIANGLEMESH_API  __declspec(dllexport)
#endif
#else
#define TRIANGLEMESH_API 
#endif


#endif
