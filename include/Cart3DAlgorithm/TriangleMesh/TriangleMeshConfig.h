#ifndef CART3D_ALGORITHM_TRIANGLEMESH_CONFIG_H
#define CART3D_ALGORITHM_TRIANGLEMESH_CONFIG_H
#include <Common/util.h>
#ifdef USING_DLL
#ifndef TRIANGLEMESH_EXPORT_DLL
#define TRIANGLEMESH_API  __declspec(dllimport)
#else
#define TRIANGLEMESH_API  __declspec(dllexport)
#endif
#else
#define TRIANGLEMESH_API 
#endif

#ifdef NDEBUG
#pragma comment(lib,"Commonx64.lib")
#pragma comment(lib,"SearchAlgox64.lib")
#pragma comment(lib,"MeshGenerationx64.lib")

#else
#pragma comment(lib,"Commonx64_d.lib")
#pragma comment(lib,"SearchAlgox64_d.lib")
#pragma comment(lib,"MeshGenerationx64_d.lib")
#endif

#endif
