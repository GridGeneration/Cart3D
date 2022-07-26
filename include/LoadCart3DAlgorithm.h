#pragma once


#ifndef LOAD_ALGORITHM_H
#define LOAD_ALGORITHM_H

#ifdef NDEBUG
#pragma comment(lib,"Commonx64.lib")
#pragma comment(lib,"MeshGenerationx64.lib")
#pragma comment(lib,"MeshDoctorx64.lib")
#pragma comment(lib,"SearchAlgox64.lib")
#pragma comment(lib,"TriangleMeshx64.lib")
#else
#pragma comment(lib,"Commonx64_d.lib")
#pragma comment(lib,"MeshGenerationx64_d.lib")
#pragma comment(lib,"MeshDoctorx64_d.lib")
#pragma comment(lib,"SearchAlgox64_d.lib")
#pragma comment(lib,"TriangleMeshx64_d.lib")
#endif


#endif