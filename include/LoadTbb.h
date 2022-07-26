#pragma once


#ifndef LOAD_TBB_H
#define LOAD_TBB_H

#ifdef NDEBUG
#pragma comment(lib,"tbb/tbb_vs2022_x64_r/tbbmalloc_proxy.lib")
#pragma comment(lib,"tbb/tbb_vs2022_x64_r/tbbmalloc.lib")
#pragma comment(lib,"tbb/tbb_vs2022_x64_r/tbb12.lib")
#else
#pragma comment(lib,"tbb/tbb_vs2022_x64_d/tbbmalloc_proxy_debug.lib")
#pragma comment(lib,"tbb/tbb_vs2022_x64_d/tbbmalloc_debug.lib")
#pragma comment(lib,"tbb/tbb_vs2022_x64_d/tbb12_debug.lib")
#endif


#endif