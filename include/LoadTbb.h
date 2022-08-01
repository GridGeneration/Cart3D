#pragma once

#ifndef LOAD_TBB_H
#define LOAD_TBB_H

#include <tbb/parallel_for.h>
#include <tbb/task.h>
#include <tbb/task_group.h>

#ifdef NDEBUG
#pragma comment(lib,"tbb12.lib")
#pragma comment(lib,"tbbmalloc.lib")
#pragma comment(lib,"tbbmalloc_proxy.lib")
#else
#pragma comment(lib,"tbb12_debug.lib")
#pragma comment(lib,"tbbmalloc_debug.lib")
#pragma comment(lib,"tbbmalloc_proxy_debug.lib")
#endif

#endif