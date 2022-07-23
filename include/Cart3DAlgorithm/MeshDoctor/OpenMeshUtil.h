#pragma once
#ifndef CART3D_ALGORITHM_OPENMESHUTIL_H
#define CART3D_ALGORITHM_OPENMESHUTIL_H

#pragma warning(push)
#pragma warning(disable:4244)

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>


#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/PolyConnectivity.hh>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/Traits.hh>

namespace Cart3DAlgorithm
{
	using OpenTriMesh = OpenMesh::TriMesh_ArrayKernelT<>;
	using OpenPolygonMesh= OpenMesh::PolyMesh_ArrayKernelT<>;
	using VertexHandle = OpenMesh::ArrayKernel::VertexHandle;
	using EdgeHandle = OpenMesh::ArrayKernel::EdgeHandle;
	using HalfedgeHandle = OpenMesh::ArrayKernel::HalfedgeHandle;
	using FaceHandle = OpenMesh::ArrayKernel::FaceHandle;
}

#pragma warning(pop)


#endif