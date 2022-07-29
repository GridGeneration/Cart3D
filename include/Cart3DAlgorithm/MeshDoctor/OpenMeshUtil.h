#pragma once
#ifndef CART3D_ALGORITHM_OPENMESHUTIL_H
#define CART3D_ALGORITHM_OPENMESHUTIL_H

#pragma warning(push)
#pragma warning(disable:4244)
#pragma warning(disable:26495)
#pragma warning(disable:26451)

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif


#include <OpenMesh/Core/IO/MeshIO.hh>
#include <Common/util.h>
#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Geometry/EigenVectorT.hh>

#include <OpenMesh/Core/Mesh/PolyMesh_ArrayKernelT.hh>
#include <OpenMesh/Core/Mesh/PolyConnectivity.hh>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Core/Mesh/Traits.hh>

namespace Cart3DAlgorithm
{
	struct EigenTraits : OpenMesh::DefaultTraits {
		using Point = cvector3d;
		using Normal = cvector3d;
		using TexCoord2D = cvector2d;
		using TexCoord3D = cvector3d;
		using TexCoord1D = cfloat;
		using TextureIndex = int;
	};


	using OpenTriMesh = OpenMesh::TriMesh_ArrayKernelT<EigenTraits>;
	using OpenPolygonMesh= OpenMesh::PolyMesh_ArrayKernelT<EigenTraits>;
	using VertexHandle = OpenMesh::ArrayKernel::VertexHandle;
	using EdgeHandle = OpenMesh::ArrayKernel::EdgeHandle;
	using HalfedgeHandle = OpenMesh::ArrayKernel::HalfedgeHandle;
	using FaceHandle = OpenMesh::ArrayKernel::FaceHandle;
}

#pragma warning(pop)


#endif