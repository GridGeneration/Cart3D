#ifndef CART3DALGORITHM_BOX_H
#define CART3DALGORITHM_BOX_H
#include "CommonConfig.h"
#include "util.h"


namespace Cart3DAlgorithm
{

	class COMMON_API  BoundingBox {
	public:

		BoundingBox();
		BoundingBox(const cvector3d& p);
	public:
		BoundingBox(const BoundingBox& box);
		//BoundingBox(const BoundingBox&& box)noexcept;
		BoundingBox& operator=(const BoundingBox& box);
	public:
		void expand_to_include(const cvector3d& p);
		void expand_to_include(const BoundingBox& b);
		cvector3d extent() const;
		void compute_squared_distance(const cvector3d& p, cfloat& d2Min, cfloat& d2Max) const;
		bool contains(const cvector3d& p) const;
		bool overlap(const BoundingBox& b) const;
		bool is_valid() const;
		int max_dimension() const;
		cvector3d centroid() const;
		cfloat surface_area() const;
		cfloat volume() const;
		BoundingBox intersect(const BoundingBox& b) const;
		BoundingBox expand_safe_box(const cvector3d& p)const;
	private:
		cvector3d pMin, pMax;
	};

} 

#endif
