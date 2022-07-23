#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable:4273)
#pragma warning(disable:4251)
#pragma warning(disable:4286)
#pragma warning(disable:4267)
#endif
#include <Cart3DAlgorithm/SearchAlgo/BBox.h>

namespace Cart3DAlgorithm
{

	BoundingBox::BoundingBox() : pMin(cvector3d::Constant(maxcfloat)),
		pMax(cvector3d::Constant(mincfloat)) {}
	BoundingBox::BoundingBox(const cvector3d& p) {
		cvector3d epsilonVector = cvector3d::Constant(epsilon_cfloat);
		pMin = p - epsilonVector;
		pMax = p + epsilonVector;
	}

	BoundingBox::BoundingBox(const BoundingBox& box):
		pMin(box.pMin),pMax(box.pMax)
	{

	}

	//BoundingBox::BoundingBox(const BoundingBox&& box)noexcept :
	//	pMin(box.pMin), pMax(box.pMax)
	//{

	//}

	BoundingBox& BoundingBox::operator=(const BoundingBox& box)
	{
		if (this != &box)
		{
			this->pMin = box.pMin;
			this->pMax = box.pMax;
		}
		return *this;
	}

	void BoundingBox::expand_to_include(const cvector3d& p) {
		cvector3d epsilonVector = cvector3d::Constant(epsilon_cfloat);
		pMin = pMin.cwiseMin(p - epsilonVector);
		pMax = pMax.cwiseMax(p + epsilonVector);
	}
	void BoundingBox::expand_to_include(const BoundingBox& b) {
		pMin = pMin.cwiseMin(b.pMin);
		pMax = pMax.cwiseMax(b.pMax);
	}
	cvector3d BoundingBox::extent() const {
		return pMax - pMin;
	}
	void BoundingBox::compute_squared_distance(const cvector3d& p, cfloat& d2Min, cfloat& d2Max) const {
		cvector3d u = pMin - p;
		cvector3d v = p - pMax;
		d2Min = u.cwiseMax(v).cwiseMax(0.0f).squaredNorm();
		d2Max = u.cwiseMin(v).squaredNorm();
	}
	bool BoundingBox::contains(const cvector3d& p) const {
		return (p.array() >= pMin.array()).all() &&
			(p.array() <= pMax.array()).all();
	}
	bool BoundingBox::overlap(const BoundingBox& b) const {
		return (b.pMax.array() >= pMin.array()).all() &&
			(b.pMin.array() <= pMax.array()).all();
	}
	bool BoundingBox::is_valid() const {
		return (pMax.array() >= pMin.array()).all();
	}
	int BoundingBox::max_dimension() const {
		int index;
		cfloat maxLength = (pMax - pMin).maxCoeff(&index);
		return index;
	}
	cvector3d BoundingBox::centroid() const {
		return (pMin + pMax) * 0.5f;
	}
	cfloat BoundingBox::surface_area() const {
		cvector3d e = extent().cwiseMax(1e-5);
		return 2.0f * cvector3d::Constant(e.prod()).cwiseQuotient(e).sum();
	}
	cfloat BoundingBox::volume() const {
		return extent().prod();
	}
	BoundingBox BoundingBox::intersect(const BoundingBox& b) const {
		BoundingBox bIntersect;
		bIntersect.pMin = pMin.cwiseMax(b.pMin);
		bIntersect.pMax = pMax.cwiseMin(b.pMax);
		return bIntersect;
	}

	BoundingBox BoundingBox::expand_safe_box(const cvector3d& p)const
	{
		BoundingBox bIntersect;
		bIntersect.pMin = pMin - p;
		bIntersect.pMax = pMax + p;
		return bIntersect;
	}
}

#if defined(_MSC_VER)
#pragma warning(pop)
#endif