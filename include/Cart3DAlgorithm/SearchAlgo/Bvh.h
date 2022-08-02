#ifndef CART3DALGORITHM_BVH_H
#define CART3DALGORITHM_BVH_H
#include "SearchAlgoConfig.h"
#include <vector>
#include <Common/BBox.h>

namespace Cart3DAlgorithm
{
    struct SEARCHALGO_API GeomBlob
    {
        int id;
        cvector3d   point;
        BoundingBox  bbox;  
        GeomBlob();
        GeomBlob(const GeomBlob& blob);
        GeomBlob(const GeomBlob&& blob)noexcept;
        GeomBlob& operator=(GeomBlob& blob);
    };


    class SEARCHALGO_API  Cart3DBvh
    {
    public:
        struct AABVHNode
        {
            AABVHNode():left(nullptr),right(nullptr) {}
            BoundingBox bbox;
            std::shared_ptr<AABVHNode> left;
            std::shared_ptr <AABVHNode> right;
            std::vector<int>  blobids;
            inline bool is_leaf() const { return left == nullptr; }
        };
    public:
        Cart3DBvh(std::vector<GeomBlob>& geoms, bool need_keep_data = true);
    public:
        void for_each_in_box(
            const BoundingBox& bbox, 
            std::function<void(int idx)>action);
        void for_each_in_box_safe(
            const BoundingBox& bbox, 
            std::function<void(int idx)>action,
            const cvector3d& deps);
        void for_each_in_box_first(
            const BoundingBox& bbox, 
            std::function<bool(int idx)>action);
        void for_each_in_box_safe_first(
            const BoundingBox& bbox, 
            std::function<bool(int idx)>action, 
            const cvector3d& deps);
    private:
        std::shared_ptr<AABVHNode> construct_tree(int begin, int end, int last_dim);
        void quick_select(int select, int begin, int end, int dim);
    private:
        std::shared_ptr<AABVHNode> root;
        std::vector<GeomBlob> blobs;
        std::vector<int> tmpids;
    };


}


#endif