#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable:4273)
#pragma warning(disable:4251)
#pragma warning(disable:4286)
#endif

#include <SearchAlgo/Bvh.h>
#include <stack>
namespace Cart3DAlgorithm
{
    namespace
    {
        static inline int tinyrnd(int max_int)
        {
            static unsigned trand = 0;
            trand = 1664525u * trand + 1013904223u;
            return (int)trand% max_int;
        }
    }

    GeomBlob::GeomBlob():
        id(-1),point(0,0,0),bbox()
    {

    }

    GeomBlob::GeomBlob(const GeomBlob& blob):
        id(blob.id),point(blob.point),bbox(blob.bbox)
    {

    }

    GeomBlob::GeomBlob(const GeomBlob&& blob)noexcept :
        id(blob.id), point(blob.point), bbox(blob.bbox)
    {

    }

    GeomBlob& GeomBlob::operator=(GeomBlob& blob)
    {
        if (this != &blob)
        {
            std::memcpy(this, &blob, sizeof(GeomBlob));
        }
        return *this;
    }

    Cart3DBvh::Cart3DBvh(std::vector<GeomBlob>& geoms, bool need_keep_data):
        root(nullptr), tmpids(geoms.size())
    {
        if (need_keep_data)
            blobs = geoms;
        else
            std::swap(blobs, geoms);
        for (int k = 0; k < tmpids.size(); ++k)
            tmpids[k] = k;
        root = construct_tree(0, (int)tmpids.size(), 2);
    }

    void Cart3DBvh::for_each_in_box(const BoundingBox& bbox, std::function<void(int idx)>action)
    {
        std::stack< std::shared_ptr<Cart3DBvh::AABVHNode>>  nodes;
        nodes.push(root);
        while (!nodes.empty()) {
            std::shared_ptr<Cart3DBvh::AABVHNode> node = nodes.top();
            nodes.pop();
            if (!bbox.overlap(node->bbox))
                continue;
            if (node->is_leaf())
            {
                for (int bid : node->blobids)
                {
                    if(bbox.overlap(blobs[bid].bbox))
                        action(blobs[bid].id);
                }
            }
            else
            {
                nodes.push(node->left);
                nodes.push(node->right);
            }
        }
    }

    void Cart3DBvh::for_each_in_box_safe(const BoundingBox& _bbox, std::function<void(int idx)>action, const cvector3d& deps)
    {
        std::stack< std::shared_ptr<Cart3DBvh::AABVHNode>>  nodes;
        BoundingBox bbox = _bbox.expand_safe_box(deps);
        nodes.push(root);
        while (!nodes.empty()) {
            std::shared_ptr<Cart3DBvh::AABVHNode> node = nodes.top();
            nodes.pop();
            if (!bbox.overlap(node->bbox))
                continue;
            if (node->is_leaf())
            {
                for (int bid : node->blobids)
                {
                    if (bbox.overlap(blobs[bid].bbox))
                        action(blobs[bid].id);
                }
            }
            else
            {
                nodes.push(node->left);
                nodes.push(node->right);
            }
        }
    }

    void Cart3DBvh::for_each_in_box_first(const BoundingBox& bbox, std::function<bool(int idx)>action)
    {
        std::stack< std::shared_ptr<Cart3DBvh::AABVHNode>>  nodes;
        nodes.push(root);
        while (!nodes.empty()) {
            std::shared_ptr<Cart3DBvh::AABVHNode> node = nodes.top();
            nodes.pop();
            if (!bbox.overlap(node->bbox))
                continue;
            if (node->is_leaf())
            {
                for (int bid : node->blobids)
                {
                    if (bbox.overlap(blobs[bid].bbox))
                        if (action(blobs[bid].id))
                            return;
                }
            }
            else
            {
                nodes.push(node->left);
                nodes.push(node->right);
            }
        }
    }

    void Cart3DBvh::for_each_in_box_safe_first(
        const BoundingBox& _bbox,
        std::function<bool(int idx)>action,
        const cvector3d& deps)
    {
        std::stack< std::shared_ptr<Cart3DBvh::AABVHNode>>  nodes;
        nodes.push(root);
        BoundingBox bbox = _bbox.expand_safe_box(deps);
        while (!nodes.empty()) {
            std::shared_ptr<Cart3DBvh::AABVHNode> node = nodes.top();
            nodes.pop();
            if (!bbox.overlap(node->bbox))
                continue;
            if (node->is_leaf())
            {
                for (int bid : node->blobids)
                {
                    if (bbox.overlap(blobs[bid].bbox))
                        if (action(blobs[bid].id))
                            return;
                }
            }
            else
            {
                nodes.push(node->left);
                nodes.push(node->right);
            }
        }
    }

    std::shared_ptr<Cart3DBvh::AABVHNode> Cart3DBvh::construct_tree(int begin, int end, int last_dim)
    {
        if (end - begin <= 8) {
            std::shared_ptr<Cart3DBvh::AABVHNode> node = std::make_shared<Cart3DBvh::AABVHNode>();
            node->blobids.resize(end - begin);
            for (int k = 0; k < end - begin; ++k) {
                int blobid = node->blobids[k] = tmpids[begin + k];
                node->bbox.expand_to_include(blobs[blobid].bbox);
            }
            return node;
        }
        int dim = (last_dim + 1) % 3;
        int mid = (begin + end) / 2;
        quick_select(mid, begin, end, dim);
        std::shared_ptr<Cart3DBvh::AABVHNode> node = std::make_shared<Cart3DBvh::AABVHNode>();
        node->left = construct_tree(begin, mid, dim);
        node->right = construct_tree(mid, end, dim);
        node->bbox = node->left->bbox;
        node->bbox.expand_to_include(node->right->bbox);
        return node;
    }

    static inline int randMod(int range) {
        return std::rand() % range;
    }

	void Cart3DBvh::quick_select(int select, int begin, int end, int dim)
	{
        if (end - 1 == select)     return;

        int pi = randMod(end - begin) + begin;
        cfloat pv = blobs[tmpids[pi]].point[dim];

        int front = begin;
        int back = end - 1;
        while (front < back) {
            if (blobs[tmpids[front]].point[dim] < pv) {
                ++front;
            }
            else if (blobs[tmpids[back]].point[dim] > pv) {
                --back;
            }
            else {
                std::swap(tmpids[front], tmpids[back]);
                ++front;
                --back;
            }
        }
        if (front == back && blobs[tmpids[front]].point[dim] <= pv) {
            front++;
        }

        if (select < int(front)) {
            quick_select(select, begin, front, dim);
        }
        else {
            quick_select(select, front, end, dim);
        }

	}
}


#if defined(_MSC_VER)
#pragma warning(pop)
#endif