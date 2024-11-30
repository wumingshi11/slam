# ivox
本片主要看iovx的线性化实现。相对简单
## 主要数据结构
```c++
    std::unordered_map<KeyType, typename std::list<std::pair<KeyType, NodeType>>::iterator, hash_vec<dim>>
        grids_map_;                                        // voxel hash map
    // 真正存储数据
    std::list<std::pair<KeyType, NodeType>> grids_cache_;  // voxel cache
```
grids_map_起索引作用，grids_cache_查找数据。

node的结构更简单
```c++
template <typename PointT, int dim = 3>
class IVoxNode {
   public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

    struct DistPoint;

    IVoxNode() = default;
    IVoxNode(const PointT& center, const float& side_length) {}  /// same with phc
    // 操作很简单，都是直接调用
    void InsertPoint(const PointT& pt);

    inline bool Empty() const;

    inline std::size_t Size() const;

    inline PointT GetPoint(const std::size_t idx) const;
    // 暴力搜索，以为一个node里数据量比较少，速度很可以
    int KNNPointByCondition(std::vector<DistPoint>& dis_points, const PointT& point, const int& K,
                            const double& max_range);

   private:
   // 唯一数据结构
    std::vector<PointT> points_;
};

// DistPoint 返回的数据结构
template <typename PointT, int dim>
struct IVoxNode<PointT, dim>::DistPoint {
    // 距离查找点的距离
    double dist = 0;
    IVoxNode* node = nullptr;
    int idx = 0;

    DistPoint() = default;
    DistPoint(const double d, IVoxNode* n, const int i) : dist(d), node(n), idx(i) {}

    PointT Get() { return node->GetPoint(idx); }

    inline bool operator()(const DistPoint& p1, const DistPoint& p2) { return p1.dist < p2.dist; }

    inline bool operator<(const DistPoint& rhs) { return dist < rhs.dist; }
};

```

## 操作
### 查询
查询是频率最高的操作。查询需要根据配置在几个（1，7，15，27）个node中查找各自最近n个点，然后合并，排序。挑选最近的n个点。
```c++
// 查找最近点，查找k近邻类似
template <int dim, IVoxNodeType node_type, typename PointType>
bool IVox<dim, node_type, PointType>::GetClosestPoint(const PointType& pt, PointType& closest_pt) {
    std::vector<DistPoint> candidates;
    auto key = Pos2Grid(ToEigen<float, dim>(pt));
    std::for_each(nearby_grids_.begin(), nearby_grids_.end(), [&key, &candidates, &pt, this](const KeyType& delta) {
        auto dkey = key + delta;
        auto iter = grids_map_.find(dkey);
        if (iter != grids_map_.end()) {
            DistPoint dist_point;
            bool found = iter->second->second.NNPoint(pt, dist_point);
            if (found) {
                candidates.emplace_back(dist_point);
            }
        }
    });

    if (candidates.empty()) {
        return false;
    }

    auto iter = std::min_element(candidates.begin(), candidates.end());
    closest_pt = iter->Get();
    return true;
}
```
### 插入 和 删除
```c++
template <int dim, IVoxNodeType node_type, typename PointType>
void IVox<dim, node_type, PointType>::AddPoints(const PointVector& points_to_add) {
    // 并发插入，这儿有bug
    std::for_each(std::execution::unseq, points_to_add.begin(), points_to_add.end(), [this](const auto& pt) {
        auto key = Pos2Grid(ToEigen<float, dim>(pt));

        auto iter = grids_map_.find(key);
        if (iter == grids_map_.end()) {
            PointType center;
            center.getVector3fMap() = key.template cast<float>() * options_.resolution_;
            // 放在最前面，同时更新索引和data
            grids_cache_.push_front({key, NodeType(center, options_.resolution_)});
            grids_map_.insert({key, grids_cache_.begin()});
            // 在node中插入点
            // bug: 并发插入可能造成数据混乱
            grids_cache_.front().second.InsertPoint(pt);
            // 基于LRU策略删除过期数据，这儿也有并发问题bug，不过不会core
            if (grids_map_.size() >= options_.capacity_) {
                grids_map_.erase(grids_cache_.back().first);
                grids_cache_.pop_back();
            }
        } else {
            iter->second->second.InsertPoint(pt);
            // a.splice(a.iter,b,b.iter); 将b.iter指向元素中元素移动到a.iter之前，移动后迭代器失效
            grids_cache_.splice(grids_cache_.begin(), grids_cache_, iter->second);
            grids_map_[key] = grids_cache_.begin();
        }
    });
}
```
