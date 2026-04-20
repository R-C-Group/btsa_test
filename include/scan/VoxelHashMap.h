#pragma once
// MIT License
//
// Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
// Stachniss.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
// NOTE: This implementation is heavily inspired in the original CT-ICP VoxelHashMap implementation,
// although it was heavily modifed and drastically simplified, but if you are using this module you
// should at least acknoowledge the work from CT-ICP by giving a star on GitHub
// #pragma once

#include <tsl/robin_map.h>

#include <Eigen/Core>
// #include "sophus/se3.h"
#include <vector>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for_each.h>
#include <tbb/parallel_reduce.h>

#include <Eigen/Core>
#include <algorithm>
#include <limits>
#include <tuple>
#include <utility>


struct Belief {
    void update() {
        // value_ += this->num > 0 ? this->sum / this->num : 0.0;
        // this->sum = 0.0;
        // this->num = 0;
        value_ = this->num > 0 ? this->sum / this->num : 0.0;
    }
    void accumulatePartialUpdate(const double &update) {
        this->sum += update;
        this->num++;
    }
    double value_ = 0.0;

protected:
    double sum = 0.0;
    int num = 0;
};

struct VoxelHashMap {
    using Voxel = Eigen::Vector3i;
    struct VoxelBlock {
        // buffer of points with a max limit of n_points
        std::vector<Eigen::Vector3d> points;
        std::vector<int> timestamps;
        Belief belief;
        int num_points_ = 0;
        bool static_voxel_ = false;
        double first_timestamp_ = -1.0;
        double last_timestamp_ = -1.0;
        inline void AddPoint(const Eigen::Vector3d &point, int timestamp) {
            if (points.size() < static_cast<size_t>(num_points_)) {
                points.push_back(point);
                timestamps.push_back(timestamp);
            }
        }
        inline void UpdateTimestamps(double timestamp) {
            if (first_timestamp_ == -1.0 || first_timestamp_ == timestamp) {
                first_timestamp_ = timestamp;
            } else {
                last_timestamp_ = timestamp;
            }
        }
        inline void setStaticVoxel(bool static_voxel) {
            static_voxel_ = static_voxel;
        }
    };
    struct VoxelHash {
        size_t operator()(const Voxel &voxel) const {
            const uint32_t *vec = reinterpret_cast<const uint32_t *>(voxel.data());
            return ((1 << 20) - 1) & (vec[0] * 73856093 ^ vec[1] * 19349669 ^ vec[2] * 83492791);
        }
    };

    explicit VoxelHashMap(double voxel_size, double max_distance, int max_points_per_voxel)
        : voxel_size_(voxel_size),
          max_distance_(max_distance),
          max_points_per_voxel_(max_points_per_voxel) {}

    inline void Clear() { map_.clear(); }
    inline bool Empty() const { return map_.empty(); }
    inline Voxel PointToVoxel(const Eigen::Vector3d &point) const {
        return Voxel(static_cast<int>(std::floor(point.x() / voxel_size_)),
                     static_cast<int>(std::floor(point.y() / voxel_size_)),
                     static_cast<int>(std::floor(point.z() / voxel_size_)));
    }
    void Update(const std::vector<Eigen::Vector3d> &points,
                const Eigen::Vector3d &origin,
                const int timestamp);
    // void Update(const std::vector<Eigen::Vector3d> &points,
    //             const Sophus::SE3 &pose,
    //             const int timestamp);
    void AddPoints(const std::vector<Eigen::Vector3d> &points, const double timestamp);
    std::vector<double> GetBelief(const std::vector<Eigen::Vector3d> &points) const;
    void UpdateBelief(const std::vector<Eigen::Vector3d> &points,
                      const std::vector<double> &updates);
    void RemovePointsFarFromLocation(const Eigen::Vector3d &origin);
    void RemovePointsFarFromTime(const double time_now, const double time_window);
    std::vector<Eigen::Vector3d> Pointcloud() const;
    std::tuple<std::vector<Eigen::Vector3d>, std::vector<int>> PointcloudWithTimestamps() const;
    std::tuple<std::vector<Voxel>, std::vector<double>> VoxelsWithBelief() const;
    std::vector<Eigen::Vector3d> GetPoints(const std::vector<Voxel> &query_voxels) const;

    void filterBoundaryPoints(const std::vector<Eigen::Vector3d> &dynamic_points, std::vector<double> &dynamic_updates, double timestamp);
    // std::vector<Voxel> GetAdjacentVoxels(const Voxel &voxel, int adjacent_voxels = 1);
    void CheckBoundary(const Eigen::Vector3d &points, const std::vector<double> &ifBoundary);
    void setStaticVoxel(const std::vector<Eigen::Vector3d> &points, const double timestamp);
    void filterFalsePositive(const std::vector<Eigen::Vector3d> &dynamic_points, std::vector<double> &dynamic_updates);
    int countStaticVoxel(const std::vector<Eigen::Vector3d> &cluster_points);
    bool isStaticVoxel(const Eigen::Vector3d &point);
    int checkPointLocation(const Eigen::Vector3d &point);
    void clearMap();
    void printParameters();
    void checkStaticVoxelIntersection(const std::vector<Eigen::Vector3d> &points,
                                                  bool &is_static,
                                                  double &overlap_percent,
                                                  Eigen::Vector3d &cluster_bbox_min,
                                                  Eigen::Vector3d &cluster_bbox_max,
                                                  Eigen::Vector3d &static_bbox_min,
                                                  Eigen::Vector3d &static_bbox_max);

    double voxel_size_;
    double max_distance_;
    int max_points_per_voxel_;
    tsl::robin_map<Voxel, VoxelBlock, VoxelHash> map_;
};

void VoxelHashMap::printParameters() {
    std::cout << "VoxelHashMap parameters:" << std::endl;
    std::cout << "     voxel_size: " << voxel_size_ << std::endl;
    std::cout << "     max_distance: " << max_distance_ << std::endl;
    std::cout << "     max_points_per_voxel: " << max_points_per_voxel_ << std::endl;
}

void VoxelHashMap::RemovePointsFarFromTime(const double time_now, const double time_window) {
    for (auto it = map_.begin(); it != map_.end();) {
        const auto &[voxel, voxel_block] = *it;
        if (voxel_block.last_timestamp_ != -1.0 && time_now - voxel_block.last_timestamp_ > time_window) {
            it = map_.erase(it);
        } else {
            ++it;
        }
    }
}

std::vector<Eigen::Vector3d> VoxelHashMap::GetPoints(const std::vector<Voxel> &query_voxels) const {
    std::vector<Eigen::Vector3d> points;
    points.reserve(query_voxels.size() * static_cast<size_t>(max_points_per_voxel_));
    std::for_each(query_voxels.cbegin(), query_voxels.cend(), [&](const auto &query) {
        auto search = map_.find(query);
        if (search != map_.end()) {
            for (const auto &point : search->second.points) {
                points.emplace_back(point);
            }
        }
    });
    return points;
}

std::vector<Eigen::Vector3d> VoxelHashMap::Pointcloud() const {
    std::vector<Eigen::Vector3d> points;
    points.reserve(max_points_per_voxel_ * map_.size());
    for (const auto &[voxel, voxel_block] : map_) {
        (void)voxel;
        for (const auto &point : voxel_block.points) {
            points.push_back(point);
        }
    }
    return points;
}

std::tuple<std::vector<Eigen::Vector3d>, std::vector<int>> VoxelHashMap::PointcloudWithTimestamps()
    const {
    std::vector<Eigen::Vector3d> points;
    std::vector<int> timestamps;
    points.reserve(max_points_per_voxel_ * map_.size());
    for (const auto &[voxel, voxel_block] : map_) {
        (void)voxel;
        for (const auto &point : voxel_block.points) {
            points.push_back(point);
        }
        for (const auto &timestamp : voxel_block.timestamps) {
            timestamps.push_back(timestamp);
        }
    }
    return std::make_tuple(points, timestamps);
}

std::tuple<std::vector<VoxelHashMap::Voxel>, std::vector<double>> VoxelHashMap::VoxelsWithBelief()
    const {
    std::vector<Voxel> voxels;
    std::vector<double> belief;
    for (auto map_element : map_) {
        voxels.push_back(map_element.first);
        belief.push_back(map_element.second.belief.value_);
    }
    return make_tuple(voxels, belief);
}

void VoxelHashMap::Update(const std::vector<Eigen::Vector3d> &points,
                          const Eigen::Vector3d &origin,
                          const int timestamp) {
    AddPoints(points, timestamp);
    RemovePointsFarFromLocation(origin);
}

void VoxelHashMap::clearMap() {
    map_.clear();
}

void VoxelHashMap::AddPoints(const std::vector<Eigen::Vector3d> &points, const double timestamp) {
    std::for_each(points.cbegin(), points.cend(), [&](const auto &point) {
        const auto voxel = PointToVoxel(point);
        auto search = map_.find(voxel);
        if (search != map_.end()) {
            auto &voxel_block = search.value();
            voxel_block.AddPoint(point, timestamp);
            voxel_block.UpdateTimestamps(timestamp);
        } else {
            map_.insert(
                {voxel, VoxelBlock{{point}, {timestamp}, {Belief{}}, max_points_per_voxel_}});
            search = map_.find(voxel);
            auto &voxel_block = search.value();
            voxel_block.UpdateTimestamps(timestamp);
        }
    });
}

void VoxelHashMap::RemovePointsFarFromLocation(const Eigen::Vector3d &origin) {
    const auto max_distance2 = max_distance_ * max_distance_;
    for (auto it = map_.begin(); it != map_.end();) {
        const auto &[voxel, voxel_block] = *it;
        Eigen::Vector3d pt(voxel[0] * voxel_size_, voxel[1] * voxel_size_, voxel[2] * voxel_size_);
        if ((pt - origin).squaredNorm() >= (max_distance2)) {
            it = map_.erase(it);
        } else {
            ++it;
        }
    }
}

void VoxelHashMap::UpdateBelief(const std::vector<Eigen::Vector3d> &points,
                                const std::vector<double> &updates) {
    std::vector<Voxel> voxels_to_update;
    voxels_to_update.reserve(points.size());
    for (size_t i = 0; i < points.size(); i++) {
        auto voxel = PointToVoxel(points[i]);
        voxels_to_update.emplace_back(voxel);
        map_[voxel].belief.accumulatePartialUpdate(updates[i]);
    }

    tbb::parallel_for_each(voxels_to_update.cbegin(), voxels_to_update.cend(),
                           [this](const auto &voxel) { map_[voxel].belief.update(); });
}

std::vector<double> VoxelHashMap::GetBelief(const std::vector<Eigen::Vector3d> &points) const {
    std::vector<double> beliefs(points.size(), 0.0);
    std::transform(points.cbegin(), points.cend(), beliefs.begin(), [this](const auto &p) {
        auto voxel = PointToVoxel(p);
        if (map_.contains(voxel)) {
            return map_.at(voxel).belief.value_;
        }
        return 0.0;
    });
    return beliefs;
} 

void VoxelHashMap::CheckBoundary(const Eigen::Vector3d &points, const std::vector<double> &ifBoundary){

    
}

void VoxelHashMap::filterBoundaryPoints(const std::vector<Eigen::Vector3d> &dynamic_points, std::vector<double> &dynamic_updates, double timestamp) {
    for (size_t i = 0; i < dynamic_points.size(); ++i) {
        const auto &point = dynamic_points[i];
        const auto voxel = PointToVoxel(point);

        // Spatial consistency check
        if (!map_.contains(voxel)) {
            dynamic_updates[i] = 0.0;
            continue;
        }

        // // Temporal consistency check
        // const auto neighbors = GetAdjacentVoxels(voxel);
        // int consistent_neighbors = 0;
        // int total_neighbors = 0;

        // for (const auto &neighbor_voxel : neighbors) {
        //     if (map_.contains(neighbor_voxel)) {
        //         const auto &neighbor_block = map_.at(neighbor_voxel);
        //         if (neighbor_block.last_timestamp_ < timestamp) {
        //             consistent_neighbors++;
        //         }
        //         total_neighbors++;
        //     }
        // }

        // if (total_neighbors > 0 && (static_cast<double>(consistent_neighbors) / total_neighbors) > 0.7) {
        //     dynamic_updates[i] = 0.0;
        // }
    }
}

void VoxelHashMap::setStaticVoxel(const std::vector<Eigen::Vector3d> &points, const double timestamp){
    for (size_t i = 0; i < points.size(); ++i) {
        const auto &point = points[i];
        const auto voxel = PointToVoxel(point);

        // Spatial consistency check
        if (!map_.contains(voxel)) {
            map_.insert(
                {voxel, VoxelBlock{{point}, {timestamp}, {Belief{}}, max_points_per_voxel_}});
            auto &voxel_block = map_.at(voxel);
            voxel_block.num_points_++;
            if(voxel_block.num_points_ > 30) voxel_block.setStaticVoxel(true);
            // voxel_block.setStaticVoxel(true);
            voxel_block.UpdateTimestamps(timestamp);
            continue;
        }

        // Temporal consistency check
        auto &voxel_block = map_.at(voxel);
        voxel_block.num_points_++;
        if(voxel_block.num_points_ > 30) voxel_block.setStaticVoxel(true);
        // voxel_block.setStaticVoxel(true);
        voxel_block.UpdateTimestamps(timestamp);
    }

    // std::for_each(points.cbegin(), points.cend(), [&](const auto &point) {
    //     const auto voxel = PointToVoxel(point);
    //     auto search = map_.find(voxel);
    //     if (search != map_.end()) {
    //         auto &voxel_block = search.value();
    //         voxel_block.AddPoint(point, timestamp);
    //         voxel_block.UpdateTimestamps(timestamp);
    //         voxel_block.setStaticVoxel(true);
    //     } else {
    //         map_.insert(
    //             {voxel, VoxelBlock{{point}, {timestamp}, {Belief{}}, max_points_per_voxel_}});
    //         search = map_.find(voxel);
    //         auto &voxel_block = search.value();
    //         voxel_block.UpdateTimestamps(timestamp);
    //         voxel_block.setStaticVoxel(true);
    //     }
    // });
}

void VoxelHashMap::filterFalsePositive(const std::vector<Eigen::Vector3d> &dynamic_points, std::vector<double> &dynamic_updates) {
    for (size_t i = 0; i < dynamic_points.size(); ++i) {
        const auto &point = dynamic_points[i];
        const auto voxel = PointToVoxel(point);

        // print the Voxel
        // printf("Voxel: %d, %d, %d\n", voxel[0], voxel[1], voxel[2]);

        // Spatial consistency check
        if (!map_.contains(voxel)) {
            dynamic_updates[i] = 1;
            // printf("Non-existing voxel\n");
            // continue;
        }

        // Temporal consistency check
        // const auto &voxel_block = map_.at(voxel);
        // if (voxel_block.static_voxel_) {
        //     dynamic_updates[i] = 0.0;
        // }
    }
}

int VoxelHashMap::countStaticVoxel(const std::vector<Eigen::Vector3d> &cluster_points) {
    int num_static_points = 0;
    for (size_t i = 0; i < cluster_points.size(); ++i) {
        const auto &point = cluster_points[i];
        const auto voxel = PointToVoxel(point);

        // Spatial consistency check
        if (!map_.contains(voxel)) {
            continue;
        }

        // Temporal consistency check
        const auto &voxel_block = map_.at(voxel);
        if (voxel_block.static_voxel_) {
            num_static_points++;
        }
    }
    return num_static_points;
}

bool VoxelHashMap::isStaticVoxel(const Eigen::Vector3d &point){
    const auto voxel = PointToVoxel(point);
    if (!map_.contains(voxel)) {
        return false;
    }
    const auto &voxel_block = map_.at(voxel);
    return voxel_block.static_voxel_;
}

int VoxelHashMap::checkPointLocation(const Eigen::Vector3d &point) {
    const auto voxel = PointToVoxel(point);
    if (!map_.contains(voxel)) {
        return 1; // Non-existing voxel
    }
    const auto &voxel_block = map_.at(voxel);
    if (voxel_block.static_voxel_) {
        return 0; // Static voxel
    }
    return -1; // Neither static nor non-existing voxel
}


void VoxelHashMap::checkStaticVoxelIntersection(
    const std::vector<Eigen::Vector3d> &points,
    bool &is_static,
    double &overlap_percent,
    Eigen::Vector3d &cluster_bbox_min,
    Eigen::Vector3d &cluster_bbox_max,
    Eigen::Vector3d &static_bbox_min,
    Eigen::Vector3d &static_bbox_max) {
    
    // Handle empty point set
    if (points.empty()) {
        is_static = false;
        overlap_percent = 0.0;
        cluster_bbox_min = cluster_bbox_max = static_bbox_min = static_bbox_max = Eigen::Vector3d::Zero();
        return;
    }

    // Compute cluster bounding box
    cluster_bbox_min = cluster_bbox_max = points.front();
    for (const auto &pt : points) {
        cluster_bbox_min = cluster_bbox_min.cwiseMin(pt);
        cluster_bbox_max = cluster_bbox_max.cwiseMax(pt);
    }

    // Calculate cluster volume
    const Eigen::Vector3d cluster_size = cluster_bbox_max - cluster_bbox_min;
    const double cluster_volume = std::max(0.0, cluster_size.x()) * 
                                 std::max(0.0, cluster_size.y()) * 
                                 std::max(0.0, cluster_size.z());

    // Quick volume-based decisions
    if (cluster_volume > 40.0) {
        is_static = true;
        overlap_percent = 0.0;
        static_bbox_min = static_bbox_max = Eigen::Vector3d::Zero();
        return;
    }

    // if (cluster_volume < 4.0) {
    //     is_static = false;
    //     overlap_percent = 0.0;
    //     static_bbox_min = static_bbox_max = Eigen::Vector3d::Zero();
    //     return;
    // }

    // Find static voxels bounding box
    bool found_static = false;
    Eigen::Vector3d s_min, s_max;

    // Calculate the number of different voxels on which dynamic points fall in the static map
    std::set<Voxel, std::function<bool(const Voxel&, const Voxel&)>> static_voxels(
        [](const Voxel& a, const Voxel& b) {
            return (a[0] < b[0]) || 
                  (a[0] == b[0] && a[1] < b[1]) || 
                  (a[0] == b[0] && a[1] == b[1] && a[2] < b[2]);
        }
    );
    
    for (const auto &pt : points) {
        const auto voxel = PointToVoxel(pt);
        
        if (!map_.contains(voxel)) continue;
        
        const auto &voxel_block = map_.at(voxel);
        if (!voxel_block.static_voxel_) continue;
        
        // Add this static voxel to our set
        static_voxels.insert(voxel);
        found_static = true;
    }

    if (!found_static) {
        is_static = false;
        overlap_percent = 0.0;
        return;
    }

    int num_static_voxels = static_voxels.size();
    double intersection_volume = voxel_size_ * voxel_size_ * voxel_size_ * num_static_voxels;
    // Calculate the percent overlap relative to the cluster bounding box.
    overlap_percent = intersection_volume / cluster_volume;
    
    // Set a threshold for considering the cluster as static. For example, 50%.
    is_static = overlap_percent >= 0.7;
    
}
