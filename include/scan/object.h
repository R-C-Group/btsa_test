#pragma once

#include <Eigen/Dense>
#include <vector>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef Eigen::Matrix<double, 7, 1> Vector7d;
typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 10, 1> Vector10d;
typedef pcl::PointXYZINormal PointType;

enum ObjectStatus
{
  UNDEFINED,
  STATIC,
  DYNAMIC
};

struct Object
{
    int id = -1;
    // Vector10d state;  // [x, y, z, heading, length, width, height, vx, vy, vz]
    Vector6d state;  // [x, y, z, length, width, height]
    int num_points;
    float density;             // points / m^2
    float avg_residuum = 0.0;  // m, average
    ObjectStatus status = UNDEFINED;
    Eigen::Vector3d min;
    Eigen::Vector3d max;
    Eigen::Vector3d min_expanded;
    Eigen::Vector3d max_expanded;
    std::vector<int> indices;
};

Object getObject(const pcl::PointCloud<PointType>& cloud)
{
    // Minimal Bounding Box
    PointType orig_min_point, orig_max_point;
    pcl::getMinMax3D(cloud, orig_min_point, orig_max_point);
    
    const double mean_X = 0.5f * (orig_max_point.x + orig_min_point.x);
    const double mean_Y = 0.5f * (orig_max_point.y + orig_min_point.y);
    const double mean_Z = 0.5f * (orig_max_point.z + orig_min_point.z);

    Eigen::Vector3d dimension(orig_max_point.x - orig_min_point.x, orig_max_point.y - orig_min_point.y, orig_max_point.z - orig_min_point.z);

    Object obj;
    obj.state << mean_X, mean_Y, mean_Z, dimension[0], dimension[1], dimension[2];  // [x, y, z, length, width, height]

    obj.num_points = cloud.size();
    double volume = dimension[0] * dimension[1] * dimension[2];
    obj.density = volume > 0 ? cloud.size() / volume : 0;

    obj.min = Eigen::Vector3d(orig_min_point.x, orig_min_point.y, orig_min_point.z);
    obj.max = Eigen::Vector3d(orig_max_point.x, orig_max_point.y, orig_max_point.z);

    // Expand the bounding box
    // In z+ direction: extend by 1/2 of the height
    // In x+, x-, y+, y- directions: extend by 1/3 of the length and width
    const double x_expansion = dimension[0] / 3.0;
    const double y_expansion = dimension[1] / 3.0;
    const double z_expansion = dimension[2] / 2.0;
    
    obj.min_expanded = Eigen::Vector3d(
        orig_min_point.x - x_expansion,
        orig_min_point.y - y_expansion,
        orig_min_point.z);
    
    obj.max_expanded = Eigen::Vector3d(
        orig_max_point.x + x_expansion,
        orig_max_point.y + y_expansion,
        orig_max_point.z + z_expansion);


    return obj;
}

bool isPointInObject(const Object& obj, const Eigen::Vector3d& point)
{
    // Check if the point is within the bounding box of the object
    return (point.x() >= obj.min.x() && point.x() <= obj.max.x() &&
            point.y() >= obj.min.y() && point.y() <= obj.max.y() &&
            point.z() >= obj.min.z() && point.z() <= obj.max.z());
}

// Overloaded function for PCL point type
bool isPointInObject(const Object& obj, const PointType& point)
{
    return (point.x >= obj.min.x() && point.x <= obj.max.x() &&
            point.y >= obj.min.y() && point.y <= obj.max.y() &&
            point.z >= obj.min.z() && point.z <= obj.max.z());
}

// Function to check if a point is in the expanded bounding box
bool isPointInExpandedObject(const Object& obj, const Eigen::Vector3d& point)
{
    return (point.x() >= obj.min_expanded.x() && point.x() <= obj.max_expanded.x() &&
            point.y() >= obj.min_expanded.y() && point.y() <= obj.max_expanded.y() &&
            point.z() >= obj.min_expanded.z() && point.z() <= obj.max_expanded.z());
}

// Overloaded function for PCL point type
bool isPointInExpandedObject(const Object& obj, const PointType& point)
{
    return (point.x >= obj.min_expanded.x() && point.x <= obj.max_expanded.x() &&
            point.y >= obj.min_expanded.y() && point.y <= obj.max_expanded.y() &&
            point.z >= obj.min_expanded.z() && point.z <= obj.max_expanded.z());
}
