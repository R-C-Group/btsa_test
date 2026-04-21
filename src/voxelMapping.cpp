// This is an advanced implementation of the algorithm described in the
// following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.

// Modifier: Livox               dev@livoxtech.com

// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#include <omp.h>
#include <mutex>
#include <math.h>
#include <thread>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <Python.h>
#include <so3_math.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include "IMU_Processing.hpp"
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <livox_ros_driver/CustomMsg.h>
#include "preprocess.h"
#include "voxel_map_util.hpp"
#include "scan/ikd_Tree_impl.h"
#include "scan/utils.h"
#include "scan/DBSCAN_kdtree.h"
#include "scan/DBSCAN_precomp.h"
#include "scan/DBSCAN_simple.h"
#include "scan/VoxelHashMap.h"
#include "scan/object.h"
#include <execution>
#include <pcl/kdtree/kdtree_flann.h>
#include <unordered_set>
#include <pcl/filters/extract_indices.h>
#include <filesystem>

#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)
#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)

/*** Time Log Variables ***/
double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
double match_time = 0, solve_time = 0, solve_const_H_time = 0;
int    kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
bool   time_sync_en = false, extrinsic_est_en = true, path_en = true;
double lidar_time_offset = 0.0;
/**************************/

float res_last[100000] = {0.0};
float DET_RANGE = 300.0f;
const float MOV_THRESHOLD = 1.5f;

mutex mtx_buffer;
condition_variable sig_buffer;

string root_dir = ROOT_DIR;
string  lid_topic, imu_topic;

double res_mean_last = 0.05, total_residual = 0.0;
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double filter_size_surf_min = 0;
double filter_size_map_min = 0;
double total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
int    effct_feat_num = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
int    iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0, pcd_index = 0;
bool   point_selected_surf[100000] = {0};
bool   lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;
bool   scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;


vector<vector<int>>  pointSearchInd_surf;
vector<PointVector>  Nearest_Points;
vector<double>       extrinT(3, 0.0);
vector<double>       extrinR(9, 0.0);
deque<double>                     time_buffer;
deque<PointCloudXYZI::Ptr>        lidar_buffer;
deque<sensor_msgs::Imu::ConstPtr> imu_buffer;

PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_undistort_world(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_dynamic(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_static_world(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr cloud_false_negatives(new PointCloudXYZI());
//PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1));
//PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1));
PointCloudXYZI::Ptr _featsArray;
std::vector<M3D> var_down_body;

pcl::VoxelGrid<PointType> downSizeFilterSurf;

std::vector<float> nn_dist_in_feats;
std::vector<float> nn_plane_std;
PointCloudXYZI::Ptr feats_with_correspondence(new PointCloudXYZI());

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);
V3D euler_cur;
V3D position_last(Zero3d);
V3D Lidar_T_wrt_IMU(Zero3d);
M3D Lidar_R_wrt_IMU(Eye3d);

// params for voxel mapping algorithm
double min_eigen_value = 0.003;
int max_layer = 0;

int max_cov_points_size = 50;
int max_points_size = 50;
double sigma_num = 2.0;
double max_voxel_size = 1.0;
std::vector<int> layer_size;

double ranging_cov = 0.0;
double angle_cov = 0.0;
std::vector<double> layer_point_size;

bool publish_voxel_map = false;
int publish_max_voxel_layer = 0;

std::unordered_map<VOXEL_LOC, OctoTree *> voxel_map;

// scan
// ---------- 动态场景 / 动态点检测相关全局量（中文说明）----------
// ikdtree：维护「历史帧」地图点的 ikd-Tree；在观测里对当前帧世界坐标点做近邻搜索，与局部平面比较以打分。
// TimeWindow_Points：按帧缓存的世界系点云队列；超过 time_slice 后触发滑窗更新并开启 flg_dynamic_detect。
// dynamic_scores：每个下采样点的「动态程度」标量；与 btsa/vel_thre 比较决定是否视为动态、是否参与 EKF 平面约束。
// out_of_crop_index：裁剪盒外点标记为 1，动态检测阶段跳过（避免远处噪声干扰）。
// feats_dynamic / feats_static_world / cloud_dynamic_refine：动态候选点、静态点、DBSCAN+SCC 后的动态点发布云。
// voxel_hash_map_：时空体素一致性检查（SCC），剔除与静态体素重叠的误检动态簇。
ikdtreeNS::KD_TREE<PointType> ikdtree;
int time_slice;
deque<PointVector>  TimeWindow_Points; 
bool flg_dynamic_detect = false;
double vel_thre;
Eigen::VectorXf dynamic_scores;
Eigen::VectorXf out_of_crop_index;
double range_x, range_y, range_z;
std::vector<bool> point_selected_surf_; 
PointCloudXYZI::Ptr feats_unstable(new PointCloudXYZI());
PointCloudXYZI::Ptr cloud_clustered(new PointCloudXYZI());
PointCloudXYZI::Ptr cloud_dynamic_refine(new PointCloudXYZI());
int match_points_num = 20;
Timer timer;
bool stop_detect = false;
int detect_cnt = 0;
bool pcd_save_en = false;
int pcd_save_interval = -1;
int detect_cnt_thre = 2;
bool if_detect_drop = false;
bool use_radius_search_ = false; // use_radius_search_for_dynamic
int neighborhood_size_;
float neighbourhood_radius_;
std::vector<Eigen::Vector3d> static_points;
VoxelHashMap voxel_hash_map_{0.1, 500, 1000};
double scc_voxel_size_;
double scc_voxel_time_;

/*** EKF inputs and output ***/
MeasureGroup Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
state_ikfom state_point;
vect3 pos_lid;

nav_msgs::Path path;
nav_msgs::Odometry odomAftMapped;
geometry_msgs::Quaternion geoQuat;
geometry_msgs::PoseStamped msg_body_pose;

shared_ptr<Preprocess> p_pre(new Preprocess());
shared_ptr<ImuProcess> p_imu(new ImuProcess());

//scan
// 动态上采样：对「当前帧中被判为动态」的下采样点，在整帧去畸变世界系点云中找邻域，
// 把邻域索引并入动态集合，从而在 feats_dynamic 中扩张动态区域（减少漏检）。
void dynamic_detection_upsampling(){
    if (!flg_dynamic_detect) return;
    if (!feats_undistort_world || feats_undistort_world->empty()) return;
    if (!feats_down_world || feats_down_world->empty()) return;

    // Build a PCL kd-tree for the current map points.
    pcl::KdTreeFLANN<PointType> kdTree;
    kdTree.setInputCloud(feats_undistort_world);
    feats_dynamic->clear();
    feats_static_world->clear();
    // feats_static_downsample->clear();
    
    std::vector<int> neighborIndices;
    std::vector<float> neighborSqDist;
    std::unordered_set<int> dynamic_indices_temp;
    dynamic_indices_temp.clear();
    for (size_t i = 0; i < feats_down_world->points.size(); ++i)
    {
        if (i >= static_cast<size_t>(dynamic_scores.size())) break;
        if (dynamic_scores(i) >= vel_thre)
        {
            if(use_radius_search_){
                if (kdTree.radiusSearch(feats_down_world->points[i], neighbourhood_radius_, neighborIndices, neighborSqDist) > 0)
                {
                    // Add each neighbor from feats_down_world to the dynamic cloud.
                    for (int idx : neighborIndices)
                    {
                        dynamic_indices_temp.insert(idx);
                    }
                }
            }else{
                if (kdTree.nearestKSearch (feats_down_world->points[i], neighborhood_size_, neighborIndices, neighborSqDist) >0)
                {
                    // Add each neighbor from feats_down_world to the dynamic cloud.
                    for (int idx : neighborIndices)
                    {
                        dynamic_indices_temp.insert(idx);
                    }
                }
            }
        }
        // else{
        //     feats_static_downsample->points.push_back(feats_down_world->points[i]);
        // }
    }
    // add dynamic points to feats_dynamic_body based on the indices
    // for (int idx : dynamic_indices_temp)
    // {
    //     feats_dynamic_body->points.push_back(feats_undistort_world->points[idx]);
    // }
    for (size_t i = 0; i < feats_undistort_world->size(); i++)
    {
        if (dynamic_indices_temp.find(i) != dynamic_indices_temp.end())
        {
            feats_dynamic->points.push_back(feats_undistort_world->points[i]);
        }else{
            feats_static_world->points.push_back(feats_undistort_world->points[i]);
        }
    }
}

// 空间一致性：对 feats_dynamic 做 DBSCAN 聚类；结合静态点云与 VoxelHashMap 做体素相交检验，
// 剔除与静态结构重叠的簇（降低假阳性），通过检查的簇写入 cloud_dynamic_refine 用于发布。
void SpatialConsistencyCheck(){
    if (!flg_dynamic_detect) return;
    if (!feats_dynamic || feats_dynamic->empty()) return;
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>);
    tree->setInputCloud(feats_dynamic);

    DBSCANKdtreeCluster<PointType> dbs;
    dbs.setCorePointMinPts(3);
    dbs.setClusterTolerance(0.5);
    dbs.setMinClusterSize(5);
    dbs.setMaxClusterSize(25000);
    dbs.setSearchMethod(tree);
    dbs.setInputCloud(feats_dynamic);
    std::vector<pcl::PointIndices> cluster_indices;
    dbs.extract(cluster_indices);

    pcl::KdTreeFLANN<PointType> tree_static;
    tree_static.setInputCloud(feats_static_world);

    // clear cloud_clustered
    cloud_clustered->clear();
    std::unordered_set<int> false_negatives_set;
    pcl::PointIndices::Ptr false_negatives(new pcl::PointIndices());
    false_negatives->indices.clear();
    cloud_false_negatives->clear();
    for (const auto& cluster : cluster_indices) {
        pcl::PointCloud<PointType> cluster_temp;
        cluster_temp.clear();
        // Use intensity to differentiate clusters, assign a unique intensity to each cluster
        float intensity = static_cast<float>(&cluster - &cluster_indices[0]) / cluster_indices.size();
        for (const auto& index : cluster.indices) {
            PointType point;
            point.x = feats_dynamic->points[index].x;
            point.y = feats_dynamic->points[index].y;
            point.z = feats_dynamic->points[index].z;
            point.intensity = intensity;
            cloud_clustered->points.push_back(point);
            cluster_temp.push_back(point);
        }
    
        Object obj = getObject(cluster_temp);
        Eigen::Vector3d cluster_min = obj.min;
        Eigen::Vector3d cluster_max = obj.max;
        double volume = (cluster_max[0] - cluster_min[0]) * (cluster_max[1] - cluster_min[1]) *
                (cluster_max[2] - cluster_min[2]);
        if (volume > 20.0) continue;
        std::vector<double> dimensions({ obj.state[3], obj.state[4], obj.state[5] });
        std::sort(dimensions.begin(), dimensions.end());  // smallest to largest
        if (dimensions[2] / dimensions[1] >= 10.0) continue;


        PointType cluster_center_point;
        cluster_center_point.x = static_cast<float>(obj.state[0]);
        cluster_center_point.y = static_cast<float>(obj.state[1]);
        cluster_center_point.z = static_cast<float>(obj.state[2]);
        Eigen::Vector3f cluster_scale = cluster_max.cast<float>() - cluster_min.cast<float>();
        float diagonal = (cluster_max - cluster_min).norm() / 2;


        std::vector<int> neighborIndices;
        neighborIndices.clear();
        std::vector<float> neighborSqDist;
        neighborSqDist.clear();
        if (tree_static.radiusSearch(cluster_center_point, diagonal * 1.1, neighborIndices, neighborSqDist) > 0)
        {
            for (int idx : neighborIndices)
            {
                // if(isPointInExpandedObject(obj, feats_static_world->points[idx])){
                    false_negatives_set.insert(idx);
                // }
            }
        }
    }

    for(auto idx : false_negatives_set){
        false_negatives->indices.push_back(idx);
        cloud_false_negatives->points.push_back(feats_static_world->points[idx]);
    }
    cloud_clustered->width = cloud_clustered->points.size();
    cloud_clustered->height = 1;

    pcl::ExtractIndices<PointType> extract;
    extract.setInputCloud(feats_static_world);
    extract.setIndices(false_negatives);
    extract.setNegative(true);
    PointCloudXYZI::Ptr cloud_filtered(new PointCloudXYZI());
    extract.filter(*feats_static_world);
    static_points.clear();

    for (size_t i = 0; i < feats_static_world->size(); i++)
    {
        static_points.push_back(Eigen::Vector3d(feats_static_world->points[i].x, feats_static_world->points[i].y, feats_static_world->points[i].z));
    }
    
    voxel_hash_map_.setStaticVoxel(static_points, lidar_end_time);
    cloud_dynamic_refine->clear();
    for (auto& getIndices: cluster_indices) {
        std::vector<Eigen::Vector3d> temp_cluster_points;
        for (auto& index : getIndices.indices) {
            temp_cluster_points.push_back(Eigen::Vector3d(feats_dynamic->points[index].x, feats_dynamic->points[index].y, feats_dynamic->points[index].z));
        }
        bool is_static = false;
        double overlap_percent = 0;
        Eigen::Vector3d cluster_bbox_min;
        Eigen::Vector3d cluster_bbox_max;
        Eigen::Vector3d static_bbox_min;
        Eigen::Vector3d static_bbox_max;
        voxel_hash_map_.checkStaticVoxelIntersection(temp_cluster_points, is_static, overlap_percent, cluster_bbox_min, cluster_bbox_max, static_bbox_min, static_bbox_max);
        if(!is_static){
            for (auto& index : getIndices.indices) {
                PointType point;
                point.x = feats_dynamic->points[index].x;
                point.y = feats_dynamic->points[index].y;
                point.z = feats_dynamic->points[index].z;
                point.intensity = 1.0; // Set intensity to 1.0 for dynamic points
                cloud_dynamic_refine->points.push_back(point);
            }
        }
    }
    voxel_hash_map_.RemovePointsFarFromTime(lidar_end_time, scc_voxel_time_);
}

void publish_dynamic_points(const ros::Publisher & pubLaserCloudDynamic)
{
    if(scan_pub_en)
    {   
        sensor_msgs::PointCloud2 laserCloudmsg;
        PointCloudXYZI::Ptr laserCloudFullRes(cloud_dynamic_refine);
        pcl::toROSMsg(*laserCloudFullRes, laserCloudmsg);
        laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.frame_id = "camera_init";
        pubLaserCloudDynamic.publish(laserCloudmsg);
    }
}

void puslish_unstable_points(const ros::Publisher & pubLaserCloudUnstable)
{
    if(flg_dynamic_detect)
    {   
        sensor_msgs::PointCloud2 laserCloudmsg;
        PointCloudXYZI::Ptr laserCloudFullRes(feats_unstable);
        // PointCloudXYZI::Ptr laserCloudFullRes(cloud_dynamic_refine);
        pcl::toROSMsg(*laserCloudFullRes, laserCloudmsg);
        laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.frame_id = "camera_init";
        pubLaserCloudUnstable.publish(laserCloudmsg);
    }
}

// 将当前 LIO 状态（时间、位置、姿态四元数）追加写入 Log/pos_log.txt；fp 为空则跳过（避免崩溃）。
inline void dump_lio_state_to_log(FILE *fp)  
{
    if (fp == nullptr) return;
    V3D rot_ang(Log(state_point.rot.toRotationMatrix()));
    geometry_msgs::Quaternion q_debug = tf::createQuaternionMsgFromRollPitchYaw(rot_ang(0),rot_ang(1),rot_ang(2));
    fprintf( fp, "%lf ", Measures.lidar_beg_time); // Time   [0]
    fprintf( fp, "%lf %lf %lf ", state_point.pos(0), state_point.pos(1), state_point.pos(2) );          // Pos    [4-6]
    fprintf( fp, "%lf %lf %lf %lf", q_debug.x, q_debug.y , q_debug.z,  q_debug.w); // omega  [7-9]
    fprintf( fp, "\n" );
    fflush( fp );
}

void SigHandle(int sig)
{
    flg_exit = true;
    ROS_WARN("catch sig %d", sig);
    sig_buffer.notify_all();
}

const bool var_contrast(pointWithCov &x, pointWithCov &y) {
    return (x.cov.diagonal().norm() < y.cov.diagonal().norm());
};

void pointBodyToWorld_ikfom(PointType const * const pi, PointType * const po, state_ikfom &s)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}


void pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
//    po->intensity = pi->intensity;
}

template<typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}

void RGBpointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

// 激光地图时间滑窗：累积帧数超过 time_slice 后，(1) 打开动态检测总开关 flg_dynamic_detect；
// (2) 从 ikdtree 删除最旧一帧点，保持地图时间厚度；(3) 生成当前帧去畸变点云的世界系副本供动态模块使用。
void lasermap_timeslice_segment(){
    if(TimeWindow_Points.size() > time_slice){
        {
            flg_dynamic_detect = true;
            PointVector oldestCloud = TimeWindow_Points.front();
            TimeWindow_Points.pop_front();
            ikdtree.Delete_Points(oldestCloud);
            // flg_dynamic_detect = false;
        } 
        feats_undistort_world->resize(feats_undistort->size());
        feats_undistort_world->points = feats_undistort->points;
        for (size_t i = 0; i < feats_undistort_world->points.size(); i++) {
            pointBodyToWorld(&(feats_undistort->points[i]), &(feats_undistort_world->points[i]));
        }
    }
    // if(scan_count < start_frame) flg_dynamic_detect = false;
}


void RGBpointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I*p_body_lidar + state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;

    po->curvature = pi->curvature;
    po->normal_x = pi->normal_x;
}

void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    auto time_offset = lidar_time_offset;
//    std::printf("lidar offset:%f\n", lidar_time_offset);
    mtx_buffer.lock();
    scan_count ++;
    double preprocess_start_time = omp_get_wtime();
    if (msg->header.stamp.toSec() + time_offset < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    if (p_pre->lidar_type == ROBOSENSE && ptr->points.empty()) {
      ROS_WARN_THROTTLE(
          5.0,
          "RoboSense 模式预处理后点云仍为空：请确认 common/lid_topic/common/imu_topic 与 "
          "bag 一致；点云需含 x,y,z（无 timestamp/ring 时会自动回退 XYZI）。");
    }
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(msg->header.stamp.toSec() + time_offset);
    last_timestamp_lidar = msg->header.stamp.toSec() + time_offset;
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

double timediff_lidar_wrt_imu = 0.0;
bool   timediff_set_flg = false;
void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg)
{
    mtx_buffer.lock();
    double preprocess_start_time = omp_get_wtime();
    scan_count ++;
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }
    last_timestamp_lidar = msg->header.stamp.toSec();

    if (!time_sync_en && abs(last_timestamp_imu - last_timestamp_lidar) > 10.0 && !imu_buffer.empty() && !lidar_buffer.empty() )
    {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n",last_timestamp_imu, last_timestamp_lidar);
    }

    if (time_sync_en && !timediff_set_flg && abs(last_timestamp_lidar - last_timestamp_imu) > 1 && !imu_buffer.empty())
    {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu = last_timestamp_lidar + 0.1 - last_timestamp_imu;
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(last_timestamp_lidar);

    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in)
{
    publish_count ++;
    // cout<<"IMU got at: "<<msg_in->header.stamp.toSec()<<endl;
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
    {
        msg->header.stamp = \
        ros::Time().fromSec(timediff_lidar_wrt_imu + msg_in->header.stamp.toSec());
    }

    double timestamp = msg->header.stamp.toSec();

    if (timestamp < last_timestamp_imu)
    {
//        ROS_WARN("imu loop back, clear buffer");
//        imu_buffer.clear();
        ROS_WARN("imu loop back, ignoring!!!");
        ROS_WARN("current T: %f, last T: %f", timestamp, last_timestamp_imu);
        return;
    }
    if (std::abs(msg->angular_velocity.x) > 10
        || std::abs(msg->angular_velocity.y) > 10
        || std::abs(msg->angular_velocity.z) > 10) {
        ROS_WARN("Large IMU measurement!!! Drop Data!!! %.3f  %.3f  %.3f",
                 msg->angular_velocity.x,
                 msg->angular_velocity.y,
                 msg->angular_velocity.z
        );
        return;
    }

//    if (is_first_imu) {
//        double acc_vec[3] = {msg_in->linear_acceleration.x, msg_in->linear_acceleration.y, msg_in->linear_acceleration.z};
//
//        R__world__o__initial = SO3(g2R(Eigen::Vector3d(acc_vec)));
//
//        is_first_imu = false;
//    }

    last_timestamp_imu = timestamp;

    mtx_buffer.lock();

    imu_buffer.push_back(msg);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

double lidar_mean_scantime = 0.0;
int    scan_num = 0;
// 同步一包激光与其时间范围内的 IMU：根据点曲率（存的是 ms 级相对扫描时间）估计扫描结束时刻，
// 再从 imu_buffer 中弹出 [beg, lidar_end_time) 的 IMU 序列，供 IMU 初始化与去畸变使用。
bool sync_packages(MeasureGroup &meas)
{
    if (lidar_buffer.empty() || imu_buffer.empty()) {
        return false;
    }

    /*** push a lidar scan ***/
    if(!lidar_pushed)
    {
        meas.lidar = lidar_buffer.front();
        meas.lidar_beg_time = time_buffer.front();
        if (meas.lidar->points.size() <= 1) // time too little
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
            ROS_WARN("Too few input point cloud!\n");
        }
        else if (meas.lidar->points.back().curvature / double(1000) < 0.5 * lidar_mean_scantime)
        {
            lidar_end_time = meas.lidar_beg_time + lidar_mean_scantime;
        }
        else
        {

//            std::printf("\nFirst 100 points: \n");
//            for(int i=0; i < 100; ++i){
//                std::printf("%f ", meas.lidar->points[i].curvature  / double(1000));
//            }
//
//            std::printf("\n Last 100 points: \n");
//            for(int i=100; i >0; --i){
//                std::printf("%f ", meas.lidar->points[meas.lidar->size() - i - 1].curvature / double(1000));
//            }
//            std::printf("last point offset time: %f\n", meas.lidar->points.back().curvature / double(1000));
            scan_num ++;
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
//            lidar_end_time = meas.lidar_beg_time + (meas.lidar->points[meas.lidar->points.size() - 20]).curvature / double(1000);
            lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
//            std::printf("pcl_bag_time: %f\n", meas.lidar_beg_time);
//            std::printf("lidar_end_time: %f\n", lidar_end_time);
        }
        if(p_pre->lidar_type == MARSIM || p_pre->lidar_type == AEVA)
            lidar_end_time = meas.lidar_beg_time;

        meas.lidar_end_time = lidar_end_time;
//        std::printf("Scan start timestamp: %f, Scan end time: %f\n", meas.lidar_beg_time, meas.lidar_end_time);

        lidar_pushed = true;
    }

    if (last_timestamp_imu < lidar_end_time)
    {
        return false;
    }

    /*** push imu data, and pop from imu buffer ***/
    double imu_time = imu_buffer.front()->header.stamp.toSec();
    meas.imu.clear();
    while ((!imu_buffer.empty()) && (imu_time < lidar_end_time))
    {
        imu_time = imu_buffer.front()->header.stamp.toSec();
        if(imu_time > lidar_end_time) break;
        meas.imu.push_back(imu_buffer.front());
        imu_buffer.pop_front();
    }

    lidar_buffer.pop_front();
    time_buffer.pop_front();
    lidar_pushed = false;
    return true;
}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1));
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());


void publish_frame_world(const ros::Publisher & pubLaserCloudFull)
{
    if(scan_pub_en)
    {
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI laserCloudWorld;
        for (int i = 0; i < size; i++)
        {
            PointType const * const p = &laserCloudFullRes->points[i];
            // if(p->intensity < 5){
            //     continue;
            // }
//            if (p->x < 0 and p->x > -4
//                    and p->y < 1.5 and p->y > -1.5
//                            and p->z < 2 and p->z > -1) {
//                continue;
//            }
            PointType p_world;

            RGBpointBodyToWorld(p, &p_world);
//            if (p_world.z > 1) {
//                continue;
//            }
            laserCloudWorld.push_back(p_world);
//            RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
//                                &laserCloudWorld->points[i]);
        }

        sensor_msgs::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(laserCloudWorld, laserCloudmsg);
        laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.frame_id = "camera_init";
        pubLaserCloudFull.publish(laserCloudmsg);
        publish_count -= PUBFRAME_PERIOD;
    }

    if (pcd_save_en)
    {
        int size = feats_undistort->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&feats_undistort->points[i], \
                                &laserCloudWorld->points[i]);
        }
        *pcl_wait_save += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num ++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0  && scan_wait_num >= pcd_save_interval)
        {
            pcd_index ++;
            string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to /PCD/" << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
}

void publish_frame_body(const ros::Publisher & pubLaserCloudFull_body)
{
//    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
    int size = laserCloudFullRes->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));
    for (int i = 0; i < size; i++)
    {
        RGBpointBodyLidarToIMU(&laserCloudFullRes->points[i], \
                            &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = "body";
    pubLaserCloudFull_body.publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}

void publish_map(const ros::Publisher & pubLaserCloudMap)
{
    sensor_msgs::PointCloud2 laserCloudMap;
    pcl::toROSMsg(*featsFromMap, laserCloudMap);
    laserCloudMap.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudMap.header.frame_id = "camera_init";
    pubLaserCloudMap.publish(laserCloudMap);
}

template<typename T>
void set_posestamp(T & out)
{
    out.pose.position.x = state_point.pos(0);
    out.pose.position.y = state_point.pos(1);
    out.pose.position.z = state_point.pos(2);
    out.pose.orientation.x = geoQuat.x;
    out.pose.orientation.y = geoQuat.y;
    out.pose.orientation.z = geoQuat.z;
    out.pose.orientation.w = geoQuat.w;

}

void publish_odometry(const ros::Publisher & pubOdomAftMapped)
{
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "body";
    odomAftMapped.header.stamp = ros::Time().fromSec(lidar_end_time);// ros::Time().fromSec(lidar_end_time);
    set_posestamp(odomAftMapped.pose);
    pubOdomAftMapped.publish(odomAftMapped);
    auto P = kf.get_P();
    for (int i = 0; i < 6; i ++)
    {
        int k = i < 3 ? i + 3 : i - 3;
        odomAftMapped.pose.covariance[i*6 + 0] = P(k, 3);
        odomAftMapped.pose.covariance[i*6 + 1] = P(k, 4);
        odomAftMapped.pose.covariance[i*6 + 2] = P(k, 5);
        odomAftMapped.pose.covariance[i*6 + 3] = P(k, 0);
        odomAftMapped.pose.covariance[i*6 + 4] = P(k, 1);
        odomAftMapped.pose.covariance[i*6 + 5] = P(k, 2);
    }

    static tf::TransformBroadcaster br;
    tf::Transform                   transform;
    tf::Quaternion                  q;
    transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x, \
                                    odomAftMapped.pose.pose.position.y, \
                                    odomAftMapped.pose.pose.position.z));
    q.setW(odomAftMapped.pose.pose.orientation.w);
    q.setX(odomAftMapped.pose.pose.orientation.x);
    q.setY(odomAftMapped.pose.pose.orientation.y);
    q.setZ(odomAftMapped.pose.pose.orientation.z);
    transform.setRotation( q );
    br.sendTransform( tf::StampedTransform( transform, odomAftMapped.header.stamp, "camera_init", "body" ) );

    static tf::TransformBroadcaster br_world;
    transform.setOrigin(tf::Vector3(0, 0, 0));
    q.setValue(p_imu->Initial_R_wrt_G.x(), p_imu->Initial_R_wrt_G.y(), p_imu->Initial_R_wrt_G.z(), p_imu->Initial_R_wrt_G.w());
    transform.setRotation(q);
    br_world.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, "world", "camera_init"));
}

void publish_path(const ros::Publisher pubPath)
{
    set_posestamp(msg_body_pose);
    msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = "camera_init";

    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;
    jjj++;
    if (jjj % 1 == 0)
    {
        path.header.stamp = msg_body_pose.header.stamp;
        path.poses.push_back(msg_body_pose);
        pubPath.publish(path);
    }
}

void transformLidar(const state_ikfom &state_point, const PointCloudXYZI::Ptr &input_cloud, PointCloudXYZI::Ptr &trans_cloud)
{
    trans_cloud->clear();
    for (size_t i = 0; i < input_cloud->size(); i++) {
        pcl::PointXYZINormal p_c = input_cloud->points[i];
        Eigen::Vector3d p_lidar(p_c.x, p_c.y, p_c.z);
        // HACK we need to specify p_body as a V3D type!!!
        V3D p_body = state_point.rot * (state_point.offset_R_L_I * p_lidar + state_point.offset_T_L_I) + state_point.pos;
        PointType pi;
        pi.x = p_body(0);
        pi.y = p_body(1);
        pi.z = p_body(2);
        pi.intensity = p_c.intensity;
        pi.curvature = p_c.curvature;
        trans_cloud->points.push_back(pi);
    }
}


M3D transformLiDARCovToWorld(Eigen::Vector3d &p_lidar, const esekfom::esekf<state_ikfom, 12, input_ikfom>& kf, const Eigen::Matrix3d& COV_lidar)
{
    M3D point_crossmat;
    point_crossmat << SKEW_SYM_MATRX(p_lidar);
    auto state = kf.get_x();

    M3D il_rot_var = kf.get_P().block<3, 3>(6, 6);
    M3D il_t_var = kf.get_P().block<3, 3>(9, 9);

    M3D COV_body =
            state.offset_R_L_I * COV_lidar * state.offset_R_L_I.conjugate()
            + state.offset_R_L_I * (-point_crossmat) * il_rot_var * (-point_crossmat).transpose() * state.offset_R_L_I.conjugate()
            + il_t_var;

    V3D p_body = state.offset_R_L_I * p_lidar + state.offset_T_L_I;

    point_crossmat << SKEW_SYM_MATRX(p_body);
    M3D rot_var = kf.get_P().block<3, 3>(3, 3);
    M3D t_var = kf.get_P().block<3, 3>(0, 0);

    // Eq. (3)
    M3D COV_world =
        state.rot * COV_body * state.rot.conjugate()
        + state.rot * (-point_crossmat) * rot_var * (-point_crossmat).transpose()  * state.rot.conjugate()
        + t_var;

    return COV_world;
//    M3D cov_world = R_body * COV_lidar * R_body.conjugate() +
//          (-point_crossmat) * rot_var * (-point_crossmat).transpose() + t_var;

}

// EKF 观测模型（共享形式）：先用体素地图 BuildResidualListOMP 匹配平面；在此之前，
// 当 stop_detect 仍为 false 时，对每个下采样点在世界系 ikdtree 中找近邻，
// 用 esti_stPlane 拟合 (x,y,z,t) 四维局部平面，|pabcd_st(3)| 作为「偏离静态流形」的动态分数 dynamic_scores；
// 大于 vel_thre 的点置 point_selected_surf_=false，使其不参与当帧平面残差（抑制动态物体拖影）。
// 每次进入本函数 detect_cnt++；超过 detect_cnt_thre 后置 stop_detect=true，仅影响本帧 EKF 后续迭代（帧首会重置）。
void observation_model_share(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{

    feats_with_correspondence->clear();
    total_residual = 0.0;

    vector<pointWithCov> pv_list;
    PointCloudXYZI::Ptr world_lidar(new PointCloudXYZI);
//    transformLidar(state_point, feats_down_body, world_lidar);
    transformLidar(s, feats_down_body, world_lidar);
    // here we control the size of the points
    // if(!stop_detect || !if_detect_drop){
    if(!stop_detect){
        point_selected_surf_.resize(feats_down_body->size());
        std::vector<size_t> index(feats_down_body->size());
        std::iota(index.begin(), index.end(), 0);
        
        std::for_each(std::execution::unseq, index.begin(), index.end(), [&](const size_t &i) {
            if(!flg_dynamic_detect || (flg_dynamic_detect && out_of_crop_index(i) == 1)) {
                point_selected_surf_[i] = true;
                dynamic_scores(i) = 0.0;
            } else {
                std::vector<float> pointSearchSqDis(match_points_num);
                PointType &point_world = world_lidar->points[i];
                PointVector points_near;
                if (ikdtree.Root_Node == nullptr) {
                    point_selected_surf_[i] = true;
                    dynamic_scores(i) = 0.0;
                } else {
                    ikdtree.Nearest_Search(point_world, match_points_num, points_near,
                                           pointSearchSqDis);
                    // esti_stPlane divides by (N-1); empty or single neighbor => UB / SIGSEGV
                    if (points_near.size() < 4) {
                        point_selected_surf_[i] = true;
                        dynamic_scores(i) = 0.0;
                    } else {
                        VF(4) pabcd_st;
                        VF(4) points_mean;
                        if (!esti_stPlane(pabcd_st, points_mean, points_near, 0.1f)) {
                            point_selected_surf_[i] = true;
                            dynamic_scores(i) = 0.0;
                        } else if (abs(pabcd_st(3)) <= vel_thre) {
                            point_selected_surf_[i] = true;
                            dynamic_scores(i) = abs(pabcd_st(3));
                        } else {
                            point_selected_surf_[i] = false;
                            dynamic_scores(i) = abs(pabcd_st(3));
                        }
                    }
                }
            }
        });
        detect_cnt ++;
        if(detect_cnt > detect_cnt_thre){
            stop_detect = true;
        }
    }

    // pv_list.resize(feats_down_body->size());
    pv_list.clear();
    for (size_t i = 0; i < feats_down_body->size(); i++) {
        if(!point_selected_surf_[i]) continue;
        pointWithCov pv;
        pv.point << feats_down_body->points[i].x, feats_down_body->points[i].y, feats_down_body->points[i].z;
        pv.point_world << world_lidar->points[i].x, world_lidar->points[i].y, world_lidar->points[i].z;
        // M3D cov_lidar = calcBodyCov(pv.point, ranging_cov, angle_cov);
        M3D cov_lidar = var_down_body[i];
        M3D cov_world = transformLiDARCovToWorld(pv.point, kf, cov_lidar);
        pv.cov = cov_world;
        pv.cov_lidar = cov_lidar;
        // pv_list[i] = pv;
        pv_list.push_back(pv);
    }

    double match_start = omp_get_wtime();
    std::vector<ptpl> ptpl_list;
    std::vector<V3D> non_match_list;
    BuildResidualListOMP(voxel_map, max_voxel_size, 3.0, max_layer, pv_list,
                         ptpl_list, non_match_list);
    double match_end = omp_get_wtime();

    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    effct_feat_num = ptpl_list.size();
    if (effct_feat_num < 1){
        ekfom_data.valid = false;
        ROS_WARN("No Effective Points! \n");
        return;
    }
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); 
    ekfom_data.h.resize(effct_feat_num);
    ekfom_data.R.resize(effct_feat_num, 1); 

#ifdef MP_EN
    omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
    for (int i = 0; i < effct_feat_num; i++)
    {

//        const PointType &laser_p  = laserCloudOri->points[i];
        V3D point_this_be(ptpl_list[i].point);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat<<SKEW_SYM_MATRX(point_this);

        /*** get the normal vector of closest surface/corner ***/
//        const PointType &norm_p = corr_normvect->points[i];
//        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);
        V3D norm_vec(ptpl_list[i].normal);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() *norm_vec);
        V3D A(point_crossmat * C);
        if (extrinsic_est_en)
        {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); //s.rot.conjugate()*norm_vec);
            // ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
            ekfom_data.h_x.block<1, 12>(i,0) << norm_vec.x(), norm_vec.y(), norm_vec.z(), VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        else
        {
            // ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            ekfom_data.h_x.block<1, 12>(i,0) << norm_vec.x(), norm_vec.y(), norm_vec.z(), VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        /*** Measuremnt: distance to the closest surface/corner ***/
//        ekfom_data.h(i) = -norm_p.intensity;
        float pd2 = norm_vec.x() * ptpl_list[i].point_world.x()
                + norm_vec.y() * ptpl_list[i].point_world.y()
                + norm_vec.z() * ptpl_list[i].point_world.z()
                + ptpl_list[i].d;
        ekfom_data.h(i) = -pd2;

        /*** Covariance ***/
//        V3D point_world = s.rot * (s.offset_R_L_I * ptpl_list[i].point + s.offset_T_L_I) + s.pos;
//        // /*** get the normal vector of closest surface/corner ***/
//        Eigen::Matrix<double, 1, 6> J_nq;
//        J_nq.block<1, 3>(0, 0) = point_world - ptpl_list[i].center;
//        J_nq.block<1, 3>(0, 3) = -ptpl_list[i].normal;
//        double sigma_l = J_nq * ptpl_list[i].plane_cov * J_nq.transpose();
//
//        M3D cov_lidar = calcBodyCov(ptpl_list[i].point, ranging_cov, angle_cov);
//        M3D R_cov_Rt = s.rot * cov_lidar * s.rot.conjugate();
//        double R_inv = 1.0 / (sigma_l + norm_vec.transpose() * R_cov_Rt * norm_vec);

        // V3D point_world = s.rot * (s.offset_R_L_I * ptpl_list[i].point + s.offset_T_L_I) + s.pos;
        V3D point_world = ptpl_list[i].point_world;
        // /*** get the normal vector of closest surface/corner ***/
        Eigen::Matrix<double, 1, 6> J_nq;
        J_nq.block<1, 3>(0, 0) = point_world - ptpl_list[i].center;
        J_nq.block<1, 3>(0, 3) = -ptpl_list[i].normal;
        double sigma_l = J_nq * ptpl_list[i].plane_cov * J_nq.transpose();

        // M3D cov_lidar = calcBodyCov(ptpl_list[i].point, ranging_cov, angle_cov);
        M3D cov_lidar = ptpl_list[i].cov_lidar;
        M3D R_cov_Rt = s.rot * s.offset_R_L_I * cov_lidar * s.offset_R_L_I.conjugate() * s.rot.conjugate();
        double R_inv = 1.0 / (sigma_l + norm_vec.transpose() * R_cov_Rt * norm_vec);

        // ekfom_data.R(i) = 1.0 / LASER_POINT_COV;
        ekfom_data.R(i) = R_inv;
    }

    // std::printf("Effective Points: %d\n", effct_feat_num);
    res_mean_last = total_residual / effct_feat_num;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laserMapping");
    ros::NodeHandle nh;

    nh.param<double>("time_offset", lidar_time_offset, 0.0);

    nh.param<bool>("publish/path_en",path_en, true);
    nh.param<bool>("publish/scan_publish_en",scan_pub_en, true);
    nh.param<bool>("publish/dense_publish_en",dense_pub_en, true);
    nh.param<bool>("publish/scan_bodyframe_pub_en",scan_body_pub_en, true);
    nh.param<string>("common/lid_topic",lid_topic,"/livox/lidar");
    nh.param<string>("common/imu_topic", imu_topic,"/livox/imu");
    nh.param<bool>("common/time_sync_en", time_sync_en, false);

    // mapping algorithm params
    nh.param<float>("mapping/det_range",DET_RANGE,300.f);
    nh.param<int>("mapping/max_iteration", NUM_MAX_ITERATIONS, 4);
    nh.param<int>("mapping/max_points_size", max_points_size, 100);
    nh.param<int>("mapping/max_cov_points_size", max_cov_points_size, 100);
    nh.param<vector<double>>("mapping/layer_point_size", layer_point_size,vector<double>());
    nh.param<int>("mapping/max_layer", max_layer, 2);
    nh.param<double>("mapping/voxel_size", max_voxel_size, 1.0);
    nh.param<double>("mapping/down_sample_size", filter_size_surf_min, 0.5);
    nh.param<double>("scan/filter_size_map", filter_size_map_min, 0.5);
    std::cout << "filter_size_surf_min:" << filter_size_surf_min << std::endl;
    nh.param<double>("mapping/plannar_threshold", min_eigen_value, 0.01);
    nh.param<bool>("mapping/extrinsic_est_en", extrinsic_est_en, true);
    nh.param<vector<double>>("mapping/extrinsic_T", extrinT, vector<double>());
    nh.param<vector<double>>("mapping/extrinsic_R", extrinR, vector<double>());

    // scan
    nh.param<int>("scan/time_slice", time_slice, 10);
    nh.param<double>("scan/vel_thre", vel_thre, 0.1);
    nh.param<double>("scan/range_x", range_x, 100);
    nh.param<double>("scan/range_y", range_y, 100);
    nh.param<double>("scan/range_z", range_z, 100);
    nh.param<int>("scan/match_points", match_points_num, 30);
    nh.param<bool>("scan/if_detect_drop", if_detect_drop, false);
    nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en, false);
    nh.param<int>("pcd_save/interval", pcd_save_interval, -1);
    nh.param<int>("pcd_save/detect_cnt_thre", detect_cnt_thre, 2);
    nh.param<bool>("scan/use_radius_search", use_radius_search_, true);
    nh.param<int>("scan/neighborhood_size", neighborhood_size_, 10);
    nh.param<float>("scan/neighbourhood_radius", neighbourhood_radius_, 0.1);
    nh.param<double>("scan/scc_voxel_size", scc_voxel_size_, 10);
    nh.param<double>("scan/scc_voxel_time", scc_voxel_time_, 10);

    // noise model params
    nh.param<double>("noise_model/ranging_cov", ranging_cov, 0.02);
    nh.param<double>("noise_model/angle_cov", angle_cov, 0.05);
    nh.param<double>("noise_model/gyr_cov",gyr_cov,0.1);
    nh.param<double>("noise_model/acc_cov",acc_cov,0.1);
    nh.param<double>("noise_model/b_gyr_cov",b_gyr_cov,0.0001);
    nh.param<double>("noise_model/b_acc_cov",b_acc_cov,0.0001);

    // visualization params
    nh.param<bool>("publish/pub_voxel_map", publish_voxel_map, false);
    nh.param<int>("publish/publish_max_voxel_layer", publish_max_voxel_layer, 0);

    nh.param<double>("preprocess/blind", p_pre->blind, 0.01);
    nh.param<double>("preprocess/height_min", p_pre->rs_height_min, -2.0);
    nh.param<double>("preprocess/height_max", p_pre->rs_height_max, 0.5);
    nh.param<int>("preprocess/lidar_type", p_pre->lidar_type, AVIA);
    nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 16);
    nh.param<int>("preprocess/scan_rate", p_pre->SCAN_RATE, 10);
    nh.param<int>("preprocess/point_filter_num", p_pre->point_filter_num, 1);
    nh.param<bool>("preprocess/feature_extract_enable", p_pre->feature_enabled, false);
    cout<<"p_pre->lidar_type "<<p_pre->lidar_type<<endl;
    for (int i = 0; i < layer_point_size.size(); i++) {
        layer_size.push_back(layer_point_size[i]);
    }

    path.header.stamp    = ros::Time::now();
    path.header.frame_id ="camera_init";

    voxel_hash_map_ = VoxelHashMap(scc_voxel_size_, 500, 1000);
    voxel_hash_map_.printParameters();

    /*** variables definition ***/
    int effect_feat_num = 0, frame_num = 0;
    bool flg_EKF_converged, EKF_stop_flg = 0;
    FILE *fp = nullptr;
    const std::filesystem::path log_dir =
        std::filesystem::path(root_dir) / "Log";
    std::error_code fs_ec;
    std::filesystem::create_directories(log_dir, fs_ec);
    if (fs_ec) {
        ROS_WARN("无法创建日志目录 %s : %s", log_dir.c_str(),
                 fs_ec.message().c_str());
    }
    const std::string pos_log_dir = (log_dir / "pos_log.txt").string();
    fp = fopen(pos_log_dir.c_str(), "w");
    if (fp == nullptr) {
        ROS_WARN("无法打开位姿日志文件（将跳过写入）: %s", pos_log_dir.c_str());
    }

    _featsArray.reset(new PointCloudXYZI());

    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));

    Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
    p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
    p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
    p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));
    p_imu->lidar_type = p_pre->lidar_type;

    double epsi[23] = {0.001};
    fill(epsi, epsi+23, 0.001);
    kf.init_dyn_share(get_f, df_dx, df_dw, observation_model_share, NUM_MAX_ITERATIONS, epsi);

    /*** ROS subscribe initialization ***/
    ros::Subscriber sub_pcl = p_pre->lidar_type == AVIA ? \
        nh.subscribe(lid_topic, 200000, livox_pcl_cbk) : \
        nh.subscribe(lid_topic, 200000, standard_pcl_cbk);
    ros::Subscriber sub_imu = nh.subscribe(imu_topic, 200000, imu_cbk);
    ros::Publisher pubLaserCloudFull = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_registered", 100000);
    ros::Publisher pubLaserCloudFull_body = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_registered_body", 100000);
    ros::Publisher pubLaserCloudEffect = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_effected", 100000);
    ros::Publisher pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>
            ("/Laser_map", 100000);
    ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>
            ("/Odometry", 100000);
    ros::Publisher pubExtrinsic = nh.advertise<nav_msgs::Odometry>
            ("/Extrinsic", 100000);
    ros::Publisher pubPath          = nh.advertise<nav_msgs::Path>
            ("/path", 100000);
    ros::Publisher voxel_map_pub =
            nh.advertise<visualization_msgs::MarkerArray>("/planes", 10000);
    ros::Publisher pubLaserCloudUnstable = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_unstable", 100000);
    ros::Publisher pubLaserCloudDynamic = nh.advertise<sensor_msgs::PointCloud2>
            ("/cloud_dynamic", 100000);
//------------------------------------------------------------------------------------------------------
    // for Plane Map
    bool init_map = false;

    double sum_optimize_time = 0, sum_update_time = 0;
    int scan_index = 0;

    signal(SIGINT, SigHandle);
    ros::Rate rate(5000);
    bool status = ros::ok();
    while (status)
    {
        if (flg_exit) break;
        ros::spinOnce();
        if(sync_packages(Measures))
        {
            if (flg_first_scan)
            {
                first_lidar_time = Measures.lidar_beg_time;
                p_imu->first_lidar_time = first_lidar_time;
                flg_first_scan = false;
                continue;
            }

            timer.Evaluate(
                [&]() {p_imu->Process(Measures, kf, feats_undistort);}, "pre-process");
            state_point = kf.get_x();
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

            if (feats_undistort->empty() || (feats_undistort == NULL))
            {
                ROS_WARN("No point, skip this scan!\n");
                continue;
            }

            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? \
                            false : true;
            // ===============================================================================================================
            if (flg_EKF_inited && !init_map) {
                PointCloudXYZI::Ptr world_lidar(new PointCloudXYZI);
                transformLidar(state_point, feats_undistort, world_lidar);
                std::vector<pointWithCov> pv_list;

                std::cout << kf.get_P() << std::endl;
                for (size_t i = 0; i < world_lidar->size(); i++) {
                    pointWithCov pv;
                    pv.point << world_lidar->points[i].x, world_lidar->points[i].y,
                            world_lidar->points[i].z;
                    V3D point_this(feats_undistort->points[i].x,
                                   feats_undistort->points[i].y,
                                   feats_undistort->points[i].z);
                    // if z=0, error will occur in calcBodyCov. To be solved
                    if (point_this[2] == 0) {
                        point_this[2] = 0.001;
                    }
                    M3D cov_lidar = calcBodyCov(point_this, ranging_cov, angle_cov);
                    M3D cov_world = transformLiDARCovToWorld(point_this, kf, cov_lidar);

                    pv.cov = cov_world;
                    pv_list.push_back(pv);
                    Eigen::Vector3d sigma_pv = pv.cov.diagonal();
                    sigma_pv[0] = sqrt(sigma_pv[0]);
                    sigma_pv[1] = sqrt(sigma_pv[1]);
                    sigma_pv[2] = sqrt(sigma_pv[2]);
                }

                buildVoxelMap(pv_list, max_voxel_size, max_layer, layer_size,
                              max_points_size, max_points_size, min_eigen_value,
                              voxel_map);
                std::cout << "build voxel map" << std::endl;

                if (publish_voxel_map) {
                    pubVoxelMap(voxel_map, publish_max_voxel_layer, voxel_map_pub);
                    publish_frame_world(pubLaserCloudFull);
                    publish_frame_body(pubLaserCloudFull_body);
                }
                init_map = true;
                continue;
            }

            /*** downsample the feature points in a scan ***/
            // 将点相对时间 curvature（ms）平移到「相对首帧雷达时刻」的全局时间轴，便于跨帧排序与滑窗。
            for (size_t i = 0; i < feats_undistort->points.size(); ++i) {
                feats_undistort->points[i].curvature += (Measures.lidar_beg_time - first_lidar_time) * 1000;
            }
            // print the time of the first point and second point in feats_undistort
            // if (feats_undistort->points.size() > 1) {
            //     std::cout << "Time of the first point: " 
            //               << feats_undistort->points[0].curvature / 1000.0 
            //               << " seconds" << std::endl;
            //     std::cout << "Time of the second point: " 
            //               << feats_undistort->points[1].curvature / 1000.0 
            //               << " seconds" << std::endl;
            // } else {
            //     std::cout << "Not enough points to print time information." << std::endl;
            // }
            
            // 体素下采样得到 feats_down_body；out_of_crop_index 标记范围外点，动态检测时跳过。
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);
            sort(feats_down_body->points.begin(), feats_down_body->points.end(), time_list);
            out_of_crop_index = Eigen::VectorXf::Zero(feats_down_body->points.size());
            for (int i = 0; i < feats_down_body->points.size(); i++)
            {
                if (feats_down_body->points[i].x > range_x || feats_down_body->points[i].x < -range_x || \
                    feats_down_body->points[i].y > range_y || feats_down_body->points[i].y < -range_y || \
                    feats_down_body->points[i].z > range_z || feats_down_body->points[i].z < -range_z)
                {
                    out_of_crop_index(i) = 1;
                }
            }
            // print the time of the first point and second point in feats_down_body

            feats_down_size = feats_down_body->points.size();
            var_down_body.clear();
            for (auto & pt:feats_down_body->points) {
                V3D point_this(pt.x, pt.y, pt.z);
                var_down_body.push_back(calcBodyCov(point_this, ranging_cov, angle_cov));
            }

            /*** ICP and iterated Kalman filter update ***/
            if (feats_down_size < 5)
            {
                ROS_WARN("No point, skip this scan!\n");
                continue;
            }
            // ===============================================================================================================
            /*** iterated state estimation ***/
            // 每帧重置动态掩码与分数；EKF 更新内部调用 observation_model_share 写入 dynamic_scores / point_selected_surf_。
            double t_update_start = omp_get_wtime();
            double solve_H_time = 0;
            point_selected_surf_.resize(feats_down_size, true);
            dynamic_scores = Eigen::VectorXf::Zero(feats_down_size);
            stop_detect = false;
            detect_cnt = 0;
            timer.Evaluate(
                [&]() {kf.update_iterated_dyn_share_diagonal();}, "dynamic_awate_estimation");
//            kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
            double t_update_end = omp_get_wtime();
            sum_optimize_time += t_update_end - t_update_start;

            state_point = kf.get_x();
            euler_cur = SO3ToEuler(state_point.rot);
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;
            geoQuat.x = state_point.rot.coeffs()[0];
            geoQuat.y = state_point.rot.coeffs()[1];
            geoQuat.z = state_point.rot.coeffs()[2];
            geoQuat.w = state_point.rot.coeffs()[3];
//
            /*** add the points to the voxel map ***/
            // 将当前帧下采样点变换到世界系 world_lidar，更新体素地图；同时推入 TimeWindow_Points 并维护 ikdtree。
            std::vector<pointWithCov> pv_list;
            PointCloudXYZI::Ptr world_lidar(new PointCloudXYZI);
            transformLidar(state_point, feats_down_body, world_lidar);
            for (size_t i = 0; i < feats_down_body->size(); i++) {
                pointWithCov pv;
                pv.point << feats_down_body->points[i].x, feats_down_body->points[i].y, feats_down_body->points[i].z;
                // M3D cov_lidar = calcBodyCov(pv.point, ranging_cov, angle_cov);
                M3D cov_lidar = var_down_body[i];

                M3D cov_world = transformLiDARCovToWorld(pv.point, kf, cov_lidar);

                pv.cov = cov_world;
                pv.point << world_lidar->points[i].x, world_lidar->points[i].y, world_lidar->points[i].z;
                pv_list.push_back(pv);
            }
            timer.Evaluate(
                [&]() {
            TimeWindow_Points.push_back(world_lidar->points);
            if(ikdtree.Root_Node == nullptr){
                if(world_lidar->size() > 5){
                    ikdtree.set_downsample_param(filter_size_map_min);
                    ikdtree.Build(world_lidar->points);
                }
            }else{
                auto add_point_size = ikdtree.Add_Points(world_lidar->points, false);
            }

            t_update_start = omp_get_wtime();
            std::sort(pv_list.begin(), pv_list.end(), var_contrast);
            updateVoxelMapOMP(pv_list, max_voxel_size, max_layer, layer_size,
                           max_points_size, max_points_size, min_eigen_value,
                           voxel_map);
            lasermap_timeslice_segment();},
            "MAP Update");

            timer.Evaluate(
                [&]() {
            feats_down_world = world_lidar;
            dynamic_detection_upsampling();
            SpatialConsistencyCheck();
            },"SpatialConsistencyCheck");

            // lasermap_timeslice_segment();
            t_update_end = omp_get_wtime();
            sum_update_time += t_update_end - t_update_start;

            scan_index++;
            /******* Publish odometry *******/
            publish_odometry(pubOdomAftMapped);
//
//            /*** add the feature points to map kdtree ***/
//            map_incremental();
//
            /******* Publish points *******/
            if (path_en)                         publish_path(pubPath);
            if (scan_pub_en)      publish_frame_world(pubLaserCloudFull);
            if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFull_body);
            if (publish_voxel_map) {
                pubVoxelMap(voxel_map, publish_max_voxel_layer, voxel_map_pub);
            }
            feats_unstable->clear();
            for (size_t i = 0; i < world_lidar->size(); i++) {
                if(dynamic_scores(i) >= vel_thre){
                    feats_unstable->points.push_back(world_lidar->points[i]);
                }
            }
            // if(1) // If you need to see map point, change to "if(1)"
            // {
            //     PointVector ().swap(ikdtree.PCL_Storage);
            //     ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, ikdtreeNS::NOT_RECORD);
            //     featsFromMap->clear();
            //     featsFromMap->points = ikdtree.PCL_Storage;
            //     // print the size of featsFromMap
            //     std::cout << "Map points size: " << featsFromMap->points.size() << std::endl;
            // }
            // print the size of feats_unstable
            // std::cout << "Unstable points size: " << feats_unstable->points.size() << std::endl;
            puslish_unstable_points(pubLaserCloudUnstable);
            publish_dynamic_points(pubLaserCloudDynamic);
            // publish_effect_world(pubLaserCloudEffect);
            // publish_map(pubLaserCloudMap);
            dump_lio_state_to_log(fp);
        }

        status = ros::ok();
        rate.sleep();
    }

    if (pcl_wait_save->size() > 0 && pcd_save_en)
    {
        string file_name = string("scans.pcd");
        string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        cout << "current scan saved to /PCD/" << file_name<<endl;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }

    timer.PrintAll();
    string time_log_dir = root_dir + "/Log/time_analysis.txt";
    timer.SaveTimingToFile(time_log_dir);

    return 0;
}
