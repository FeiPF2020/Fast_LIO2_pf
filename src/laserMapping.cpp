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
#include <visualization_msgs/Marker.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Vector3.h>
#include <livox_ros_driver/CustomMsg.h>
#include "preprocess.h"
#include <ikd-Tree/ikd_Tree.h>


#define INIT_TIME           (0.1)
#define LASER_POINT_COV     (0.001)
#define MAXN                (720000)
#define PUBFRAME_PERIOD     (20)

// by wpf 69-142
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>

// ros::Publisher pub_filtered_points;

//  class PclTestCore
// {
// public:
//     void point_clipping(const sensor_msgs::PointCloud2ConstPtr& in_cloud_ptr){
//         pcl::PointCloud<pcl::PointXYZI>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
//         pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);

//         pcl::fromROSMsg(*in_cloud_ptr, *current_pc_ptr);

//         pcl::VoxelGrid<pcl::PointXYZI> vg;

//         vg.setInputCloud(current_pc_ptr);
//         vg.setLeafSize(10.0f, 10.0f, 10.0f);
//         vg.filter(*filtered_pc_ptr);

//         sensor_msgs::PointCloud2 pub_pc;
//         pcl::toROSMsg(*filtered_pc_ptr, pub_pc);

//         pub_pc.header = in_cloud_ptr->header;

//         pub_filtered_points_.publish(pub_pc);
//     }


//     PclTestCore(ros::NodeHandle &nh){
//         sub_point_cloud_ = nh.subscribe("/cloud_registered",100000, &PclTestCore::point_clipping, this);
//         pub_filtered_points_ = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points", 100000);
//         // ros::spin();

//     }
//     ros::Subscriber sub_point_cloud_;
//     ros::Publisher pub_filtered_points_;
//     ~PclTestCore();
//     // void Spin();
// };

//  PclTestCore::PclTestCore(ros::NodeHandle &nh){

//     //ros::Publisher pubLaserCloudFull = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100000);

   

//     ros::spin();
// }
// PclTestCore::~PclTestCore(){}

// void PclTestCore::Spin(){
    
// }


// void point_clips(const sensor_msgs::PointCloud2ConstPtr & in_cloud_ptr){

//     pcl::PointCloud<pcl::PointXYZI>::Ptr current_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);
//     pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_pc_ptr(new pcl::PointCloud<pcl::PointXYZI>);

//     pcl::fromROSMsg(*in_cloud_ptr, *current_pc_ptr);

//     pcl::VoxelGrid<pcl::PointXYZI> vg;

//     vg.setInputCloud(current_pc_ptr);
//     vg.setLeafSize(2.0f, 2.0f, 2.0f);
//     vg.filter(*filtered_pc_ptr);

//     sensor_msgs::PointCloud2 pub_pc;
//     pcl::toROSMsg(*filtered_pc_ptr, pub_pc);

//     pub_pc.header = in_cloud_ptr->header;
//     // pub_filtered_points.publish(pub_pc);
// }



/*** Time Log Variables ***/
double kdtree_incremental_time = 0.0, kdtree_search_time = 0.0, kdtree_delete_time = 0.0;
/**
 *  T1为雷达初始时间戳，s_plot为整个流程耗时，s_plot2特征点数量,s_plot3为kdtree增量时间，s_plot4为kdtree搜索耗时，s_plot5为kdtree删除点数量，
 *  s_plot6为kdtree删除耗时，s_plot7为kdtree初始大小，s_plot8为kdtree结束大小,s_plot9为平均消耗时间，s_plot10为添加点数量，s_plot11为点云预处理的总时间
 */
double T1[MAXN], s_plot[MAXN], s_plot2[MAXN], s_plot3[MAXN], s_plot4[MAXN], s_plot5[MAXN], s_plot6[MAXN], s_plot7[MAXN], s_plot8[MAXN], s_plot9[MAXN], s_plot10[MAXN], s_plot11[MAXN];
double match_time = 0, solve_time = 0, solve_const_H_time = 0;
int    kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
bool   runtime_pos_log = false, pcd_save_en = false, time_sync_en = false, extrinsic_est_en = true, path_en = true;

/**************************/
float res_last[100000] = {0.0};//残差，点到面距离平方和
float DET_RANGE = 300.0f;//雷达最大的探测距离--avia450m,velodyne100m， //设置的当前雷达系中心到各个地图边缘的距离
const float MOV_THRESHOLD = 1.5f;//移动阈值 1.5 float//设置的当前雷达系中心到各个地图边缘的权重
double time_diff_lidar_to_imu = 0.0;

mutex mtx_buffer;
condition_variable sig_buffer;

string root_dir = ROOT_DIR;
string map_file_path, lid_topic, imu_topic;

double res_mean_last = 0.05, total_residual = 0.0;//设置残差平均值，残差总和
double last_timestamp_lidar = 0, last_timestamp_imu = -1.0;
double gyr_cov = 0.1, acc_cov = 0.1, b_gyr_cov = 0.0001, b_acc_cov = 0.0001;
double filter_size_corner_min = 0, filter_size_surf_min = 0, filter_size_map_min = 0, fov_deg = 0;//设置滤波器的最小尺寸，地图的最小尺寸，视野角度
//设置立方体长度，视野一半的角度，视野总角度，总距离，雷达结束时间，雷达初始时间
double cube_len = 0, HALF_FOV_COS = 0, FOV_DEG = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
//设置有效特征点数，时间log计数器, scan_count：接收到的激光雷达Msg的总数，publish_count：接收到的IMU的Msg的总数
int    effct_feat_num = 0, time_log_counter = 0, scan_count = 0, publish_count = 0;
//设置迭代次数，下采样的点数，最大迭代次数，有效点数
int    iterCount = 0, feats_down_size = 0, NUM_MAX_ITERATIONS = 0, laserCloudValidNum = 0, pcd_save_interval = -1, pcd_index = 0;
bool   point_selected_surf[100000] = {0};// 是否为平面特征点
// lidar_pushed：用于判断激光雷达数据是否从缓存队列中拿到meas中的数据, flg_EKF_inited用于判断EKF是否初始化完成
bool   lidar_pushed, flg_first_scan = true, flg_exit = false, flg_EKF_inited;
//设置是否发布激光雷达数据，是否发布稠密数据，是否发布激光雷达数据的身体数据
bool   scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;

vector<vector<int>>  pointSearchInd_surf; //每个点的索引,暂时没用到
vector<BoxPointType> cub_needrm;// ikd-tree中，地图需要移除的包围盒序列
vector<PointVector>  Nearest_Points; //每个点的最近点序列
vector<double>       extrinT(3, 0.0);//雷达相对于IMU的外参T
vector<double>       extrinR(9, 0.0);//雷达相对于IMU的外参R
deque<double>                     time_buffer;// 激光雷达数据时间戳缓存队列
deque<PointCloudXYZI::Ptr>        lidar_buffer;//记录特征提取或间隔采样后的lidar（特征）数据
deque<sensor_msgs::Imu::ConstPtr> imu_buffer;

PointCloudXYZI::Ptr featsFromMap(new PointCloudXYZI()); //提取地图中的特征点，IKD-tree获得
PointCloudXYZI::Ptr feats_undistort(new PointCloudXYZI());//去畸变的特征，lidar坐标系
PointCloudXYZI::Ptr feats_down_body(new PointCloudXYZI());  //畸变纠正后降采样的单帧点云，lidar系
PointCloudXYZI::Ptr feats_down_world(new PointCloudXYZI());
PointCloudXYZI::Ptr normvec(new PointCloudXYZI(100000, 1)); //特征点在地图中对应点的，局部平面参数,w系
PointCloudXYZI::Ptr laserCloudOri(new PointCloudXYZI(100000, 1)); // laserCloudOri是畸变纠正后降采样的单帧点云，body系
PointCloudXYZI::Ptr corr_normvect(new PointCloudXYZI(100000, 1)); //对应点法相量
PointCloudXYZI::Ptr _featsArray; // ikd-tree中，map需要移除的点云序列

pcl::VoxelGrid<PointType> downSizeFilterSurf; //单帧内降采样使用voxel grid
pcl::VoxelGrid<PointType> downSizeFilterMap; //未使用

KD_TREE<PointType> ikdtree;// ikd-tree类

V3F XAxisPoint_body(LIDAR_SP_LEN, 0.0, 0.0);//雷达相对于body系的X轴方向的点，12，0，0
V3F XAxisPoint_world(LIDAR_SP_LEN, 0.0, 0.0);//雷达相对于world系的X轴方向的点，12，0，0
V3D euler_cur;//当前的欧拉角
V3D position_last(Zero3d);
V3D Lidar_T_wrt_IMU(Zero3d);// T lidar to imu (imu = r * lidar + t)
M3D Lidar_R_wrt_IMU(Eye3d);// R lidar to imu (imu = r * lidar + t)

/*** EKF inputs and output-------------滤波器实例化，状态，噪声维度，输入 ***/
MeasureGroup Measures;
esekfom::esekf<state_ikfom, 12, input_ikfom> kf;
state_ikfom state_point;// 相当于论文中的一个x
vect3 pos_lid;// world系下lidar坐标
//输出的路径参数
nav_msgs::Path path;  //包含了一系列位姿
nav_msgs::Odometry odomAftMapped;//只包含了一个位姿
geometry_msgs::Quaternion geoQuat; //四元数
geometry_msgs::PoseStamped msg_body_pose;//位姿，header+pose

shared_ptr<Preprocess> p_pre(new Preprocess());// 定义指向激光雷达数据的预处理类Preprocess的智能指针
shared_ptr<ImuProcess> p_imu(new ImuProcess());// 定义指向IMU数据预处理类ImuProcess的智能指针
//按下ctrl+c后唤醒所有线程
void SigHandle(int sig)
{
    flg_exit = true;
    ROS_WARN("catch sig %d", sig);
    sig_buffer.notify_all();//  会唤醒所有等待队列中阻塞的线程 线程被唤醒后，会通过轮询方式获得锁，获得锁前也一直处理运行状态，不会被再次阻塞。
}
//将fast lio2信息打印到log中
inline void dump_lio_state_to_log(FILE *fp)  
{
    V3D rot_ang(Log(state_point.rot.toRotationMatrix()));
    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                   // Angle
    fprintf(fp, "%lf %lf %lf ", state_point.pos(0), state_point.pos(1), state_point.pos(2)); // Pos  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // omega  
    fprintf(fp, "%lf %lf %lf ", state_point.vel(0), state_point.vel(1), state_point.vel(2)); // Vel  
    fprintf(fp, "%lf %lf %lf ", 0.0, 0.0, 0.0);                                        // Acc  
    fprintf(fp, "%lf %lf %lf ", state_point.bg(0), state_point.bg(1), state_point.bg(2));    // Bias_g  
    fprintf(fp, "%lf %lf %lf ", state_point.ba(0), state_point.ba(1), state_point.ba(2));    // Bias_a  
    fprintf(fp, "%lf %lf %lf ", state_point.grav[0], state_point.grav[1], state_point.grav[2]); // Bias_a  
    fprintf(fp, "\r\n");  
    fflush(fp);
}
//把点从body系转到world系，通过ikfom的位置和姿态
void pointBodyToWorld_ikfom(PointType const * const pi, PointType * const po, state_ikfom &s)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

//把点从body系转到world系
void pointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}
//把点从body系转到world系
template<typename T>
void pointBodyToWorld(const Matrix<T, 3, 1> &pi, Matrix<T, 3, 1> &po)
{
    V3D p_body(pi[0], pi[1], pi[2]);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po[0] = p_global(0);
    po[1] = p_global(1);
    po[2] = p_global(2);
}
// 含有RGB的点云从body系转到world系
void RGBpointBodyToWorld(PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(state_point.rot * (state_point.offset_R_L_I*p_body + state_point.offset_T_L_I) + state_point.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}
// 含有RGB的点云从Lidar系转到IMU系
void RGBpointBodyLidarToIMU(PointType const * const pi, PointType * const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(state_point.offset_R_L_I*p_body_lidar + state_point.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}
//得到被剔除的点
void points_cache_collect()
{
    PointVector points_history;
    ikdtree.acquire_removed_points(points_history); //返回被剔除的点
    // for (int i = 0; i < points_history.size(); i++) _featsArray->push_back(points_history[i]);//存入到缓存中，后面没有用到该数据
}

BoxPointType LocalMap_Points;
bool Localmap_Initialized = false;
void lasermap_fov_segment()
{
    cub_needrm.clear();
    kdtree_delete_counter = 0;
    kdtree_delete_time = 0.0;    
    pointBodyToWorld(XAxisPoint_body, XAxisPoint_world);
    V3D pos_LiD = pos_lid;
    if (!Localmap_Initialized){
        for (int i = 0; i < 3; i++){
            LocalMap_Points.vertex_min[i] = pos_LiD(i) - cube_len / 2.0;
            LocalMap_Points.vertex_max[i] = pos_LiD(i) + cube_len / 2.0;
        }
        Localmap_Initialized = true;
        return;
    }
    float dist_to_map_edge[3][2];
    bool need_move = false;
    for (int i = 0; i < 3; i++){
        dist_to_map_edge[i][0] = fabs(pos_LiD(i) - LocalMap_Points.vertex_min[i]);
        dist_to_map_edge[i][1] = fabs(pos_LiD(i) - LocalMap_Points.vertex_max[i]);
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE || dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE) need_move = true;
    }
    if (!need_move) return;
    BoxPointType New_LocalMap_Points, tmp_boxpoints;
    New_LocalMap_Points = LocalMap_Points;
    float mov_dist = max((cube_len - 2.0 * MOV_THRESHOLD * DET_RANGE) * 0.5 * 0.9, double(DET_RANGE * (MOV_THRESHOLD -1)));
    for (int i = 0; i < 3; i++){
        tmp_boxpoints = LocalMap_Points;
        if (dist_to_map_edge[i][0] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] -= mov_dist;
            New_LocalMap_Points.vertex_min[i] -= mov_dist;
            tmp_boxpoints.vertex_min[i] = LocalMap_Points.vertex_max[i] - mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        } else if (dist_to_map_edge[i][1] <= MOV_THRESHOLD * DET_RANGE){
            New_LocalMap_Points.vertex_max[i] += mov_dist;
            New_LocalMap_Points.vertex_min[i] += mov_dist;
            tmp_boxpoints.vertex_max[i] = LocalMap_Points.vertex_min[i] + mov_dist;
            cub_needrm.push_back(tmp_boxpoints);
        }
    }
    LocalMap_Points = New_LocalMap_Points;

    points_cache_collect();
    double delete_begin = omp_get_wtime();
    if(cub_needrm.size() > 0) kdtree_delete_counter = ikdtree.Delete_Point_Boxes(cub_needrm);
    kdtree_delete_time = omp_get_wtime() - delete_begin;
}

void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg) 
{
    mtx_buffer.lock();
    scan_count ++;
    double preprocess_start_time = omp_get_wtime();
    if (msg->header.stamp.toSec() < last_timestamp_lidar)
    {
        ROS_ERROR("lidar loop back, clear buffer");
        lidar_buffer.clear();
    }

    PointCloudXYZI::Ptr  ptr(new PointCloudXYZI());
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);
    time_buffer.push_back(msg->header.stamp.toSec());
    last_timestamp_lidar = msg->header.stamp.toSec();
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

double timediff_lidar_wrt_imu = 0.0;
bool   timediff_set_flg = false;
// 雷达点云预处理
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
    // 特征提取和降采样，这里的特征提取主要是面特征的提取，降采样直接是等距采样，当成面特征
    p_pre->process(msg, ptr);
    lidar_buffer.push_back(ptr);// 特征提取和U间隔采样的
    time_buffer.push_back(last_timestamp_lidar);
    
    s_plot11[scan_count] = omp_get_wtime() - preprocess_start_time;
    mtx_buffer.unlock();
    // 需要学习和补充
    sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in) 
{
    publish_count ++;
    // cout<<"IMU got at: "<<msg_in->header.stamp.toSec()<<endl;
    sensor_msgs::Imu::Ptr msg(new sensor_msgs::Imu(*msg_in));

    msg->header.stamp = ros::Time().fromSec(msg_in->header.stamp.toSec() - time_diff_lidar_to_imu);
    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
    {
        msg->header.stamp = \
        ros::Time().fromSec(timediff_lidar_wrt_imu + msg_in->header.stamp.toSec());
    }

    double timestamp = msg->header.stamp.toSec();

    mtx_buffer.lock();

    if (timestamp < last_timestamp_imu)
    {
        ROS_WARN("imu loop back, clear buffer");
        imu_buffer.clear();
    }

    last_timestamp_imu = timestamp;

    imu_buffer.push_back(msg);
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

double lidar_mean_scantime = 0.0;
int    scan_num = 0;
//数据同步之后，加入imu的数据
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
            scan_num ++;
            lidar_end_time = meas.lidar_beg_time + meas.lidar->points.back().curvature / double(1000);
            lidar_mean_scantime += (meas.lidar->points.back().curvature / double(1000) - lidar_mean_scantime) / scan_num;
        }

        meas.lidar_end_time = lidar_end_time;

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

int process_increments = 0;
void map_incremental()
{
    PointVector PointToAdd;
    PointVector PointNoNeedDownsample;
    PointToAdd.reserve(feats_down_size);
    PointNoNeedDownsample.reserve(feats_down_size);
    for (int i = 0; i < feats_down_size; i++)
    {
        /* transform to world frame */
        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
        /* decide if need add to map */
        if (!Nearest_Points[i].empty() && flg_EKF_inited)
        {
            const PointVector &points_near = Nearest_Points[i];
            bool need_add = true;
            BoxPointType Box_of_Point;
            PointType downsample_result, mid_point; 
            mid_point.x = floor(feats_down_world->points[i].x/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.y = floor(feats_down_world->points[i].y/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            mid_point.z = floor(feats_down_world->points[i].z/filter_size_map_min)*filter_size_map_min + 0.5 * filter_size_map_min;
            float dist  = calc_dist(feats_down_world->points[i],mid_point);
            if (fabs(points_near[0].x - mid_point.x) > 0.5 * filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * filter_size_map_min){
                PointNoNeedDownsample.push_back(feats_down_world->points[i]);
                continue;
            }
            for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i ++)
            {
                if (points_near.size() < NUM_MATCH_POINTS) break;
                if (calc_dist(points_near[readd_i], mid_point) < dist)
                {
                    need_add = false;
                    break;
                }
            }
            if (need_add) PointToAdd.push_back(feats_down_world->points[i]);
        }
        else
        {
            PointToAdd.push_back(feats_down_world->points[i]);
        }
    }

    double st_time = omp_get_wtime();
    add_point_size = ikdtree.Add_Points(PointToAdd, true);
    ikdtree.Add_Points(PointNoNeedDownsample, false); 
    add_point_size = PointToAdd.size() + PointNoNeedDownsample.size();
    kdtree_incremental_time = omp_get_wtime() - st_time;
}

PointCloudXYZI::Ptr pcl_wait_pub(new PointCloudXYZI(500000, 1));
PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());
// 发布世界坐标下的点云，对应的帧是camera——init
void publish_frame_world(const ros::Publisher & pubLaserCloudFull)
{
    if(scan_pub_en)
    {
        // 判断稠密的是否打开
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? feats_undistort : feats_down_body);
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&laserCloudFullRes->points[i], \
                                &laserCloudWorld->points[i]);
        }

        sensor_msgs::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        laserCloudmsg.header.frame_id = "camera_init";// 世界的参考帧
        pubLaserCloudFull.publish(laserCloudmsg);// 发布对应的雷达的点云信息
        publish_count -= PUBFRAME_PERIOD;
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* TODO 2. noted that pcd save will influence the real-time performences **/
    if (pcd_save_en)
    {
        // 去畸变的点云，去畸变的点云直接累加
        int size = feats_undistort->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( \
                        new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&feats_undistort->points[i], \
                                &laserCloudWorld->points[i]);
        }
        *pcl_wait_save += *laserCloudWorld;// 点云直接相加？？？？

        static int scan_wait_num = 0;
        scan_wait_num ++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0  && scan_wait_num >= pcd_save_interval)
        {
            pcd_index ++;
            string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to /PCD/" << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();// 存储之后
            scan_wait_num = 0;
        }
    }
}
// PointCloudXYZI::Ptr pcl_wait_acc_100(new PointCloudXYZI());
// void points_acc_100(){
//     for(int i = 0;i<100;i++){
//         int size = feats_undistort->points.size();
//         PointCloudXYZI::Ptr laserCloudWorld( new PointCloudXYZI(size, 1));
//         for (int i = 0; i < size; i++)
//         {
//             RGBpointBodyToWorld(&feats_undistort->points[i], \
//                                 &laserCloudWorld->points[i]);
//         }
//     }

// }
PointCloudXYZI::Ptr pcl_wait_save1(new PointCloudXYZI());
void publish_Accmap(const ros::Publisher & pubLaser_pointAcc_map)
{
        // PointCloudXYZI::Ptr laserCloudFullRes1(feats_undistort);
        // int size = laserCloudFullRes1->points.size();
        // PointCloudXYZI::Ptr laserCloudWorld1( new PointCloudXYZI(size, 1));
        // for (int i = 0; i < size; i++)
        // {
        //     RGBpointBodyToWorld(&laserCloudFullRes1->points[i], &laserCloudWorld1->points[i]);
        // }
        // sensor_msgs::PointCloud2 laserCloudmsg_1;
        // pcl::toROSMsg(*laserCloudWorld, laserCloudmsg_1);
        // laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
        // laserCloudmsg.header.frame_id = "camera_init";// 世界的参考帧
        // pubLaserCloudFull.publish(laserCloudmsg);// 发布对应的雷达的点云信息
        // publish_count -= PUBFRAME_PERIOD;

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* TODO 2. noted that pcd save will influence the real-time performences **/

        // 去畸变的点云，去畸变的点云直接累加
        int size = feats_undistort->points.size();
        PointCloudXYZI::Ptr laserCloudWorld( new PointCloudXYZI(size, 1));
        for (int i = 0; i < size; i++)
        {
            RGBpointBodyToWorld(&feats_undistort->points[i], \
                                &laserCloudWorld->points[i]);
        }
        // 这里设置一个点云队列，只存储一定时间的点云
        *pcl_wait_save1 += *laserCloudWorld;// 点云直接相加？？？？
    sensor_msgs::PointCloud2 laserCloudMap;
    pcl::toROSMsg(*pcl_wait_save1, laserCloudMap);
    laserCloudMap.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudMap.header.frame_id = "camera_init";
    pubLaser_pointAcc_map.publish(laserCloudMap);
}


// 发布雷达全部的点，body坐标系，对应的帧是body
void publish_frame_body(const ros::Publisher & pubLaserCloudFull_body)
{
    int size = feats_undistort->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        RGBpointBodyLidarToIMU(&feats_undistort->points[i], \
                            &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudmsg.header.frame_id = "body";
    pubLaserCloudFull_body.publish(laserCloudmsg);
    publish_count -= PUBFRAME_PERIOD;
}
// 发布有效点的数量-camera_init
void publish_effect_world(const ros::Publisher & pubLaserCloudEffect)
{
    PointCloudXYZI::Ptr laserCloudWorld( \
                    new PointCloudXYZI(effct_feat_num, 1));
    for (int i = 0; i < effct_feat_num; i++)
    {
        RGBpointBodyToWorld(&laserCloudOri->points[i], \
                            &laserCloudWorld->points[i]);
    }
    sensor_msgs::PointCloud2 laserCloudFullRes3;
    pcl::toROSMsg(*laserCloudWorld, laserCloudFullRes3);
    laserCloudFullRes3.header.stamp = ros::Time().fromSec(lidar_end_time);
    laserCloudFullRes3.header.frame_id = "camera_init";
    pubLaserCloudEffect.publish(laserCloudFullRes3);
}
// 发布地图
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
// 发布里程计"camera_init"; "body";
void publish_odometry(const ros::Publisher & pubOdomAftMapped)
{
    // 定义帧
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "body";
    odomAftMapped.header.stamp = ros::Time().fromSec(lidar_end_time);// ros::Time().fromSec(lidar_end_time);
    set_posestamp(odomAftMapped.pose);// 仅有一个位姿，得到pose和对应的旋转四元数
    pubOdomAftMapped.publish(odomAftMapped);// 将其发布出去
    auto P = kf.get_P(); // 位置协方差
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
    // 位姿的四元数发布，transform，tf发布
    static tf::TransformBroadcaster br;
    tf::Transform                   transform;// 对应旋转和平移
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
}
// 发布路径
void publish_path(const ros::Publisher pubPath)
{
    set_posestamp(msg_body_pose);
    msg_body_pose.header.stamp = ros::Time().fromSec(lidar_end_time);
    msg_body_pose.header.frame_id = "camera_init";

    /*** if path is too large, the rvis will crash ，不能太大***/
    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0) // 这里间隔10个点取对应的位姿，进行发布
    {
        path.poses.push_back(msg_body_pose);
        pubPath.publish(path);
    }
}

void h_share_model(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
    double match_start = omp_get_wtime();
    laserCloudOri->clear(); 
    corr_normvect->clear(); 
    total_residual = 0.0; 

    /** closest surface search and residual computation **/
    // 最近面计算和残差计算
    #ifdef MP_EN
        omp_set_num_threads(MP_PROC_NUM);
        #pragma omp parallel for
    #endif
    for (int i = 0; i < feats_down_size; i++)
    {
        PointType &point_body  = feats_down_body->points[i]; 
        PointType &point_world = feats_down_world->points[i]; 

        /* transform to world frame */
        V3D p_body(point_body.x, point_body.y, point_body.z);
        V3D p_global(s.rot * (s.offset_R_L_I*p_body + s.offset_T_L_I) + s.pos);
        point_world.x = p_global(0);
        point_world.y = p_global(1);
        point_world.z = p_global(2);
        point_world.intensity = point_body.intensity;

        vector<float> pointSearchSqDis(NUM_MATCH_POINTS);
        auto &points_near = Nearest_Points[i];

        if (ekfom_data.converge)
        {
            /** Find the closest surfaces in the map **/            // 找到五个点，且最近的点小于5m
            ikdtree.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
            point_selected_surf[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
        }

        if (!point_selected_surf[i]) continue;

        VF(4) pabcd;
        point_selected_surf[i] = false;
        // 估计一个平面
        if (esti_plane(pabcd, points_near, 0.1f))
        {
            // 点到平面的距离
            float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
            float s = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

            if (s > 0.9)
            {
                point_selected_surf[i] = true;
                normvec->points[i].x = pabcd(0);
                normvec->points[i].y = pabcd(1);
                normvec->points[i].z = pabcd(2);
                normvec->points[i].intensity = pd2;
                res_last[i] = abs(pd2);
            }
        }
    }
    
    effct_feat_num = 0;

    for (int i = 0; i < feats_down_size; i++)
    {
        if (point_selected_surf[i])
        {
            laserCloudOri->points[effct_feat_num] = feats_down_body->points[i];
            corr_normvect->points[effct_feat_num] = normvec->points[i];
            total_residual += res_last[i];
            effct_feat_num ++;
        }
    }

    if (effct_feat_num < 1)
    {
        ekfom_data.valid = false;
        ROS_WARN("No Effective Points! \n");
        return;
    }

    res_mean_last = total_residual / effct_feat_num;//残差的均值
    match_time  += omp_get_wtime() - match_start;
    double solve_start_  = omp_get_wtime();
    
    /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
    ekfom_data.h_x = MatrixXd::Zero(effct_feat_num, 12); //23
    ekfom_data.h.resize(effct_feat_num);

    for (int i = 0; i < effct_feat_num; i++)
    {
        const PointType &laser_p  = laserCloudOri->points[i];
        V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
        M3D point_be_crossmat;
        point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
        V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
        M3D point_crossmat;
        point_crossmat<<SKEW_SYM_MATRX(point_this);

        /*** get the normal vector of closest surface/corner ***/
        const PointType &norm_p = corr_normvect->points[i];
        V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

        /*** calculate the Measuremnt Jacobian matrix H ***/
        V3D C(s.rot.conjugate() *norm_vec);
        V3D A(point_crossmat * C);
        if (extrinsic_est_en)
        {
            V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C); //s.rot.conjugate()*norm_vec);
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
        }
        else
        {
            ekfom_data.h_x.block<1, 12>(i,0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        }

        /*** Measuremnt: distance to the closest surface/corner ***/
        ekfom_data.h(i) = -norm_p.intensity;
    }
    solve_time += omp_get_wtime() - solve_start_;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laserMapping");
    ros::NodeHandle nh;
// 从yaml文件处读取参数
    nh.param<bool>("publish/path_en",path_en, true); // 是否发布路径的topic
    nh.param<bool>("publish/scan_publish_en",scan_pub_en, true);// 是否发布当前正在扫描的点云的topic
    nh.param<bool>("publish/dense_publish_en",dense_pub_en, true);// 是否在全局帧点云扫描中降低点数
    nh.param<bool>("publish/scan_bodyframe_pub_en",scan_body_pub_en, true);// 是否发布经过运动畸变校正注册到IMU坐标系的点云的topic
    nh.param<int>("max_iteration",NUM_MAX_ITERATIONS,4); // 卡尔曼滤波的最大迭代次数
    nh.param<string>("map_file_path",map_file_path,"");
    // 这是话题参数
    nh.param<string>("common/lid_topic",lid_topic,"/livox/lidar");
    nh.param<string>("common/imu_topic", imu_topic,"/livox/imu");
    nh.param<bool>("common/time_sync_en", time_sync_en, false);
    nh.param<double>("common/time_offset_lidar_to_imu", time_diff_lidar_to_imu, 0.0);
    nh.param<double>("filter_size_corner",filter_size_corner_min,0.5);// VoxelGrid降采样时的体素大小//角点
    nh.param<double>("filter_size_surf",filter_size_surf_min,0.5);//平面点
    nh.param<double>("filter_size_map",filter_size_map_min,0.5);//地图
    nh.param<double>("cube_side_length",cube_len,200); // 地图的局部区域的长度（FastLio2论文中有解释）
    nh.param<float>("mapping/det_range",DET_RANGE,300.f);// 激光雷达的最大探测范围
    nh.param<double>("mapping/fov_degree",fov_deg,180);// 激光雷达的视场角
    nh.param<double>("mapping/gyr_cov",gyr_cov,0.1);
    nh.param<double>("mapping/acc_cov",acc_cov,0.1);
    nh.param<double>("mapping/b_gyr_cov",b_gyr_cov,0.0001);
    nh.param<double>("mapping/b_acc_cov",b_acc_cov,0.0001);
    // 雷达的基本参数
    nh.param<double>("preprocess/blind", p_pre->blind, 0.01);// 最小距离阈值，即过滤掉0～blind范围内的点云
    nh.param<int>("preprocess/lidar_type", p_pre->lidar_type, AVIA);
    nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 16);// 激光雷达扫描的线数（livox avia为6线）
    nh.param<int>("preprocess/timestamp_unit", p_pre->time_unit, US);
    nh.param<int>("preprocess/scan_rate", p_pre->SCAN_RATE, 10);

    nh.param<int>("point_filter_num", p_pre->point_filter_num, 2);// 采样间隔，即每隔point_filter_num个点取1个点
    nh.param<bool>("feature_extract_enable", p_pre->feature_enabled, false);// 是否提取特征点（FAST_LIO2默认不进行特征点提取）
    nh.param<bool>("runtime_pos_log_enable", runtime_pos_log, 0); // 是否输出调试log信息
    nh.param<bool>("mapping/extrinsic_est_en", extrinsic_est_en, true);// 是否进行在线的外参估计
    nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en, false);
    nh.param<int>("pcd_save/interval", pcd_save_interval, -1);// 每一个PCD文件保存多少个雷达帧（-1表示所有雷达帧都保存在一个PCD文件中）
    nh.param<vector<double>>("mapping/extrinsic_T", extrinT, vector<double>());// 雷达相对于IMU的外参T（即雷达在IMU坐标系中的坐标
    nh.param<vector<double>>("mapping/extrinsic_R", extrinR, vector<double>());// 雷达相对于IMU的外参R
    cout<<"p_pre->lidar_type "<<p_pre->lidar_type<<endl;
    // nav_msgs::Path 消息类型，初始化path的header（包括时间戳和帧id），path：用于保存odemetry的路径
    path.header.stamp    = ros::Time::now();
    path.header.frame_id ="camera_init";

    /*** variables definition ***/
    /**
     * * effect_feat_num          有效的特征点数量（未用到该变量）
 * frame_num                雷达总帧数
 * deltaT                 平移增量  （未用到该变量）
 * deltaR                   旋转增量（未用到该变量）
 * aver_time_consu          每帧平均的处理总时间
 * aver_time_icp            每帧中icp的平均时间
 * aver_time_match          每帧中匹配的平均时间
 * aver_time_incre          每帧中ikd-tree增量处理的平均时间
 * aver_time_solve          每帧中计算的平均时间
 * aver_time_const_H_time   每帧中计算的平均时间（当H恒定时）
 * flg_EKF_converged        扩展卡尔曼滤波收敛标志（未用到该变量）
 * EKF_stop_flg             扩展卡尔曼滤波停止标志（未用到该变量）
 * FOV_DEG                  视场角度（未用到该变量）
 * HALF_FOV_COS             视场角度半值的cos值（未用到该变量）
 * _featsArray              特征点数组（未用到该变量）
    */
    int effect_feat_num = 0, frame_num = 0;
    double deltaT, deltaR, aver_time_consu = 0, aver_time_icp = 0, aver_time_match = 0, aver_time_incre = 0, aver_time_solve = 0, aver_time_const_H_time = 0;
    bool flg_EKF_converged, EKF_stop_flg = 0;
    
    FOV_DEG = (fov_deg + 10.0) > 179.9 ? 179.9 : (fov_deg + 10.0);
    HALF_FOV_COS = cos((FOV_DEG) * 0.5 * PI_M / 180.0);//不太明白这里这样处理的意义（虽然没有用到）

    _featsArray.reset(new PointCloudXYZI());
    // 将从点云中挑选的平面点数组point_selected_surf内元素的值全部设为true，数组point_selected_surf用于平面特征
    memset(point_selected_surf, true, sizeof(point_selected_surf));
    // 数组res_last用于点与平面间的残差距离
    memset(res_last, -1000.0f, sizeof(res_last));
    // VoxelGrid滤波器参数，即进行滤波时的创建的体素边长为filter_size_surf_min
    downSizeFilterSurf.setLeafSize(filter_size_surf_min, filter_size_surf_min, filter_size_surf_min);
    downSizeFilterMap.setLeafSize(filter_size_map_min, filter_size_map_min, filter_size_map_min);
    // memset函数用于将一段内存区域的值设置为指定的字节值。
    memset(point_selected_surf, true, sizeof(point_selected_surf));
    memset(res_last, -1000.0f, sizeof(res_last));
    // 输出雷达相对于IMU的外参R和T
    Lidar_T_wrt_IMU<<VEC_FROM_ARRAY(extrinT);
    Lidar_R_wrt_IMU<<MAT_FROM_ARRAY(extrinR);
    // 设置IMU的参数，对p_imu进行初始化，其中p_imu为ImuProcess的智能指针（ImuProcess是进行IMU处理的类）
    p_imu->set_extrinsic(Lidar_T_wrt_IMU, Lidar_R_wrt_IMU);
    p_imu->set_gyr_cov(V3D(gyr_cov, gyr_cov, gyr_cov));
    p_imu->set_acc_cov(V3D(acc_cov, acc_cov, acc_cov));
    p_imu->set_gyr_bias_cov(V3D(b_gyr_cov, b_gyr_cov, b_gyr_cov));
    p_imu->set_acc_bias_cov(V3D(b_acc_cov, b_acc_cov, b_acc_cov));

    // 这两句代码都是用来初始化数组epsi的元素值为0.001。
    double epsi[23] = {0.001};
    fill(epsi, epsi+23, 0.001);
    /**
     * 扩展卡尔曼滤波器的初始化
     * 定义了平面搜索和残差计算
    */
    kf.init_dyn_share(get_f, df_dx, df_dw, h_share_model, NUM_MAX_ITERATIONS, epsi);

    /*** debug record ***/
    // 调试记录的日志文件开启
    FILE *fp;
    string pos_log_dir = root_dir + "/Log/pos_log.txt";
    fp = fopen(pos_log_dir.c_str(),"w");

    ofstream fout_pre, fout_out, fout_dbg;
    fout_pre.open(DEBUG_FILE_DIR("mat_pre.txt"),ios::out);
    fout_out.open(DEBUG_FILE_DIR("mat_out.txt"),ios::out);
    fout_dbg.open(DEBUG_FILE_DIR("dbg.txt"),ios::out);
    if (fout_pre && fout_out)
        cout << "~~~~"<<ROOT_DIR<<" file opened" << endl;
    else
        cout << "~~~~"<<ROOT_DIR<<" doesn't exist" << endl;

    /*** ROS subscribe initialization ***/
    // 雷达点云的订阅器sub_pcl，订阅点云的topic//放入缓存器中
    ros::Subscriber sub_pcl = p_pre->lidar_type == AVIA ? \
        nh.subscribe(lid_topic, 200000, livox_pcl_cbk) : \
        nh.subscribe(lid_topic, 200000, standard_pcl_cbk);
    ros::Subscriber sub_imu = nh.subscribe(imu_topic, 200000, imu_cbk);



    ros::Publisher pubLaserCloudFull = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered", 100000);    // 发布当前正在扫描的点云，topic名字为/cloud_registered
    ros::Publisher pubLaserCloudFull_body = nh.advertise<sensor_msgs::PointCloud2>("/cloud_registered_body", 100000);   // 发布经过运动畸变校正注册到IMU坐标系的点云，topic名字为/cloud_registered_body
    ros::Publisher pubLaserCloudEffect = nh.advertise<sensor_msgs::PointCloud2>("/cloud_effected", 100000);
    ros::Publisher pubLaserCloudMap = nh.advertise<sensor_msgs::PointCloud2>("/Laser_map", 100000);
    // 发布当前里程计信息，topic名字为/Odometry
    ros::Publisher pubOdomAftMapped = nh.advertise<nav_msgs::Odometry> ("/Odometry", 100000);
            // 发布里程计总的路径，topic名字为/path
    ros::Publisher pubPath          = nh.advertise<nav_msgs::Path> ("/path", 100000);
    // 发布累积的点云,发布这样一个点云
    ros::Publisher pubLaser_pointAcc_map = nh.advertise<sensor_msgs::PointCloud2>("/Laser_ACCmap", 100000);


    // ros::Subscriber sub_point_cloud_ = nh.subscribe("/cloud_registered",100000, &point_clips);

    // ros::Publisher pub_filtered_points_ = nh.advertise<sensor_msgs::PointCloud2>("/filtered_points", 100000);
//------------------------------------------------------------------------------------------------------
    signal(SIGINT, SigHandle);// 中断处理函数，如果有中断信号（比如Ctrl+C），则执行第二个参数里面的SigHandle函数
    ros::Rate rate(5000);// 设置ROS程序主循环每次运行的时间至少为0.0002秒（5000Hz）
    bool status = ros::ok();
    // 程序主循环
    while (status)
    {
        if (flg_exit) break;// 如果有中断产生，则结束主循环
        ros::spinOnce();// ROS消息回调处理函数，放在ROS的主循环中
        if(sync_packages(Measures)) // 将激光雷达点云数据和IMU数据从缓存队列中取出，进行时间对齐，并保存到Measures中
        {
            if (flg_first_scan) // 激光雷达第一次扫描
            {
                first_lidar_time = Measures.lidar_beg_time;// 记录激光雷达第一次扫描的时间
                p_imu->first_lidar_time = first_lidar_time;
                flg_first_scan = false;
                continue;
            }
//IKD树建图以及相关算法求解所用时间记录//用于后续输出到日志文件中//看来是记录各个模块的处理时间的
            double t0,t1,t2,t3,t4,t5,match_start, solve_start, svd_time;

            match_time = 0;
            kdtree_search_time = 0.0;
            solve_time = 0;
            solve_const_H_time = 0;
            svd_time   = 0;
            t0 = omp_get_wtime();
            // 对IMU数据进行预处理（初始化），其中包含了点云畸变处理

            p_imu->Process(Measures, kf, feats_undistort);
            state_point = kf.get_x();
            //激光雷达的位姿
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;

            if (feats_undistort->empty() || (feats_undistort == NULL))// 点云的去畸变为空
            {
                ROS_WARN("No point, skip this scan!\n");
                continue;
            }
            //EKF初始化完成
            flg_EKF_inited = (Measures.lidar_beg_time - first_lidar_time) < INIT_TIME ? \
                            false : true;
            /*** Segment the map in lidar FOV ***/
            lasermap_fov_segment();

            /*** downsample the feature points in a scan ***/
            downSizeFilterSurf.setInputCloud(feats_undistort);
            downSizeFilterSurf.filter(*feats_down_body);
            t1 = omp_get_wtime();
            feats_down_size = feats_down_body->points.size();
            /*** initialize the map kdtree ***/
            if(ikdtree.Root_Node == nullptr)//是否还没有根节点
            {
                if(feats_down_size > 5)//降采样后的特征点树是否多于5个
                {
                    ikdtree.set_downsample_param(filter_size_map_min);//设置IKDtree的降采样参数//0.5米的方块
                    feats_down_world->resize(feats_down_size);//世界系下降采样点数空间大小重置
                    for(int i = 0; i < feats_down_size; i++)
                    {
                        pointBodyToWorld(&(feats_down_body->points[i]), &(feats_down_world->points[i]));
                    }
                    ikdtree.Build(feats_down_world->points);//根据世界系下的降采样特征点构建ikdtree

                }
                continue;
            }
            int featsFromMapNum = ikdtree.validnum();//获取ikdtree中有效点的数目
            kdtree_size_st = ikdtree.size();//获取ikdtree的大小？它的大小和有效点数目的区别是什么呢？
            
            // cout<<"[ mapping ]: In num: "<<feats_undistort->points.size()<<" downsamp "<<feats_down_size<<" Map num: "<<featsFromMapNum<<"effect num:"<<effct_feat_num<<endl;

            /*** ICP and iterated Kalman filter update ***/
            if (feats_down_size < 5)
            {
                ROS_WARN("No point, points too small, skip this scan!\n");
                continue;
            }
            
            normvec->resize(feats_down_size);
            feats_down_world->resize(feats_down_size);

            V3D ext_euler = SO3ToEuler(state_point.offset_R_L_I);//状态中Imu与雷达的旋转偏差转换为欧拉角、
            //计算并输出状态处理结果到日志文件中
            fout_pre<<setw(20)<<Measures.lidar_beg_time - first_lidar_time<<" "<<euler_cur.transpose()<<" "<< state_point.pos.transpose()<<" "<<ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<< " " << state_point.vel.transpose() \
            <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<< endl;
            // 待定，后续研究
            if(1) // If you need to see map point, change to "if(1)"
            {
                PointVector ().swap(ikdtree.PCL_Storage);
                ikdtree.flatten(ikdtree.Root_Node, ikdtree.PCL_Storage, NOT_RECORD);
                featsFromMap->clear();
                featsFromMap->points = ikdtree.PCL_Storage;
            }
            //对搜索点索引数组以及最近点数组大小进行更新
            pointSearchInd_surf.resize(feats_down_size);
            Nearest_Points.resize(feats_down_size);
            int  rematch_num = 0;
            bool nearest_search_en = true; //

            t2 = omp_get_wtime();
            
            /*** iterated state estimation ***/
            double t_update_start = omp_get_wtime();
            double solve_H_time = 0;
            //卡尔曼迭代函数前者针对一个特定系统修改的迭代误差状态EKF更新，后者求解时间
            kf.update_iterated_dyn_share_modified(LASER_POINT_COV, solve_H_time);
            state_point = kf.get_x();//更新后的状态
            euler_cur = SO3ToEuler(state_point.rot);
            pos_lid = state_point.pos + state_point.rot * state_point.offset_T_L_I;//激光雷达位姿
            geoQuat.x = state_point.rot.coeffs()[0];
            geoQuat.y = state_point.rot.coeffs()[1];
            geoQuat.z = state_point.rot.coeffs()[2];
            geoQuat.w = state_point.rot.coeffs()[3];//相应的四元数

            double t_update_end = omp_get_wtime();

            /******* Publish odometry *******/
            publish_odometry(pubOdomAftMapped);

            /*** add the feature points to map kdtree ***/
            t3 = omp_get_wtime();
            map_incremental();//ikdtree的增量式建图
            t5 = omp_get_wtime();
            
            /******* Publish points 应该在这里处理*******/
            if (path_en)                         publish_path(pubPath);
            if (scan_pub_en || pcd_save_en)      publish_frame_world(pubLaserCloudFull);//当前扫描点
            if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFull_body);//去畸变后载体系的点云
            // PclTestCore pub_fliterss(nh);
            // publish_effect_world(pubLaserCloudEffect);
            publish_map(pubLaserCloudMap);

            publish_Accmap(pubLaser_pointAcc_map);


            //

            /*** Debug variables ***/
            if (runtime_pos_log)
            {
                frame_num ++;//统计帧数
                kdtree_size_end = ikdtree.size();//统计ikdtree的大小
                aver_time_consu = aver_time_consu * (frame_num - 1) / frame_num + (t5 - t0) / frame_num;//每帧点云在框架中计算所用平均时间
                aver_time_icp = aver_time_icp * (frame_num - 1)/frame_num + (t_update_end - t_update_start) / frame_num;//每帧点云ICP迭代所用平均时间
                aver_time_match = aver_time_match * (frame_num - 1)/frame_num + (match_time)/frame_num;//每帧点云进行匹配所用平均时间
                aver_time_incre = aver_time_incre * (frame_num - 1)/frame_num + (kdtree_incremental_time)/frame_num;//每帧点云进行ikdtree增量操作所用平均时间
                aver_time_solve = aver_time_solve * (frame_num - 1)/frame_num + (solve_time + solve_H_time)/frame_num;//总的求解时间平均值
                aver_time_const_H_time = aver_time_const_H_time * (frame_num - 1)/frame_num + solve_time / frame_num;
                T1[time_log_counter] = Measures.lidar_beg_time;
                s_plot[time_log_counter] = t5 - t0;//当前帧所用总时间
                s_plot2[time_log_counter] = feats_undistort->points.size();//去畸变后特征点数量
                s_plot3[time_log_counter] = kdtree_incremental_time;//当前帧Ikdtree增量时间
                s_plot4[time_log_counter] = kdtree_search_time;//当前帧KDtree搜索所用时间
                s_plot5[time_log_counter] = kdtree_delete_counter;//当前帧kdtree删减计数器
                s_plot6[time_log_counter] = kdtree_delete_time;//当前帧kdtree删减所用时间
                s_plot7[time_log_counter] = kdtree_size_st;//当前帧添加特征点到地图之前KDTREE的大小
                s_plot8[time_log_counter] = kdtree_size_end;//当前帧添加特征点到地图之后KDTREE的大小
                s_plot9[time_log_counter] = aver_time_consu;//上面计算出的平均计算时间
                s_plot10[time_log_counter] = add_point_size;//kdtree中插入的点数量
                time_log_counter ++;
                printf("[ mapping ]: time: IMU + Map + Input Downsample: %0.6f ave match: %0.6f ave solve: %0.6f  ave ICP: %0.6f  map incre: %0.6f ave total: %0.6f icp: %0.6f construct H: %0.6f \n",t1-t0,aver_time_match,aver_time_solve,t3-t1,t5-t3,aver_time_consu,aver_time_icp, aver_time_const_H_time);
                ext_euler = SO3ToEuler(state_point.offset_R_L_I);
                fout_out << setw(20) << Measures.lidar_beg_time - first_lidar_time << " " << euler_cur.transpose() << " " << state_point.pos.transpose()<< " " << ext_euler.transpose() << " "<<state_point.offset_T_L_I.transpose()<<" "<< state_point.vel.transpose() \
                <<" "<<state_point.bg.transpose()<<" "<<state_point.ba.transpose()<<" "<<state_point.grav<<" "<<feats_undistort->points.size()<<endl;
                dump_lio_state_to_log(fp);
            }
        }

        status = ros::ok();
        rate.sleep();
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. pcd save will largely influence the real-time performences **/
    if (pcl_wait_save->size() > 0 && pcd_save_en)
    {
        string file_name = string("scans.pcd");
        string all_points_dir(string(string(ROOT_DIR) + "PCD/") + file_name);
        pcl::PCDWriter pcd_writer;
        cout << "current scan saved to /PCD/" << file_name<<endl;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }

    fout_out.close();
    fout_pre.close();

    if (runtime_pos_log)
    {
        vector<double> t, s_vec, s_vec2, s_vec3, s_vec4, s_vec5, s_vec6, s_vec7;    
        FILE *fp2;
        string log_dir = root_dir + "/Log/fast_lio_time_log.csv";
        fp2 = fopen(log_dir.c_str(),"w");
        fprintf(fp2,"time_stamp, total time, scan point size, incremental time, search time, delete size, delete time, tree size st, tree size end, add point size, preprocess time\n");
        for (int i = 0;i<time_log_counter; i++){
            fprintf(fp2,"%0.8f,%0.8f,%d,%0.8f,%0.8f,%d,%0.8f,%d,%d,%d,%0.8f\n",T1[i],s_plot[i],int(s_plot2[i]),s_plot3[i],s_plot4[i],int(s_plot5[i]),s_plot6[i],int(s_plot7[i]),int(s_plot8[i]), int(s_plot10[i]), s_plot11[i]);
            t.push_back(T1[i]);
            s_vec.push_back(s_plot9[i]);
            s_vec2.push_back(s_plot3[i] + s_plot6[i]);
            s_vec3.push_back(s_plot4[i]);
            s_vec5.push_back(s_plot[i]);
        }
        fclose(fp2);
    }

    return 0;
}
