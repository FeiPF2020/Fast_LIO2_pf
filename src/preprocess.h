#include <ros/ros.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <livox_ros_driver/CustomMsg.h>

using namespace std;
// 未使用
#define IS_VALID(a)  ((abs(a)>1e8) ? true : false)

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

enum LID_TYPE {
    AVIA = 1, VELO16, OUST64
}; //{1, 2, 3}
enum TIME_UNIT {
    SEC = 0, MS = 1, US = 2, NS = 3
};
/**
 * Nor,正常点
 * Poss_Plane,可能的平面点
 * Real_Plane, 确定是平面点
 * Edge_Jump, 有跨越的边
 * Edge_Plane, 边上的平面点
 * Wire, 线段
 * ZeroPoint 无效点，未使用
 */
enum Feature {
    Nor, Poss_Plane, Real_Plane, Edge_Jump, Edge_Plane, Wire, ZeroPoint
};
/**
 * 位置标识，前一个后一个
 */
enum Surround {
    Prev, Next
};
// 枚举类型：表示有跨越边的类型
enum E_jump {
    Nr_nor, Nr_zero, Nr_180, Nr_inf, Nr_blind
};//有跨越边的类型 ：正常、0、180、无穷大、在盲区？
//用于记录每个点的距离、角度、特征种类等属性用于存储激光雷达点的一些其他属性
struct orgtype//用于存储点云的其他属性————这里的所有属性，都是服务于提取点云特征
{
    double range; // 点云在xy平面离雷达中心的距离
    double dista; // 当前点与后一个点之间的距离的平方
    //假设雷达原点为O 前一个点为M 当前点为A 后一个点为N//假设雷达原点为O 前一个点为M 当前点为A 后一个点为N
    //假设雷达原点为O 前一个点为M 当前点为A 后一个点为N
    double angle[2];// 这个是角OAM和角OAN的cos值// cos(当前点指向前一点或后一点的向量, ray)
    double intersect;// 这个是角MAN的cos值    // 当前点与相邻两点的夹角cos值     //判断点云特征用
    E_jump edj[2];// 前后两点的类型    // 点前后两个方向的edge_jump类型         //判断特征用
    Feature ftype;// 点类型                    //判断特征用
    orgtype() {
        range = 0;
        edj[Prev] = Nr_nor;
        edj[Next] = Nr_nor;
        ftype = Nor;//默认是正常点
        intersect = 2;
    }
};


// velodyne数据结构
namespace velodyne_ros {
    struct EIGEN_ALIGN16 Point {
        PCL_ADD_POINT4D; // 4D点坐标类型
        float intensity;// 强度
        float time;
        uint16_t ring; // 点所属的圈数

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // 内存对齐
    };
}  // namespace velodyne_ros
// 注册velodyne_ros的Point类型
POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_ros::Point,
                                  (float, x, x)
                                          (float, y, y)
                                          (float, z, z)
                                          (float, intensity, intensity)
                                          (float, time, time)
                                          (uint16_t, ring, ring)
)
// ouster数据结构
namespace ouster_ros {
    struct EIGEN_ALIGN16 Point {
        PCL_ADD_POINT4D;
        float intensity;
        uint32_t t;// 时间
        uint16_t reflectivity;// 反射率
        uint8_t ring;
        uint16_t ambient;// 没用到
        uint32_t range; // 距离

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}  // namespace ouster_ros

// clang-format off
POINT_CLOUD_REGISTER_POINT_STRUCT(ouster_ros::Point,
                                  (float, x, x)
                                          (float, y, y)
                                          (float, z, z)
                                          (float, intensity, intensity)
                                          // use std::uint32_t to avoid conflicting with pcl::uint32_t
                                          (std::uint32_t, t, t)
                                          (std::uint16_t, reflectivity, reflectivity)
                                          (std::uint8_t, ring, ring)
                                          (std::uint16_t, ambient, ambient)
                                          (std::uint32_t, range, range)
)
// Preproscess类：用于对激光雷达点云数据进行预处理
class Preprocess {
public:
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Preprocess();

    ~Preprocess();

    void process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);//对livox自定义msg格式的雷达数据进行处理
    void process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);//对ros的Msg格式的雷达数据进行处理
    void set(bool feat_en, int lid_type, double bld, int pfilt_num);

    // sensor_msgs::PointCloud2::ConstPtr pointcloud;
    PointCloudXYZI pl_full, pl_corn, pl_surf;//全部点//储存全部点(特征提取或间隔采样后）、角点、面特征点
    PointCloudXYZI pl_buff[128]; //maximum 128 line lidar
    vector<orgtype> typess[128]; //maximum 128 line lidar
    float time_unit_scale;
    int lidar_type, point_filter_num, N_SCANS, SCAN_RATE, time_unit;//雷达类型、采样间隔、扫描线数、扫描频率
    double blind;  //最小距离值（盲区）
    bool feature_enabled, given_offset_time;//是否特征提取、是否时间偏移
    ros::Publisher pub_full, pub_surf, pub_corn;//发布全部点、发布平面点、发布边缘点


private:
    void avia_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg);// 用于对Livox激光雷达数据进行处理
    void oust64_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg);

    void give_feature(PointCloudXYZI &pl, vector<orgtype> &types);// 没有用到

    void pub_func(PointCloudXYZI &pl, const ros::Time &ct);

    int
    plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, uint &i_nex, Eigen::Vector3d &curr_direct);

    bool small_plane(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex,
                     Eigen::Vector3d &curr_direct);

    bool edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir);

    int group_size;//计算平面特征时需要的最少局部点数
    double disA, disB, inf_bound;
    double limit_maxmid, limit_midmin, limit_maxmin;
    double p2l_ratio;
    double jump_up_limit, jump_down_limit;
    double cos160;
    double edgea, edgeb;
    double smallp_intersect, smallp_ratio;
    double vx, vy, vz;
};
