#include "preprocess.h"

#define RETURN0     0x00
#define RETURN0AND1 0x10

Preprocess::Preprocess()
        : feature_enabled(0), lidar_type(AVIA), blind(0.01), point_filter_num(1) {
    inf_bound = 10;// 有效点集合,大于10m则是盲区
    N_SCANS = 6;////多线激光雷达的线数
    SCAN_RATE = 10;//不单独服务特征提取
    group_size = 8;//8个点为一组
    disA = 0.01;// 点集合的距离阈值,判断是否为平面
    disA = 0.1; // B?
    p2l_ratio = 225; // 点到线的距离阈值，需要大于这个值才能判断组成面
    limit_maxmid = 6.25;// 中点到左侧的距离变化率范围
    limit_midmin = 6.25;// 中点到右侧的距离变化率范围
    limit_maxmin = 3.24;// 左侧到右侧的距离变化率范围
    jump_up_limit = 170.0;
    jump_down_limit = 8.0;
    cos160 = 160.0;
    edgea = 2;//点与点距离超过两倍则认为遮挡
    edgeb = 0.1;//点与点距离超过0.1m则认为遮挡
    smallp_intersect = 172.5;
    smallp_ratio = 1.2;//三个点如果角度大于172.5度，且比例小于1.2倍，则认为是平面
    given_offset_time = false;

    jump_up_limit = cos(jump_up_limit / 180 * M_PI);//角度大于170度的点跳过，认为在
    jump_down_limit = cos(jump_down_limit / 180 * M_PI); //角度小于8度的点跳过
    cos160 = cos(cos160 / 180 * M_PI);//夹角限制
    smallp_intersect = cos(smallp_intersect / 180 * M_PI);//三个点如果角度大于172.5度，且比例小于1.2倍，则认为是平面
}


Preprocess::~Preprocess() {}

//可是该函数并未被使用
void Preprocess::set(bool feat_en, int lid_type, double bld, int pfilt_num) {
    feature_enabled = feat_en;//是否提取特征点
    lidar_type = lid_type;
    blind = bld;
    point_filter_num = pfilt_num;
}

/**
 * @brief Livox激光雷达点云预处理函数
 * @param msg livox激光雷达点云数据，格式为livox_ros_driver::CustomMsg
 * @param pcl_out 输出处理后的点云数据，格式为pcl::PointCloud<pcl::PointXYZINormal>
 * 仅仅考虑的是面特征
*/
void Preprocess::process(const livox_ros_driver::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out) {
    avia_handler(msg);
    *pcl_out = pl_surf; //所有点云中有效的点云 —— pl_surf,用地址的解引用，第二个参数的地址内的东西 已经变成pl_surf，
}

void Preprocess::process(const sensor_msgs::PointCloud2::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out) {
    switch (time_unit) {
        case SEC:
            time_unit_scale = 1.e3f;
            break;
        case MS:
            time_unit_scale = 1.f;
            break;
        case US:
            time_unit_scale = 1.e-3f;
            break;
        case NS:
            time_unit_scale = 1.e-6f;
            break;
        default:
            time_unit_scale = 1.f;
            break;
    }

    switch (lidar_type) {
        case OUST64:
            oust64_handler(msg);
            break;

        case VELO16:
            velodyne_handler(msg);
            break;

        default:
            printf("Error LiDAR Type");
            break;
    }
    *pcl_out = pl_surf;
}
/***
 * 对Livox激光雷达点云数据进行预处理
 * @param msg livox激光雷达点云数据，格式为livox_ros_driver::CustomMsg

// Header header             # ROS standard message header
// uint64 timebase          # The time of first point       //此帧的第一个点的时间
// uint32 point_num      # Total number of pointclouds      //总点云数量
// uint8 lidar_id               # Lidar device id number    //雷达设备的id
// uint8[3] rsvd                 # Reserved use             //？
// CustomPoint[] points    # Pointcloud data                //点云数据（类似容器）
 */
// 最主要是做点云的类型转换（默认不用特征，否则在加点云特征提取、判断），一帧的数据
void Preprocess::avia_handler(const livox_ros_driver::CustomMsg::ConstPtr &msg) {
    pl_surf.clear();// 清除之前的点云缓存
    pl_corn.clear();// PointCloudXYZI类型点云
    pl_full.clear();
    double t1 = omp_get_wtime();
    int plsize = msg->point_num;//获取此帧点云的数量
    // cout<<"plsie: "<<plsize<<endl;

    pl_corn.reserve(plsize);//reverse按照数量设置点云大小（分配内存空间）———— 点云PointCloudXYZI的本质还是容器，所以resize也是容器的成员函数
    pl_surf.reserve(plsize);
    pl_full.resize(plsize);

    for (int i = 0; i < N_SCANS; i++) {
        pl_buff[i].clear();//线数对应的数据
        pl_buff[i].reserve(plsize);// 缓存的每一线数的点的数量 每一个scan保存的点云数量
    }
    uint valid_num = 0;//无符号整数————有效点数量
    if (feature_enabled) {
        // 特征提取的话
        for (uint i = 1; i < plsize; i++) {
            if ((msg->points[i].line < N_SCANS) &&
                ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00)) { //把点云从livox数据类型转换为pcl的形式
                pl_full[i].x = msg->points[i].x;
                pl_full[i].y = msg->points[i].y;
                pl_full[i].z = msg->points[i].z;
                pl_full[i].intensity = msg->points[i].reflectivity;
                pl_full[i].curvature =
                        msg->points[i].offset_time / float(1000000); //use curvature as time of each laser points

                bool is_new = false;
                // 过滤了一下点云
                // 只有当前点和上一点的间距足够大（>1e-7时），才将当前点认为是有用的点，分别加入到对应line的pl_buff队列中 //任何一轴太近都不行
                if ((abs(pl_full[i].x - pl_full[i - 1].x) > 1e-7)
                    || (abs(pl_full[i].y - pl_full[i - 1].y) > 1e-7)
                    || (abs(pl_full[i].z - pl_full[i - 1].z) > 1e-7)) {
                    pl_buff[msg->points[i].line].push_back(pl_full[i]);//追加到点云的缓存器中
                    //缓存器中第i条线(点在livox类型下的第几条线，就也放在缓存器中的第几条线).追加进来（每个点依次追加）
                }
            }
        } //把此组所有点云搬到了pcl的数据类型下——————————————————完毕
        static int count = 0;
        static double time = 0.0;
        count++;
        double t0 = omp_get_wtime();
        // 对每个line中的激光雷达分别进行处理
        for (int j = 0; j < N_SCANS; j++) {
            // 如果当前的线的点太少，直接处理下一条
            if (pl_buff[j].size() <= 5) continue;
            pcl::PointCloud<PointType> &pl = pl_buff[j];//当前缓存所有有用的点
            plsize = pl.size();
            // 存储判断提取特征类型的点云，以每一线进行处理
            vector<orgtype> &types = typess[j];
            types.clear();
            types.resize(plsize);
//      plsize--;/// 算最后一个点，是存在的，可以算距离的，计算对应的属性
            for (uint i = 0; i < plsize - 1; i++) {
                types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
                vx = pl[i].x - pl[i + 1].x;
                vy = pl[i].y - pl[i + 1].y;
                vz = pl[i].z - pl[i + 1].z;
                types[i].dista = sqrt(vx * vx + vy * vy + vz * vz);
            }
//处理最后一个点只有range信息，没有distance信息
            types[plsize].range = sqrt(pl[plsize].x * pl[plsize].x + pl[plsize].y * pl[plsize].y);
            give_feature(pl, types);// 点和点的属性，计算对应的特征
            // pl_surf += pl;
        }
        time += omp_get_wtime() - t0;//计算点云预处理所花费的时间
        printf("Feature extraction time: %lf \n", time / count);
    } else {
        // 遍历每一个点
        for (uint i = 1; i < plsize; i++) {
            // 线数在scan内部， 回波次序为0或者为1的点
            if ((msg->points[i].line < N_SCANS) &&
                ((msg->points[i].tag & 0x30) == 0x10 || (msg->points[i].tag & 0x30) == 0x00)) {
                valid_num++;
                // 等间隔降采样，每间隔一定的point——filter—num 进行一次降采样
                // pl_full 采用的是点云类型
                if (valid_num % point_filter_num == 0) {
                    pl_full[i].x = msg->points[i].x;
                    pl_full[i].y = msg->points[i].y;
                    pl_full[i].z = msg->points[i].z;
                    pl_full[i].intensity = msg->points[i].reflectivity;
                    pl_full[i].curvature = msg->points[i].offset_time /
                                           float(1000000); // use curvature as time of each laser points, curvature unit: ms
                    // 如果距离太近（x,y,z）三者中的一个以及点的距离大于盲区的距离，认为是有效点
                    if (((abs(pl_full[i].x - pl_full[i - 1].x) > 1e-7)
                         || (abs(pl_full[i].y - pl_full[i - 1].y) > 1e-7)
                         || (abs(pl_full[i].z - pl_full[i - 1].z) > 1e-7))
                        && (pl_full[i].x * pl_full[i].x + pl_full[i].y * pl_full[i].y + pl_full[i].z * pl_full[i].z >
                            (blind * blind))) {
                        pl_surf.push_back(pl_full[i]);// 认为降采样的点为平面点
                    }
                }
            }
        }
    }
}

void Preprocess::oust64_handler(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    pl_surf.clear();
    pl_corn.clear();
    pl_full.clear();
    pcl::PointCloud<ouster_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.size();
    pl_corn.reserve(plsize);
    pl_surf.reserve(plsize);
    if (feature_enabled) {
        for (int i = 0; i < N_SCANS; i++) {
            pl_buff[i].clear();
            pl_buff[i].reserve(plsize);
        }

        for (uint i = 0; i < plsize; i++) {
            double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                           pl_orig.points[i].z * pl_orig.points[i].z;
            if (range < (blind * blind)) continue;
            Eigen::Vector3d pt_vec;
            PointType added_pt;
            added_pt.x = pl_orig.points[i].x;
            added_pt.y = pl_orig.points[i].y;
            added_pt.z = pl_orig.points[i].z;
            added_pt.intensity = pl_orig.points[i].intensity;
            added_pt.normal_x = 0;
            added_pt.normal_y = 0;
            added_pt.normal_z = 0;
            double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.3;
            if (yaw_angle >= 180.0)
                yaw_angle -= 360.0;
            if (yaw_angle <= -180.0)
                yaw_angle += 360.0;

            added_pt.curvature = pl_orig.points[i].t * time_unit_scale;
            if (pl_orig.points[i].ring < N_SCANS) {
                pl_buff[pl_orig.points[i].ring].push_back(added_pt);
            }
        }

        for (int j = 0; j < N_SCANS; j++) {
            PointCloudXYZI &pl = pl_buff[j];
            int linesize = pl.size();
            vector<orgtype> &types = typess[j];
            types.clear();
            types.resize(linesize);
            linesize--;
            for (uint i = 0; i < linesize; i++) {
                types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
                vx = pl[i].x - pl[i + 1].x;
                vy = pl[i].y - pl[i + 1].y;
                vz = pl[i].z - pl[i + 1].z;
                types[i].dista = vx * vx + vy * vy + vz * vz;
            }
            types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
            give_feature(pl, types);
        }
    } else {
        double time_stamp = msg->header.stamp.toSec();
        // cout << "===================================" << endl;
        // printf("Pt size = %d, N_SCANS = %d\r\n", plsize, N_SCANS);
        for (int i = 0; i < pl_orig.points.size(); i++) {
            if (i % point_filter_num != 0) continue;

            double range = pl_orig.points[i].x * pl_orig.points[i].x + pl_orig.points[i].y * pl_orig.points[i].y +
                           pl_orig.points[i].z * pl_orig.points[i].z;

            if (range < (blind * blind)) continue;

            Eigen::Vector3d pt_vec;
            PointType added_pt;
            added_pt.x = pl_orig.points[i].x;
            added_pt.y = pl_orig.points[i].y;
            added_pt.z = pl_orig.points[i].z;
            added_pt.intensity = pl_orig.points[i].intensity;
            added_pt.normal_x = 0;
            added_pt.normal_y = 0;
            added_pt.normal_z = 0;
            added_pt.curvature = pl_orig.points[i].t * time_unit_scale; // curvature unit: ms

            pl_surf.points.push_back(added_pt);
        }
    }
    // pub_func(pl_surf, pub_full, msg->header.stamp);
    // pub_func(pl_surf, pub_corn, msg->header.stamp);
}

void Preprocess::velodyne_handler(const sensor_msgs::PointCloud2::ConstPtr &msg) {
    pl_surf.clear();
    pl_corn.clear();
    pl_full.clear();

    pcl::PointCloud<velodyne_ros::Point> pl_orig;
    pcl::fromROSMsg(*msg, pl_orig);
    int plsize = pl_orig.points.size();
    if (plsize == 0) return;
    pl_surf.reserve(plsize);

    /*** These variables only works when no point timestamps given ***/
    double omega_l = 0.361 * SCAN_RATE;       // scan angular velocity
    std::vector<bool> is_first(N_SCANS, true);
    std::vector<double> yaw_fp(N_SCANS, 0.0);      // yaw of first scan point
    std::vector<float> yaw_last(N_SCANS, 0.0);   // yaw of last scan point
    std::vector<float> time_last(N_SCANS, 0.0);  // last offset time
    /*****************************************************************/

    if (pl_orig.points[plsize - 1].time > 0) {
        given_offset_time = true;
    } else {
        given_offset_time = false;
        double yaw_first = atan2(pl_orig.points[0].y, pl_orig.points[0].x) * 57.29578;
        double yaw_end = yaw_first;
        int layer_first = pl_orig.points[0].ring;
        for (uint i = plsize - 1; i > 0; i--) {
            if (pl_orig.points[i].ring == layer_first) {
                yaw_end = atan2(pl_orig.points[i].y, pl_orig.points[i].x) * 57.29578;
                break;
            }
        }
    }

    if (feature_enabled) {
        for (int i = 0; i < N_SCANS; i++) {
            pl_buff[i].clear();
            pl_buff[i].reserve(plsize);
        }

        for (int i = 0; i < plsize; i++) {
            PointType added_pt;
            added_pt.normal_x = 0;
            added_pt.normal_y = 0;
            added_pt.normal_z = 0;
            int layer = pl_orig.points[i].ring;
            if (layer >= N_SCANS) continue;
            added_pt.x = pl_orig.points[i].x;
            added_pt.y = pl_orig.points[i].y;
            added_pt.z = pl_orig.points[i].z;
            added_pt.intensity = pl_orig.points[i].intensity;
            added_pt.curvature = pl_orig.points[i].time * time_unit_scale; // units: ms

            if (!given_offset_time) {
                double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;
                if (is_first[layer]) {
                    // printf("layer: %d; is first: %d", layer, is_first[layer]);
                    yaw_fp[layer] = yaw_angle;
                    is_first[layer] = false;
                    added_pt.curvature = 0.0;
                    yaw_last[layer] = yaw_angle;
                    time_last[layer] = added_pt.curvature;
                    continue;
                }

                if (yaw_angle <= yaw_fp[layer]) {
                    added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l;
                } else {
                    added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
                }

                if (added_pt.curvature < time_last[layer]) added_pt.curvature += 360.0 / omega_l;

                yaw_last[layer] = yaw_angle;
                time_last[layer] = added_pt.curvature;
            }

            pl_buff[layer].points.push_back(added_pt);
        }

        for (int j = 0; j < N_SCANS; j++) {
            PointCloudXYZI &pl = pl_buff[j];
            int linesize = pl.size();
            if (linesize < 2) continue;
            vector<orgtype> &types = typess[j];
            types.clear();
            types.resize(linesize);
            linesize--;
            for (uint i = 0; i < linesize; i++) {
                types[i].range = sqrt(pl[i].x * pl[i].x + pl[i].y * pl[i].y);
                vx = pl[i].x - pl[i + 1].x;
                vy = pl[i].y - pl[i + 1].y;
                vz = pl[i].z - pl[i + 1].z;
                types[i].dista = vx * vx + vy * vy + vz * vz;
            }
            types[linesize].range = sqrt(pl[linesize].x * pl[linesize].x + pl[linesize].y * pl[linesize].y);
            give_feature(pl, types);
        }
    } else {
        for (int i = 0; i < plsize; i++) {
            PointType added_pt;
            // cout<<"!!!!!!"<<i<<" "<<plsize<<endl;

            added_pt.normal_x = 0;
            added_pt.normal_y = 0;
            added_pt.normal_z = 0;
            added_pt.x = pl_orig.points[i].x;
            added_pt.y = pl_orig.points[i].y;
            added_pt.z = pl_orig.points[i].z;
            added_pt.intensity = pl_orig.points[i].intensity;
            added_pt.curvature =
                    pl_orig.points[i].time * time_unit_scale;  // curvature unit: ms // cout<<added_pt.curvature<<endl;

            if (!given_offset_time) {
                int layer = pl_orig.points[i].ring;
                double yaw_angle = atan2(added_pt.y, added_pt.x) * 57.2957;

                if (is_first[layer]) {
                    // printf("layer: %d; is first: %d", layer, is_first[layer]);
                    yaw_fp[layer] = yaw_angle;
                    is_first[layer] = false;
                    added_pt.curvature = 0.0;
                    yaw_last[layer] = yaw_angle;
                    time_last[layer] = added_pt.curvature;
                    continue;
                }

                // compute offset time
                if (yaw_angle <= yaw_fp[layer]) {
                    added_pt.curvature = (yaw_fp[layer] - yaw_angle) / omega_l;
                } else {
                    added_pt.curvature = (yaw_fp[layer] - yaw_angle + 360.0) / omega_l;
                }

                if (added_pt.curvature < time_last[layer]) added_pt.curvature += 360.0 / omega_l;

                yaw_last[layer] = yaw_angle;
                time_last[layer] = added_pt.curvature;
            }

            if (i % point_filter_num == 0) {
                if (added_pt.x * added_pt.x + added_pt.y * added_pt.y + added_pt.z * added_pt.z > (blind * blind)) {
                    pl_surf.points.push_back(added_pt);
                }
            }
        }
    }
}

/**
 * 特征提取
 * @param pl 一条line的点云信息
 * @param types 点云信息对应的属性
 */
void Preprocess::give_feature(pcl::PointCloud<PointType> &pl, vector<orgtype> &types) {
    int plsize = pl.size();// pl线的粗过滤点数
    int plsize2;// 用于估计特征的点数，剩余的点数
    if (plsize == 0) {
        printf("something wrong\n");
        return;
    }
    uint head = 0;
    ///更新第一个head为大于blind的范围的点索引，在盲区之外
    while (types[head].range < blind) {
        head++;
    }
    // Surf 判断当前点之后是否含有8个点，来确定一个面
    // 判断当前点后面是否还有8个点 够的话就逐渐减少
    plsize2 = (plsize > group_size) ? (plsize - group_size) : 0;

    Eigen::Vector3d curr_direct(Eigen::Vector3d::Zero());//当前平面的法向量
    Eigen::Vector3d last_direct(Eigen::Vector3d::Zero());//上一个平面的法向量

    uint i_nex = 0, i2;// i2为当前点的下一个点
    uint last_i = 0;// last_i为上一个点的保存的索引
    uint last_i_nex = 0;// last_i_nex为上一个点的下一个点的索引
    int last_state = 0;//为1代表上个状态为平面 否则为0
    int plane_type;
    // 判断平面特征，使用八个点
    for (uint i = head; i < plsize2; i++) {
        if (types[i].range < blind) continue;
        i2 = i;// 记录当前点 //求得平面，并返回类型0 1 2
        plane_type = plane_judge(pl, types, i, i_nex, curr_direct);
        // 表明是一个平面
        if (plane_type == 1)//正常退出
        {
            // 可能是边界的点
            for (uint j = i; j <= i_nex; j++) {
                if (j != i && j != i_nex) {
                    types[j].ftype = Real_Plane;
                } else {
                    types[j].ftype = Poss_Plane;
                }
            }
            // if(last_state==1 && fabs(last_direct.sum())>0.5)
            //最开始last_state=0直接跳过
            //之后last_state=1
            //如果之前状态是平面则判断当前点是处于两平面边缘的点还是较为平坦的平面的点
            if (last_state == 1 && last_direct.norm() > 0.1) {
                //修改ftype，两个面法向量夹角在45度和135度之间 认为是两平面边缘上的点
                double mod = last_direct.transpose() * curr_direct;
                if (mod > -0.707 && mod < 0.707) {
                    types[i].ftype = Edge_Plane;
                } else {
                    types[i].ftype = Real_Plane;
                }
            }

            i = i_nex - 1;
            last_state = 1;
        } else // if(plane_type == 2)
        {
            i = i_nex;
            last_state = 0;//设置为不是平面点
        }
        // else if(plane_type == 0)
        // {
        //   if(last_state == 1)
        //   {
        //     uint i_nex_tem;
        //     uint j;
        //     for(j=last_i+1; j<=last_i_nex; j++)
        //     {
        //       uint i_nex_tem2 = i_nex_tem;
        //       Eigen::Vector3d curr_direct2;

        //       uint ttem = plane_judge(pl, types, j, i_nex_tem, curr_direct2);

        //       if(ttem != 1)
        //       {
        //         i_nex_tem = i_nex_tem2;
        //         break;
        //       }
        //       curr_direct = curr_direct2;
        //     }

        //     if(j == last_i+1)
        //     {
        //       last_state = 0;
        //     }
        //     else
        //     {
        //       for(uint k=last_i_nex; k<=i_nex_tem; k++)
        //       {
        //         if(k != i_nex_tem)
        //         {
        //           types[k].ftype = Real_Plane;
        //         }
        //         else
        //         {
        //           types[k].ftype = Poss_Plane;
        //         }
        //       }
        //       i = i_nex_tem-1;
        //       i_nex = i_nex_tem;
        //       i2 = j-1;
        //       last_state = 1;
        //     }

        //   }
        // }

        last_i = i2;
        last_i_nex = i_nex;
        last_direct = curr_direct;
    }
// 判断edge特征
//如果剩下的点数小于3则不判断边缘点，否则计算哪些点是边缘点
    plsize2 = plsize > 3 ? plsize - 3 : 0;
    for (uint i = head + 3; i < plsize2; i++) {
        //点不能在盲区 或者 点必须属于正常点和可能的平面点
        if (types[i].range < blind || types[i].ftype >= Real_Plane) {
            continue;
        }

        if (types[i - 1].dista < 1e-16 || types[i].dista < 1e-16) {
            continue;
        }

        Eigen::Vector3d vec_a(pl[i].x, pl[i].y, pl[i].z);//当前点组成的向量
        Eigen::Vector3d vecs[2];

        for (int j = 0; j < 2; j++) {
            int m = -1;
            if (j == 1) {
                m = 1;
            }
            //若当前的前/后一个点在盲区内（4m)
            if (types[i + m].range < blind) {
                if (types[i].range > inf_bound) { //若其大于10m
                    types[i].edj[j] = Nr_inf; //赋予该点Nr_inf(跳变较远)
                } else {
                    types[i].edj[j] = Nr_blind;//赋予该点Nr_blind(在盲区)
                }
                continue;
            }

            vecs[j] = Eigen::Vector3d(pl[i + m].x, pl[i + m].y, pl[i + m].z);
            vecs[j] = vecs[j] - vec_a;//前/后点指向当前点的向量
            //若雷达坐标系原点为O 当前点为A 前/后一点为M和N
            //则下面OA点乘MA/（|OA|*|MA|）
            //得到的是cos角OAM的大小

            types[i].angle[j] = vec_a.dot(vecs[j]) / vec_a.norm() / vecs[j].norm();
            if (types[i].angle[j] < jump_up_limit) {
                types[i].edj[j] = Nr_180;
            } else if (types[i].angle[j] > jump_down_limit) {
                types[i].edj[j] = Nr_zero;
            }
        }
        //角MAN的cos值
        types[i].intersect = vecs[Prev].dot(vecs[Next]) / vecs[Prev].norm() / vecs[Next].norm();

        if (types[i].edj[Prev] == Nr_nor && types[i].edj[Next] == Nr_zero && types[i].dista > 0.0225 &&
            types[i].dista > 4 * types[i - 1].dista) {
            if (types[i].intersect > cos160) {
                if (edge_jump_judge(pl, types, i, Prev)) {
                    types[i].ftype = Edge_Jump;
                }
            }
        } else if (types[i].edj[Prev] == Nr_zero && types[i].edj[Next] == Nr_nor && types[i - 1].dista > 0.0225 &&
                   types[i - 1].dista > 4 * types[i].dista) {
            if (types[i].intersect > cos160) {
                if (edge_jump_judge(pl, types, i, Next)) {
                    types[i].ftype = Edge_Jump;
                }
            }
        } else if (types[i].edj[Prev] == Nr_nor && types[i].edj[Next] == Nr_inf) {
            if (edge_jump_judge(pl, types, i, Prev)) {
                types[i].ftype = Edge_Jump;
            }
        } else if (types[i].edj[Prev] == Nr_inf && types[i].edj[Next] == Nr_nor) {
            if (edge_jump_judge(pl, types, i, Next)) {
                types[i].ftype = Edge_Jump;
            }

        } else if (types[i].edj[Prev] > Nr_nor && types[i].edj[Next] > Nr_nor) {
            if (types[i].ftype == Nor) {
                types[i].ftype = Wire;
            }
        }
    }

    plsize2 = plsize - 1;
    double ratio;
//  继续寻找平面点
    for (uint i = head + 1; i < plsize2; i++) {
        if (types[i].range < blind || types[i - 1].range < blind || types[i + 1].range < blind) {
            continue;
        }

        if (types[i - 1].dista < 1e-8 || types[i].dista < 1e-8) {
            continue;
        }

        if (types[i].ftype == Nor) {
            if (types[i - 1].dista > types[i].dista) {
                ratio = types[i - 1].dista / types[i].dista;
            } else {
                ratio = types[i].dista / types[i - 1].dista;
            }

            if (types[i].intersect < smallp_intersect && ratio < smallp_ratio) {
                if (types[i - 1].ftype == Nor) {
                    types[i - 1].ftype = Real_Plane;
                }
                if (types[i + 1].ftype == Nor) {
                    types[i + 1].ftype = Real_Plane;
                }
                types[i].ftype = Real_Plane;
            }
        }
    }

    int last_surface = -1;
    // 进行存储对应的点和角点等特征
    for (uint j = head; j < plsize; j++) {
        if (types[j].ftype == Poss_Plane || types[j].ftype == Real_Plane) {
            if (last_surface == -1) {
                last_surface = j;
            }

            if (j == uint(last_surface + point_filter_num - 1)) {
                PointType ap;
                ap.x = pl[j].x;
                ap.y = pl[j].y;
                ap.z = pl[j].z;
                ap.intensity = pl[j].intensity;
                ap.curvature = pl[j].curvature;
                pl_surf.push_back(ap);

                last_surface = -1;
            }
        } else {
            if (types[j].ftype == Edge_Jump || types[j].ftype == Edge_Plane) {
                pl_corn.push_back(pl[j]);
            }
            if (last_surface != -1) {
                PointType ap;
                for (uint k = last_surface; k < j; k++) {
                    ap.x += pl[k].x;
                    ap.y += pl[k].y;
                    ap.z += pl[k].z;
                    ap.intensity += pl[k].intensity;
                    ap.curvature += pl[k].curvature;
                }
                ap.x /= (j - last_surface);
                ap.y /= (j - last_surface);
                ap.z /= (j - last_surface);
                ap.intensity /= (j - last_surface);
                ap.curvature /= (j - last_surface);
                pl_surf.push_back(ap);
            }
            last_surface = -1;
        }
    }
}

void Preprocess::pub_func(PointCloudXYZI &pl, const ros::Time &ct) {
    pl.height = 1;
    pl.width = pl.size();
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(pl, output);
    output.header.frame_id = "livox";
    output.header.stamp = ct;
}

/**
 *
 * @param pl
 * @param types
 * @param i_cur 当前点
 * @param i_nex 最后一个点
 * @param curr_direct 归一化之后的局部范围最后一个点与第一个点的坐标差值，向量从I_cur指向i——nex
 * @return 1，0，分别对应正常退出和中途break，curr_direct置零
 */
int Preprocess::plane_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i_cur, uint &i_nex,
                            Eigen::Vector3d &curr_direct) {
    double group_dis = disA * types[i_cur].range + disB;
    group_dis = group_dis * group_dis;
    // i_nex = i_cur;

    double two_dis;
    vector<double> disarr;
    disarr.reserve(20);

    for (i_nex = i_cur; i_nex < i_cur + group_size; i_nex++) {
        if (types[i_nex].range < blind)// 如果存在小于range的点，就返回2法向量设置为0；
        {
            curr_direct.setZero();
            return 2;
        }
        disarr.push_back(types[i_nex].dista);// 保存最后一个点的距离
    }

    for (;;) {
        if ((i_cur >= pl.size()) || (i_nex >= pl.size())) break;

        if (types[i_nex].range < blind) {
            curr_direct.setZero();
            return 2;
        }
        vx = pl[i_nex].x - pl[i_cur].x;
        vy = pl[i_nex].y - pl[i_cur].y;
        vz = pl[i_nex].z - pl[i_cur].z;
        two_dis = vx * vx + vy * vy + vz * vz;
        if (two_dis >= group_dis) {
            break;
        }
        disarr.push_back(types[i_nex].dista);
        i_nex++;
    }

    double leng_wid = 0;// 局部范围内最大平行四边行的面积
    double v1[3], v2[3];
    for (uint j = i_cur + 1; j < i_nex; j++) {
        if ((j >= pl.size()) || (i_cur >= pl.size())) break;
        v1[0] = pl[j].x - pl[i_cur].x;
        v1[1] = pl[j].y - pl[i_cur].y;
        v1[2] = pl[j].z - pl[i_cur].z;

        v2[0] = v1[1] * vz - vy * v1[2];
        v2[1] = v1[2] * vx - v1[0] * vz;
        v2[2] = v1[0] * vy - vx * v1[1];

        double lw = v2[0] * v2[0] + v2[1] * v2[1] + v2[2] * v2[2];
        if (lw > leng_wid) {
            leng_wid = lw;
        }
    }


    if ((two_dis * two_dis / leng_wid) < p2l_ratio) {
        curr_direct.setZero();
        return 0;
    }

    uint disarrsize = disarr.size();
    for (uint j = 0; j < disarrsize - 1; j++) {
        for (uint k = j + 1; k < disarrsize; k++) {
            if (disarr[j] < disarr[k]) {
                leng_wid = disarr[j];
                disarr[j] = disarr[k];
                disarr[k] = leng_wid;
            }
        }
    }

    if (disarr[disarr.size() - 2] < 1e-16) {
        curr_direct.setZero();
        return 0;
    }

    if (lidar_type == AVIA) {
        double dismax_mid = disarr[0] / disarr[disarrsize / 2];
        double dismid_min = disarr[disarrsize / 2] / disarr[disarrsize - 2];

        if (dismax_mid >= limit_maxmid || dismid_min >= limit_midmin) {
            curr_direct.setZero();
            return 0;
        }
    } else {
        double dismax_min = disarr[0] / disarr[disarrsize - 2];
        if (dismax_min >= limit_maxmin) {
            curr_direct.setZero();
            return 0;
        }
    }

    curr_direct << vx, vy, vz;
    curr_direct.normalize();
    return 1;
}

bool Preprocess::edge_jump_judge(const PointCloudXYZI &pl, vector<orgtype> &types, uint i, Surround nor_dir) {
    if (nor_dir == 0) {
        if (types[i - 1].range < blind || types[i - 2].range < blind) {
            return false;
        }
    } else if (nor_dir == 1) {
        if (types[i + 1].range < blind || types[i + 2].range < blind) {
            return false;
        }
    }
    double d1 = types[i + nor_dir - 1].dista;
    double d2 = types[i + 3 * nor_dir - 2].dista;
    double d;

    if (d1 < d2) {
        d = d1;
        d1 = d2;
        d2 = d;
    }

    d1 = sqrt(d1);
    d2 = sqrt(d2);


    if (d1 > edgea * d2 || (d1 - d2) > edgeb) {
        return false;
    }

    return true;
}
