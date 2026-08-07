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
//
// This file is the ROS1 glue layer: it converts ROS messages/params to/from
// the plain types the ROS-free fastlio library (libs/fastlio) operates on.
// The actual ESIKF/mapping algorithm lives in fastlio::LioCore; buffering
// and lidar/IMU synchronization lives in fastlio::PacketSync.
#include <mutex>
#include <csignal>
#include <cstdio>
#include <condition_variable>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <livox_ros_driver/CustomMsg.h>

#include <fastlio/lio_core.h>
#include <fastlio/packet_sync.h>
#include <fastlio/preprocess.h>
#include <fastlio/use-ikfom.hpp>

using fastlio::LioConfig;
using fastlio::LioCore;
using fastlio::ImuSample;
using fastlio::LivoxScan;

mutex mtx_buffer;
condition_variable sig_buffer;

string root_dir = ROOT_DIR;
string lid_topic, imu_topic;

bool path_en = true, scan_pub_en = false, dense_pub_en = false, scan_body_pub_en = false;
bool pcd_save_en = false, time_sync_en = false, runtime_pos_log = false;
double time_diff_lidar_to_imu = 0.0;
int pcd_save_interval = -1, pcd_index = 0;
bool flg_exit = false;

// Livox-specific IMU/lidar self-sync (see livox_pcl_cbk below).
double timediff_lidar_wrt_imu = 0.0;
bool timediff_set_flg = false;

fastlio::PacketSync sync_buf;
shared_ptr<Preprocess> p_pre(new Preprocess());
shared_ptr<LioCore> lio;

nav_msgs::Path path;
geometry_msgs::PoseStamped msg_body_pose;

PointCloudXYZI::Ptr pcl_wait_save(new PointCloudXYZI());

void SigHandle(int sig)
{
    flg_exit = true;
    ROS_WARN("catch sig %d", sig);
    sig_buffer.notify_all();
}

void standard_pcl_cbk(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    mtx_buffer.lock();
    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());

    switch (p_pre->lidar_type)
    {
        case OUST64:
        {
            pcl::PointCloud<ouster_ros::Point> cloud;
            pcl::fromROSMsg(*msg, cloud);
            p_pre->process(cloud, ptr);
            break;
        }
        case VELO16:
        {
            pcl::PointCloud<velodyne_ros::Point> cloud;
            pcl::fromROSMsg(*msg, cloud);
            p_pre->process(cloud, ptr);
            break;
        }
        case MARSIM:
        {
            pcl::PointCloud<pcl::PointXYZI> cloud;
            pcl::fromROSMsg(*msg, cloud);
            p_pre->process(cloud, ptr);
            break;
        }
        default:
            ROS_ERROR("Unsupported preprocess/lidar_type %d for a sensor_msgs/PointCloud2 topic", p_pre->lidar_type);
            mtx_buffer.unlock();
            return;
    }

    if (!sync_buf.pushLidar(ptr, msg->header.stamp.toSec()))
    {
        ROS_ERROR("lidar loop back, clear buffer");
    }
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void livox_pcl_cbk(const livox_ros_driver::CustomMsg::ConstPtr &msg)
{
    mtx_buffer.lock();

    double last_timestamp_imu = sync_buf.lastImuTime();
    double last_timestamp_lidar = sync_buf.lastLidarTime();
    double this_lidar_time = msg->header.stamp.toSec();

    if (!time_sync_en && abs(last_timestamp_imu - this_lidar_time) > 10.0 && last_timestamp_imu > 0 && last_timestamp_lidar > 0)
    {
        printf("IMU and LiDAR not Synced, IMU time: %lf, lidar header time: %lf \n", last_timestamp_imu, this_lidar_time);
    }

    if (time_sync_en && !timediff_set_flg && abs(this_lidar_time - last_timestamp_imu) > 1 && last_timestamp_imu > 0)
    {
        timediff_set_flg = true;
        timediff_lidar_wrt_imu = this_lidar_time + 0.1 - last_timestamp_imu;
        printf("Self sync IMU and LiDAR, time diff is %.10lf \n", timediff_lidar_wrt_imu);
    }

    LivoxScan scan;
    scan.timestamp = this_lidar_time;
    scan.points.resize(msg->point_num);
    for (uint i = 0; i < msg->point_num; i++)
    {
        scan.points[i].x = msg->points[i].x;
        scan.points[i].y = msg->points[i].y;
        scan.points[i].z = msg->points[i].z;
        scan.points[i].reflectivity = msg->points[i].reflectivity;
        scan.points[i].tag = msg->points[i].tag;
        scan.points[i].line = msg->points[i].line;
        scan.points[i].offset_time = msg->points[i].offset_time;
    }

    PointCloudXYZI::Ptr ptr(new PointCloudXYZI());
    p_pre->process(scan, ptr);

    if (!sync_buf.pushLidar(ptr, this_lidar_time))
    {
        ROS_ERROR("lidar loop back, clear buffer");
    }

    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void imu_cbk(const sensor_msgs::Imu::ConstPtr &msg_in)
{
    double stamp = msg_in->header.stamp.toSec() - time_diff_lidar_to_imu;
    if (abs(timediff_lidar_wrt_imu) > 0.1 && time_sync_en)
    {
        stamp = timediff_lidar_wrt_imu + msg_in->header.stamp.toSec();
    }

    ImuSample sample;
    sample.timestamp = stamp;
    sample.acc = V3D(msg_in->linear_acceleration.x, msg_in->linear_acceleration.y, msg_in->linear_acceleration.z);
    sample.gyro = V3D(msg_in->angular_velocity.x, msg_in->angular_velocity.y, msg_in->angular_velocity.z);

    mtx_buffer.lock();
    if (!sync_buf.pushImu(sample))
    {
        ROS_WARN("imu loop back, clear buffer");
    }
    mtx_buffer.unlock();
    sig_buffer.notify_all();
}

void pointBodyToWorld(const state_ikfom &s, PointType const * const pi, PointType * const po)
{
    V3D p_body(pi->x, pi->y, pi->z);
    V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);

    po->x = p_global(0);
    po->y = p_global(1);
    po->z = p_global(2);
    po->intensity = pi->intensity;
}

void pointBodyLidarToIMU(const state_ikfom &s, PointType const * const pi, PointType * const po)
{
    V3D p_body_lidar(pi->x, pi->y, pi->z);
    V3D p_body_imu(s.offset_R_L_I * p_body_lidar + s.offset_T_L_I);

    po->x = p_body_imu(0);
    po->y = p_body_imu(1);
    po->z = p_body_imu(2);
    po->intensity = pi->intensity;
}

void publish_frame_world(const ros::Publisher &pubLaserCloudFull, const LioCore::FrameResult &res)
{
    if (scan_pub_en)
    {
        PointCloudXYZI::Ptr laserCloudFullRes(dense_pub_en ? res.body_undistorted : res.down_body);
        int size = laserCloudFullRes->points.size();
        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            pointBodyToWorld(res.state, &laserCloudFullRes->points[i], &laserCloudWorld->points[i]);
        }

        sensor_msgs::PointCloud2 laserCloudmsg;
        pcl::toROSMsg(*laserCloudWorld, laserCloudmsg);
        laserCloudmsg.header.stamp = ros::Time().fromSec(res.time);
        laserCloudmsg.header.frame_id = "camera_init";
        pubLaserCloudFull.publish(laserCloudmsg);
    }

    /**************** save map ****************/
    /* 1. make sure you have enough memories
    /* 2. noted that pcd save will influence the real-time performences **/
    if (pcd_save_en)
    {
        int size = res.body_undistorted->points.size();
        PointCloudXYZI::Ptr laserCloudWorld(new PointCloudXYZI(size, 1));

        for (int i = 0; i < size; i++)
        {
            pointBodyToWorld(res.state, &res.body_undistorted->points[i], &laserCloudWorld->points[i]);
        }
        *pcl_wait_save += *laserCloudWorld;

        static int scan_wait_num = 0;
        scan_wait_num++;
        if (pcl_wait_save->size() > 0 && pcd_save_interval > 0 && scan_wait_num >= pcd_save_interval)
        {
            pcd_index++;
            string all_points_dir(string(string(ROOT_DIR) + "PCD/scans_") + to_string(pcd_index) + string(".pcd"));
            pcl::PCDWriter pcd_writer;
            cout << "current scan saved to /PCD/" << all_points_dir << endl;
            pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
            pcl_wait_save->clear();
            scan_wait_num = 0;
        }
    }
}

void publish_frame_body(const ros::Publisher &pubLaserCloudFull_body, const LioCore::FrameResult &res)
{
    int size = res.body_undistorted->points.size();
    PointCloudXYZI::Ptr laserCloudIMUBody(new PointCloudXYZI(size, 1));

    for (int i = 0; i < size; i++)
    {
        pointBodyLidarToIMU(res.state, &res.body_undistorted->points[i], &laserCloudIMUBody->points[i]);
    }

    sensor_msgs::PointCloud2 laserCloudmsg;
    pcl::toROSMsg(*laserCloudIMUBody, laserCloudmsg);
    laserCloudmsg.header.stamp = ros::Time().fromSec(res.time);
    laserCloudmsg.header.frame_id = "body";
    pubLaserCloudFull_body.publish(laserCloudmsg);
}

void publish_odometry(const ros::Publisher &pubOdomAftMapped, const LioCore::FrameResult &res)
{
    nav_msgs::Odometry odomAftMapped;
    odomAftMapped.header.frame_id = "camera_init";
    odomAftMapped.child_frame_id = "body";
    odomAftMapped.header.stamp = ros::Time().fromSec(res.time);
    odomAftMapped.pose.pose.position.x = res.state.pos(0);
    odomAftMapped.pose.pose.position.y = res.state.pos(1);
    odomAftMapped.pose.pose.position.z = res.state.pos(2);
    odomAftMapped.pose.pose.orientation.x = res.state.rot.coeffs()[0];
    odomAftMapped.pose.pose.orientation.y = res.state.rot.coeffs()[1];
    odomAftMapped.pose.pose.orientation.z = res.state.rot.coeffs()[2];
    odomAftMapped.pose.pose.orientation.w = res.state.rot.coeffs()[3];
    for (int i = 0; i < 6; i++)
    {
        int k = i < 3 ? i + 3 : i - 3;
        odomAftMapped.pose.covariance[i * 6 + 0] = res.P(k, 3);
        odomAftMapped.pose.covariance[i * 6 + 1] = res.P(k, 4);
        odomAftMapped.pose.covariance[i * 6 + 2] = res.P(k, 5);
        odomAftMapped.pose.covariance[i * 6 + 3] = res.P(k, 0);
        odomAftMapped.pose.covariance[i * 6 + 4] = res.P(k, 1);
        odomAftMapped.pose.covariance[i * 6 + 5] = res.P(k, 2);
    }
    pubOdomAftMapped.publish(odomAftMapped);

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(odomAftMapped.pose.pose.position.x,
                                     odomAftMapped.pose.pose.position.y,
                                     odomAftMapped.pose.pose.position.z));
    q.setW(odomAftMapped.pose.pose.orientation.w);
    q.setX(odomAftMapped.pose.pose.orientation.x);
    q.setY(odomAftMapped.pose.pose.orientation.y);
    q.setZ(odomAftMapped.pose.pose.orientation.z);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, odomAftMapped.header.stamp, "camera_init", "body"));
}

void publish_path(const ros::Publisher &pubPath, const LioCore::FrameResult &res)
{
    msg_body_pose.pose.position.x = res.state.pos(0);
    msg_body_pose.pose.position.y = res.state.pos(1);
    msg_body_pose.pose.position.z = res.state.pos(2);
    msg_body_pose.pose.orientation.x = res.state.rot.coeffs()[0];
    msg_body_pose.pose.orientation.y = res.state.rot.coeffs()[1];
    msg_body_pose.pose.orientation.z = res.state.rot.coeffs()[2];
    msg_body_pose.pose.orientation.w = res.state.rot.coeffs()[3];
    msg_body_pose.header.stamp = ros::Time().fromSec(res.time);
    msg_body_pose.header.frame_id = "camera_init";

    /*** if path is too large, the rvis will crash ***/
    static int jjj = 0;
    jjj++;
    if (jjj % 10 == 0)
    {
        path.poses.push_back(msg_body_pose);
        pubPath.publish(path);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laserMapping");
    ros::NodeHandle nh;

    LioConfig cfg;
    int lidar_type = AVIA;
    vector<double> extrinT(3, 0.0), extrinR(9, 0.0);

    nh.param<bool>("publish/path_en", path_en, true);
    nh.param<bool>("publish/scan_publish_en", scan_pub_en, true);
    nh.param<bool>("publish/dense_publish_en", dense_pub_en, true);
    nh.param<bool>("publish/scan_bodyframe_pub_en", scan_body_pub_en, true);
    nh.param<int>("max_iteration", cfg.max_iterations, 4);
    nh.param<string>("common/lid_topic", lid_topic, "/livox/lidar");
    nh.param<string>("common/imu_topic", imu_topic, "/livox/imu");
    nh.param<bool>("common/time_sync_en", time_sync_en, false);
    nh.param<double>("common/time_offset_lidar_to_imu", time_diff_lidar_to_imu, 0.0);
    nh.param<double>("filter_size_surf", cfg.filter_size_surf_min, 0.5);
    nh.param<double>("filter_size_map", cfg.filter_size_map_min, 0.5);
    nh.param<double>("cube_side_length", cfg.cube_side_length, 200);
    nh.param<float>("mapping/det_range", cfg.det_range, 300.f);
    nh.param<double>("mapping/gyr_cov", cfg.gyr_cov, 0.1);
    nh.param<double>("mapping/acc_cov", cfg.acc_cov, 0.1);
    nh.param<double>("mapping/b_gyr_cov", cfg.b_gyr_cov, 0.0001);
    nh.param<double>("mapping/b_acc_cov", cfg.b_acc_cov, 0.0001);
    nh.param<double>("preprocess/blind", p_pre->blind, 0.01);
    nh.param<int>("preprocess/lidar_type", lidar_type, AVIA);
    nh.param<int>("preprocess/scan_line", p_pre->N_SCANS, 16);
    nh.param<int>("preprocess/timestamp_unit", p_pre->time_unit, US);
    nh.param<int>("preprocess/scan_rate", p_pre->SCAN_RATE, 10);
    nh.param<int>("point_filter_num", p_pre->point_filter_num, 2);
    nh.param<bool>("feature_extract_enable", p_pre->feature_enabled, false);
    nh.param<bool>("runtime_pos_log_enable", runtime_pos_log, false);
    nh.param<bool>("mapping/extrinsic_est_en", cfg.extrinsic_est_en, true);
    nh.param<bool>("pcd_save/pcd_save_en", pcd_save_en, false);
    nh.param<int>("pcd_save/interval", pcd_save_interval, -1);
    nh.param<vector<double>>("mapping/extrinsic_T", extrinT, vector<double>());
    nh.param<vector<double>>("mapping/extrinsic_R", extrinR, vector<double>());

    cfg.lidar_type = lidar_type;
    cfg.extrinsic_T << VEC_FROM_ARRAY(extrinT);
    cfg.extrinsic_R << MAT_FROM_ARRAY(extrinR);

    p_pre->lidar_type = lidar_type;
    cout << "p_pre->lidar_type " << p_pre->lidar_type << endl;

    sync_buf.setLidarType(lidar_type);

    path.header.stamp = ros::Time::now();
    path.header.frame_id = "camera_init";

    // LioCore embeds a KD_TREE (its Rebuild_Logger alone is a fixed
    // ~1e6-entry buffer) and must be heap-allocated, not stack/value.
    lio.reset(new LioCore(cfg));

    /*** debug record ***/
    FILE *fp = nullptr;
    if (runtime_pos_log)
    {
        string pos_log_dir = root_dir + "/Log/pos_log.txt";
        fp = fopen(pos_log_dir.c_str(), "w");
    }

    /*** ROS subscribe initialization ***/
    ros::Subscriber sub_pcl = p_pre->lidar_type == AVIA ?
        nh.subscribe(lid_topic, 200000, livox_pcl_cbk) :
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
    ros::Publisher pubPath          = nh.advertise<nav_msgs::Path>
            ("/path", 100000);
//------------------------------------------------------------------------------------------------------
    signal(SIGINT, SigHandle);
    ros::Rate rate(5000);
    bool status = ros::ok();
    double first_lidar_time = -1.0;
    while (status)
    {
        if (flg_exit) break;
        ros::spinOnce();

        MeasureGroup Measures;
        if (sync_buf.nextMeasurement(Measures))
        {
            if (first_lidar_time < 0) first_lidar_time = Measures.lidar_beg_time;

            LioCore::FrameResult res = lio->processFrame(Measures);
            if (!res.ok)
            {
                continue; // first scan consumed / map bootstrap / too few points this scan
            }

            /******* Publish odometry *******/
            publish_odometry(pubOdomAftMapped, res);

            /******* Publish points *******/
            if (path_en)                         publish_path(pubPath, res);
            if (scan_pub_en || pcd_save_en)      publish_frame_world(pubLaserCloudFull, res);
            if (scan_pub_en && scan_body_pub_en) publish_frame_body(pubLaserCloudFull_body, res);

            /*** Debug variables ***/
            if (runtime_pos_log)
            {
                V3D rot_ang(Log(res.state.rot.toRotationMatrix()));
                if (fp)
                {
                    fprintf(fp, "%lf ", Measures.lidar_beg_time - first_lidar_time);
                    fprintf(fp, "%lf %lf %lf ", rot_ang(0), rot_ang(1), rot_ang(2));                       // Angle
                    fprintf(fp, "%lf %lf %lf ", res.state.pos(0), res.state.pos(1), res.state.pos(2));     // Pos
                    fprintf(fp, "%lf %lf %lf ", res.state.vel(0), res.state.vel(1), res.state.vel(2));     // Vel
                    fprintf(fp, "%lf %lf %lf ", res.state.bg(0), res.state.bg(1), res.state.bg(2));        // Bias_g
                    fprintf(fp, "%lf %lf %lf ", res.state.ba(0), res.state.ba(1), res.state.ba(2));        // Bias_a
                    fprintf(fp, "%lf %lf %lf ", res.state.grav[0], res.state.grav[1], res.state.grav[2]);  // Gravity
                    fprintf(fp, "\r\n");
                    fflush(fp);
                }
                printf("[ mapping ]: time: total: %0.6f match: %0.6f solve: %0.6f incre: %0.6f eff pts: %d\n",
                       res.match_time + res.solve_time + res.kdtree_incremental_time,
                       res.match_time, res.solve_time, res.kdtree_incremental_time, res.effect_feat_num);
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
        cout << "current scan saved to /PCD/" << file_name << endl;
        pcd_writer.writeBinary(all_points_dir, *pcl_wait_save);
    }

    if (fp) fclose(fp);

    return 0;
}
