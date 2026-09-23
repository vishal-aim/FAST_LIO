#ifndef FASTLIO_TYPES_H
#define FASTLIO_TYPES_H

#include <cstdint>
#include <vector>
#include <Eigen/Eigen>

namespace fastlio
{

// A single IMU measurement, decoupled from sensor_msgs::Imu.
struct ImuSample
{
    double timestamp = 0.0;
    Eigen::Vector3d acc = Eigen::Vector3d::Zero();
    Eigen::Vector3d gyro = Eigen::Vector3d::Zero();
};

// Preintegrated lidar state at the time of an IMU measurement.
// Replaces the generated fast_lio::Pose6D ROS message, which was never
// published on a topic -- only used internally for IMU-integration bookkeeping.
struct Pose6D
{
    double offset_time = 0.0; // offset time of the IMU measurement w.r.t. the first lidar point
    double acc[3] = {0, 0, 0};  // preintegrated total acceleration (global frame) at the lidar origin
    double gyr[3] = {0, 0, 0};  // unbiased angular velocity (body frame) at the lidar origin
    double vel[3] = {0, 0, 0};  // preintegrated velocity (global frame) at the lidar origin
    double pos[3] = {0, 0, 0};  // preintegrated position (global frame) at the lidar origin
    double rot[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0}; // preintegrated rotation (global frame) at the lidar origin
};

// A single Livox point, decoupled from livox_ros_driver::CustomMsg::points[i].
struct LivoxPoint
{
    float x = 0, y = 0, z = 0;
    float reflectivity = 0;
    uint8_t tag = 0;
    uint8_t line = 0;
    uint32_t offset_time = 0; // ns, offset from the scan's header stamp
};

// A full Livox custom scan, decoupled from livox_ros_driver::CustomMsg.
struct LivoxScan
{
    double timestamp = 0.0; // header stamp, seconds
    std::vector<LivoxPoint> points;
};

} // namespace fastlio

#endif
