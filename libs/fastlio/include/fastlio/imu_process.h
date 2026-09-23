#ifndef FASTLIO_IMU_PROCESS_H
#define FASTLIO_IMU_PROCESS_H

#include <cmath>
#include <deque>
#include <fastlio/so3_math.h>
#include <Eigen/Eigen>
#include <fastlio/common_lib.h>
#include <fastlio/types.h>
#include <fastlio/use-ikfom.hpp>

/// *************IMU Process and undistortion
class ImuProcess
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ImuProcess();
  ~ImuProcess();

  void Reset();
  void Reset(double start_timestamp, const fastlio::ImuSample &lastimu);
  void set_extrinsic(const V3D &transl, const M3D &rot);
  void set_extrinsic(const V3D &transl);
  void set_extrinsic(const MD(4,4) &T);
  void set_gyr_cov(const V3D &scaler);
  void set_acc_cov(const V3D &scaler);
  void set_gyr_bias_cov(const V3D &b_g);
  void set_acc_bias_cov(const V3D &b_a);
  Eigen::Matrix<double, 12, 12> Q;
  void Process(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI::Ptr pcl_un_);

  // Bias-corrected angular velocity at the last IMU sample folded into the
  // most recent Process() call (i.e. ~lidar_end_time) -- set inside
  // UndistortPcl's forward integration, not a filter state itself.
  const V3D &angvel() const { return angvel_last; }

  V3D cov_acc;
  V3D cov_gyr;
  V3D cov_acc_scale;
  V3D cov_gyr_scale;
  V3D cov_bias_gyr;
  V3D cov_bias_acc;
  double first_lidar_time;
  int lidar_type;

 private:
  void IMU_init(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, int &N);
  void UndistortPcl(const MeasureGroup &meas, esekfom::esekf<state_ikfom, 12, input_ikfom> &kf_state, PointCloudXYZI &pcl_in_out);

  PointCloudXYZI::Ptr cur_pcl_un_;
  fastlio::ImuSample last_imu_;
  std::deque<fastlio::ImuSample> v_imu_;
  std::vector<Pose6D> IMUpose;
  std::vector<M3D> v_rot_pcl_;
  M3D Lidar_R_wrt_IMU;
  V3D Lidar_T_wrt_IMU;
  V3D mean_acc;
  V3D mean_gyr;
  V3D angvel_last;
  V3D acc_s_last;
  double start_timestamp_;
  double last_lidar_end_time_;
  int    init_iter_num = 1;
  bool   b_first_frame_ = true;
  bool   imu_need_init_ = true;
};

#endif
