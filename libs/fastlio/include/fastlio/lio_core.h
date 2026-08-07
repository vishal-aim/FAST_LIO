#ifndef FASTLIO_LIO_CORE_H
#define FASTLIO_LIO_CORE_H

#include <vector>
#include <fastlio/common_lib.h>
#include <fastlio/imu_process.h>
#include <fastlio/use-ikfom.hpp>
#include <ikd-Tree/ikd_Tree.h>
#include <pcl/filters/voxel_grid.h>

namespace fastlio
{

// The ~20 fields previously loaded via nh.param<...>("mapping/...", ...) /
// nh.param<...>("common/...", ...) in laserMapping.cpp's main(). Preprocess
// (feature extraction) config is intentionally not included here -- that's a
// separate object the caller owns and feeds into LioCore via MeasureGroup.
struct LioConfig
{
  int max_iterations = 4;
  int lidar_type = 1; // see preprocess.h::LID_TYPE (AVIA=1, VELO16, OUST64, MARSIM)
  double filter_size_surf_min = 0.5;
  double filter_size_map_min = 0.5;
  double cube_side_length = 200.0;
  float det_range = 300.f;
  double gyr_cov = 0.1;
  double acc_cov = 0.1;
  double b_gyr_cov = 0.0001;
  double b_acc_cov = 0.0001;
  bool extrinsic_est_en = true;
  V3D extrinsic_T = V3D::Zero();
  M3D extrinsic_R = M3D::Identity();
};

// The ESIKF state-estimation + incremental-map core of FAST-LIO, extracted
// from laserMapping.cpp's globals/while-loop body. Contains no ROS types --
// callers (the ROS1 node, the standalone MCAP tool) feed it MeasureGroups
// built via PacketSync and read back whatever they need to publish/visualize
// from FrameResult.
//
// Must be heap-allocated (e.g. std::unique_ptr<LioCore>), not a stack/value
// member: it embeds a KD_TREE, whose Rebuild_Logger holds a fixed ~1e6-entry
// buffer (tens of MB). The original code got away with a plain value because
// `ikdtree` was a global there; LioCore has no such luxury as a local.
class LioCore
{
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  explicit LioCore(const LioConfig &config);

  struct FrameResult
  {
    // false on every early-return path (first scan consumed just to record
    // first_lidar_time, initial map bootstrap, or too few points this scan)
    // -- mirrors the `continue` statements in the original main() loop.
    bool ok = false;
    double time = 0.0; // MeasureGroup::lidar_end_time
    state_ikfom state;
    esekfom::esekf<state_ikfom, 12, input_ikfom>::cov P; // kf.get_P()
    PointCloudXYZI::Ptr body_undistorted;  // feats_undistort, body frame
    PointCloudXYZI::Ptr down_body;         // feats_down_body, body frame
    PointCloudXYZI::Ptr down_world;        // feats_down_world, world frame
    int effect_feat_num = 0;

    // Timing/diagnostics, only meaningful when ok -- exposed so a caller can
    // reproduce the original runtime_pos_log_enable CSV dump if it wants to.
    double match_time = 0, solve_time = 0, solve_h_time = 0;
    double kdtree_incremental_time = 0, kdtree_search_time = 0, kdtree_delete_time = 0;
    int kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0, kdtree_delete_counter = 0;
  };

  FrameResult processFrame(const MeasureGroup &meas);

  bool mapInitialized() const { return ikdtree_.Root_Node != nullptr; }

 private:
  void lasermapFovSegment();
  void mapIncremental();
  void pointBodyToWorld(PointType const *pi, PointType *po) const;

  void hShareModelImpl(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data);
  static void hShareModel(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data);

  LioConfig config_;
  ImuProcess imu_;
  esekfom::esekf<state_ikfom, 12, input_ikfom> kf_;
  KD_TREE<PointType> ikdtree_;
  pcl::VoxelGrid<PointType> down_size_filter_surf_;

  state_ikfom state_point_;
  vect3 pos_lid_;

  BoxPointType local_map_points_;
  bool local_map_initialized_ = false;
  bool flg_ekf_inited_ = false;
  bool first_scan_ = true;
  double first_lidar_time_ = 0.0;

  int feats_down_size_ = 0;
  int effect_feat_num_ = 0;
  double res_mean_last_ = 0.05;
  double total_residual_ = 0.0;

  std::vector<float> res_last_;
  std::vector<bool> point_selected_surf_;

  std::vector<BoxPointType> cub_needrm_;
  std::vector<PointVector> nearest_points_;

  PointCloudXYZI::Ptr feats_undistort_;
  PointCloudXYZI::Ptr feats_down_body_;
  PointCloudXYZI::Ptr feats_down_world_;
  PointCloudXYZI::Ptr normvec_;
  PointCloudXYZI::Ptr laser_cloud_ori_;
  PointCloudXYZI::Ptr corr_normvect_;

  double match_time_ = 0, solve_time_ = 0;
  double kdtree_incremental_time_ = 0, kdtree_delete_time_ = 0;
  int kdtree_delete_counter_ = 0, add_point_size_ = 0;

  static LioCore *active_instance_;
};

} // namespace fastlio

#endif
