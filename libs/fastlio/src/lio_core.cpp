#include <fastlio/lio_core.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <omp.h>

namespace fastlio
{

namespace
{
constexpr double kInitTime = 0.1;
constexpr double kLaserPointCov = 0.001;
constexpr float kMovThreshold = 1.5f;
}

LioCore *LioCore::active_instance_ = nullptr;

LioCore::LioCore(const LioConfig &config)
    : config_(config)
{
  active_instance_ = this;

  feats_undistort_.reset(new PointCloudXYZI());
  feats_down_body_.reset(new PointCloudXYZI());
  feats_down_world_.reset(new PointCloudXYZI());
  normvec_.reset(new PointCloudXYZI(100000, 1));
  laser_cloud_ori_.reset(new PointCloudXYZI(100000, 1));
  corr_normvect_.reset(new PointCloudXYZI(100000, 1));

  res_last_.assign(100000, -1000.0f);
  point_selected_surf_.assign(100000, true);

  down_size_filter_surf_.setLeafSize(config_.filter_size_surf_min, config_.filter_size_surf_min, config_.filter_size_surf_min);

  imu_.set_extrinsic(config_.extrinsic_T, config_.extrinsic_R);
  imu_.set_gyr_cov(V3D(config_.gyr_cov, config_.gyr_cov, config_.gyr_cov));
  imu_.set_acc_cov(V3D(config_.acc_cov, config_.acc_cov, config_.acc_cov));
  imu_.set_gyr_bias_cov(V3D(config_.b_gyr_cov, config_.b_gyr_cov, config_.b_gyr_cov));
  imu_.set_acc_bias_cov(V3D(config_.b_acc_cov, config_.b_acc_cov, config_.b_acc_cov));
  imu_.lidar_type = config_.lidar_type;

  double epsi[23];
  std::fill(epsi, epsi + 23, 0.001);
  kf_.init_dyn_share(get_f, df_dx, df_dw, &LioCore::hShareModel, config_.max_iterations, epsi);
}

void LioCore::pointBodyToWorld(PointType const * const pi, PointType * const po) const
{
  V3D p_body(pi->x, pi->y, pi->z);
  V3D p_global(state_point_.rot * (state_point_.offset_R_L_I * p_body + state_point_.offset_T_L_I) + state_point_.pos);

  po->x = p_global(0);
  po->y = p_global(1);
  po->z = p_global(2);
  po->intensity = pi->intensity;
}

void LioCore::lasermapFovSegment()
{
  cub_needrm_.clear();
  kdtree_delete_counter_ = 0;
  kdtree_delete_time_ = 0.0;
  V3D pos_LiD = pos_lid_;
  if (!local_map_initialized_)
  {
    for (int i = 0; i < 3; i++)
    {
      local_map_points_.vertex_min[i] = pos_LiD(i) - config_.cube_side_length / 2.0;
      local_map_points_.vertex_max[i] = pos_LiD(i) + config_.cube_side_length / 2.0;
    }
    local_map_initialized_ = true;
    return;
  }
  float dist_to_map_edge[3][2];
  bool need_move = false;
  for (int i = 0; i < 3; i++)
  {
    dist_to_map_edge[i][0] = fabs(pos_LiD(i) - local_map_points_.vertex_min[i]);
    dist_to_map_edge[i][1] = fabs(pos_LiD(i) - local_map_points_.vertex_max[i]);
    if (dist_to_map_edge[i][0] <= kMovThreshold * config_.det_range || dist_to_map_edge[i][1] <= kMovThreshold * config_.det_range) need_move = true;
  }
  if (!need_move) return;
  BoxPointType new_local_map_points = local_map_points_, tmp_boxpoints;
  float mov_dist = std::max((config_.cube_side_length - 2.0 * kMovThreshold * config_.det_range) * 0.5 * 0.9, double(config_.det_range * (kMovThreshold - 1)));
  for (int i = 0; i < 3; i++)
  {
    tmp_boxpoints = local_map_points_;
    if (dist_to_map_edge[i][0] <= kMovThreshold * config_.det_range)
    {
      new_local_map_points.vertex_max[i] -= mov_dist;
      new_local_map_points.vertex_min[i] -= mov_dist;
      tmp_boxpoints.vertex_min[i] = local_map_points_.vertex_max[i] - mov_dist;
      cub_needrm_.push_back(tmp_boxpoints);
    }
    else if (dist_to_map_edge[i][1] <= kMovThreshold * config_.det_range)
    {
      new_local_map_points.vertex_max[i] += mov_dist;
      new_local_map_points.vertex_min[i] += mov_dist;
      tmp_boxpoints.vertex_max[i] = local_map_points_.vertex_min[i] + mov_dist;
      cub_needrm_.push_back(tmp_boxpoints);
    }
  }
  local_map_points_ = new_local_map_points;

  PointVector points_history;
  ikdtree_.acquire_removed_points(points_history);

  double delete_begin = omp_get_wtime();
  if (!cub_needrm_.empty()) kdtree_delete_counter_ = ikdtree_.Delete_Point_Boxes(cub_needrm_);
  kdtree_delete_time_ = omp_get_wtime() - delete_begin;
}

void LioCore::mapIncremental()
{
  PointVector PointToAdd;
  PointVector PointNoNeedDownsample;
  PointToAdd.reserve(feats_down_size_);
  PointNoNeedDownsample.reserve(feats_down_size_);
  for (int i = 0; i < feats_down_size_; i++)
  {
    /* transform to world frame */
    pointBodyToWorld(&(feats_down_body_->points[i]), &(feats_down_world_->points[i]));
    /* decide if need add to map */
    if (!nearest_points_[i].empty() && flg_ekf_inited_)
    {
      const PointVector &points_near = nearest_points_[i];
      bool need_add = true;
      PointType mid_point;
      mid_point.x = floor(feats_down_world_->points[i].x / config_.filter_size_map_min) * config_.filter_size_map_min + 0.5 * config_.filter_size_map_min;
      mid_point.y = floor(feats_down_world_->points[i].y / config_.filter_size_map_min) * config_.filter_size_map_min + 0.5 * config_.filter_size_map_min;
      mid_point.z = floor(feats_down_world_->points[i].z / config_.filter_size_map_min) * config_.filter_size_map_min + 0.5 * config_.filter_size_map_min;
      float dist = calc_dist(feats_down_world_->points[i], mid_point);
      if (fabs(points_near[0].x - mid_point.x) > 0.5 * config_.filter_size_map_min && fabs(points_near[0].y - mid_point.y) > 0.5 * config_.filter_size_map_min && fabs(points_near[0].z - mid_point.z) > 0.5 * config_.filter_size_map_min)
      {
        PointNoNeedDownsample.push_back(feats_down_world_->points[i]);
        continue;
      }
      for (int readd_i = 0; readd_i < NUM_MATCH_POINTS; readd_i++)
      {
        if (points_near.size() < NUM_MATCH_POINTS) break;
        if (calc_dist(points_near[readd_i], mid_point) < dist)
        {
          need_add = false;
          break;
        }
      }
      if (need_add) PointToAdd.push_back(feats_down_world_->points[i]);
    }
    else
    {
      PointToAdd.push_back(feats_down_world_->points[i]);
    }
  }

  double st_time = omp_get_wtime();
  add_point_size_ = ikdtree_.Add_Points(PointToAdd, true);
  ikdtree_.Add_Points(PointNoNeedDownsample, false);
  add_point_size_ = PointToAdd.size() + PointNoNeedDownsample.size();
  kdtree_incremental_time_ = omp_get_wtime() - st_time;
}

void LioCore::hShareModel(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
  active_instance_->hShareModelImpl(s, ekfom_data);
}

void LioCore::hShareModelImpl(state_ikfom &s, esekfom::dyn_share_datastruct<double> &ekfom_data)
{
  double match_start = omp_get_wtime();
  laser_cloud_ori_->clear();
  corr_normvect_->clear();
  total_residual_ = 0.0;

  /** closest surface search and residual computation **/
#ifdef MP_EN
  omp_set_num_threads(MP_PROC_NUM);
#pragma omp parallel for
#endif
  for (int i = 0; i < feats_down_size_; i++)
  {
    PointType &point_body = feats_down_body_->points[i];
    PointType &point_world = feats_down_world_->points[i];

    /* transform to world frame */
    V3D p_body(point_body.x, point_body.y, point_body.z);
    V3D p_global(s.rot * (s.offset_R_L_I * p_body + s.offset_T_L_I) + s.pos);
    point_world.x = p_global(0);
    point_world.y = p_global(1);
    point_world.z = p_global(2);
    point_world.intensity = point_body.intensity;

    vector<float> pointSearchSqDis(NUM_MATCH_POINTS);

    auto &points_near = nearest_points_[i];

    if (ekfom_data.converge)
    {
      /** Find the closest surfaces in the map **/
      ikdtree_.Nearest_Search(point_world, NUM_MATCH_POINTS, points_near, pointSearchSqDis);
      point_selected_surf_[i] = points_near.size() < NUM_MATCH_POINTS ? false : pointSearchSqDis[NUM_MATCH_POINTS - 1] > 5 ? false : true;
    }

    if (!point_selected_surf_[i]) continue;

    VF(4) pabcd;
    point_selected_surf_[i] = false;
    if (esti_plane(pabcd, points_near, 0.1f))
    {
      float pd2 = pabcd(0) * point_world.x + pabcd(1) * point_world.y + pabcd(2) * point_world.z + pabcd(3);
      float s_score = 1 - 0.9 * fabs(pd2) / sqrt(p_body.norm());

      if (s_score > 0.9)
      {
        point_selected_surf_[i] = true;
        normvec_->points[i].x = pabcd(0);
        normvec_->points[i].y = pabcd(1);
        normvec_->points[i].z = pabcd(2);
        normvec_->points[i].intensity = pd2;
        res_last_[i] = abs(pd2);
      }
    }
  }

  effect_feat_num_ = 0;

  for (int i = 0; i < feats_down_size_; i++)
  {
    if (point_selected_surf_[i])
    {
      laser_cloud_ori_->points[effect_feat_num_] = feats_down_body_->points[i];
      corr_normvect_->points[effect_feat_num_] = normvec_->points[i];
      total_residual_ += res_last_[i];
      effect_feat_num_++;
    }
  }

  if (effect_feat_num_ < 1)
  {
    ekfom_data.valid = false;
    std::cerr << "[fastlio] No Effective Points!" << std::endl;
    return;
  }

  res_mean_last_ = total_residual_ / effect_feat_num_;
  match_time_ += omp_get_wtime() - match_start;
  double solve_start_ = omp_get_wtime();

  /*** Computation of Measuremnt Jacobian matrix H and measurents vector ***/
  ekfom_data.h_x = MatrixXd::Zero(effect_feat_num_, 12);
  ekfom_data.h.resize(effect_feat_num_);

  for (int i = 0; i < effect_feat_num_; i++)
  {
    const PointType &laser_p = laser_cloud_ori_->points[i];
    V3D point_this_be(laser_p.x, laser_p.y, laser_p.z);
    M3D point_be_crossmat;
    point_be_crossmat << SKEW_SYM_MATRX(point_this_be);
    V3D point_this = s.offset_R_L_I * point_this_be + s.offset_T_L_I;
    M3D point_crossmat;
    point_crossmat << SKEW_SYM_MATRX(point_this);

    /*** get the normal vector of closest surface/corner ***/
    const PointType &norm_p = corr_normvect_->points[i];
    V3D norm_vec(norm_p.x, norm_p.y, norm_p.z);

    /*** calculate the Measuremnt Jacobian matrix H ***/
    V3D C(s.rot.conjugate() * norm_vec);
    V3D A(point_crossmat * C);
    if (config_.extrinsic_est_en)
    {
      V3D B(point_be_crossmat * s.offset_R_L_I.conjugate() * C);
      ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), VEC_FROM_ARRAY(B), VEC_FROM_ARRAY(C);
    }
    else
    {
      ekfom_data.h_x.block<1, 12>(i, 0) << norm_p.x, norm_p.y, norm_p.z, VEC_FROM_ARRAY(A), 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    }

    /*** Measuremnt: distance to the closest surface/corner ***/
    ekfom_data.h(i) = -norm_p.intensity;
  }
  solve_time_ += omp_get_wtime() - solve_start_;
}

LioCore::FrameResult LioCore::processFrame(const MeasureGroup &meas)
{
  FrameResult result;
  result.time = meas.lidar_end_time;

  if (first_scan_)
  {
    first_lidar_time_ = meas.lidar_beg_time;
    imu_.first_lidar_time = first_lidar_time_;
    first_scan_ = false;
    return result;
  }

  match_time_ = 0;
  solve_time_ = 0;

  imu_.Process(meas, kf_, feats_undistort_);
  state_point_ = kf_.get_x();
  pos_lid_ = state_point_.pos + state_point_.rot * state_point_.offset_T_L_I;

  if (feats_undistort_->empty() || feats_undistort_ == nullptr)
  {
    return result; // "No point, skip this scan!"
  }

  flg_ekf_inited_ = (meas.lidar_beg_time - first_lidar_time_) >= kInitTime;
  lasermapFovSegment();

  down_size_filter_surf_.setInputCloud(feats_undistort_);
  down_size_filter_surf_.filter(*feats_down_body_);
  feats_down_size_ = feats_down_body_->points.size();

  if (ikdtree_.Root_Node == nullptr)
  {
    if (feats_down_size_ > 5)
    {
      ikdtree_.set_downsample_param(config_.filter_size_map_min);
      feats_down_world_->resize(feats_down_size_);
      for (int i = 0; i < feats_down_size_; i++)
      {
        pointBodyToWorld(&(feats_down_body_->points[i]), &(feats_down_world_->points[i]));
      }
      ikdtree_.Build(feats_down_world_->points);
    }
    return result;
  }

  result.kdtree_size_st = ikdtree_.size();

  if (feats_down_size_ < 5)
  {
    return result; // "No point, skip this scan!"
  }

  normvec_->resize(feats_down_size_);
  feats_down_world_->resize(feats_down_size_);
  nearest_points_.resize(feats_down_size_);

  double solve_h_time = 0;
  kf_.update_iterated_dyn_share_modified(kLaserPointCov, solve_h_time);
  state_point_ = kf_.get_x();
  pos_lid_ = state_point_.pos + state_point_.rot * state_point_.offset_T_L_I;

  mapIncremental();

  result.ok = true;
  result.state = state_point_;
  result.angvel = imu_.angvel();
  result.P = kf_.get_P();
  result.body_undistorted = feats_undistort_;
  result.down_body = feats_down_body_;
  result.down_world = feats_down_world_;
  result.effect_feat_num = effect_feat_num_;
  result.match_time = match_time_;
  result.solve_time = solve_time_ + solve_h_time;
  result.solve_h_time = solve_h_time;
  result.kdtree_incremental_time = kdtree_incremental_time_;
  result.kdtree_search_time = 0.0; // ikd-Tree does not report search time separately from match_time
  result.kdtree_delete_time = kdtree_delete_time_;
  result.kdtree_delete_counter = kdtree_delete_counter_;
  result.add_point_size = add_point_size_;
  result.kdtree_size_end = ikdtree_.size();
  return result;
}

} // namespace fastlio
