/******************************************************************************
 * Copyright 2018 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/
#include "modules/perception/fusion/lib/data_fusion/motion_fusion/kalman_motion_fusion/kalman_motion_fusion.h"

#include <algorithm>
#include <iomanip>
#include <limits>

#include "modules/perception/base/sensor_meta.h"
#include "modules/perception/common/geometry/basic.h"
#include "modules/perception/common/sensor_manager/sensor_manager.h"

namespace {
constexpr double kDoubleEpsilon = std::numeric_limits<double>::epsilon();
}

namespace apollo {
namespace perception {
namespace fusion {

int KalmanMotionFusion::s_eval_window_ = 3;
size_t KalmanMotionFusion::s_history_size_maximum_ = 20;

bool KalmanMotionFusion::Init() {
  if (track_ref_ == nullptr) {
    return false;
  }
  // 对这个航迹进行初始化
  if (track_ref_->GetLatestLidarObject() != nullptr) {
    filter_init_ = InitFilter(track_ref_->GetLatestLidarObject()); // 得到最近的Lidar障碍物
  } else if (track_ref_->GetLatestRadarObject() != nullptr) {
    filter_init_ = InitFilter(track_ref_->GetLatestRadarObject());// 如果没有lidar，得到最近的Radar障碍物
  }
  return true;
}

// 输入为最新lidar/radar的历史观测  track_ref_->GetLatestLidarObject()==========
bool KalmanMotionFusion::InitFilter(const SensorObjectConstPtr& sensor_object) {
  // 加速度故障
  const std::vector<bool> gain_break_down = {0, 0, 0, 0, 1, 1};
  // 速度故障
  const std::vector<bool> value_break_down = {0, 0, 1, 1, 0, 0};
  // 故障阈值，加速度2 速度0.05
  const float gain_break_down_threshold = 2.0f;
  const float value_break_down_threshold = 0.05f;
  // 协方差矩阵P(预测协方差矩阵)=========================================
  Eigen::MatrixXd global_uncertainty;
  // 状态X(预测当前值)=================================================
  Eigen::VectorXd global_states;
  // global_states: center(2), velocity(2), acceleration(2)
  global_uncertainty.setIdentity(6, 6);
  global_states.setZero(6, 1);

  // 融合锚点
  fused_anchor_point_ =
      sensor_object->GetBaseObject()->anchor_point.cast<double>();
  // 融合速度
  fused_velocity_ = sensor_object->GetBaseObject()->velocity.cast<double>();
  // 融合加速度
  fused_acceleration_ = Eigen::Vector3d(0, 0, 0);

  // 设置X参数（这些都是需要更新的参数=================================）
  global_states(0) = sensor_object->GetBaseObject()->anchor_point(0);//？？？
  global_states(1) = sensor_object->GetBaseObject()->anchor_point(1);
  global_states(2) = sensor_object->GetBaseObject()->velocity(0);// 速度
  global_states(3) = sensor_object->GetBaseObject()->velocity(1);
  global_states(4) = 0.0;                                       //加速度
  global_states(5) = 0.0;
  // 设置P
  global_uncertainty.topLeftCorner(2, 2) =
      sensor_object->GetBaseObject()
          ->center_uncertainty.topLeftCorner(2, 2)
          .cast<double>();
  global_uncertainty.block<2, 2>(2, 2) =
      sensor_object->GetBaseObject()
          ->velocity_uncertainty.topLeftCorner(2, 2)
          .cast<double>();
  global_uncertainty.bottomRightCorner(2, 2) =
      sensor_object->GetBaseObject()
          ->velocity_uncertainty.topLeftCorner(2, 2)
          .cast<double>();

  // 保存到历史观测数据中，用于后续计算加速度
  if (sensor_object->GetBaseObject()->velocity_converged) {
    UpdateSensorHistory(sensor_object->GetSensorType(),
                        sensor_object->GetBaseObject()->velocity.cast<double>(),
                        sensor_object->GetTimestamp());
  }

  // 状态初始化
  if (!kalman_filter_.Init(global_states, global_uncertainty)) { //传参到kalman_filter.cc===============
    return false;
  }
  // 故障初始化
  if (!kalman_filter_.SetGainBreakdownThresh(gain_break_down,
                                             gain_break_down_threshold) ||
      !kalman_filter_.SetValueBreakdownThresh(value_break_down,
                                              value_break_down_threshold)) {
    return false;
  }

  return true;
}

//
void KalmanMotionFusion::GetStates(Eigen::Vector3d* anchor_point,
                                   Eigen::Vector3d* velocity) {
  *anchor_point = fused_anchor_point_;
  *velocity = fused_velocity_;
}

// 更新未匹配的航迹
void KalmanMotionFusion::UpdateWithoutMeasurement(const std::string& sensor_id,
                                                  double measurement_timestamp,
                                                  double target_timestamp) {
  SensorObjectConstPtr lidar_ptr = track_ref_->GetLatestLidarObject();
  SensorObjectConstPtr radar_ptr = track_ref_->GetLatestRadarObject();
  SensorObjectConstPtr camera_ptr = track_ref_->GetLatestCameraObject();
  // 判断是否存活
  bool is_alive =
      (lidar_ptr != nullptr || radar_ptr != nullptr || camera_ptr != nullptr);
  if (filter_init_ && is_alive) {
    double time_diff =
        measurement_timestamp - track_ref_->GetFusedObject()->GetTimestamp();
    MotionFusionWithoutMeasurement(time_diff); //预测kalman_filter_.Predict=============================
    fused_anchor_point_(0) = kalman_filter_.GetStates()(0);//  更新使用的是通过卡尔曼滤波得到预测得到的结果
    fused_anchor_point_(1) = kalman_filter_.GetStates()(1);
    fused_velocity_(0) = kalman_filter_.GetStates()(2);
    fused_velocity_(1) = kalman_filter_.GetStates()(3);
    fused_acceleration_(0) = kalman_filter_.GetStates()(4);
    fused_acceleration_(1) = kalman_filter_.GetStates()(5);
  }
  // Originally, we would reset filter_init_ to false, when there is no
  // valid lidar & radar measurement. now, as the quality of estimation
  // of camera improved, this step is not needed.
  UpdateMotionState(); //更新状态=======================
}

// 卡尔曼滤波融合KalmanMotionFusion!!!!!=========================================================main主函数
void KalmanMotionFusion::UpdateWithMeasurement(
    const SensorObjectConstPtr& measurement, double target_timestamp) {
  // 状态值
  fused_anchor_point_ =
      measurement->GetBaseObject()->anchor_point.cast<double>();
  fused_velocity_ = measurement->GetBaseObject()->velocity.cast<double>();
  fused_acceleration_ =
      measurement->GetBaseObject()->acceleration.cast<double>();
  // 协方差
  center_uncertainty_ = measurement->GetBaseObject()->center_uncertainty;
  velo_uncertainty_ = measurement->GetBaseObject()->velocity_uncertainty;
  acc_uncertainty_ = measurement->GetBaseObject()->acceleration_uncertainty;

  // 时间差
  double time_diff = measurement->GetTimestamp() -
                     track_ref_->GetFusedObject()->GetTimestamp();
  bool is_lidar = IsLidar(measurement);
  bool is_radar = IsRadar(measurement);
  bool is_camera = IsCamera(measurement);
  SensorObjectConstPtr lidar_ptr = track_ref_->GetLatestLidarObject();
  SensorObjectConstPtr radar_ptr = track_ref_->GetLatestRadarObject();
  SensorObjectConstPtr camera_ptr = track_ref_->GetLatestCameraObject();
  // Motion fusion
  // 卡尔曼滤波预测和更新
  if (is_lidar || is_radar || is_camera) {
    if (filter_init_) {
      MotionFusionWithMeasurement(measurement, time_diff); // 预测更新入口================================
    } else {
      filter_init_ = InitFilter(measurement);
    }
    if (!filter_init_) {
      // No kalman result, no matter which sensortype
      // of measurement, use measurement's
      // anchor point and velocity
      return;
    }
  }
  // 根据传感器，读卡尔曼滤波器的状态值，跟Apollo特性相关
  // shape & location fusion
  if (is_lidar || is_radar) {
    if (is_lidar || (is_radar && lidar_ptr == nullptr)) {
      // 1) measurement is lidar: use lidar's anchor point
      //    , use fused velocity
      // 2) measurement is radar, and has no history lidar:
      // use radar's anchor point, use fused velocity
      fused_velocity_(0) = kalman_filter_.GetStates()(2); // 修改measurement的一些状态
      fused_velocity_(1) = kalman_filter_.GetStates()(3);
    }
    if (is_radar && lidar_ptr != nullptr) {
      // measurement is radar, has history lidar:
      // use fused achor point, use fused velocity
      fused_anchor_point_(0) = kalman_filter_.GetStates()(0);
      fused_anchor_point_(1) = kalman_filter_.GetStates()(1);
      fused_velocity_(0) = kalman_filter_.GetStates()(2);
      fused_velocity_(1) = kalman_filter_.GetStates()(3);
    }
  } else if (is_camera) {
    if (filter_init_) {
      if (lidar_ptr != nullptr) {
        // measurement is camera, has history lidar
        // use fused position, use fused velocity
        fused_anchor_point_(0) = kalman_filter_.GetStates()(0);
        fused_anchor_point_(1) = kalman_filter_.GetStates()(1);
        fused_velocity_(0) = kalman_filter_.GetStates()(2);
        fused_velocity_(1) = kalman_filter_.GetStates()(3);
      } else if (radar_ptr != nullptr) {
        // measurement is camera, has no history lidar, but
        // has history radar, use history radar's position
        // use fused velocity
        fused_anchor_point_ =
            radar_ptr->GetBaseObject()->anchor_point.cast<double>();
        fused_velocity_(0) = kalman_filter_.GetStates()(2);
        fused_velocity_(1) = kalman_filter_.GetStates()(3);
      } else {
        // measurement is camera, has no history lidar
        // or radar, use measurement's anchor point and
        // velocity, do nothing(at the beginning of this
        // function, fused velocity/anchor point has been
        // set to as same as measurement)
      }
    }
  }
  fused_acceleration_(0) = kalman_filter_.GetStates()(4);
  fused_acceleration_(1) = kalman_filter_.GetStates()(5);
  // Originally, we would reset filter_init_ to false, when there is no
  // valid lidar & radar measurement. now, as the quality of estimation
  // of camera improved, this step is not needed.
  UpdateMotionState(); // 更新tracker的融合障碍物属性===========================
}

void KalmanMotionFusion::MotionFusionWithoutMeasurement(
    const double time_diff) {
  Eigen::MatrixXd transform_matrix;
  Eigen::MatrixXd env_uncertainty;
  transform_matrix.setIdentity(6, 6);
  transform_matrix(0, 2) = time_diff;
  transform_matrix(1, 3) = time_diff;
  env_uncertainty.setZero(6, 6);
  kalman_filter_.Predict(transform_matrix, env_uncertainty); // 预测结果
}

// tracker的运动物理量做卡尔曼滤波处理具体实现===============================
void KalmanMotionFusion::MotionFusionWithMeasurement(
    const SensorObjectConstPtr& measurement, double time_diff) {
  // we use kalman filter to update our tracker.
  // The pipeline is detailed as follows:
  // 1) compute the time diff to predict the tracker
  //    (although we introduce the acceleration, we
  //    doesn't use it to update the position)
  // 2) DeCorrelation the uncertainty matrix (we belief
  //    that the velocity won`t be affected by position)
  // 3) compute the acceleration of the measurement
  // 4) use the history radar or lidar(depend on which sensor
  //    type in current) to correct the observation
  // 5) set r_matrix according to converged or not
  // 6) use kalman to correct the predict before
  // 7) use correction breakdown to eliminate the unreasonable
  //    acceleration gain or velocity noise
  Eigen::MatrixXd transform_matrix;
  Eigen::MatrixXd env_uncertainty;

  transform_matrix.setIdentity(6, 6);
  transform_matrix(0, 2) = time_diff;
  transform_matrix(1, 3) = time_diff;
  transform_matrix(2, 4) = time_diff;
  transform_matrix(3, 5) = time_diff;

  env_uncertainty.setIdentity(6, 6);
  env_uncertainty *= 0.5;

  // 3.1 预测(modules/perception/fusion/common/kalman_filter.cc)
  // 一步预测
  kalman_filter_.Predict(transform_matrix, env_uncertainty); // 预测当前值和协方差矩阵

  // 3.2 计算加速度
  // 根据最近三帧计算加速度
  // 对camera目标，直接使用卡尔曼滤波一步预测后的加速度。应该是相机目标的加速度测的不准，不信赖其加速度？
  Eigen::Vector3d measured_acceleration = Eigen::Vector3d::Zero();
  measured_acceleration = ComputeAccelerationMeasurement(
      measurement->GetSensorType(),
      measurement->GetBaseObject()->velocity.cast<double>(), // 速度
      measurement->GetTimestamp());

  // 观测值
  Eigen::VectorXd observation;
  observation.setZero(6, 1);
  observation(0) = measurement->GetBaseObject()->center(0); //中心点坐标
  observation(1) = measurement->GetBaseObject()->center(1);
  observation(2) = measurement->GetBaseObject()->velocity(0); // 速度
  observation(3) = measurement->GetBaseObject()->velocity(1);
  observation(4) = measured_acceleration(0); // 加速度
  observation(5) = measured_acceleration(1);

  // 3.3 R矩阵初始化（观测噪声协方差矩阵）
  // 使用观测的距离和速度方差更新
  Eigen::MatrixXd r_matrix;
  r_matrix.setIdentity(6, 6);
  r_matrix.topLeftCorner(2, 2) = measurement->GetBaseObject()
                                     ->center_uncertainty.topLeftCorner(2, 2)
                                     .cast<double>();
  r_matrix.block<2, 2>(2, 2) = measurement->GetBaseObject()
                                   ->velocity_uncertainty.topLeftCorner(2, 2)
                                   .cast<double>();

  ADEBUG << "fusion_original_measurement@(" << std::setprecision(10)
         << observation(0) << "," << observation(1) << "," << observation(2)
         << "," << observation(3) << ")";
  ADEBUG << "fusion_original_measurement_covariance@(" << r_matrix(0, 0) << ","
         << r_matrix(0, 1) << "," << r_matrix(1, 0) << "," << r_matrix(1, 1)
         << "," << r_matrix(2, 2) << "," << r_matrix(2, 3) << ","
         << r_matrix(3, 2) << "," << r_matrix(3, 3) << ")";

  // Compute pseudo measurement
  // 3.4 修正观测值z_t  主要函数ComputePseudoMeasurement
  // Apollo特有的，就是根据历史的radar和lidar观测来修正当前帧的观测值，减少过大的突变，
  Eigen::Vector4d temp_observation = observation.head(4); // 之前的观测值
  Eigen::Vector4d pseudo_measurement =
      ComputePseudoMeasurement(temp_observation, measurement->GetSensorType()); // 具体实现得到测方程结果z_t
  // 现在的测量观测方程结果z_t：observation
  observation.head(4) = pseudo_measurement;

  // 3.5 保存修正后的观测数据
  // 速度收敛则储存到历史观测数据，用于计算加速度
  if (measurement->GetBaseObject()->velocity_converged) {
    UpdateSensorHistory(measurement->GetSensorType(),
                        measurement->GetBaseObject()->velocity.cast<double>(),
                        measurement->GetTimestamp());
  }

  // Adapt noise level to rewarding status
  // 3.6 调整R矩阵（观测噪声）
  // 针对是否lidar观测和是否收敛，调整R矩阵
  RewardRMatrix(measurement->GetSensorType(),
                measurement->GetBaseObject()->velocity_converged, &r_matrix);

  // 观测结果z_t
  ADEBUG << "fusion_pseudo_measurement@(" << std::setprecision(10)
         << observation(0) << "," << observation(1) << "," << observation(2)
         << "," << observation(3) << ")";
  ADEBUG << "fusion_pseudo_measurement_covariance@(" << r_matrix(0, 0) << ","
         << r_matrix(0, 1) << "," << r_matrix(1, 0) << "," << r_matrix(1, 1)
         << "," << r_matrix(2, 2) << "," << r_matrix(2, 3) << ","
         << r_matrix(3, 2) << "," << r_matrix(3, 3) << ")";

  // 3.7 解协方差((modules/perception/fusion/common/kalman_filter.cc))
  // 认为速度和位置互不影响，状态协方差矩阵P（6*6）的(2，0) (2,1) (3,0) (3,1)位置设为0
  kalman_filter_.DeCorrelation(2, 0, 2, 2);
  // 3.8 卡尔曼更新（输入：）  实现kalman_filter.cc
  kalman_filter_.Correct(observation, r_matrix);
  // 3.9 修正更新后的加速度和速度
  kalman_filter_.CorrectionBreakdown();

  ADEBUG << "fusion_filter_belief@(" << std::setprecision(10)
         << kalman_filter_.GetStates()(0) << ","
         << kalman_filter_.GetStates()(1) << ","
         << kalman_filter_.GetStates()(2) << ","
         << kalman_filter_.GetStates()(3) << ")";
  ADEBUG << "fusion_filter_belief_covariance@("
         << kalman_filter_.GetUncertainty()(0, 0) << ","
         << kalman_filter_.GetUncertainty()(0, 1) << ","
         << kalman_filter_.GetUncertainty()(1, 0) << ","
         << kalman_filter_.GetUncertainty()(1, 1) << ","
         << kalman_filter_.GetUncertainty()(2, 2) << ","
         << kalman_filter_.GetUncertainty()(2, 3) << ","
         << kalman_filter_.GetUncertainty()(3, 2) << ","
         << kalman_filter_.GetUncertainty()(3, 3) << ")";
}

// 最终更新结果送到GetFusedObject==================
void KalmanMotionFusion::UpdateMotionState() {
  base::ObjectPtr obj = track_ref_->GetFusedObject()->GetBaseObject();  // 融合障碍物结果
  obj->anchor_point = fused_anchor_point_.cast<double>();
  // it seems that it is the only place to update the FusedObject's `center`
  // who will be used in CollectFusedObjects
  obj->center = obj->anchor_point;
  obj->velocity = fused_velocity_.cast<float>(); // 速度 加速度
  obj->acceleration = fused_acceleration_.cast<float>();
  // Previously, obj velocity uncertainty would be updated according to the
  // uncertainty within kalman filter. however, rewarding strategy within
  // motion fusion could change kalman filter's uncertainty, which could
  // not reflect the reality of cov.
  // TEMPORARYLY, we keep obj's velocity covariance as it computed in
  // single sensor. THIS IS AS SAME AS 2-1-19-1.
  // a more general probabilisitic method would be try in this week, and
  // codes would be updated after benchmarking.
  obj->center_uncertainty = center_uncertainty_; // 不确定性
  obj->velocity_uncertainty = velo_uncertainty_;
  obj->acceleration_uncertainty = acc_uncertainty_;
}

// 3.2 计算加速度
Eigen::VectorXd KalmanMotionFusion::ComputeAccelerationMeasurement(
    const base::SensorType& sensor_type, const Eigen::Vector3d& velocity,
    const double& timestamp) {
  Eigen::Vector3d acceleration_measurement = Eigen::Vector3d::Zero();
  // 对camera目标，直接使用卡尔曼滤波一步预测后的加速度
  if (common::SensorManager::Instance()->IsCamera(sensor_type)) {
    acceleration_measurement(0) = kalman_filter_.GetStates()(4); //加速度
    acceleration_measurement(1) = kalman_filter_.GetStates()(5);
    return acceleration_measurement;
  }
  // 该观测传感器保存了有3帧以上
  if (GetSensorHistoryLength(sensor_type) >= s_eval_window_) {
    size_t history_index = GetSensorHistoryIndex(sensor_type, s_eval_window_);
    if (history_index >= history_velocity_.size()) {
      AERROR << "illegal history index";
      return Eigen::Vector3d::Zero();
    }
    // 最新观测和第一帧的速度差/时间差（隔了两帧？
    acceleration_measurement = velocity - history_velocity_[history_index];
    acceleration_measurement /= (timestamp - history_timestamp_[history_index]);
  }
  return acceleration_measurement;
}

// 3.6 调整R矩阵
// 针对是否lidar观测和是否收敛，调整R矩阵
void KalmanMotionFusion::RewardRMatrix(const base::SensorType& sensor_type,
                                       const bool& converged,
                                       Eigen::MatrixXd* r_matrix) {
  common::SensorManager* sensor_manager = common::SensorManager::Instance();
  const float converged_scale = 0.01f;
  const float unconverged_scale = 1000.0f;
  // R=6*6矩阵
  // if 观测是lidar：
  //     if 速度收敛：
  //         设置协方差：位置和速度0.01，加速度1
  //     else：
  //         设置协方差：位置0.01，速度1000，加速度1

  // else if 观测是radar或者camera：
  //     设置协方差：位置和速度乘以2

  //     if VELODYNE_64的历史观测长度 > 0:
  //         速度协方差=1000
  // 加速度协方差×0.5
  if (sensor_manager->IsLidar(sensor_type)) {
    if (converged) {
      r_matrix->setIdentity();
      // 收敛则设置协方差：位置和速度0.01，加速度1
      r_matrix->block<4, 4>(0, 0) *= converged_scale;
    } else {
      r_matrix->setIdentity();
      // 不收敛则设置协方差：位置0.01，速度1000，加速度1
      r_matrix->block<2, 2>(0, 0) *= converged_scale;
      r_matrix->block<2, 2>(2, 2) *= unconverged_scale;
    }
  } else if (sensor_manager->IsRadar(sensor_type) ||
             sensor_manager->IsCamera(sensor_type)) {
    // 设置协方差：位置和速度乘以2
    r_matrix->block<4, 4>(0, 0) *= 2.0;
    int lidar_history_length =
        GetSensorHistoryLength(base::SensorType::VELODYNE_64);
    if (lidar_history_length > 0) {
      // 速度协方差=1000
      r_matrix->block<2, 2>(2, 2).setIdentity();
      r_matrix->block<2, 2>(2, 2) *= unconverged_scale;
    }
  }
  // 加速度协方差×0.5
  r_matrix->block<2, 2>(4, 4) *= 0.5;
}
// 3.4 修正观测值
// Apollo特有的，就是根据历史的radar和lidar观测来修正当前帧的观测值，减少过大的突变，
Eigen::Vector4d KalmanMotionFusion::  ComputePseudoMeasurement(
    const Eigen::Vector4d& measurement, const base::SensorType& sensor_type) {
  // What is a pseudo-lidar estimation? if given lidar estimation could trace
  // a good radar estimation within a short window, then project radar
  // estimation on given lidar one. otherwise, use original lidar estimation.
  // what is a pseudo-radar estimation? if given radar estimation is good,
  // project it on its last good lidar estimation within a short window.
  // otherwise, use current belief
  common::SensorManager* sensor_manager = common::SensorManager::Instance();
  // (1)lidar观测修正
  if (sensor_manager->IsLidar(sensor_type)) {
    return ComputePseudoLidarMeasurement(measurement);
  }
  // (2)camera观测修正
  if (sensor_manager->IsRadar(sensor_type)) {
    return ComputePseudoRadarMeasurement(measurement);
  }
  // (3)radar观测修正
  if (sensor_manager->IsCamera(sensor_type)) {
    return ComputePseudoCameraMeasurement(measurement);
  }
  AINFO << "unsupport sensor type for pseudo measurement computation!";
  Eigen::Vector4d pseudo_measurement = measurement;
  return pseudo_measurement;
}

Eigen::Vector4d KalmanMotionFusion::ComputePseudoLidarMeasurement(
    const Eigen::Vector4d& measurement) {
  // Initialize status variables
  int trace_count = 0;
  const float velocity_angle_change_thresh_ = static_cast<float>(M_PI / 20.0);
  const float acceleration_angle_change_thresh_ =
      static_cast<float>(M_PI / 3.0);
  Eigen::Vector4d pseudo_measurement = measurement;
  Eigen::Vector3d lidar_velocity =
      Eigen::Vector3d(measurement(2), measurement(3), 0);
  Eigen::Vector3d fused_acceleration = Eigen::Vector3d::Zero();
  fused_acceleration(0) = kalman_filter_.GetStates()(4);
  fused_acceleration(1) = kalman_filter_.GetStates()(5);
  // Return if lidar velocity is already small enough
  double lidar_velocity_norm = lidar_velocity.norm();
  if (lidar_velocity_norm < kDoubleEpsilon) {
    return pseudo_measurement;
  }
  // Trace back radar velocity history, try to find good radar measurement
  // which could help lidar velocity get a more accurate pseudo measurement
  for (size_t count = 1; count < history_sensor_type_.size(); ++count) {
    size_t history_index = history_sensor_type_.size() - count;
    base::SensorType& history_type = history_sensor_type_[history_index];
    if (common::SensorManager::Instance()->IsRadar(history_type)) {
      trace_count++;
      Eigen::Vector3d radar_velocity = history_velocity_[history_index];
      double radar_velocity_norm = radar_velocity.norm();
      // Abandon radar history, if their velocity lengths are too different
      if (fabs(radar_velocity_norm - lidar_velocity_norm) >
          velocity_length_change_thresh_) {
        continue;
      }
      // Abandon radar history, if its velocity angle change is too large
      double velocity_angle_change =
          common::CalculateTheta2DXY(radar_velocity, lidar_velocity);
      if (fabs(velocity_angle_change) > velocity_angle_change_thresh_) {
        continue;
      }
      // Abandon radar history, if its acceleration angle change is too large
      Eigen::Vector3d radar_velocity_project_on_lidar_velocity =
          common::Calculate2DXYProjectVector(radar_velocity, lidar_velocity);
      Eigen::Vector3d radar_velocity_project_on_lidar_velocity_gain =
          radar_velocity_project_on_lidar_velocity - lidar_velocity;
      double acceleration_angle_change = common::CalculateTheta2DXY(
          fused_acceleration, radar_velocity_project_on_lidar_velocity_gain);
      if (fabs(acceleration_angle_change) > acceleration_angle_change_thresh_) {
        continue;
      }
      // Compute normalized velocity gain
      double normalized_radar_velocity_project_on_lidar_velocity_gain =
          radar_velocity_project_on_lidar_velocity_gain.head(2).norm() /
          std::max(radar_velocity_project_on_lidar_velocity.head(2).norm(),
                   lidar_velocity.head(2).norm());
      // Compute normalized velocity angle change
      double normalized_velocity_angle_change =
          fabs(velocity_angle_change) / velocity_angle_change_thresh_;
      // Abandon radar history, if normalized diff is too large
      if (normalized_radar_velocity_project_on_lidar_velocity_gain *
              normalized_velocity_angle_change >
          0.5) {
        continue;
      }
      pseudo_measurement(2) = radar_velocity_project_on_lidar_velocity(0);
      pseudo_measurement(3) = radar_velocity_project_on_lidar_velocity(1);
      return pseudo_measurement;
    }
    if (trace_count == s_eval_window_) {
      pseudo_measurement = measurement;
      return pseudo_measurement;
    }
  }
  pseudo_measurement = measurement;
  return pseudo_measurement;
}

Eigen::Vector4d KalmanMotionFusion::ComputePseudoCameraMeasurement(
    const Eigen::Vector4d& measurement) {
  // Initialize status variables
  int trace_count = 0;
  const float velocity_angle_change_thresh_ = static_cast<float>(M_PI / 10.0);
  const float acceleration_angle_change_thresh_ =
      static_cast<float>(M_PI / 3.0);
  Eigen::Vector4d pseudo_measurement = measurement;
  Eigen::Vector3d camera_velocity =
      Eigen::Vector3d(measurement(2), measurement(3), 0);
  Eigen::Vector3d fused_acceleration = Eigen::Vector3d::Zero();
  fused_acceleration(0) = kalman_filter_.GetStates()(4);
  fused_acceleration(1) = kalman_filter_.GetStates()(5);
  // Return if camera velocity is already small enough
  double camera_velocity_norm = camera_velocity.norm();
  if (camera_velocity_norm < kDoubleEpsilon) {
    return pseudo_measurement;
  }
  // Trace back radar velocity history, try to find good radar measurement
  // which could help camera velocity get a more accurate pseudo measurement
  for (size_t count = 1; count < history_sensor_type_.size(); ++count) {
    size_t history_index = history_sensor_type_.size() - count;
    base::SensorType& history_type = history_sensor_type_[history_index];
    if (common::SensorManager::Instance()->IsRadar(history_type)) {
      trace_count++;
      Eigen::Vector3d radar_velocity = history_velocity_[history_index];
      double radar_velocity_norm = radar_velocity.norm();
      // Abandon radar history, if their velocity lengths are too different
      if (fabs(radar_velocity_norm - camera_velocity_norm) >
          velocity_length_change_thresh_) {
        continue;
      }
      // Abandon radar history, if its velocity angle change is too large
      double velocity_angle_change =
          common::CalculateTheta2DXY(radar_velocity, camera_velocity);
      if (fabs(velocity_angle_change) > velocity_angle_change_thresh_) {
        continue;
      }
      // Abandon radar history, if its acceleration angle change is too large
      Eigen::Vector3d radar_velocity_project_on_camera_velocity =
          common::Calculate2DXYProjectVector(radar_velocity, camera_velocity);
      Eigen::Vector3d radar_velocity_project_on_camera_velocity_gain =
          radar_velocity_project_on_camera_velocity - camera_velocity;
      double acceleration_angle_change = common::CalculateTheta2DXY(
          fused_acceleration, radar_velocity_project_on_camera_velocity_gain);
      if (fabs(acceleration_angle_change) > acceleration_angle_change_thresh_) {
        continue;
      }
      // Compute normalized velocity gain
      double normalized_radar_velocity_project_on_camera_velocity_gain =
          radar_velocity_project_on_camera_velocity_gain.head(2).norm() /
          std::max(radar_velocity_project_on_camera_velocity.head(2).norm(),
                   camera_velocity.head(2).norm());
      // Compute normalized velocity angle change
      double normalized_velocity_angle_change =
          fabs(velocity_angle_change) / velocity_angle_change_thresh_;
      // Abandon radar history, if normalized diff is too large
      if (normalized_radar_velocity_project_on_camera_velocity_gain *
              normalized_velocity_angle_change >
          0.3) {
        continue;
      }
      pseudo_measurement(2) = radar_velocity_project_on_camera_velocity(0);
      pseudo_measurement(3) = radar_velocity_project_on_camera_velocity(1);
      return pseudo_measurement;
    }
    if (trace_count == s_eval_window_) {
      pseudo_measurement = measurement;
      return pseudo_measurement;
    }
  }
  pseudo_measurement = measurement;
  return pseudo_measurement;
}

Eigen::Vector4d KalmanMotionFusion::ComputePseudoRadarMeasurement(
    const Eigen::Vector4d& measurement) {
  // Initialize status variables
  int lidar_trace_count = 0;
  int camera_trace_count = 0;
  Eigen::Vector4d pseudo_measurement = measurement;
  int lidar_camera_history_length =
      GetSensorHistoryLength(base::SensorType::VELODYNE_64) +
      GetSensorHistoryLength(base::SensorType::MONOCULAR_CAMERA);
  // Keep motion if lidar & camera history is empty
  if (lidar_camera_history_length == 0) {
    pseudo_measurement(2) = kalman_filter_.GetStates()(2);
    pseudo_measurement(3) = kalman_filter_.GetStates()(3);
    return pseudo_measurement;
  }
  Eigen::Vector3d radar_velocity =
      Eigen::Vector3d(measurement(2), measurement(3), 0);
  Eigen::Vector3d fused_acceleration = Eigen::Vector3d::Zero();
  fused_acceleration(0) = kalman_filter_.GetStates()(4);
  fused_acceleration(1) = kalman_filter_.GetStates()(5);
  common::SensorManager* sensor_manager = common::SensorManager::Instance();
  // Trace back lidar and camera history, try to find good lidar/camera
  // measurement which could help radar velocity get a more robust pseudo
  // measurement.
  for (size_t count = 1; count < history_sensor_type_.size(); ++count) {
    size_t history_index = history_sensor_type_.size() - count;
    base::SensorType& history_type = history_sensor_type_[history_index];
    if (!sensor_manager->IsLidar(history_type) &&
        !sensor_manager->IsCamera(history_type)) {
      continue;
    }
    // Keep motion, if no good history been found
    if (lidar_trace_count == s_eval_window_ ||
        camera_trace_count == s_eval_window_) {
      pseudo_measurement(2) = kalman_filter_.GetStates()(2);
      pseudo_measurement(3) = kalman_filter_.GetStates()(3);
      return pseudo_measurement;
    }
    Eigen::Vector3d history_velocity = history_velocity_[history_index];
    // Abandon history measurement, if its speed is too small
    if (history_velocity.norm() < kDoubleEpsilon) {
      pseudo_measurement(2) = history_velocity(0);
      pseudo_measurement(3) = history_velocity(1);
      return pseudo_measurement;
    }
    // Compute velocity angle change and acceleration angle change
    double velocity_angle_change =
        common::CalculateTheta2DXY(radar_velocity, history_velocity);
    Eigen::Vector3d radar_velocity_project_on_history_velocity =
        common::Calculate2DXYProjectVector(radar_velocity, history_velocity);
    Eigen::Vector3d radar_velocity_project_on_history_velocity_gain =
        radar_velocity_project_on_history_velocity - history_velocity;
    double acceleration_angle_change = common::CalculateTheta2DXY(
        fused_acceleration, radar_velocity_project_on_history_velocity_gain);
    // Compute normalized velocity gain
    double normalized_radar_velocity_project_on_history_velocity_gain =
        radar_velocity_project_on_history_velocity_gain.head(2).norm() /
        std::max(radar_velocity_project_on_history_velocity.head(2).norm(),
                 history_velocity.head(2).norm());
    // Handle lidar history
    if (sensor_manager->IsLidar(history_type)) {
      lidar_trace_count++;
      // Abandon lidar measurement, if its velocity angle change is too big
      if (fabs(velocity_angle_change) > M_PI / 20) {
        continue;
      }
      // Abandon lidar measurement, if its acceleration angle change is too
      // big
      if (fabs(acceleration_angle_change) > M_PI / 3) {
        continue;
      }
      // Compute normalized velocity angle change
      double normalized_velocity_angle_change =
          fabs(velocity_angle_change) / (M_PI / 20);
      // Abandon lidar measurement, if normalized diff is too big
      if (normalized_radar_velocity_project_on_history_velocity_gain *
              normalized_velocity_angle_change >
          0.5) {
        continue;
      }
      pseudo_measurement(2) = radar_velocity_project_on_history_velocity(0);
      pseudo_measurement(3) = radar_velocity_project_on_history_velocity(1);
      return pseudo_measurement;
    } else {
      ++camera_trace_count;
      // Abandon camera measurement, if its velocity angle change is too big
      if (fabs(velocity_angle_change) > M_PI / 10) {
        continue;
      }
      // Abandon camera measurement, if its acceleration angle change is too
      // big
      if (fabs(acceleration_angle_change) > M_PI / 3) {
        continue;
      }
      // Compute normalized velocity angle change
      double normalized_velocity_angle_change =
          fabs(velocity_angle_change) / (M_PI / 10);
      // Abandon lidar measurement, if normalized diff is too big
      if (normalized_radar_velocity_project_on_history_velocity_gain *
              normalized_velocity_angle_change >
          0.3) {
        continue;
      }
      pseudo_measurement(2) = radar_velocity_project_on_history_velocity(0);
      pseudo_measurement(3) = radar_velocity_project_on_history_velocity(1);
      return pseudo_measurement;
    }
  }
  // Use original measurement, if history is not enough
  pseudo_measurement = measurement;
  return pseudo_measurement;
}

void KalmanMotionFusion::UpdateSensorHistory(
    const base::SensorType& sensor_type, const Eigen::Vector3d& velocity,
    const double& timestamp) {
  int lidar_history_length =
      GetSensorHistoryLength(base::SensorType::VELODYNE_64);
  int radar_history_length =
      GetSensorHistoryLength(base::SensorType::LONG_RANGE_RADAR);
  if ((lidar_history_length >= s_eval_window_ &&
       radar_history_length >= s_eval_window_) ||
      history_velocity_.size() > s_history_size_maximum_) {
    history_velocity_.pop_front();
    history_timestamp_.pop_front();
    history_sensor_type_.pop_front();
  }
  history_velocity_.push_back(velocity);
  history_timestamp_.push_back(timestamp);
  history_sensor_type_.push_back(sensor_type);
}

int KalmanMotionFusion::GetSensorHistoryLength(
    const base::SensorType& sensor_type) {
  int sensor_history_length = 0;
  for (size_t i = 0; i < history_sensor_type_.size(); ++i) {
    if (history_sensor_type_[i] == sensor_type) {
      ++sensor_history_length;
    }
  }
  return sensor_history_length;
}

int KalmanMotionFusion::GetSensorHistoryIndex(
    const base::SensorType& sensor_type, const int& trace_length) {
  int history_index = 0;
  int history_count = 0;
  for (size_t i = 1; i <= history_sensor_type_.size(); ++i) {
    history_index = static_cast<int>(history_sensor_type_.size() - i);
    if (history_sensor_type_[history_index] == sensor_type) {
      ++history_count;
    }
    if (history_count == trace_length) {
      return history_index;
    }
  }

  return -1;
}

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
