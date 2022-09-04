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
#include "modules/perception/fusion/lib/data_fusion/tracker/pbf_tracker/pbf_tracker.h"

#include "cyber/common/file.h"
#include "modules/common/util/string_util.h"
#include "modules/perception/fusion/lib/data_fusion/existence_fusion/dst_existence_fusion/dst_existence_fusion.h"
#include "modules/perception/fusion/lib/data_fusion/motion_fusion/kalman_motion_fusion/kalman_motion_fusion.h"
#include "modules/perception/fusion/lib/data_fusion/shape_fusion/pbf_shape_fusion/pbf_shape_fusion.h"
#include "modules/perception/fusion/lib/data_fusion/type_fusion/dst_type_fusion/dst_type_fusion.h"
#include "modules/perception/lib/config_manager/config_manager.h"

namespace apollo {
namespace perception {
namespace fusion {

using cyber::common::GetAbsolutePath;

// TODO(all) fix the static string lint issue
std::string PbfTracker::s_type_fusion_method_ = "DstTypeFusion";  // NOLINT
std::string PbfTracker::s_existence_fusion_method_ =              // NOLINT
    "DstExistenceFusion";
std::string PbfTracker::s_motion_fusion_method_ =  // NOLINT
    "KalmanMotionFusion";
std::string PbfTracker::s_shape_fusion_method_ = "PbfShapeFusion";  // NOLINT

PbfTracker::PbfTracker() {}

PbfTracker::~PbfTracker() {}

bool PbfTracker::InitParams() {
  BaseInitOptions options;
  if (!GetFusionInitOptions("PbfTracker", &options)) {
    return false;
  }

  std::string woork_root_config = GetAbsolutePath(
      lib::ConfigManager::Instance()->work_root(), options.root_dir);

  std::string config = GetAbsolutePath(woork_root_config, options.conf_file);
  AINFO << "Config file : " << config;
  PbfTrackerConfig params;
  if (!cyber::common::GetProtoFromFile(config, &params)) {
    AERROR << "Read config failed: " << config;
    return false;
  }

  // modules/perception/proto/pbf_tracker_config.proto
  AINFO << "Load PbfTrackerConfig: " << params.type_fusion_method() << ","
        << params.motion_fusion_method() << "," << params.shape_fusion_method()
        << "," << params.existence_fusion_method();
  s_type_fusion_method_ = params.type_fusion_method();
  s_motion_fusion_method_ = params.motion_fusion_method();
  s_existence_fusion_method_ = params.existence_fusion_method();
  s_shape_fusion_method_ = params.shape_fusion_method();

  return true;
}

bool PbfTracker::InitMethods() {
  if (s_type_fusion_method_ == "DstTypeFusion") {
    type_fusion_.reset(new DstTypeFusion(track_));
  } else {
    AERROR << "Unknown type fusion : " << s_type_fusion_method_;
    return false;
  }

  if (s_motion_fusion_method_ == "KalmanMotionFusion") {
    motion_fusion_.reset(new KalmanMotionFusion(track_)); // class类
  } else {
    AERROR << "Unknown motion fusion : " << s_motion_fusion_method_;
    return false;
  }

  if (s_existence_fusion_method_ == "DstExistenceFusion") {
    existence_fusion_.reset(new DstExistenceFusion(track_));
  } else {
    AERROR << "Unknown existence fusion : " << s_existence_fusion_method_;
    return false;
  }

  if (s_shape_fusion_method_ == "PbfShapeFusion") {
    shape_fusion_.reset(new PbfShapeFusion(track_));
  } else {
    AERROR << "Unknown shape fusion : " << s_shape_fusion_method_;
    return false;
  }

  return true;
}

bool PbfTracker::Init(TrackPtr track, SensorObjectPtr measurement) {
  track_ = track;
  if (!InitMethods()) {
    return false;
  }
  motion_fusion_->Init();
  return true;
}

// 观测更新tracker
// tracker更新的函数中会更新四个部分，existence、motion、shape、type和tracker的信息
// 主要是DS theory和Kalman更新tracker的属性
// 前四个fusion的配置参数在modules/perception/proto/pbf_tracker_config.proto，就是init中的默认值。
// 个人文档：https://www.yuque.com/huangzhongqing/crvg1o/ayi6dc
void PbfTracker::UpdateWithMeasurement(const TrackerOptions& options,
                                       const SensorObjectPtr measurement,
                                       double target_timestamp) {
  std::string sensor_id = measurement->GetSensorId();
  ADEBUG << "fusion_updating..." << track_->GetTrackId() << " with "
         << sensor_id << "..." << measurement->GetBaseObject()->track_id << "@"
         << FORMAT_TIMESTAMP(measurement->GetTimestamp());
  // options.match_distance = 0
  // 1 DstExistenceFusion（更新tracker的存在性）
  // 证据推理（DS theory）更新tracker的存在性modules/perception/fusion/lib/data_fusion/existence_fusion/dst_existence_fusion/dst_existence_fusion.cc
  existence_fusion_->UpdateWithMeasurement(measurement, target_timestamp,
                                           options.match_distance);

  // 2 * KalmanMotionFusion!!!!!motion_fusion/kalman_motion_fusion/kalman_motion_fusion.cc=======================================================================
  // 鲁棒卡尔曼滤波更新tracker的运动属性
  // trackers_[track_ind]->UpdateWithMeasurement(frame->GetForegroundObjects()[obj_ind], frame->GetTimestamp());
  motion_fusion_->UpdateWithMeasurement(measurement, target_timestamp);

  // 3 PbfShapeFusion modules/perception/fusion/lib/data_fusion/shape_fusion/pbf_shape_fusion/pbf_shape_fusion.cc
  // 更新tracker的形状
  shape_fusion_->UpdateWithMeasurement(measurement, target_timestamp);

  // 4 * DstTypeFusion!!!!!（更新tracker的属性）=============================================================================
  // modules/perception/fusion/lib/data_fusion/type_fusion/dst_type_fusion/dst_type_fusion.cc
  // 证据推理（DS theory）更新tracker的属性
  type_fusion_->UpdateWithMeasurement(measurement, target_timestamp);

  // 最后更新
  track_->UpdateWithSensorObject(measurement);
}

// 同上UpdateAssignedTracks一样，对没有匹配到观测的tracker，更新同样的参数
void PbfTracker::UpdateWithoutMeasurement(const TrackerOptions& options,
                                          const std::string& sensor_id,
                                          double measurement_timestamp,
                                          double target_timestamp) {
  existence_fusion_->UpdateWithoutMeasurement(sensor_id, measurement_timestamp,
                                              target_timestamp,
                                              options.match_distance);
  motion_fusion_->UpdateWithoutMeasurement(sensor_id, measurement_timestamp,
                                           target_timestamp);
  shape_fusion_->UpdateWithoutMeasurement(sensor_id, measurement_timestamp,
                                          target_timestamp);
  type_fusion_->UpdateWithoutMeasurement(sensor_id, measurement_timestamp,
                                         target_timestamp,
                                         options.match_distance);
  track_->UpdateWithoutSensorObject(sensor_id, measurement_timestamp);
}

std::string PbfTracker::Name() const { return "PbfTracker"; }

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
