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
#include "modules/perception/fusion/lib/data_fusion/shape_fusion/pbf_shape_fusion/pbf_shape_fusion.h"

namespace apollo {
namespace perception {
namespace fusion {

bool PbfShapeFusion::s_use_camera_3d_ = true;
float PbfShapeFusion::s_camera_radar_time_diff_th_ = 0.3f;

bool PbfShapeFusion::Init() { return true; }

//形状
void PbfShapeFusion::UpdateWithMeasurement(const SensorObjectPtr measurement, //输入 SensorObject：单个目标
                                           double target_timestamp) {
  // base::SensorType sensor_type = measurement->GetSensorType();
  SensorObjectConstPtr latest_lidar = track_ref_->GetLatestLidarObject(); // 当前航迹最近的历史传感器数据
  SensorObjectConstPtr latest_radar = track_ref_->GetLatestRadarObject();
  SensorObjectConstPtr latest_camera = track_ref_->GetLatestCameraObject();

  /*
    逻辑：
    优先使用lidar的形状，最近的历史关联的观测中有lidar的话，radar和camera观测都不会做更新
    其次优先camera的形状，间隔更新时间比较小的话，radar观测仅仅更新中心，形状用历史camera更新
    最后最近的历史观测中lidar和camera都没有的话，才使用radar的观测更新

    仅一处用历史观测做更新，其余全是用measurement做更新。
    UpdateState = UpdateShape + UpdateCenter

    观测是lidar：
      UpdateState(measurement)

    观测是radar：
      if 最新的lidar历史观测为空：
        if 最新的camera历史观测不为空：
          if 观测-camera 时间戳<0.3
            UpdateShape(camera)
            UpdateCenter(measurement)
        else 最新的camera历史观测为空：
          UpdateState(measurement)
      else
        不更新

    观测是camera：
      if 最新的lidar历史观测为空：
        UpdateState(measurement)
      else
        不更新
  */
  if (IsLidar(measurement)) {
    UpdateState(measurement); //1 优先使用lidar的形状
  } else if (IsRadar(measurement)) { // 2 观测radar
    if (latest_lidar == nullptr) {
      if (latest_camera != nullptr) { //最新历史观测的无lidar，有camera
        if (std::fabs(measurement->GetTimestamp() -
                      latest_camera->GetTimestamp()) <
            s_camera_radar_time_diff_th_) { // 观测-camera 时间戳<0.3s
          // 间隔更新时间比较小的话，radar观测仅仅更新中心，形状用历史camera更新
          UpdateShape(latest_camera); // 仅一处用历史观测做更新，其余全是用measurement做更新。
          UpdateCenter(measurement); //观测radar的center更新
        } else {
          // nothing to do
        }
      } else { // 最新历史观测的无lidar，无camera
        UpdateState(measurement); // 使用radar观测的更新
      }
    } else {
      // nothing to do
    }
  } else if (IsCamera(measurement) && s_use_camera_3d_) { // 3 观测camera
    if (latest_lidar == nullptr) {
      UpdateState(measurement); // 使用camera的更新
    } else {
      // nothing to do
    }
  } else {
    // nothing to do;
  }
}

//更新UpdateWithoutMeasurement
void PbfShapeFusion::UpdateWithoutMeasurement(const std::string& sensor_id,
                                              double measurement_timestamp,
                                              double target_timestamp) {}

std::string PbfShapeFusion::Name() const { return "PbfShapeFusion"; }

// 下面被上面调用============================================================================
// 包括更新Shape和Center
void PbfShapeFusion::UpdateState(const SensorObjectConstPtr& measurement) {
  UpdateShape(measurement);
  UpdateCenter(measurement);
}

// Shape包括size,direcation,theta,polygon
void PbfShapeFusion::UpdateShape(const SensorObjectConstPtr& measurement) {
  base::ObjectPtr dst_obj = track_ref_->GetFusedObject()->GetBaseObject(); // 融合的结果track被更新
  base::ObjectConstPtr src_obj = measurement->GetBaseObject(); //

  dst_obj->size = src_obj->size; //修改融合障碍物的size
  dst_obj->direction = src_obj->direction;
  dst_obj->theta = src_obj->theta;
  dst_obj->polygon = src_obj->polygon;
}

void PbfShapeFusion::UpdateCenter(const SensorObjectConstPtr& measurement) {
  base::ObjectPtr dst_obj = track_ref_->GetFusedObject()->GetBaseObject();
  base::ObjectConstPtr src_obj = measurement->GetBaseObject();

  dst_obj->center = src_obj->center;
  dst_obj->anchor_point = src_obj->anchor_point;
}

// FUSION_REGISTER_SHAPEFUSION(PbfShapeFusion)

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
