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
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "cyber/component/component.h"
#include "modules/perception/base/object.h"
#include "modules/perception/fusion/app/obstacle_multi_sensor_fusion.h"
#include "modules/perception/fusion/lib/interface/base_fusion_system.h"
#include "modules/perception/map/hdmap/hdmap_input.h"
#include "modules/perception/onboard/inner_component_messages/inner_component_messages.h"
#include "modules/perception/onboard/proto/fusion_component_config.pb.h"

namespace apollo {
namespace perception {
namespace onboard {

class FusionComponent : public cyber::Component<SensorFrameMessage> {
 public:
  FusionComponent() = default;
  ~FusionComponent() = default;
  bool Init() override; // 由 Cyber RT 在启动组件过程中进行间接调用，只会执行一次
  bool Proc(const std::shared_ptr<SensorFrameMessage>& message) override; // 由 Cyber RT 在启动组件过程中注册为消息回调函数

 private:
  bool InitAlgorithmPlugin();  // 组件内部的初始化方法，由 Init 方法进行调用
  //  // 组件内部的消息处理方法，具体算法流程入口，由 Proc 方法进行调用
  bool InternalProc(const std::shared_ptr<SensorFrameMessage const>& in_message,
                    std::shared_ptr<PerceptionObstacles> out_message,
                    std::shared_ptr<SensorFrameMessage> viz_message);

 private:
  static std::mutex s_mutex_;
  static uint32_t s_seq_num_;

  std::string fusion_method_; // 融合方法
  std::string fusion_main_sensor_;  // 融合主传感器
  bool object_in_roi_check_ = false; // 是否开启 HD Map ROI 融合障碍物校验
  double radius_for_roi_object_check_ = 0; // HD Map ROI 融合障碍物校验半径

  std::unique_ptr<fusion::ObstacleMultiSensorFusion> fusion_;  // 独占智能指针，用于管理多传感器融合抽象基类对象，最为关键的数据成员
  map::HDMapInput* hdmap_input_ = nullptr;  // HD Map 输入
  std::shared_ptr<apollo::cyber::Writer<PerceptionObstacles>> writer_;  // Cyber Writer 对象，用于输出 protobuf 格式的经 HD Map ROI 校验过的有效融合障碍物信息
  std::shared_ptr<apollo::cyber::Writer<SensorFrameMessage>> inner_writer_; // Cyber Writer 对象，用于输出可视化信息
};

CYBER_REGISTER_COMPONENT(FusionComponent);

}  // namespace onboard
}  // namespace perception
}  // namespace apollo
