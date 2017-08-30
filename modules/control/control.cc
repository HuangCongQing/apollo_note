/******************************************************************************
  * Copyright 2017 The Apollo Authors. All Rights Reserved.
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
#include "modules/control/control.h"

#include <iomanip>
#include <string>

#include "ros/include/std_msgs/String.h"

#include "modules/localization/proto/localization.pb.h"

#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/adapters/adapter_manager.h"
#include "modules/common/log.h"
#include "modules/common/time/time.h"
#include "modules/common/vehicle_state/vehicle_state.h"
#include "modules/control/common/control_gflags.h"

namespace apollo {
namespace control {

using apollo::canbus::Chassis;
using apollo::common::ErrorCode;
using apollo::common::Status;
using apollo::common::adapter::AdapterManager;
using apollo::common::monitor::MonitorMessageItem;
using apollo::common::time::Clock;
using apollo::localization::LocalizationEstimate;
using apollo::planning::ADCTrajectory;

std::string Control::Name() const { return FLAGS_node_name; }

Status Control::Init() {
  AINFO << "Control init, starting ...";
  CHECK(::apollo::common::util::GetProtoFromFile(FLAGS_control_conf_file,
                                                 &control_conf_))
      << "Unable to load control conf file: " + FLAGS_control_conf_file;

  AINFO << "Conf file: " << FLAGS_control_conf_file << " is loaded.";

  AdapterManager::Init(FLAGS_adapter_config_path);

  apollo::common::monitor::MonitorBuffer buffer(&monitor_);

  // set controller
  if (!controller_agent_.Init(&control_conf_).ok()) {
    std::string error_msg = "Control init controller failed! Stopping...";
    buffer.ERROR(error_msg);
    return Status(ErrorCode::CONTROL_INIT_ERROR, error_msg);
  }

  // lock it in case for after sub, init_vehicle not ready, but msg trigger
  // come
  CHECK(AdapterManager::GetLocalization())
      << "Localization is not initialized.";

  CHECK(AdapterManager::GetChassis()) << "Chassis is not initialized.";

  CHECK(AdapterManager::GetPlanning()) << "Planning is not initialized.";

  CHECK(AdapterManager::GetPad()) << "Pad is not initialized.";

  CHECK(AdapterManager::GetControlCommand())
      << "ControlCommand publisher is not initialized.";

  AdapterManager::AddPadCallback(&Control::OnPad, this);
  AdapterManager::AddMonitorCallback(&Control::OnMonitor, this);

  return Status::OK();
}

Status Control::Start() {
  // set initial vehicle state by cmd
  // need to sleep, because advertised channel is not ready immediately
  // simple test shows a short delay of 80 ms or so
  AINFO << "Control resetting vehicle state, sleeping for 1000 ms ...";
  usleep(1000 * 1000);

  // should init_vehicle first, let car enter work status, then use status msg
  // trigger control

  AINFO << "Control default driving action is "
        << DrivingAction_Name(control_conf_.action());
  pad_msg_.set_action(control_conf_.action());

  timer_ = AdapterManager::CreateTimer(
      ros::Duration(control_conf_.control_period()), &Control::OnTimer, this);

  AINFO << "Control init done!";

  apollo::common::monitor::MonitorBuffer buffer(&monitor_);
  buffer.INFO("control started");

  return Status::OK();
}

void Control::OnPad(const PadMessage &pad) {
  pad_msg_ = pad;
  ADEBUG << "Received Pad Msg:" << pad.DebugString();
  AERROR_IF(!pad_msg_.has_action()) << "pad message check failed!";

  // do something according to pad message
  if (pad_msg_.action() == DrivingAction::RESET) {
    AINFO << "Control received RESET action!";
    estop_ = false;
  }
  pad_received_ = true;
}

void Control::OnMonitor(
    const apollo::common::monitor::MonitorMessage &monitor_message) {
  for (const auto &item : monitor_message.item()) {
    if (item.log_level() == MonitorMessageItem::FATAL) {
      estop_ = true;
      return;
    }
  }
}

Status Control::ProduceControlCommand(ControlCommand *control_command) {
  Status status = CheckInput();
  // check data
  if (!status.ok()) {
    AERROR << "Control input data failed: " << status.error_message();
    estop_ = true;
  } else {
    Status status_ts = CheckTimestamp();
    if (!status_ts.ok()) {
      AERROR << "Input messages timeout";
      estop_ = true;
      status = status_ts;
    }
  }

  // check estop
  estop_ = estop_ || trajectory_.estop().is_estop();

  // if planning set estop, then no control process triggered
  if (!estop_) {
    if (chassis_.driving_mode() == Chassis::COMPLETE_MANUAL) {
      controller_agent_.Reset();
      AINFO << "Reset Controllers in Manual Mode";
    }

    auto debug = control_command->mutable_debug()->mutable_input_debug();
    debug->mutable_localization_header()->CopyFrom(localization_.header());
    debug->mutable_canbus_header()->CopyFrom(chassis_.header());
    debug->mutable_trajectory_header()->CopyFrom(trajectory_.header());

    Status status_compute = controller_agent_.ComputeControlCommand(
        &localization_, &chassis_, &trajectory_, control_command);

    if (!status_compute.ok()) {
      AERROR << "Control main function failed"
             << " with localization: " << localization_.ShortDebugString()
             << " with chassis: " << chassis_.ShortDebugString()
             << " with trajectory: " << trajectory_.ShortDebugString()
             << " with cmd: " << control_command->ShortDebugString()
             << " status:" << status_compute.error_message();
      estop_ = true;
      status = status_compute;
    }
  }

  if (estop_) {
    AWARN_EVERY(100) << "Estop triggered! No control core method executed!";
    // set Estop command
    control_command->set_speed(0);
    control_command->set_throttle(0);
    control_command->set_brake(control_conf_.soft_estop_brake());
    control_command->set_gear_location(Chassis::GEAR_DRIVE);
  }
  // check signal
  if (trajectory_.has_signal()) {
    control_command->mutable_signal()->CopyFrom(trajectory_.signal());
  }
  return status;
}

void Control::OnTimer(const ros::TimerEvent &) {
  double start_timestamp = Clock::NowInSecond();

  ControlCommand control_command;

  Status status = ProduceControlCommand(&control_command);
  if (!status.ok()) {
    AERROR << "Failed to produce control command:" << status.error_message();
  }

  double end_timestamp = Clock::NowInSecond();

  if (pad_received_) {
    control_command.mutable_pad_msg()->CopyFrom(pad_msg_);
    pad_received_ = false;
  }

  const double time_diff_ms = (end_timestamp - start_timestamp) * 1000;
  control_command.mutable_latency_stats()->set_total_time_ms(time_diff_ms);
  AINFO_EVERY(1000) << "control cycle time is: " << time_diff_ms << " ms.";
  status.Save(control_command.mutable_header()->mutable_status());

  SendCmd(&control_command);
}

Status Control::CheckInput() {
  AdapterManager::Observe();
  auto localization_adapter = AdapterManager::GetLocalization();
  if (localization_adapter->Empty()) {
    AWARN_EVERY(100) << "No Localization msg yet. ";
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "No localization msg");
  }
  localization_ = localization_adapter->GetLatestObserved();
  ADEBUG << "Received localization:" << localization_.ShortDebugString();

  auto chassis_adapter = AdapterManager::GetChassis();
  if (chassis_adapter->Empty()) {
    AWARN_EVERY(100) << "No Chassis msg yet. ";
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "No chassis msg");
  }
  chassis_ = chassis_adapter->GetLatestObserved();
  ADEBUG << "Received chassis:" << chassis_.ShortDebugString();

  auto trajectory_adapter = AdapterManager::GetPlanning();
  if (trajectory_adapter->Empty()) {
    AWARN_EVERY(100) << "No planning msg yet. ";
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "No planning msg");
  }
  trajectory_ = trajectory_adapter->GetLatestObserved();
  if (trajectory_.trajectory_point_size() == 0) {
    AWARN_EVERY(100) << "planning has no trajectory point. ";
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR,
                  "planning has no trajectory point.");
  }

  common::VehicleState::instance()->Update(localization_, chassis_);

  return Status::OK();
}

Status Control::CheckTimestamp() {
  if (!FLAGS_enable_input_timestamp_check || FLAGS_is_control_test_mode) {
    ADEBUG << "Skip input timestamp check by gflags.";
    return Status::OK();
  }
  double current_timestamp = Clock::NowInSecond();
  double localization_diff =
      current_timestamp - localization_.header().timestamp_sec();
  if (localization_diff >
      (FLAGS_max_localization_miss_num * control_conf_.localization_period())) {
    AERROR << "Localization msg lost for " << std::setprecision(6)
           << localization_diff << "s";
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "Localization msg timeout");
  }

  double chassis_diff = current_timestamp - chassis_.header().timestamp_sec();
  if (chassis_diff >
      (FLAGS_max_chassis_miss_num * control_conf_.chassis_period())) {
    AERROR << "Chassis msg lost for " << std::setprecision(6) << chassis_diff
           << "s";
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "Chassis msg timeout");
  }

  double trajectory_diff =
      current_timestamp - trajectory_.header().timestamp_sec();
  if (trajectory_diff >
      (FLAGS_max_planning_miss_num * control_conf_.trajectory_period())) {
    AERROR << "Trajectory msg lost for " << std::setprecision(6)
           << trajectory_diff << "s";
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR, "Trajectory msg timeout");
  }
  return Status::OK();
}

void Control::SendCmd(ControlCommand *control_command) {
  // set header
  AdapterManager::FillControlCommandHeader(Name(), control_command);

  ADEBUG << control_command->ShortDebugString();
  if (FLAGS_is_control_test_mode) {
    ADEBUG << "Skip publish control command in test mode";
    return;
  }
  AdapterManager::PublishControlCommand(*control_command);
}

void Control::Stop() {}

}  // namespace control
}  // namespace apollo
