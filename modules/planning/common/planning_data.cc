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

/**
 * @file planning_data.cc
 **/

#include "modules/planning/common/planning_data.h"

#include <utility>

#include "modules/common/log.h"
#include "modules/common/util/string_util.h"
#include "modules/planning/common/path/path_data.h"

namespace apollo {
namespace planning {

const DecisionData& PlanningData::decision_data() const {
  return *decision_data_.get();
}

DecisionData* PlanningData::mutable_decision_data() const {
  return decision_data_.get();
}

void PlanningData::set_decision_data(
    std::shared_ptr<DecisionData>& decision_data) {
  decision_data_ = decision_data;
}

const PathData& PlanningData::path_data() const { return path_data_; }

const SpeedData& PlanningData::speed_data() const { return speed_data_; }

PathData* PlanningData::mutable_path_data() { return &path_data_; }

SpeedData* PlanningData::mutable_speed_data() { return &speed_data_; }

bool PlanningData::aggregate(const double time_resolution,
                             const double relative_time,
                             PublishableTrajectory* trajectory) {
  CHECK(time_resolution > 0.0);
  CHECK(trajectory != nullptr);

  for (double cur_rel_time = 0.0; cur_rel_time < speed_data_.total_time();
       cur_rel_time += time_resolution) {
    common::SpeedPoint speed_point;
    if (!speed_data_.get_speed_point_with_time(cur_rel_time, &speed_point)) {
      AERROR << "Fail to get speed point with relative time " << cur_rel_time;
      return false;
    }

    if (speed_point.s() > path_data_.discretized_path().length()) {
      break;
    }
    common::PathPoint path_point;
    if (!path_data_.get_path_point_with_path_s(speed_point.s(), &path_point)) {
      AERROR << "Fail to get path data with s " << speed_point.s()
             << "path total length " << path_data_.discretized_path().length();
      return false;
    }

    common::TrajectoryPoint trajectory_point;
    trajectory_point.mutable_path_point()->CopyFrom(path_point);
    trajectory_point.set_v(speed_point.v());
    trajectory_point.set_a(speed_point.a());
    trajectory_point.set_relative_time(speed_point.t() + relative_time);
    trajectory->append_trajectory_point(trajectory_point);
  }
  return true;
}

std::string PlanningData::DebugString() const {
  return apollo::common::util::StrCat("path_data:", path_data_.DebugString(),
                                      "speed_data:", speed_data_.DebugString());
}

}  // namespace planning
}  // namespace apollo
