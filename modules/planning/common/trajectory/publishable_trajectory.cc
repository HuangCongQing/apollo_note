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
 * @file publishable_trajectory.cpp
 **/

#include "modules/planning/common/trajectory/publishable_trajectory.h"

namespace apollo {
namespace planning {

using apollo::common::TrajectoryPoint;

TrajectoryPoint PublishableTrajectory::evaluate_absolute_time(
    const double abs_time) const {
  return Evaluate(abs_time - _header_time);
}

TrajectoryPoint
PublishableTrajectory::evaluate_linear_approximation_absolute_time(
    const double abs_time) const {
  return EvaluateUsingLinearApproximation(abs_time - _header_time);
}

std::uint32_t PublishableTrajectory::query_nearest_point_absolute_time(
    const double abs_time) const {
  return QueryNearestPoint(abs_time - _header_time);
}

double PublishableTrajectory::header_time() const { return _header_time; }

void PublishableTrajectory::set_header_time(const double header_time) {
  _header_time = header_time;
}

void PublishableTrajectory::populate_trajectory_protobuf(
    ADCTrajectory* trajectory_pb) const {
  trajectory_pb->mutable_header()->set_timestamp_sec(_header_time);
  trajectory_pb->mutable_trajectory_point()->CopyFrom(
      {_trajectory_points.begin(), _trajectory_points.end()});
}

}  // namespace planning
}  // namespace apollo
