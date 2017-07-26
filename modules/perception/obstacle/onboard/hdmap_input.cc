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

#include "modules/perception/obstacle/onboard/hdmap_input.h"
#include <stdlib.h>
#include <algorithm>
#include <vector>

#include "modules/common/log.h"
#include "modules/perception/common/define.h"
#include "modules/perception/common/perception_gflags.h"

namespace apollo {
namespace perception {

bool HDMapInput::Init() {
  inited_ = true;
  return true;
}

bool HDMapInput::GetROI(const pcl_util::PointD& pointd, HdmapStructPtr mapptr) {
  return true;
}

/*
using apollo::hdmap::LineSegment;
using apollo::hdmap::BoundaryEdge;
using apollo::hdmap::RoadBoundaryPtr;
using apollo::hdmap::JunctionInfoConstPtr;
using apollo::hdmap::BoundaryEdge_Type_LEFT_BOUNDARY;
using apollo::hdmap::BoundaryEdge_Type_RIGHT_BOUNDARY;
using apollo::common::math::Vec2d;
using apollo::common::math::Polygon2d;
using pcl_util::PointD;
using pcl_util::PointDCloud;
using pcl_util::PointDCloudPtr;
using std::string;
using std::vector;

// HDMapInput
bool HDMapInput::Init() {
  std::unique_lock<std::mutex> lock(mutex_);
  if (inited_) {
    return true;
  }

  hdmap_.reset(new apollo::hdmap::HDMap());
  if (hdmap_->load_map_from_file(FLAGS_map_file) != SUCC) {
    AERROR << "Failed to load map file: " << FLAGS_map_file;
    inited_ = false;
    return false;
  }

  AINFO << "Init HDMap successfully, "
        << "load map file: " << FLAGS_map_file;
  inited_ = true;
  return true;
}

bool HDMapInput::GetROI(const PointD& pointd, HdmapStructPtr mapptr) {
  std::unique_lock<std::mutex> lock(mutex_);
  if (mapptr == nullptr) {
    mapptr.reset(new HdmapStruct);
  }
  apollo::hdmap::Point point;
  point.set_x(pointd.x);
  point.set_y(pointd.y);
  point.set_z(pointd.z);
  std::vector<RoadBoundaryPtr> boundary_vec;
  std::vector<JunctionInfoConstPtr> junctions_vec;

  int status = hdmap_->get_road_boundaries(point, FLAGS_map_radius,
                                           &boundary_vec, &junctions_vec);
  if (status != SUCC) {
    AERROR << "Failed to get road boundaries for point " << point.DebugString();
    return false;
  }
  AINFO << "Get road boundaries : num_boundary = " << boundary_vec.size()
        << " num_junction = " << junctions_vec.size();

  if (MergeBoundaryJunction(boundary_vec, junctions_vec, mapptr) != SUCC) {
    AERROR << "merge boundary and junction to hdmap struct failed.";
    return false;
  }
  return true;
}

int HDMapInput::MergeBoundaryJunction(
    std::vector<RoadBoundaryPtr>& boundaries,
    std::vector<JunctionInfoConstPtr>& junctions, HdmapStructPtr mapptr) {
  if (mapptr == nullptr) {
    AERROR << "the HdmapStructPtr mapptr is null";
    return FAIL;
  }
  mapptr->road_boundary.resize(boundaries.size());
  mapptr->junction.resize(junctions.size());

  for (size_t i = 0; i < boundaries.size(); i++) {
    for (size_t rbi = 0; rbi < boundaries[i]->road_boundary.size(); ++rbi) {
      const apollo::hdmap::RoadBoundary& kRoadBoundary =
          boundaries[i]->road_boundary[rbi];
      if (kRoadBoundary.has_outer_polygon() &&
          kRoadBoundary.outer_polygon().edge_size() > 0) {
        for (int j = 0; j < kRoadBoundary.outer_polygon().edge_size(); ++j) {
          const BoundaryEdge& edge = kRoadBoundary.outer_polygon().edge(j);
          if (edge.type() == BoundaryEdge_Type_LEFT_BOUNDARY &&
              edge.has_curve()) {
            for (int k = 0; k < edge.curve().segment_size(); ++k) {
              if (edge.curve().segment(k).has_line_segment()) {
                DownSampleBoundary(edge.curve().segment(k).line_segment(),
                                   &(mapptr->road_boundary[i].left_boundary));
              }
            }
          } else if (edge.type() == BoundaryEdge_Type_RIGHT_BOUNDARY &&
                     edge.has_curve()) {
            for (int k = 0; k < edge.curve().segment_size(); ++k) {
              if (edge.curve().segment(k).has_line_segment()) {
                DownSampleBoundary(edge.curve().segment(k).line_segment(),
                                   &(mapptr->road_boundary[i].right_boundary));
              }
            }
          }
        }
      }
    }
  }

  for (size_t i = 0; i < junctions.size(); i++) {
    const Polygon2d& polygon = junctions[i]->polygon();
    const vector<Vec2d>& points = polygon.points();
    mapptr->junction[i].reserve(points.size());
    for (size_t idj = 0; idj < points.size(); ++idj) {
      PointD pointd;
      pointd.x = points[idj].x();
      pointd.y = points[idj].y();
      pointd.z = 0.0;
      mapptr->junction[i].push_back(pointd);
    }
  }

  return SUCC;
}

void HDMapInput::DownSampleBoundary(const apollo::hdmap::LineSegment& line,
                                    PolygonDType* out_boundary_line) const {
  PointDCloudPtr raw_cloud(new PointDCloud);
  for (int i = 0; i < line.point_size(); ++i) {
    if (i % FLAGS_map_sample_step == 0) {
      PointD pointd;
      pointd.x = line.point(i).x();
      pointd.y = line.point(i).y();
      pointd.z = line.point(i).z();
      raw_cloud->push_back(pointd);
    }
  }

  const double kCumulateThetaError = 5.0;
  size_t spt = 0;
  double acos_theta = 0.0;
  size_t raw_cloud_size = raw_cloud->points.size();
  if (raw_cloud_size <= 3) {
    AINFO << "Points num < 3, so no need to downsample.";
    return;
  }
  out_boundary_line->points.reserve(out_boundary_line->points.size() +
                                    raw_cloud_size);
  // the first point
  out_boundary_line->push_back(raw_cloud->points[0]);
  for (size_t i = 2; i < raw_cloud_size; ++i) {
    const PointD& point_0 = raw_cloud->points[spt];
    const PointD& point_1 = raw_cloud->points[i - 1];
    const PointD& point_2 = raw_cloud->points[i];
    common::math::Vec2d v1(point_1.x - point_0.x, point_1.y - point_0.y);
    common::math::Vec2d v2(point_2.x - point_1.x, point_2.y - point_1.y);
    double vector_dist =
        sqrt(v1.cwiseProduct(v1).sum()) * sqrt(v2.cwiseProduct(v2).sum());
    // judge duplicate points
    if (vector_dist < DBL_EPSILON) {
      continue;
    }
    double cos_theta = (v1.cwiseProduct(v2)).sum() / vector_dist;
    if (cos_theta > 1.0) {
      cos_theta = 1.0;
    } else if (cos_theta < -1.0) {
      cos_theta = -1.0;
    }
    double angle = (acos(cos_theta) * kRadianToDegree);
    acos_theta += angle;
    if ((acos_theta - kCumulateThetaError) > DBL_EPSILON) {
      out_boundary_line->push_back(point_1);
      spt = i - 1;
      acos_theta = 0.0;
    }
  }
  // the last point
  out_boundary_line->push_back(raw_cloud->points[raw_cloud_size - 1]);
}
*/

}  // namespace perception
}  // namespace apollo
