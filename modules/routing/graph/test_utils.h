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

#ifndef MODULES_ROUTING_GRAPH_TEST_UTILS_H_
#define MODULES_ROUTING_GRAPH_TEST_UTILS_H_

#include <string>

#include "modules/routing/common/utils.h"
#include "modules/routing/graph/topo_graph.h"
#include "modules/routing/graph/topo_node.h"

#include "gtest/gtest.h"
#include "modules/routing/proto/topo_graph.pb.h"

namespace apollo {
namespace routing {

const std::string TEST_MAP_VERSION = "1.0.1";
const std::string TEST_MAP_DISTRICT = "";

const std::string TEST_L1 = "L1";
const std::string TEST_L2 = "L2";
const std::string TEST_L3 = "L3";
const std::string TEST_L4 = "L4";
const std::string TEST_L5 = "L5";
const std::string TEST_L6 = "L6";

const std::string TEST_R1 = "R1";
const std::string TEST_R2 = "R2";
const std::string TEST_R3 = "R3";

const double TEST_LANE_LENGTH = 100.0;
const double TEST_LANE_COST = 1.1;
const double TEST_EDGE_COST = 2.2;

const double TEST_START_S = 0.0;
const double TEST_MIDDLE_S = 0.0;
const double TEST_END_S = TEST_LANE_LENGTH;

void GetNodeDetailForTest(::apollo::routing::Node* const node,
                              const std::string& lane_id,
                              const std::string& road_id);

void GetNodeForTest(::apollo::routing::Node* const node,
                       const std::string& lane_id, const std::string& road_id);

void GetEdgeForTest(::apollo::routing::Edge* const edge,
                       const std::string& lane_id_1,
                       const std::string& lane_id_2,
                       const ::apollo::routing::Edge::DirectionType& type);

void GetGraphForTest(::apollo::routing::Graph* graph);

void GetGraph2ForTest(::apollo::routing::Graph* graph);

void GetGraph3ForTest(::apollo::routing::Graph* graph);

}  // namespace routing
}  // namespace apollo

#endif  // MODULES_ROUTING_GRAPH_TEST_UTILS_H_
