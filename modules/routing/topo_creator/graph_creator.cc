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

#include "modules/routing/topo_creator/graph_creator.h"

#include "glog/logging.h"

#include "modules/common/util/file.h"
#include "modules/routing/topo_creator/edge_creator.h"
#include "modules/routing/topo_creator/node_creator.h"

namespace apollo {
namespace routing {

using ::google::protobuf::RepeatedPtrField;
using ::apollo::hdmap::Id;
using ::apollo::routing::Node;
using ::apollo::routing::Edge;

GraphCreator::GraphCreator(const std::string& base_map_file_path,
                           const std::string& dump_topo_file_path)
    : base_map_file_path_(base_map_file_path),
      dump_topo_file_path_(dump_topo_file_path) {}

bool GraphCreator::Create() {
  if (!::apollo::common::util::GetProtoFromFile(base_map_file_path_, &pbmap_)) {
    AERROR << "Failed to load base map file from " << base_map_file_path_;
    return false;
  }
  AINFO << "Number of lanes: " << pbmap_.lane_size();

  graph_.set_hdmap_version(pbmap_.header().version());
  graph_.set_hdmap_district(pbmap_.header().district());

  node_index_map_.clear();
  road_id_map_.clear();
  showed_edge_id_set_.clear();

  for (const auto& road : pbmap_.road()) {
    for (const auto& section : road.section()) {
      for (const auto& lane_id : section.lane_id()) {
        road_id_map_[lane_id.id()] = road.id().id();
      }
    }
  }

  for (const auto& lane : pbmap_.lane()) {
    AINFO << "Current lane id: " << lane.id().id();
    node_index_map_[lane.id().id()] = graph_.node_size();
    const auto iter = road_id_map_.find(lane.id().id());
    if (iter != road_id_map_.end()) {
      NodeCreator::GetPbNode(lane, iter->second, graph_.add_node());
    } else {
      LOG(WARNING) << "Failed to find road id of lane " << lane.id().id();
      NodeCreator::GetPbNode(lane, "", graph_.add_node());
    }
  }

  std::string edge_id = "";
  for (const auto& lane : pbmap_.lane()) {
    const auto& from_node = graph_.node(node_index_map_[lane.id().id()]);
    AddEdge(from_node, lane.left_neighbor_forward_lane_id(), Edge::LEFT);
    AddEdge(from_node, lane.right_neighbor_forward_lane_id(), Edge::RIGHT);
    AddEdge(from_node, lane.successor_id(), Edge::FORWARD);
  }

  if (!::apollo::common::util::SetProtoToASCIIFile(graph_,
                                                   dump_topo_file_path_)) {
    AERROR << "Failed to dump topo data into file " << dump_topo_file_path_;
    return false;
  }
  AINFO << "File is dumped successfully. Path: " << dump_topo_file_path_;
  return true;
}

std::string GraphCreator::GetEdgeID(const std::string& from_id,
                                    const std::string& to_id) {
  return from_id + "->" + to_id;
}

void GraphCreator::AddEdge(const Node& from_node,
                           const RepeatedPtrField<Id>& to_node_vec,
                           const Edge::DirectionType& type) {
  std::string edge_id = "";
  for (const auto& to_id : to_node_vec) {
    edge_id = GetEdgeID(from_node.lane_id(), to_id.id());
    if (showed_edge_id_set_.count(edge_id) != 0) {
      continue;
    }
    showed_edge_id_set_.insert(edge_id);
    const auto& iter = node_index_map_.find(to_id.id());
    if (iter == node_index_map_.end()) {
      continue;
    }
    const auto& to_node = graph_.node(iter->second);
    EdgeCreator::GetPbEdge(from_node, to_node, type, graph_.add_edge());
  }
}

}  // namespace routing
}  // namespace apollo
