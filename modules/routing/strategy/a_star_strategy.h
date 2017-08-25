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

#ifndef MODULES_ROUTING_STRATEGY_A_STAR_STRATEGY_H_
#define MODULES_ROUTING_STRATEGY_A_STAR_STRATEGY_H_

#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "modules/routing/strategy/strategy.h"

namespace apollo {
namespace routing {

class AStarStrategy : public Strategy {
 public:
  AStarStrategy() = default;
  ~AStarStrategy() = default;

  virtual bool Search(const TopoGraph* graph, const TopoNode* src_node,
                      const TopoNode* dest_node,
                      const std::unordered_set<const TopoNode*>& black_list,
                      std::vector<const TopoNode*>* const result_nodes);

 private:
  void Clear();
  double HeuristicCost(const TopoNode* src_node, const TopoNode* dest_node);

 private:
  std::unordered_set<const TopoNode*> open_set_;
  std::unordered_set<const TopoNode*> closed_set_;
  std::unordered_map<const TopoNode*, const TopoNode*> came_from_;
  std::unordered_map<const TopoNode*, double> g_score_;
};

}  // namespace routing
}  // namespace apollo

#endif  // MODULES_ROUTING_STRATEGY_A_STAR_STRATEGY_H_
