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
#ifndef MODULES_PERCEPTION_TRAFFIC_LIGHT_CLASSIFY_H
#define MODULES_PERCEPTION_TRAFFIC_LIGHT_CLASSIFY_H

#include <string>
#include <utility>
#include <vector>
#include "caffe/caffe.hpp"
#include "modules/perception/traffic_light/base/light.h"
#include "modules/perception/traffic_light/interface/green_interface.h"

namespace apollo {
namespace perception {
namespace traffic_light {

class ClassifyBySimple : public IRefine {
 public:
  ClassifyBySimple(const std::string &_class_net,
                   const std::string &_class_model, float threshold,
                   unsigned int resize_width, unsigned int resize_height);

  void Init(const std::string &_class_net, const std::string &_class_model,
            float threshold, unsigned int resize_width,
            unsigned int resize_height);

  virtual void Perform(const cv::Mat &ros_image, std::vector<LightPtr> *lights);

  void SetCropBox(const cv::Rect &box) override;

  ~ClassifyBySimple();

 private:
  void ProbToColor(const float *out_put_data, float threshold, LightPtr light);
  caffe::Net<float> *classify_net_ptr_;
  cv::Rect crop_box_;
  int resize_width_;
  int resize_height_;
  float unknown_threshold_;
};
}  // namespace traffic_light
}  // namespace perception
}  // namespace apollo
#endif
