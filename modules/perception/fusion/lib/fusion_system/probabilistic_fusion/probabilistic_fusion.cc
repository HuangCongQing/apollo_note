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
#include "modules/perception/fusion/lib/fusion_system/probabilistic_fusion/probabilistic_fusion.h"

#include <map>
#include <utility>

#include "cyber/common/file.h"
#include "modules/common/util/perf_util.h"
#include "modules/common/util/string_util.h"
#include "modules/perception/base/object_pool_types.h"
#include "modules/perception/fusion/base/base_init_options.h"
#include "modules/perception/fusion/base/track_pool_types.h"
#include "modules/perception/fusion/lib/data_association/hm_data_association/hm_tracks_objects_match.h"
#include "modules/perception/fusion/lib/data_fusion/existence_fusion/dst_existence_fusion/dst_existence_fusion.h"
#include "modules/perception/fusion/lib/data_fusion/tracker/pbf_tracker/pbf_tracker.h"
#include "modules/perception/fusion/lib/data_fusion/type_fusion/dst_type_fusion/dst_type_fusion.h"
#include "modules/perception/fusion/lib/gatekeeper/pbf_gatekeeper/pbf_gatekeeper.h"
#include "modules/perception/lib/config_manager/config_manager.h"
#include "modules/perception/proto/probabilistic_fusion_config.pb.h"

namespace apollo {
namespace perception {
namespace fusion {

using cyber::common::GetAbsolutePath;

ProbabilisticFusion::ProbabilisticFusion() {}

ProbabilisticFusion::~ProbabilisticFusion() {}

//  初始化
// modules/perception/production/data/perception/fusion/probabilistic_fusion.pt
bool ProbabilisticFusion::Init(const FusionInitOptions& init_options) {
  main_sensor_ = init_options.main_sensor; // 主sensor:lidar

  BaseInitOptions options;
  if (!GetFusionInitOptions("ProbabilisticFusion", &options)) {
    return false;
  }

  std::string work_root_config = GetAbsolutePath(
      lib::ConfigManager::Instance()->work_root(), options.root_dir);

  std::string config = GetAbsolutePath(work_root_config, options.conf_file);
  ProbabilisticFusionConfig params;

  if (!cyber::common::GetProtoFromFile(config, &params)) {
    AERROR << "Read config failed: " << config;
    return false;
  }
  params_.use_lidar = params.use_lidar();
  params_.use_radar = params.use_radar();
  params_.use_camera = params.use_camera();
  params_.tracker_method = params.tracker_method();
  params_.data_association_method = params.data_association_method();
  params_.gate_keeper_method = params.gate_keeper_method();
  for (int i = 0; i < params.prohibition_sensors_size(); ++i) {
    params_.prohibition_sensors.push_back(params.prohibition_sensors(i));
  }

  // static member initialization from PB config
  Track::SetMaxLidarInvisiblePeriod(params.max_lidar_invisible_period());
  Track::SetMaxRadarInvisiblePeriod(params.max_radar_invisible_period());
  Track::SetMaxCameraInvisiblePeriod(params.max_camera_invisible_period());
  Sensor::SetMaxCachedFrameNumber(params.max_cached_frame_num());

  scenes_.reset(new Scene()); //// 初始化用于管理场景的共享智能指针，场景中包含所有的前景航迹与背景航迹
  if (params_.data_association_method == "HMAssociation") {
    matcher_.reset(new HMTrackersObjectsAssociation());
  } else {
    AERROR << "Unknown association method: " << params_.data_association_method;
    return false;
  }
  if (!matcher_->Init()) {
    AERROR << "Failed to init matcher.";
    return false;
  }

  if (params_.gate_keeper_method == "PbfGatekeeper") {
    gate_keeper_.reset(new PbfGatekeeper());
  } else {
    AERROR << "Unknown gate keeper method: " << params_.gate_keeper_method;
    return false;
  }
  if (!gate_keeper_->Init()) {
    AERROR << "Failed to init gatekeeper.";
    return false;
  }

  bool state = DstTypeFusion::Init() && DstExistenceFusion::Init() &&
               PbfTracker::InitParams();

  return state;
}

// 概率融合算法(核心函数)===========================================================================
// 文档:https://www.yuque.com/huangzhongqing/crvg1o/qnfy09
// 一些类的简单定义：
// SensorObject：单个目标
// SensorFrame：一帧所有的目标
// Sensor：历史十帧数据
// SensorDataManager：所有传感器的历史十帧数据
bool ProbabilisticFusion::Fuse(const FusionOptions& options,
                               const base::FrameConstPtr& sensor_frame, //一帧数据
                               std::vector<base::ObjectPtr>* fused_objects) {
  if (fused_objects == nullptr) {
    AERROR << "fusion error: fused_objects is nullptr";
    return false;
  }

  // SensorDataManager：所有传感器的历史十帧数据
  auto* sensor_data_manager = SensorDataManager::Instance();
  // 1. save frame data 保存数据
  {
    std::lock_guard<std::mutex> data_lock(data_mutex_);
    // 三个if全是false
    if (!params_.use_lidar && sensor_data_manager->IsLidar(sensor_frame)) {
      return true;
    }
    if (!params_.use_radar && sensor_data_manager->IsRadar(sensor_frame)) {
      return true;
    }
    if (!params_.use_camera && sensor_data_manager->IsCamera(sensor_frame)) {
      return true;
    }

    // velodyne128
    bool is_publish_sensor = this->IsPublishSensor(sensor_frame); // 判断是不是main_sensor_
    if (is_publish_sensor) {
      started_ = true; // 主雷达设置为true
    }

    // 有lidar进来才开始save
    if (started_) {
      AINFO << "add sensor measurement: " << sensor_frame->sensor_info.name
            << ", obj_cnt : " << sensor_frame->objects.size() << ", "
            << FORMAT_TIMESTAMP(sensor_frame->timestamp);
      // 传感器数据管理，保存最新数据到一个map结构中，map为每个sensor对应的数据队列
      sensor_data_manager->AddSensorMeasurements(sensor_frame); // lidar/radar
    }

    // 不是主传感器就return
    if (!is_publish_sensor) {
      return true;
    }
  }

  // 2. query related sensor_frames for fusion
  // 查询所有传感器的最新一帧数据，插入到frames中，按时间顺序排序好。
  std::lock_guard<std::mutex> fuse_lock(fuse_mutex_);
  double fusion_time = sensor_frame->timestamp; // 得到时间戳
  std::vector<SensorFramePtr> frames; // 一帧所有的目标
  // 查询所有传感器的最新一帧数据，插入到frames中，按时间顺序排序好。============
  sensor_data_manager->GetLatestFrames(fusion_time, &frames); //vector<一帧所有的目标(分为前景和后景)>
  AINFO << "Get " << frames.size() << " related frames for fusion";

  // 3. perform fusion on related frames
  // 序贯滤波的定义，是对同一时刻的N个传感器按序号做序贯更新，做一遍预测后，N遍更新航迹
  // 按时间先后融合最新一帧，进行处理======
  for (const auto& frame : frames) { // lidar radar camera
    FuseFrame(frame); // 开始融合了!!!!
  }

  // 4. collect fused objects
  // 规则门限过滤，最新匹配更新过track的obj放入到fused_objects中，并publish
  CollectFusedObjects(fusion_time, fused_objects);
  return true;
}

std::string ProbabilisticFusion::Name() const { return "ProbabilisticFusion"; }

// 是不是主传感器:main_sensor_
bool ProbabilisticFusion::IsPublishSensor(
    const base::FrameConstPtr& sensor_frame) const {
  std::string sensor_id = sensor_frame->sensor_info.name;
  return sensor_id == main_sensor_;
  // const std::vector<std::string>& pub_sensors =
  //   params_.publish_sensor_ids;
  // const auto& itr = std::find(
  //   pub_sensors.begin(), pub_sensors.end(), sensor_id);
  // if (itr != pub_sensors.end()) {
  //   return true;
  // } else {
  //   return false;
  // }
}

// FuseFrame单帧融合的详细细节：主要是观测和航迹的关联匹配，航迹更新
// 6.1 FuseForegroundTrack函数
// 6.1.1 HMTrackersObjectsAssociation::Associate函数
// 6.1.2 UpdateAssignedTracks
// 6.1.3 UpdateUnassignedTracks
// 6.1.4 CreateNewTracks
// 6.2 FusebackgroundTrack函数
// 6.3 RemoveLostTrack函数
void ProbabilisticFusion::FuseFrame(const SensorFramePtr& frame) { // frame已经分好前景和后景(里面有单个sensor的障碍物)
  AINFO << "Fusing frame: " << frame->GetSensorId()
        << ", 前景foreground_object_number: "
        << frame->GetForegroundObjects().size()
        << ", 后景background_object_number: "
        << frame->GetBackgroundObjects().size()
        << ", timestamp: " << FORMAT_TIMESTAMP(frame->GetTimestamp());
  // 01 融合前景
  this->FuseForegroundTrack(frame);
  // 02 融合背景，主要是来自激光雷达的背景数据
  this->FusebackgroundTrack(frame);
  // 03 删除未更新的航迹
  this->RemoveLostTrack();
}

// 01 融合前景========
void ProbabilisticFusion::FuseForegroundTrack(const SensorFramePtr& frame) {
  PERF_BLOCK_START();
  std::string indicator = "fusion_" + frame->GetSensorId();

  // 1.1关联匹配--HMTrackersObjectsAssociation >>>>>>>>>
  // modules/perception/fusion/lib/data_association/hm_data_association/hm_tracks_objects_match.cc
  AssociationOptions options;
  AssociationResult association_result; // 得到association结果
  matcher_->Associate(options, frame, scenes_, &association_result); // 场景scenes_
  PERF_BLOCK_END_WITH_INDICATOR(indicator, "association");

  // 1.2更新匹配的航迹(四块更新)========================================
  const std::vector<TrackMeasurmentPair>& assignments =
      association_result.assignments;
  this->UpdateAssignedTracks(frame, assignments);
  PERF_BLOCK_END_WITH_INDICATOR(indicator, "update_assigned_track");

  // 1.3更新未匹配的航迹track
  const std::vector<size_t>& unassigned_track_inds =
      association_result.unassigned_tracks;
  this->UpdateUnassignedTracks(frame, unassigned_track_inds);
  PERF_BLOCK_END_WITH_INDICATOR(indicator, "update_unassigned_track");

  // 1.4未匹配上的obj量测新建航迹=====新建!
  const std::vector<size_t>& unassigned_obj_inds =
      association_result.unassigned_measurements;
  this->CreateNewTracks(frame, unassigned_obj_inds);
  PERF_BLOCK_END_WITH_INDICATOR(indicator, "create_track");
}

// 1.2更新匹配的航迹
// 匹配上的结果做更新，使用观测更新tracker，tracker类型是pbf_tracker，
void ProbabilisticFusion::UpdateAssignedTracks(
    const SensorFramePtr& frame,
    const std::vector<TrackMeasurmentPair>& assignments) {
  // Attention: match_distance should be used
  // in ExistenceFusion to calculate existence score.
  // We set match_distance to zero if track and object are matched,
  // which only has a small difference compared with actural match_distance
  TrackerOptions options;
  options.match_distance = 0;
  for (size_t i = 0; i < assignments.size(); ++i) {
    size_t track_ind = assignments[i].first; // ??
    size_t obj_ind = assignments[i].second; // ??
    // pbf_tracker,观测更新tracker-->data_fusion/tracker/pbf_tracker/pbf_tracker.cc
    trackers_[track_ind]->UpdateWithMeasurement(  // >>>>>>>modules/perception/fusion/lib/data_fusion/tracker/pbf_tracker/pbf_tracker.cc
        options, frame->GetForegroundObjects()[obj_ind], frame->GetTimestamp()); //更新tracker_状态
  }
}

// 1.3更新未匹配的航迹
void ProbabilisticFusion::UpdateUnassignedTracks(
    const SensorFramePtr& frame,
    const std::vector<size_t>& unassigned_track_inds) {
  // Attention: match_distance(min_match_distance) should be used
  // in ExistenceFusion to calculate toic score.
  // Due to it hasn't been used(mainly for front radar object pub in
  // gatekeeper),
  // we do not set match_distance temporarily.
  TrackerOptions options;
  options.match_distance = 0;
  std::string sensor_id = frame->GetSensorId();
  for (size_t i = 0; i < unassigned_track_inds.size(); ++i) {
    size_t track_ind = unassigned_track_inds[i];
    trackers_[track_ind]->UpdateWithoutMeasurement( // data_fusion/tracker/pbf_tracker/pbf_tracker.cc
        options, sensor_id, frame->GetTimestamp(), frame->GetTimestamp()); //更新tracker_状态
  }
}

// 1.4未匹配上的量测新建航迹tracker
// 对没有匹配到tracker的观测object，新建航迹tracker，主要是最后的两个Init函数。可以详细看下Track和BaseTracker两个类。
void ProbabilisticFusion::CreateNewTracks(
    const SensorFramePtr& frame,
    const std::vector<size_t>& unassigned_obj_inds) {
  for (size_t i = 0; i < unassigned_obj_inds.size(); ++i) {
    size_t obj_ind = unassigned_obj_inds[i];

    bool prohibition_sensor_flag = false;
    // 泛型，radar_front不新建航迹
    std::for_each(params_.prohibition_sensors.begin(),
                  params_.prohibition_sensors.end(),
                  [&](std::string sensor_name) {
                    if (sensor_name == frame->GetSensorId())
                      prohibition_sensor_flag = true;
                  });
    if (prohibition_sensor_flag) {
      continue;
    }
    // 新建track，并初始化,添加到scenes_中
    TrackPtr track = TrackPool::Instance().Get();
    track->Initialize(frame->GetForegroundObjects()[obj_ind]); //主要是最后的两个Init函数======================
    scenes_->AddForegroundTrack(track); //// 将新的前景航迹添加到场景的背景航迹列表中

    ADEBUG << "object id: "
           << frame->GetForegroundObjects()[obj_ind]->GetBaseObject()->track_id
           << ", create new track: " << track->GetTrackId();

    // PbfTracker：新建tracker，track初始化tracker，tracker插入到航迹集合trackers_中
    if (params_.tracker_method == "PbfTracker") {
      std::shared_ptr<BaseTracker> tracker;
      tracker.reset(new PbfTracker());
      tracker->Init(track, frame->GetForegroundObjects()[obj_ind]); //主要是最后的两个Init函数======================
      trackers_.emplace_back(tracker);
    }
  }
}

// 02 融合背景========
void ProbabilisticFusion::FusebackgroundTrack(const SensorFramePtr& frame) {
  // 1. association
  size_t track_size = scenes_->GetBackgroundTracks().size(); // // 所有的背景航迹
  size_t obj_size = frame->GetBackgroundObjects().size();
  std::map<int, size_t> local_id_2_track_ind_map;
  std::vector<bool> track_tag(track_size, false);
  std::vector<bool> object_tag(obj_size, false);
  std::vector<TrackMeasurmentPair> assignments;

  std::vector<TrackPtr>& background_tracks = scenes_->GetBackgroundTracks();
  for (size_t i = 0; i < track_size; ++i) {
    const FusedObjectPtr& obj = background_tracks[i]->GetFusedObject();
    int local_id = obj->GetBaseObject()->track_id;
    local_id_2_track_ind_map[local_id] = i;
  }

  // SensorObject：单个目标
  std::vector<SensorObjectPtr>& frame_objs = frame->GetBackgroundObjects();
  for (size_t i = 0; i < obj_size; ++i) {
    int local_id = frame_objs[i]->GetBaseObject()->track_id;
    const auto& it = local_id_2_track_ind_map.find(local_id);
    if (it != local_id_2_track_ind_map.end()) {
      size_t track_ind = it->second;
      assignments.push_back(std::make_pair(track_ind, i));
      track_tag[track_ind] = true;
      object_tag[i] = true;
      continue;
    }
  }

  // 2. update assigned track
  for (size_t i = 0; i < assignments.size(); ++i) {
    int track_ind = static_cast<int>(assignments[i].first);
    int obj_ind = static_cast<int>(assignments[i].second);
    background_tracks[track_ind]->UpdateWithSensorObject(frame_objs[obj_ind]);
  }

  // 3. update unassigned track
  std::string sensor_id = frame->GetSensorId();
  for (size_t i = 0; i < track_tag.size(); ++i) {
    if (!track_tag[i]) {
      background_tracks[i]->UpdateWithoutSensorObject(sensor_id,
                                                      frame->GetTimestamp());
    }
  }

  // 4. create new track
  for (size_t i = 0; i < object_tag.size(); ++i) {
    if (!object_tag[i]) {
      TrackPtr track = TrackPool::Instance().Get();
      track->Initialize(frame->GetBackgroundObjects()[i], true);
      scenes_->AddBackgroundTrack(track); //添加背景
    }
  }
}

// 03 删除未更新的航迹========
// 前景航迹和背景航迹，当该航迹所有匹配的传感器都没有更新过，移除掉该航迹
void ProbabilisticFusion::RemoveLostTrack() {
  // need to remove tracker at the same time
  size_t foreground_track_count = 0; // 存活的前景航迹计数，也代表了下一个存活的前景航迹的新的索引
  std::vector<TrackPtr>& foreground_tracks = scenes_->GetForegroundTracks(); // // 得到scenes的当前背景航迹
  for (size_t i = 0; i < foreground_tracks.size(); ++i) {
    // track里面所有匹配过的传感器是否存在
    // 不存在就删掉，不能直接erase？
    if (foreground_tracks[i]->IsAlive()) {  // 对于每一个前景航迹
      if (i != foreground_track_count) {  // 当前存活的前景航迹之前存在失活航迹
        foreground_tracks[foreground_track_count] = foreground_tracks[i]; // 将当前存活的前景航迹移动到前景航迹列表新的位置
        trackers_[foreground_track_count] = trackers_[i];// 将当前存活的前景航迹对应的 tracker 移动到 tracker 列表新的位置
      }
      foreground_track_count++; // 存活的前景航迹 数量
    }
  }
  AINFO << "Remove " << foreground_tracks.size() - foreground_track_count
        << " foreground tracks";
  foreground_tracks.resize(foreground_track_count); // 析构前景航迹列表尾部多余的元素
  trackers_.resize(foreground_track_count); // 析构 tracker 列表尾部多余的元素

  // only need to remove frame track
  size_t background_track_count = 0;
  std::vector<TrackPtr>& background_tracks = scenes_->GetBackgroundTracks();// 得到scenes的背景航迹
  for (size_t i = 0; i < background_tracks.size(); ++i) {
    if (background_tracks[i]->IsAlive()) {
      if (i != background_track_count) {
        background_tracks[background_track_count] = background_tracks[i];
      }
      background_track_count++;
    }
  }
  AINFO << "Remove " << background_tracks.size() - background_track_count
        << " background tracks";
  background_tracks.resize(background_track_count);
}

// 融合的
void ProbabilisticFusion::CollectFusedObjects(
    double timestamp, std::vector<base::ObjectPtr>* fused_objects) {
  fused_objects->clear();

  size_t fg_obj_num = 0;
  const std::vector<TrackPtr>& foreground_tracks =
      scenes_->GetForegroundTracks();
  for (size_t i = 0; i < foreground_tracks.size(); ++i) {
    if (gate_keeper_->AbleToPublish(foreground_tracks[i])) {
      this->CollectObjectsByTrack(timestamp, foreground_tracks[i],
                                  fused_objects);
      ++fg_obj_num;
    }
  }

  size_t bg_obj_num = 0;
  const std::vector<TrackPtr>& background_tracks =
      scenes_->GetBackgroundTracks();
  for (size_t i = 0; i < background_tracks.size(); ++i) {
    if (gate_keeper_->AbleToPublish(background_tracks[i])) {
      this->CollectObjectsByTrack(timestamp, background_tracks[i],
                                  fused_objects);
      ++bg_obj_num;
    }
  }

  AINFO << "collect objects : fg_obj_cnt = " << fg_obj_num
        << ", bg_obj_cnt = " << bg_obj_num
        << ", timestamp = " << FORMAT_TIMESTAMP(timestamp);
}

// 追踪的
void ProbabilisticFusion::CollectObjectsByTrack(
    double timestamp, const TrackPtr& track,
    std::vector<base::ObjectPtr>* fused_objects) {
  const FusedObjectPtr& fused_object = track->GetFusedObject(); // 融合障碍物结果
  base::ObjectPtr obj = base::ObjectPool::Instance().Get();
  *obj = *(fused_object->GetBaseObject());
  const SensorId2ObjectMap& lidar_measurements = track->GetLidarObjects();
  const SensorId2ObjectMap& radar_measurements = track->GetRadarObjects();
  const SensorId2ObjectMap& camera_measurements = track->GetCameraObjects();
  int num_measurements =
      static_cast<int>(lidar_measurements.size() + camera_measurements.size() +
                       radar_measurements.size());
  obj->fusion_supplement.on_use = true;
  std::vector<base::SensorObjectMeasurement>& measurements =
      obj->fusion_supplement.measurements;
  measurements.resize(num_measurements);
  int m_id = 0;
  for (auto it = lidar_measurements.begin(); it != lidar_measurements.end();
       ++it, m_id++) {
    base::SensorObjectMeasurement* measurement = &(measurements[m_id]);
    CollectSensorMeasurementFromObject(it->second, measurement);
  }
  for (auto it = camera_measurements.begin(); it != camera_measurements.end();
       ++it, m_id++) {
    base::SensorObjectMeasurement* measurement = &(measurements[m_id]);
    CollectSensorMeasurementFromObject(it->second, measurement);
  }
  for (auto it = radar_measurements.begin(); it != radar_measurements.end();
       ++it, m_id++) {
    base::SensorObjectMeasurement* measurement = &(measurements[m_id]);
    CollectSensorMeasurementFromObject(it->second, measurement);
  }

  obj->track_id = track->GetTrackId();
  obj->latest_tracked_time = timestamp;
  obj->tracking_time = track->GetTrackingPeriod();
  fused_objects->emplace_back(obj);
  ADEBUG << "fusion_reporting..." << obj->track_id << "@"
         << FORMAT_TIMESTAMP(timestamp) << "@(" << std::setprecision(10)
         << obj->center(0) << "," << obj->center(1) << ","
         << obj->center_uncertainty(0, 0) << ","
         << obj->center_uncertainty(0, 1) << ","
         << obj->center_uncertainty(1, 0) << ","
         << obj->center_uncertainty(1, 1) << "," << obj->velocity(0) << ","
         << obj->velocity(1) << "," << obj->velocity_uncertainty(0, 0) << ","
         << obj->velocity_uncertainty(0, 1) << ","
         << obj->velocity_uncertainty(1, 0) << ","
         << obj->velocity_uncertainty(1, 1) << ")";
}

// 得到SensorMeasurement测量信息(附加信息)
void ProbabilisticFusion::CollectSensorMeasurementFromObject(
    const SensorObjectConstPtr& object,
    base::SensorObjectMeasurement* measurement) {
  measurement->sensor_id = object->GetSensorId();
  measurement->timestamp = object->GetTimestamp();
  measurement->track_id = object->GetBaseObject()->track_id;
  measurement->center = object->GetBaseObject()->center;
  measurement->theta = object->GetBaseObject()->theta;
  measurement->size = object->GetBaseObject()->size;
  measurement->velocity = object->GetBaseObject()->velocity;
  measurement->type = object->GetBaseObject()->type;
  if (IsCamera(object)) {
    measurement->box = object->GetBaseObject()->camera_supplement.box;
  }
}

FUSION_REGISTER_FUSIONSYSTEM(ProbabilisticFusion);

}  // namespace fusion
}  // namespace perception
}  // namespace apollo
