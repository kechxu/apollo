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

#include "modules/prediction/prediction_component.h"

#include <algorithm>
#include <vector>

#include "cyber/common/file.h"
#include "cyber/record/record_reader.h"
#include "modules/common/adapters/adapter_gflags.h"
#include "modules/common/util/message_util.h"

#include "modules/prediction/common/feature_output.h"
#include "modules/prediction/common/junction_analyzer.h"
#include "modules/prediction/common/prediction_gflags.h"
#include "modules/prediction/common/prediction_system_gflags.h"
#include "modules/prediction/common/validation_checker.h"
#include "modules/prediction/evaluator/evaluator_manager.h"
#include "modules/prediction/predictor/predictor_manager.h"
#include "modules/prediction/proto/offline_features.pb.h"
#include "modules/prediction/scenario/scenario_manager.h"
#include "modules/prediction/util/data_extraction.h"

namespace apollo {
namespace prediction {

using apollo::common::adapter::AdapterConfig;
using apollo::common::time::Clock;
using apollo::localization::LocalizationEstimate;
using apollo::perception::PerceptionObstacle;
using apollo::perception::PerceptionObstacles;
using apollo::planning::ADCTrajectory;

PredictionComponent::~PredictionComponent() {}

std::string PredictionComponent::Name() const {
  return FLAGS_prediction_module_name;
}

void PredictionComponent::OfflineProcessFeatureProtoFile(
    const std::string& features_proto_file_name) {
  auto obstacles_container_ptr =
      ContainerManager::Instance()->GetContainer<ObstaclesContainer>(
          AdapterConfig::PERCEPTION_OBSTACLES);
  obstacles_container_ptr->Clear();
  Features features;
  apollo::cyber::common::GetProtoFromBinaryFile(features_proto_file_name,
                                                &features);
  for (const Feature& feature : features.feature()) {
    obstacles_container_ptr->InsertFeatureProto(feature);
    Obstacle* obstacle_ptr = obstacles_container_ptr->GetObstacle(feature.id());
    EvaluatorManager::Instance()->EvaluateObstacle(obstacle_ptr,
                                                   obstacles_container_ptr);
  }
}

bool PredictionComponent::Init() {
  component_start_time_ = Clock::NowInSeconds();

  if (!MessageProcess::Init()) {
    return false;
  }

  planning_reader_ = node_->CreateReader<ADCTrajectory>(
      FLAGS_planning_trajectory_topic, nullptr);

  localization_reader_ =
      node_->CreateReader<localization::LocalizationEstimate>(
          FLAGS_localization_topic, nullptr);

  storytelling_reader_ = node_->CreateReader<storytelling::Stories>(
      FLAGS_storytelling_topic, nullptr);

  prediction_writer_ =
      node_->CreateWriter<PredictionObstacles>(FLAGS_prediction_topic);

  container_writer_ =
      node_->CreateWriter<ContainerOutput>(FLAGS_container_topic_name);

  adc_container_writer_ = node_->CreateWriter<ADCTrajectoryContainer>(
      FLAGS_adccontainer_topic_name);

  return true;
}

bool PredictionComponent::Proc(
    const std::shared_ptr<PerceptionObstacles>& perception_message) {
  const double frame_start_time = Clock::NowInSeconds();
  // Read localization info. and call OnLocalization to update
  // the PoseContainer.
  localization_reader_->Observe();
  auto ptr_localization_msg = localization_reader_->GetLatestObserved();
  if (ptr_localization_msg == nullptr) {
    AERROR << "Prediction: cannot receive any localization message.";
    return false;
  }
  auto localization_msg = *ptr_localization_msg;
  MessageProcess::OnLocalization(localization_msg);

  // Read planning info. of last frame and call OnPlanning to update
  // the ADCTrajectoryContainer
  planning_reader_->Observe();
  auto ptr_trajectory_msg = planning_reader_->GetLatestObserved();
  if (ptr_trajectory_msg != nullptr) {
    auto trajectory_msg = *ptr_trajectory_msg;
    MessageProcess::OnPlanning(trajectory_msg);
  }
  MessageProcess::ContainerProcess(*perception_message);

  auto obstacles_container_ptr =
      ContainerManager::Instance()->GetContainer<ObstaclesContainer>(
          AdapterConfig::PERCEPTION_OBSTACLES);
  CHECK_NOTNULL(obstacles_container_ptr);

  auto adc_trajectory_container_ptr =
      ContainerManager::Instance()->GetContainer<ADCTrajectoryContainer>(
          AdapterConfig::PLANNING_TRAJECTORY);
  CHECK_NOTNULL(adc_trajectory_container_ptr);

  SubmoduleOutput submodule_output =
      obstacles_container_ptr->GetSubmoduleOutput();
  submodule_output.set_perception_header(perception_message->header());
  submodule_output.set_perception_error_code(perception_message->error_code());
  submodule_output.set_frame_start_time(frame_start_time);
  ContainerOutput container_output(std::move(submodule_output));
  container_writer_->Write(std::make_shared<ContainerOutput>(container_output));
  ADCTrajectoryContainer adc_container = *adc_trajectory_container_ptr;
  adc_container_writer_->Write(
      std::make_shared<ADCTrajectoryContainer>(adc_container));
  return true;
}

}  // namespace prediction
}  // namespace apollo
