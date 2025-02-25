// Copyright 2024 PAL Robotics S.L.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "SemanticObject.hpp"
#include "SimItem.hpp"

namespace rqt_human_radar
{

class ZoneOfInterestItem : public SimItem, public SemanticObject
{
  Q_OBJECT

public:
  enum { Type = UserType + 5 };

  ZoneOfInterestItem(
    rclcpp::Node::SharedPtr node, const std::string & name,
    const std::string & svg_file,
    bool randomize_id = false);

  ZoneOfInterestItem(
    rclcpp::Node::SharedPtr node, const std::string & name,
    bool randomize_id = false);

  ~ZoneOfInterestItem();

  QRectF boundingRect() const override;
  int type() const override {return Type;}

private:
  void init();

  rclcpp::Node::SharedPtr node_;
};

}  // namespace rqt_human_radar
