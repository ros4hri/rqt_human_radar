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

#include <QGraphicsScene>
#include <QGraphicsSceneMouseEvent>

#include "rqt_human_radar/ZoneOfInterestItem.hpp"
#include "rqt_human_radar/SimScene.hpp"

using namespace std::chrono_literals;

namespace rqt_human_radar
{

ZoneOfInterestItem::ZoneOfInterestItem(
  rclcpp::Node::SharedPtr node,
  const std::string & name,
  const std::string & svg_file,
  bool randomize_id)
: SimItem(node, svg_file),
  SemanticObject(node, name, ORO_ZONE_OF_INTEREST, randomize_id),
  node_(node)
{
  init();
}

ZoneOfInterestItem::ZoneOfInterestItem(
  rclcpp::Node::SharedPtr node,
  const std::string & name,
  bool randomize_id)
: SimItem(node),
  SemanticObject(node, name, ORO_ZONE_OF_INTEREST, randomize_id),
  node_(node)
{
  init();
}

void ZoneOfInterestItem::init()
{
  setLabel(id_);
  setZValue(-100);

  setPhysicalWidth(0.2);   // 20cm

  auto font = label_->font();
  font.setStyle(QFont::Style::StyleItalic);
  font.setPointSize(14);
  label_->setFont(font);
  label_->setOpacity(0.4);
}

ZoneOfInterestItem::~ZoneOfInterestItem()
{
}

QRectF ZoneOfInterestItem::boundingRect() const
{
  auto bb = SimItem::boundingRect();

  label_->setPos(
    -bb.width() / 2 + 0.1 * SimScene::pixelsPerMeter,
    bb.height() / 2 - 0.25 * SimScene::pixelsPerMeter);

  return bb;
}


}  // namespace rqt_human_radar
