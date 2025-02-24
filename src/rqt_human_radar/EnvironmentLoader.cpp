// Copyright 2024 pal-robotics
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

#include <QFile>
#include <QTransform>

#include "rqt_human_radar/EnvironmentLoader.hpp"
#include "rqt_human_radar/SimScene.hpp"
#include "rqt_human_radar/LocalObjectItem.hpp"

#include "rqt_human_radar/SemanticObject.hpp"

namespace rqt_human_radar
{

void EnvironmentLoader::loadMap(
  rclcpp::Node::SharedPtr node, QGraphicsScene * scene,
  const std::string & filename)
{
  RCLCPP_INFO_STREAM(node->get_logger(), "Loading SVG file:" << filename);
  QFile file(QString::fromStdString(filename));
  if (!file.open(QIODevice::ReadOnly)) {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Could not open file:" << filename);
    return;
  }

  if (!doc_.setContent(&file)) {
    RCLCPP_ERROR_STREAM(node->get_logger(), "Could not parse file:" << filename);
    file.close();
    return;
  }

  file.close();

  renderer_.load(doc_.toString().toUtf8());


  // Retrieve all static objects, ie SVG elements that:
  // - are direct children of the SVG group with inkscape:label='static_objects'
  // - whose own inkscape:label does not start with '_'

  // get elementsByTag 'g' and only keep the one that has the label inkscape:label='static_objects' or 'zones'
  QDomElement static_objects;
  QDomElement zones_of_interest;
  QDomElement walls;
  QDomNodeList groups = doc_.elementsByTagName("g");

  for (int i = 0; i < groups.size(); i++) {
    QDomElement group = groups.at(i).toElement();
    if (group.attribute("inkscape:label") == "static_objects") {
      static_objects = group;
    }
    if (group.attribute("inkscape:label") == "zones") {
      zones_of_interest = group;
    }
    if (group.attribute("inkscape:label") == "walls") {
      walls = group;
    }
  }

  loadElements(node, static_objects, scene, ORO_OBJECT);
  loadElements(node, zones_of_interest, scene, ORO_ZONE_OF_INTEREST);
  loadElements(node, walls, scene, "");

}

void EnvironmentLoader::loadElements(
  rclcpp::Node::SharedPtr node,
  QDomElement & root, QGraphicsScene * scene,
  const std::string & default_class)
{

  auto viewBox = renderer_.viewBoxF();
  // (0,0) is the top-left corner of the SVG
  // we assume landscape orientation, with the robot at the middle of X=0 axis
  // as such:
  double yOrigin = viewBox.height() / 2;
  double xOrigin = 0;

  for (auto obj = root.firstChildElement();
    !obj.isNull();
    obj = obj.nextSiblingElement())
  {
    QString elementLabel = obj.attribute("inkscape:label");
    QString elementId = obj.attribute("id");
    if (!elementLabel.isEmpty() && !elementLabel.startsWith("_")) {
      QStringList parts = elementLabel.split(u' ');
      std::string name, classname;
      name = parts[0].toStdString();
      if (parts.size() >= 2) {
        classname = parts[1].toStdString();
      }

      auto bounds = getElementBounds(node, elementId.toStdString(), name);

      LocalObjectItem * item;


      if (default_class.empty()) {
        // if not default case, assume these SVG elements are
        // purely visual, and do not have any semantic meaning
        // in the simulation
        //auto svg_item = new QGraphicsSvgItem();
        //svg_item->setSharedRenderer(&renderer_);
        //svg_item->setElementId(elementId);
        //svg_item->setZValue(100);
        //scene->addItem(svg_item);
        //svg_item->setPos(
        //  (bounds.center().x() - xOrigin) / 1000 * SimScene::pixelsPerMeter,
        //  (bounds.center().y() - yOrigin) / 1000 * SimScene::pixelsPerMeter);
        auto svg_item = new SimItem(node);
        svg_item->setSharedRenderer(&renderer_);
        svg_item->setElementId(elementId);
        svg_item->setZValue(100);
        svg_item->setPhysicalWidth(bounds.width() / 1000);
        scene->addItem(svg_item);
        svg_item->setPos(
          (bounds.center().x() - xOrigin) / 1000 * SimScene::pixelsPerMeter,
          (bounds.center().y() - yOrigin) / 1000 * SimScene::pixelsPerMeter);

        continue;
      }
      if (default_class == ORO_ZONE_OF_INTEREST) {
        item = new LocalObjectItem(node, name, default_class);
        item->setStatic();
        item->setZValue(-100);
      } else {
        if (classname.empty()) {
          item = new LocalObjectItem(node, name, default_class);
        } else {
          item = new LocalObjectItem(node, name, classname);
        }
      }


      item->setSharedRenderer(&renderer_);
      item->setElementId(elementId);

      item->setPhysicalWidth(bounds.width() / 1000);

      scene->addItem(item);

      item->setPos(
        (bounds.center().x() - xOrigin) / 1000 * SimScene::pixelsPerMeter,
        (bounds.center().y() - yOrigin) / 1000 * SimScene::pixelsPerMeter);
    }
  }
}


QRectF EnvironmentLoader::getElementBounds(
  rclcpp::Node::SharedPtr node,
  const std::string & elementId, const std::string & label)
{
  if (!renderer_.elementExists(QString::fromStdString(elementId))) {
    RCLCPP_ERROR_STREAM(
      node->get_logger(),
      "Element <" << label << "> (ID:" << elementId << ") not found or has no renderable path.");
    return QRectF();
  }

  auto transform = renderer_.transformForElement(QString::fromStdString(elementId));
  auto bounds = transform.mapRect(renderer_.boundsOnElement(QString::fromStdString(elementId)));

  RCLCPP_INFO_STREAM(
    node->get_logger(),
    "Object <" << label << "> (SVG ID:" << elementId << "):" << bounds.x() << "mm, " <<
      bounds.y() << "mm, w=" << bounds.width() << "mm, h=" << bounds.height() << "mm");

  return bounds;
}


}  // namespace rqt_human_radar
