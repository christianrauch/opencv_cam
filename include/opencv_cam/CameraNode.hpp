// Copyright 2015 Open Source Robotics Foundation, Inc.
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

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/videoio.hpp>
#include <image_transport/image_transport.h>


// Node which captures images from a camera using OpenCV and publishes them.
// Images are annotated with this process's id as well as the message's ptr.
class CameraNode : public rclcpp::Node
{
public:
  CameraNode(
    const std::string & node_name = "camera_node",
    int device = 0, int width = 320, int height = 240);

  virtual ~CameraNode();

  void loop();

private:
  image_transport::CameraPublisher pub_cam;
  std::thread thread_;
  std::atomic<bool> canceled_;

  cv::VideoCapture cap_;
  cv::Mat frame_;
};

