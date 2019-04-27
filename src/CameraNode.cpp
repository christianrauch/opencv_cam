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


#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/videoio.hpp>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>

#include <string>
#include <thread>

static const std::map<int, std::string> cv_encoding {
  {CV_8UC1, "mono8"},
  {CV_8UC3, "bgr8"},
  {CV_16SC1, "mono16"},
  {CV_8UC4, "rgba8"},
};

class CameraNode : public rclcpp::Node
{
public:
  CameraNode(const rclcpp::NodeOptions options = rclcpp::NodeOptions())
  : Node("camera_node", rclcpp::NodeOptions(options).use_intra_process_comms(true)),
    canceled_(false)
  {
    int device;
    get_parameter_or<int>("device", device, 0);

    int width, height;
    get_parameter_or<int>("width", width, 320);
    get_parameter_or<int>("height", height, 240);

    cap_.open(device, cv::CAP_ANY);
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(width));
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(height));

    if (!cap_.isOpened()) {
      throw std::runtime_error("Could not open video stream!");
    }

    pub_cam = image_transport::create_camera_publisher(this, "image", rmw_qos_profile_default);

    ci_manager = std::make_unique<camera_info_manager::CameraInfoManager>(this);

    std::string camera_info_url;
    if(! (get_parameter<std::string>("camera_info_url", camera_info_url) &&
          ci_manager->loadCameraInfo(camera_info_url)) )
    {
      // no configuration file provided or found, use default values
      sensor_msgs::msg::CameraInfo ci;
      ci.width = cap_.get(cv::CAP_PROP_FRAME_WIDTH);
      ci.height = cap_.get(cv::CAP_PROP_FRAME_HEIGHT);
      ci_manager->setCameraInfo(ci);
    }

    thread_ = std::thread(std::bind(&CameraNode::loop, this));
  }

  virtual ~CameraNode()
  {
    // Make sure to join the thread on shutdown.
    canceled_.store(true);
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  void loop()
  {
    while (rclcpp::ok() && !canceled_.load()) {
      cap_ >> frame_;

      if (frame_.empty()) { continue; }

      sensor_msgs::msg::CameraInfo::SharedPtr ci(new sensor_msgs::msg::CameraInfo(ci_manager->getCameraInfo()));
      ci->header.stamp = now();
      ci->header.frame_id = "camera_frame";

      sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(ci->header, cv_encoding.at(frame_.type()), frame_).toImageMsg();

      pub_cam.publish(msg, ci);
    }
  }

private:
  image_transport::CameraPublisher pub_cam;
  std::unique_ptr<camera_info_manager::CameraInfoManager> ci_manager;
  std::thread thread_;
  std::atomic<bool> canceled_;

  cv::VideoCapture cap_;
  cv::Mat frame_;
};

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(CameraNode)
