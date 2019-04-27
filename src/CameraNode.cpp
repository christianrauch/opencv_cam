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
  CameraNode(
    const rclcpp::NodeOptions options = rclcpp::NodeOptions(),
    const int device = 0, const int width = 320, const int height = 240)
  : Node("camera_node", rclcpp::NodeOptions(options).use_intra_process_comms(true)),
    canceled_(false)
  {
    // Initialize OpenCV
    cap_.open(device);
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(width));
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(height));
    if (!cap_.isOpened()) {
      throw std::runtime_error("Could not open video stream!");
    }
    // Create a publisher on the output topic.
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

    // Create the camera reading loop.
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
    // While running...
    while (rclcpp::ok() && !canceled_.load()) {
      // Capture a frame from OpenCV.
      cap_ >> frame_;
      if (frame_.empty()) {
        continue;
      }
      // Create a new unique_ptr to an Image message for storage.
      sensor_msgs::msg::Image::SharedPtr msg(new sensor_msgs::msg::Image());

      // Pack the OpenCV image into the ROS image.
      msg->header.stamp = now();
      msg->header.frame_id = "camera_frame";
      msg->height = frame_.rows;
      msg->width = frame_.cols;
      msg->encoding = cv_encoding.at(frame_.type());
      msg->is_bigendian = false;
      msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame_.step);
      msg->data.assign(frame_.datastart, frame_.dataend);

      sensor_msgs::msg::CameraInfo::SharedPtr ci(new sensor_msgs::msg::CameraInfo(ci_manager->getCameraInfo()));
      ci->header = msg->header;

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
