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

#include <opencv_cam/CameraNode.hpp>
#include <chrono>
#include <string>
#include <thread>

std::string
mat_type2encoding(int mat_type)
{
  switch (mat_type) {
    case CV_8UC1:
      return "mono8";
    case CV_8UC3:
      return "bgr8";
    case CV_16SC1:
      return "mono16";
    case CV_8UC4:
      return "rgba8";
    default:
      throw std::runtime_error("Unsupported encoding type");
  }
}

CameraNode::CameraNode(
  rclcpp::NodeOptions options,
  const std::string & node_name,
  int device, int width, int height)
: Node(node_name, "camera", options.use_intra_process_comms(true)),
  canceled_(false)
{
  // Initialize OpenCV
  cap_.open(device);
  // TODO(jacobperron): Remove pre-compiler check when we drop support for Xenial
#if CV_MAJOR_VERSION < 3
  cap_.set(CV_CAP_PROP_FRAME_WIDTH, static_cast<double>(width));
  cap_.set(CV_CAP_PROP_FRAME_HEIGHT, static_cast<double>(height));
#else
  cap_.set(cv::CAP_PROP_FRAME_WIDTH, static_cast<double>(width));
  cap_.set(cv::CAP_PROP_FRAME_HEIGHT, static_cast<double>(height));
#endif
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

CameraNode::~CameraNode()
{
  // Make sure to join the thread on shutdown.
  canceled_.store(true);
  if (thread_.joinable()) {
    thread_.join();
  }
}

void CameraNode::loop()
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
    // msg->header.frame_id = "camera_frame";
    msg->height = frame_.rows;
    msg->width = frame_.cols;
    msg->encoding = mat_type2encoding(frame_.type());
    msg->is_bigendian = false;
    msg->step = static_cast<sensor_msgs::msg::Image::_step_type>(frame_.step);
    msg->data.assign(frame_.datastart, frame_.dataend);

    sensor_msgs::msg::CameraInfo::SharedPtr ci(new sensor_msgs::msg::CameraInfo(ci_manager->getCameraInfo()));
    ci->header = msg->header;

    std::stringstream ss;
    ss << "cam ptr: " << (void*)msg->data.data();
    msg->header.frame_id = ss.str();

    pub_cam.publish(msg, ci);
  }
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(CameraNode)
