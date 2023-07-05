// Copyright 2023 Meta-Team
#include "solais_camera/solais_ip_camera_node.hpp"

namespace solais_camera
{
IPCameraNode::IPCameraNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("ip_camera", options)
{
  auto device_url = this->declare_parameter("rtsp_url", "rtsp://172.19.192.1:554/live");
  _camera = std::make_shared<IPCamera>(device_url, params_);
  _server = std::make_shared<CameraServer>(std::shared_ptr<rclcpp::Node>(this), _camera, params_);
}
}  // namespace solais_camera

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(solais_camera::IPCameraNode)
