#include "drone/perception/detection_adapter.hpp"

#include <vision_msgs/msg/detection2_d.hpp>
#include <vision_msgs/msg/object_hypothesis_with_pose.hpp>

namespace drone
{

namespace
{

std::string class_id_to_string(int id)
{
  switch (id) {
    case 0:
      return "circle";
    case 1:
      return "stuffed";
    case 2:
      return "h";
    default:
      return std::to_string(id);
  }
}

}  // namespace

vision_msgs::msg::Detection2DArray to_detection2d_array(
  const std::vector<Detection> & detections, const rclcpp::Time & stamp,
  const std::string & frame_id)
{
  vision_msgs::msg::Detection2DArray msg;
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;

  for (const auto & det : detections) {
    vision_msgs::msg::Detection2D d;
    d.header = msg.header;

    // Bounding box
    d.bbox.center.position.x = det.cx;
    d.bbox.center.position.y = det.cy;
    d.bbox.size_x = det.w;
    d.bbox.size_y = det.h;

    // Classification result
    vision_msgs::msg::ObjectHypothesisWithPose hyp;
    hyp.hypothesis.class_id = class_id_to_string(det.class_id);
    hyp.hypothesis.score = det.confidence;
    d.results.push_back(hyp);

    msg.detections.push_back(d);
  }

  return msg;
}

}  // namespace drone
