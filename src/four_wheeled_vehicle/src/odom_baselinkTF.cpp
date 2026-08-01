// # Copyright 2025 <Ming2zun:https://github.com/Ming2zun/Pure-tracking-slam-automatic-navigation-system>
// #                <喵了个水蓝蓝:https://www.bilibili.com/video/BV1kzEwzuEFw?spm_id_from=333.788.videopod.sections&vd_source=134c12873ff478ea447a06d652426f8f>
// #
// # Licensed under the Apache License, Version 2.0 (the "License");
// # you may not use this file except in compliance with the License.
// # You may obtain a copy of the License at
// #
// #     http://www.apache.org/licenses/LICENSE-2.0
// #
// # Unless required by applicable law or agreed to in writing, software
// # distributed under the License is distributed on an "AS IS" BASIS,
// # WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// # See the License for the specific language governing permissions and
// # limitations under the License.



#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

class OdomToBaseLinkTF : public rclcpp::Node {
public:
  OdomToBaseLinkTF() : Node("odom_to_baselink_tf")
  {
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom", 10,
      std::bind(&OdomToBaseLinkTF::odomCallback, this, std::placeholders::_1));

    tf_pub_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    RCLCPP_INFO(this->get_logger(), "Publishing dynamic TF: odom -> base_footprint");
  }

private:
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    geometry_msgs::msg::TransformStamped tf;

    tf.header.stamp = msg->header.stamp;
    tf.header.frame_id = "odom";
    tf.child_frame_id  = "base_footprint";

    tf.transform.translation.x = msg->pose.pose.position.x;
    tf.transform.translation.y = msg->pose.pose.position.y;
    tf.transform.translation.z = msg->pose.pose.position.z;

    tf.transform.rotation = msg->pose.pose.orientation;

    tf_pub_->sendTransform(tf);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomToBaseLinkTF>());
  rclcpp::shutdown();
  return 0;
}
