// cloud_merge_node.cpp
// Subscribes to 5x ToF point clouds, transforms each into base_link frame,
// concatenates them, and publishes a single fused PointCloud2.
//
// Subscriptions : /drone/tof_{0..4}/points  (sensor_msgs/PointCloud2)
// Publishes     : /drone/tof_merged/points  (sensor_msgs/PointCloud2, frame=base_link)

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl_ros/transforms.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using PC2 = sensor_msgs::msg::PointCloud2;
using ApproxSync5 = message_filters::sync_policies::ApproximateTime<
    PC2, PC2, PC2, PC2, PC2>;

class CloudMergeNode : public rclcpp::Node
{
public:
  CloudMergeNode() : Node("cloud_merge_node")
  {
    tf_buf_    = std::make_shared<tf2_ros::Buffer>(get_clock());
    tf_listen_ = std::make_shared<tf2_ros::TransformListener>(*tf_buf_);

    target_frame_ = declare_parameter<std::string>("target_frame", "base_link");
    tf_timeout_   = rclcpp::Duration::from_seconds(
                      declare_parameter<double>("tf_timeout_sec", 0.1));

    // Use Reliable QoS so octomap_server (which expects Reliable) can subscribe.
    // Individual /drone/tof_N/points remain BestEffort (sensor data).
    pub_ = create_publisher<PC2>("/drone/tof_merged/points",
             rclcpp::QoS(10).reliable().durability_volatile());

    auto qos = rclcpp::SensorDataQoS();
    for (int i = 0; i < 5; ++i) {
      sub_[i] = std::make_shared<message_filters::Subscriber<PC2>>(
          this, "/drone/tof_" + std::to_string(i) + "/points",
          qos.get_rmw_qos_profile());
    }

    sync_ = std::make_shared<message_filters::Synchronizer<ApproxSync5>>(
        ApproxSync5(20), *sub_[0], *sub_[1], *sub_[2], *sub_[3], *sub_[4]);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.05));
    sync_->registerCallback(std::bind(&CloudMergeNode::cb, this,
        std::placeholders::_1, std::placeholders::_2, std::placeholders::_3,
        std::placeholders::_4, std::placeholders::_5));

    RCLCPP_INFO(get_logger(), "CloudMergeNode ready — target frame: %s",
                target_frame_.c_str());
  }

private:
  void cb(const PC2::ConstSharedPtr& c0, const PC2::ConstSharedPtr& c1,
          const PC2::ConstSharedPtr& c2, const PC2::ConstSharedPtr& c3,
          const PC2::ConstSharedPtr& c4)
  {
    const std::array<const PC2::ConstSharedPtr, 5> clouds = {c0, c1, c2, c3, c4};
    pcl::PointCloud<pcl::PointXYZ> merged;

    for (const auto& cloud_msg : clouds) {
      // Look up transform: tof_N_link → base_link
      geometry_msgs::msg::TransformStamped tf_stamped;
      try {
        tf_stamped = tf_buf_->lookupTransform(
            target_frame_,
            cloud_msg->header.frame_id,
            rclcpp::Time(0, 0, RCL_ROS_TIME),  // latest available
            tf_timeout_);
      } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "TF lookup failed [%s]: %s",
            cloud_msg->header.frame_id.c_str(), ex.what());
        continue;
      }

      // pcl_ros::transformPointCloud uses Eigen under the hood — no tf2_sensor_msgs needed
      pcl::PointCloud<pcl::PointXYZ> pcl_in, pcl_transformed;
      pcl::fromROSMsg(*cloud_msg, pcl_in);

      // Build Eigen transform from stamped TF
      Eigen::Matrix4f mat = Eigen::Matrix4f::Identity();
      const auto& tr = tf_stamped.transform.translation;
      const auto& ro = tf_stamped.transform.rotation;
      Eigen::Quaternionf q(
          static_cast<float>(ro.w), static_cast<float>(ro.x),
          static_cast<float>(ro.y), static_cast<float>(ro.z));
      mat.block<3,3>(0,0) = q.toRotationMatrix();
      mat(0,3) = static_cast<float>(tr.x);
      mat(1,3) = static_cast<float>(tr.y);
      mat(2,3) = static_cast<float>(tr.z);

      pcl::transformPointCloud(pcl_in, pcl_transformed, mat);

      // Filter NaN and out-of-sensor-range points (A010: 0.2–3.0 m)
      for (const auto& pt : pcl_transformed) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z))
          continue;
        const float r = std::sqrt(pt.x*pt.x + pt.y*pt.y + pt.z*pt.z);
        if (r < 0.15f || r > 3.2f) continue;
        merged.push_back(pt);
      }
    }

    if (merged.empty()) return;

    PC2 out;
    pcl::toROSMsg(merged, out);
    // Use node clock instead of sensor stamp — octomap_server looks up
    // TF at this timestamp, and sensor stamps may lag the TF buffer.
    out.header.stamp    = this->get_clock()->now();
    out.header.frame_id = target_frame_;
    pub_->publish(out);
  }

  std::string      target_frame_;
  rclcpp::Duration tf_timeout_{0, 100'000'000};

  std::shared_ptr<tf2_ros::Buffer>            tf_buf_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listen_;
  rclcpp::Publisher<PC2>::SharedPtr           pub_;

  std::array<std::shared_ptr<message_filters::Subscriber<PC2>>, 5> sub_;
  std::shared_ptr<message_filters::Synchronizer<ApproxSync5>>       sync_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CloudMergeNode>());
  rclcpp::shutdown();
}