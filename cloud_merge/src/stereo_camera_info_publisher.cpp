// Workaround for Gazebo Harmonic not emitting CameraInfo for camera sensors.
// Publishes static intrinsics for the stereo IR pair so cuVSLAM can initialize.

#include <cmath>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

class StereoCameraInfoPublisher : public rclcpp::Node {
public:
  StereoCameraInfoPublisher() : Node("stereo_camera_info_publisher") {
    const int width      = declare_parameter("width", 1280);
    const int height     = declare_parameter("height", 720);
    const double hfov    = declare_parameter("hfov_rad", 1.134);
    const double baseline = declare_parameter("baseline_m", 0.055);
    const double rate_hz = declare_parameter("publish_rate_hz", 30.0);

    const std::string left_topic  = declare_parameter("left_topic",  std::string("/drone/stereo/left/camera_info"));
    const std::string right_topic = declare_parameter("right_topic", std::string("/drone/stereo/right/camera_info"));
    const std::string left_frame  = declare_parameter("left_frame",  std::string("stereo_left_cam_link"));
    const std::string right_frame = declare_parameter("right_frame", std::string("stereo_right_cam_link"));

    const double fx = width / (2.0 * std::tan(hfov / 2.0));
    const double fy = fx;  // square pixels
    const double cx = width / 2.0;
    const double cy = height / 2.0;

    left_msg_  = makeCameraInfo(width, height, fx, fy, cx, cy, 0.0, left_frame);
    right_msg_ = makeCameraInfo(width, height, fx, fy, cx, cy, -fx * baseline, right_frame);

    auto qos = rclcpp::QoS(10).reliable();
    pub_left_  = create_publisher<sensor_msgs::msg::CameraInfo>(left_topic, qos);
    pub_right_ = create_publisher<sensor_msgs::msg::CameraInfo>(right_topic, qos);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_hz),
      [this]() {
        const auto now = this->now();
        left_msg_.header.stamp = now;
        right_msg_.header.stamp = now;
        pub_left_->publish(left_msg_);
        pub_right_->publish(right_msg_);
      });

    RCLCPP_INFO(get_logger(),
      "Publishing stereo camera_info: %dx%d, fx=%.1f, baseline=%.3fm, rate=%.1fHz",
      width, height, fx, baseline, rate_hz);
  }

private:
  static sensor_msgs::msg::CameraInfo makeCameraInfo(
      int w, int h, double fx, double fy, double cx, double cy,
      double Tx, const std::string& frame_id) {
    sensor_msgs::msg::CameraInfo msg;
    msg.header.frame_id = frame_id;
    msg.width  = static_cast<uint32_t>(w);
    msg.height = static_cast<uint32_t>(h);
    msg.distortion_model = "plumb_bob";
    msg.d = {0.0, 0.0, 0.0, 0.0, 0.0};
    msg.k = {fx, 0.0, cx,
             0.0, fy, cy,
             0.0, 0.0, 1.0};
    msg.r = {1.0, 0.0, 0.0,
             0.0, 1.0, 0.0,
             0.0, 0.0, 1.0};
    msg.p = {fx, 0.0, cx, Tx,
             0.0, fy, cy, 0.0,
             0.0, 0.0, 1.0, 0.0};
    return msg;
  }

  sensor_msgs::msg::CameraInfo left_msg_;
  sensor_msgs::msg::CameraInfo right_msg_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_left_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr pub_right_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StereoCameraInfoPublisher>());
  rclcpp::shutdown();
  return 0;
}
