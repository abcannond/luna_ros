#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

using std::placeholders::_1;

class DepthToPointCloud : public rclcpp::Node {
public:
  DepthToPointCloud()
  : Node("depth_to_pointcloud_node")
  {
    this->declare_parameter<std::string>("depth_topic", "/camera/camera/depth/image_rect_raw");
    this->declare_parameter<std::string>("camera_info_topic", "/camera/camera/depth/camera_info");
    this->declare_parameter<std::string>("points_topic", "/camera/camera/depth/color/points");
    this->declare_parameter<std::string>("frame_id", "");

    auto depth_topic = this->get_parameter("depth_topic").as_string();
    auto info_topic = this->get_parameter("camera_info_topic").as_string();
    auto points_topic = this->get_parameter("points_topic").as_string();
    frame_id_override_ = this->get_parameter("frame_id").as_string();

    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      depth_topic, 10,
      std::bind(&DepthToPointCloud::depth_callback, this, _1));

    info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      info_topic, 1,
      std::bind(&DepthToPointCloud::info_callback, this, _1));

    pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      points_topic, 10);

    H_ = 0;
    W_ = 0;

    RCLCPP_INFO(this->get_logger(),
      "DepthToPointCloud: %s + %s -> %s",
      depth_topic.c_str(), info_topic.c_str(), points_topic.c_str());
  }

private:
  void info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    fx_ = msg->k[0];
    fy_ = msg->k[4];
    cx_ = msg->k[2];
    cy_ = msg->k[5];
    H_ = msg->height;
    W_ = msg->width;
  }

  void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if(H_ == 0 || W_ == 0) return;

    const float* depth_data = reinterpret_cast<const float*>(msg->data.data());
    size_t num_points = H_ * W_;

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = msg->header;
    if (!frame_id_override_.empty()) {
      cloud.header.frame_id = frame_id_override_;
    }
    cloud.height = 1;
    cloud.width = num_points;
    cloud.is_dense = false;

    sensor_msgs::PointCloud2Modifier modifier(cloud);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(num_points);

    sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");

    for(size_t i = 0; i < num_points; ++i, ++iter_x, ++iter_y, ++iter_z) {
      float z = depth_data[i];
      if(std::isfinite(z)) {
        int row = i / W_;
        int col = i % W_;
        *iter_x = (col - cx_) * z / fx_;
        *iter_y = (row - cy_) * z / fy_;
        *iter_z = z;
      } else {
        *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
      }
    }

    pc_pub_->publish(cloud);
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;

  std::string frame_id_override_;
  double fx_, fy_, cx_, cy_;
  int H_, W_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DepthToPointCloud>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
