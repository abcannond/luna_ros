#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <Eigen/Dense>

using std::placeholders::_1;

class DepthToPointCloud : public rclcpp::Node {
public:
  DepthToPointCloud()
  : Node("depth_to_pointcloud_node")
  {
    depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/depth_camera/depth/image_raw", 10,
      std::bind(&DepthToPointCloud::depth_callback, this, _1));

    info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
      "/depth_camera/depth/camera_info", 1,
      std::bind(&DepthToPointCloud::info_callback, this, _1));

    pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/depth_camera/points", 10);

    H_ = 0;
    W_ = 0;
  }

private:
  void info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
    fx_ = msg->k[0];
    fy_ = msg->k[4];
    cx_ = msg->k[2];
    cy_ = msg->k[5];
    H_ = msg->height;
    W_ = msg->width;

    // Precompute pixel grids
    Eigen::VectorXd u = Eigen::VectorXd::LinSpaced(W_, 0, W_-1);
    Eigen::VectorXd v = Eigen::VectorXd::LinSpaced(H_, 0, H_-1);
    uu_ = u.replicate(1, H_).transpose();
    vv_ = v.replicate(1, W_);
  }

  void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    if(H_ == 0 || W_ == 0) return;  // wait for camera info

    const float* depth_data = reinterpret_cast<const float*>(msg->data.data());
    size_t num_points = H_ * W_;

    sensor_msgs::msg::PointCloud2 cloud;
    cloud.header = msg->header;
    cloud.header.frame_id = "base_link";
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

  Eigen::MatrixXd uu_, vv_;
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

// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/image.hpp>
// #include <sensor_msgs/msg/camera_info.hpp>
// #include <sensor_msgs/msg/point_cloud2.hpp>
// #include <sensor_msgs/point_cloud2_iterator.hpp>
// #include <cv_bridge/cv_bridge.h>
// #include <Eigen/Dense>
// #include <vector>
// #include <cmath>

// class DepthToPointCloud : public rclcpp::Node
// {
// public:
//     DepthToPointCloud()
//     : Node("depth_to_pointcloud_node"), camera_info_received_(false)
//     {
//         // Fixed transform from camera -> base_link
//         tx_ = 0.275; ty_ = 0.12; tz_ = 1.17;
//         roll_ = 0.0; pitch_ = 0.6; yaw_ = 0.0;
//         R_ = rotationMatrix(roll_, pitch_, yaw_);

//         // Publisher
//         pc_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/depth_camera/points", 10);

//         // Subscriptions
//         depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
//             "/depth_camera/depth/image_raw", 10,
//             std::bind(&DepthToPointCloud::depthCallback, this, std::placeholders::_1));

//         info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
//             "/depth_camera/depth/camera_info", 10,
//             std::bind(&DepthToPointCloud::infoCallback, this, std::placeholders::_1));
//     }

// private:
//     void infoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
//     {
//         if (!camera_info_received_)
//         {
//             camera_info_ = msg;
//             H_ = msg->height;
//             W_ = msg->width;
//             fx_ = msg->k[0];
//             fy_ = msg->k[4];
//             cx_ = msg->k[2];
//             cy_ = msg->k[5];

//             // Precompute pixel grid
//             uu_.resize(H_ * W_);
//             vv_.resize(H_ * W_);
//             for (size_t v = 0; v < H_; ++v)
//                 for (size_t u = 0; u < W_; ++u)
//                 {
//                     uu_[v * W_ + u] = u;
//                     vv_[v * W_ + u] = v;
//                 }

//             camera_info_received_ = true;
//             RCLCPP_INFO(this->get_logger(), "Camera info received, pixel grid precomputed.");
//         }
//     }

//     void depthCallback(const sensor_msgs::msg::Image::SharedPtr msg)
//     {
//         if (!camera_info_received_)
//             return;

//         cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
//         auto depth = cv_ptr->image;

//         std::vector<Eigen::Vector3f> points;
//         points.reserve(H_ * W_);

//         for (size_t idx = 0; idx < uu_.size(); ++idx)
//         {
//             float z = depth.at<float>(vv_[idx], uu_[idx]);
//             if (!std::isfinite(z) || z <= 0.0f)
//                 continue;

//             float x = (uu_[idx] - cx_) * z / fx_;
//             float y = (vv_[idx] - cy_) * z / fy_;
//             Eigen::Vector3f pt(x, y, z);
//             pt = R_ * pt + Eigen::Vector3f(tx_, ty_, tz_);
//             points.push_back(pt);
//         }

//         sensor_msgs::msg::PointCloud2 pc_msg;
//         pc_msg.header = msg->header;
//         pc_msg.header.frame_id = "base_link";
//         pc_msg.height = 1;
//         pc_msg.width = points.size();
//         pc_msg.is_dense = false;
//         pc_msg.is_bigendian = false;
//         pc_msg.fields = sensor_msgs::msg::PointField::get_point_fields();
//         pc_msg.point_step = 12;
//         pc_msg.row_step = pc_msg.point_step * pc_msg.width;
//         pc_msg.data.resize(points.size() * pc_msg.point_step);

//         float* data_ptr = reinterpret_cast<float*>(pc_msg.data.data());
//         for (auto& pt : points)
//         {
//             *data_ptr++ = pt.x();
//             *data_ptr++ = pt.y();
//             *data_ptr++ = pt.z();
//         }

//         pc_pub_->publish(pc_msg);
//     }

//     Eigen::Matrix3f rotationMatrix(float roll, float pitch, float yaw)
//     {
//         Eigen::Matrix3f Rx, Ry, Rz;
//         Rx << 1, 0, 0,
//               0, cos(roll), -sin(roll),
//               0, sin(roll), cos(roll);
//         Ry << cos(pitch), 0, sin(pitch),
//               0, 1, 0,
//               -sin(pitch), 0, cos(pitch);
//         Rz << cos(yaw), -sin(yaw), 0,
//               sin(yaw), cos(yaw), 0,
//               0, 0, 1;
//         return Rz * Ry * Rx;
//     }

//     bool camera_info_received_;
//     sensor_msgs::msg::CameraInfo::SharedPtr camera_info_;
//     size_t H_, W_;
//     float fx_, fy_, cx_, cy_;
//     std::vector<size_t> uu_, vv_;

//     float tx_, ty_, tz_;
//     float roll_, pitch_, yaw_;
//     Eigen::Matrix3f R_;

//     rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pc_pub_;
//     rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
//     rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
// };

// int main(int argc, char** argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<DepthToPointCloud>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }
