#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_cloud.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <message_filters/subscriber.h>
#include <tf2_ros/message_filter.h>
#include <tf2_ros/create_timer_ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <std_srvs/srv/empty.hpp>
#include <fstream>

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

class ElevationMappingNode : public rclcpp::Node
{
public:
  using SharedPtr = std::shared_ptr<ElevationMappingNode>;

  ElevationMappingNode()
  : rclcpp::Node("elevation_mapping_node"),
    map_center_(0.5, 0.0),
    map_widths_(1.0, 1.0),
    resolution_(0.005)
  {
    map_lim_min_ = map_center_ - 0.5 * map_widths_;
    map_lim_max_ = map_center_ + 0.5 * map_widths_;

    grid_dim_.x() = std::round(map_widths_.x() / resolution_) + 1;
    grid_dim_.y() = std::round(map_widths_.y() / resolution_) + 1;

    grid_ = new float[grid_dim_.x() * grid_dim_.y()];
    sigma_ = new float[grid_dim_.x() * grid_dim_.y()];

    memset(grid_, -1.f, grid_dim_.x() * grid_dim_.y() * sizeof(float));
    memset(sigma_, 1e1f, grid_dim_.x() * grid_dim_.y() * sizeof(float));
    
    // Initialize service
    save_map_srv_ = create_service<std_srvs::srv::Empty>(
      "save_map", std::bind(&ElevationMappingNode::save_map, this, _1, _2));

    // Initialize publisher
    elevation_map_msg_.header.frame_id = world_frame_id_;
    elevation_map_msg_.points.resize(grid_dim_.x() * grid_dim_.y());
    for (int i = 0; i < grid_dim_.x(); ++i) {
      for (int j = 0; j < grid_dim_.y(); ++j) {
        const int idx = i * grid_dim_.y() + j;
        elevation_map_msg_.points[idx].x = map_lim_min_.x() + i * resolution_;
        elevation_map_msg_.points[idx].y = map_lim_min_.y() + j * resolution_;
      }
    }
    map_pub_ = create_publisher<sensor_msgs::msg::PointCloud>("elevation_map_pts", 10);

    // Initialize tf2 buffer and listener
    tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      get_node_base_interface(), get_node_timers_interface());
    tf2_buffer_->setCreateTimerInterface(timer_interface);
    tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

    // Initialize tf2 subscriber
    pc_sub_.subscribe(this, "cloud_in", rmw_qos_profile_sensor_data);
    tf_pc_sub_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>>(
      pc_sub_, *tf2_buffer_, world_frame_id_, 5, this->get_node_logging_interface(),
      this->get_node_clock_interface(), 5s);
    tf_pc_sub_->registerCallback(&ElevationMappingNode::insert_pc_callback, this);
  }

  ~ElevationMappingNode() {
    delete[] grid_;
    delete[] sigma_;
  }

private:
  // Frame names
  std::string world_frame_id_ = "base_link";

  // Map parameter
  Eigen::Vector2f map_center_;
  Eigen::Vector2f map_widths_;

  Eigen::Vector2f map_lim_min_;
  Eigen::Vector2f map_lim_max_;

  Eigen::Vector2i grid_dim_;

  float resolution_;

  float * grid_;

  float * sigma_;

  float sigma_meas_ = 0.1;

  bool first = true;

  // Point cloud tf subscriber
  message_filters::Subscriber<sensor_msgs::msg::PointCloud2> pc_sub_;
  std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::PointCloud2>> tf_pc_sub_;

  // TF2
  std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;

  // Publisher
  sensor_msgs::msg::PointCloud elevation_map_msg_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr map_pub_;

  // Service
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr save_map_srv_;

  void insert_pc_callback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr cloud)
  {
    geometry_msgs::msg::TransformStamped sensor_to_world_tf;
    sensor_msgs::msg::PointCloud2 cloud_transformed;
    try {
      sensor_to_world_tf = tf2_buffer_->lookupTransform(
        world_frame_id_, cloud->header.frame_id, cloud->header.stamp,
        rclcpp::Duration::from_seconds(1.0));
      tf2::doTransform(*cloud, cloud_transformed, sensor_to_world_tf);

      sensor_msgs::PointCloud2Iterator<float> x_iter(cloud_transformed, "x");
      sensor_msgs::PointCloud2Iterator<float> y_iter(cloud_transformed, "y");
      sensor_msgs::PointCloud2Iterator<float> z_iter(cloud_transformed, "z");

      for (; x_iter != x_iter.end(); ++x_iter, ++y_iter, ++z_iter) {
        const Eigen::Vector2f xy(*x_iter, *y_iter);
        const float z = *z_iter;
        if (isnan(xy.x()) || isnan(xy.y()) || isnan(z)) {
          continue;
        }
        const Eigen::Vector2i ij = get_ij_from_xy(xy);
        if (ij.x() < 0 || ij.x() >= grid_dim_.x()) {
          continue;
        }
        if (ij.y() < 0 || ij.y() >= grid_dim_.y()) {
          continue;
        }
        const int idx = ij.x() * grid_dim_.y() + ij.y();
        if (first) {
          grid_[idx] = z; //(sigma_meas_ * grid_[idx] + sigma_[idx] * z) / (sigma_meas_ + sigma_[idx]);
        } else {
          grid_[idx] = (sigma_meas_ * grid_[idx] + sigma_[idx] * z) / (sigma_meas_ + sigma_[idx]);
        }
        sigma_[idx] = std::max<float>(sigma_meas_ * sigma_[idx] / (sigma_meas_ + sigma_[idx]), 1e-4f);
      }

      first = false;

      elevation_map_msg_.header.stamp = cloud->header.stamp;
      for (int idx = 0; idx < grid_dim_.x() * grid_dim_.y(); ++idx) {
        elevation_map_msg_.points[idx].z = grid_[idx];
      }
      map_pub_->publish(elevation_map_msg_);

      RCLCPP_INFO(get_logger(), "Published Elevation Map.");

    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN(get_logger(), "%s", ex.what());
      return;
    }
  }

  Eigen::Vector2i get_ij_from_xy(const Eigen::Vector2f & xy)
  {
    const int i = (int) ((xy.x() - map_lim_min_.x()) / resolution_);
    const int j = (int) ((xy.y() - map_lim_min_.y()) / resolution_);

    return Eigen::Vector2i(i, j);
  }

  void save_map(
    const std::shared_ptr<std_srvs::srv::Empty::Request>/*request*/,
    const std::shared_ptr<std_srvs::srv::Empty::Response>/*response*/)
  {
    std::ofstream outfile;
    outfile.open("elevation_map.txt");
    if (!outfile) {
      RCLCPP_ERROR(this->get_logger(), "Could not open esdf out file");
    }
    RCLCPP_INFO_STREAM(get_logger(), "Min. Map Point (x,y) in m in base_link_frame: " << map_lim_min_.transpose());
    RCLCPP_INFO_STREAM(get_logger(), "Grid dimensions: " << grid_dim_.transpose());
    RCLCPP_INFO_STREAM(get_logger(), "Resolution in m: " << resolution_);
    for (int i = 0; i < grid_dim_.x() * grid_dim_.y(); ++i)
    {
      outfile << " " << grid_[i];
    }
    RCLCPP_INFO_STREAM(get_logger(), "Map saved!");
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  const auto node_ptr = std::make_shared<ElevationMappingNode>();

  RCLCPP_INFO(node_ptr->get_logger(), "Starting %s, shut down with CTRL-C.", node_ptr->get_name());
  rclcpp::spin(node_ptr);
  RCLCPP_INFO(node_ptr->get_logger(), "Keyboard interrupt, shutting down.");

  rclcpp::shutdown();
  return 0;
}
