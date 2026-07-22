#include <algorithm>
#include <cstdint>
#include <cstring>
#include <string>

#include <livox_ros_driver2/CustomMsg.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

namespace
{
struct PointXYZI
{
  float x;
  float y;
  float z;
  float intensity;
};

static_assert(sizeof(PointXYZI) == 16, "PointCloud2 point layout must be 16 bytes");
}  // namespace

class LivoxCustomToPointCloud
{
public:
  LivoxCustomToPointCloud()
    : nh_(), pnh_("~")
  {
    pnh_.param<std::string>("input_topic", input_topic_, "/livox/lidar");
    pnh_.param<std::string>("output_topic", output_topic_, "/livox/lidar_pcl2");
    pnh_.param("subscriber_queue_size", subscriber_queue_size_, 1);
    pnh_.param("publisher_queue_size", publisher_queue_size_, 1);

    subscriber_queue_size_ = std::max(1, subscriber_queue_size_);
    publisher_queue_size_ = std::max(1, publisher_queue_size_);

    publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(
        output_topic_, publisher_queue_size_);
    subscriber_ = nh_.subscribe(
        input_topic_, subscriber_queue_size_, &LivoxCustomToPointCloud::callback, this,
        ros::TransportHints().tcpNoDelay());

    ROS_INFO_STREAM("livox_custom_to_pcl: " << input_topic_ << " -> " << output_topic_
                    << " (queues: " << subscriber_queue_size_ << ", "
                    << publisher_queue_size_ << ")");
  }

private:
  void callback(const livox_ros_driver2::CustomMsgConstPtr& input)
  {
    const std::size_t point_count = input->points.size();

    sensor_msgs::PointCloud2 output;
    output.header = input->header;
    output.height = 1;
    output.width = static_cast<std::uint32_t>(point_count);
    output.is_bigendian = false;
    output.is_dense = false;
    output.point_step = sizeof(PointXYZI);
    output.row_step = output.point_step * output.width;
    output.fields.resize(4);
    setField(output.fields[0], "x", 0);
    setField(output.fields[1], "y", 4);
    setField(output.fields[2], "z", 8);
    setField(output.fields[3], "intensity", 12);
    output.data.resize(point_count * sizeof(PointXYZI));

    for (std::size_t i = 0; i < point_count; ++i)
    {
      const auto& input_point = input->points[i];
      const PointXYZI output_point{
          input_point.x,
          input_point.y,
          input_point.z,
          static_cast<float>(input_point.reflectivity)};
      std::memcpy(output.data.data() + i * sizeof(PointXYZI),
                  &output_point, sizeof(PointXYZI));
    }

    publisher_.publish(output);
  }

  static void setField(sensor_msgs::PointField& field, const std::string& name,
                       std::uint32_t offset)
  {
    field.name = name;
    field.offset = offset;
    field.datatype = sensor_msgs::PointField::FLOAT32;
    field.count = 1;
  }

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Publisher publisher_;
  ros::Subscriber subscriber_;
  std::string input_topic_;
  std::string output_topic_;
  int subscriber_queue_size_;
  int publisher_queue_size_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "custom_to_pcl");
  LivoxCustomToPointCloud converter;
  ros::spin();
  return 0;
}
