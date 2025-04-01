#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

class StaticTFPublisher : public rclcpp::Node
{
public:
    StaticTFPublisher() : Node("static_tf_publisher")
    {
        // 创建一个静态 TF 广播器（一个广播器可以发布多个变换）
        static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // 发布第一个变换：camera_link -> imu
        geometry_msgs::msg::TransformStamped imu_to_camera = create_transform(
            "camera_link", 
            "imu", 
            0.09, 0.023, -0.07,  // 平移 (x, y, z)
            0.493,-0.516,0.494, 0.497  // 旋转 (x, y, z, w)
        );

        // 发布第二个变换：camera_link -> camera_infra1_optical_frame
        geometry_msgs::msg::TransformStamped camera_to_optical = create_transform(
            "camera_link", 
            "camera_infra1_optical_frame", 
            0.0, 0.0, 0.0,  // 平移 (x, y, z)
            0.0, 0, 0, 1.0  // 旋转 (x, y, z, w)
        );

        // 将两个变换合并为一个消息数组并发布
        std::vector<geometry_msgs::msg::TransformStamped> transforms = {imu_to_camera, camera_to_optical};   // imu_to_camera,
        static_broadcaster_->sendTransform(transforms);

        RCLCPP_INFO(this->get_logger(), "Published static transforms: camera_link -> imu and camera_link -> camera_infra1_optical_frame");
    }

private:
    // 辅助函数：创建 TransformStamped 消息
    geometry_msgs::msg::TransformStamped create_transform(
        const std::string& parent_frame,
        const std::string& child_frame,
        double x, double y, double z,
        double qx, double qy, double qz, double qw)
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->get_clock()->now();
        transform.header.frame_id = parent_frame;
        transform.child_frame_id = child_frame;
        transform.transform.translation.x = x;
        transform.transform.translation.y = y;
        transform.transform.translation.z = z;
        transform.transform.rotation.x = qx;
        transform.transform.rotation.y = qy;
        transform.transform.rotation.z = qz;
        transform.transform.rotation.w = qw;
        return transform;
    }

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_broadcaster_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StaticTFPublisher>());
    rclcpp::shutdown();
    return 0;
}