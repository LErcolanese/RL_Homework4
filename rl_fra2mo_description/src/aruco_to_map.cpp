#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Transform.h>

using namespace std::chrono_literals;


class FrameListener : public rclcpp::Node
{
public:
  FrameListener()
  : Node("aruco_map_transform_listener")
  {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    publisher_ = this->create_publisher<geometry_msgs::msg::TransformStamped>("transformed_frame", 10);

    timer_ = this->create_wall_timer(500ms, std::bind(&FrameListener::on_timer, this));
  }

private:
  void on_timer()
  {
    try {
      // Ottieni la trasformazione da "aruco_marker_frame" a "map"
      geometry_msgs::msg::TransformStamped aruco_to_map =
        tf_buffer_->lookupTransform("map", "aruco_marker_frame", tf2::TimePointZero);

      // Converti la trasformazione in tf2::Transform
      tf2::Transform tf_aruco_to_map;
      tf2::fromMsg(aruco_to_map.transform, tf_aruco_to_map);

      // Calcola manualmente la trasformazione da "map" a "world"
      // Parametri della trasformazione "map" -> "world"
      double tx = -3.5;  // Traslazione in x
      double ty = 3.0;   // Traslazione in y
      double tz = 0.0;   // Traslazione in z
      double yaw = -M_PI / 2.0; // Rotazione di -90° (in radianti)

      // Costruisci manualmente la matrice di rotazione
      double cos_yaw = std::cos(yaw);
      double sin_yaw = std::sin(yaw);

      tf2::Matrix3x3 rotation_matrix(
        cos_yaw, -sin_yaw, 0.0,
        sin_yaw,  cos_yaw, 0.0,
        0.0,      0.0,     1.0
      );

      tf2::Quaternion rotation_quaternion;
      rotation_matrix.getRotation(rotation_quaternion);

      // Costruisci la trasformazione completa "map" -> "world"
      tf2::Transform map_to_world(rotation_quaternion, tf2::Vector3(tx, ty, tz));

      // Calcola la trasformazione inversa "world" -> "map"
      tf2::Transform world_to_map = map_to_world.inverse();

      // Applica la trasformazione inversa al frame "aruco"
      tf2::Transform aruco_to_world = world_to_map * tf_aruco_to_map;

      // Converti la trasformazione risultante in geometry_msgs
      geometry_msgs::msg::TransformStamped result;
      result.header.stamp = this->get_clock()->now();
      result.header.frame_id = "world";
      result.child_frame_id = "aruco_in_world";
      result.transform = tf2::toMsg(aruco_to_world);

      // Stampa o pubblica il risultato
      RCLCPP_INFO(this->get_logger(), "Aruco in world: x=%.2f, y=%.2f, yaw=%.2f",
        aruco_to_world.getOrigin().x(),
        aruco_to_world.getOrigin().y(),
        tf2::getYaw(aruco_to_world.getRotation()));
    } catch (const tf2::TransformException &ex) {
      RCLCPP_WARN(this->get_logger(), "Could not get transform: %s", ex.what());
    }
  }

  std::string map_frame_;
  std::string aruco_frame_;
  std::string output_frame_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::TransformStamped>::SharedPtr publisher_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FrameListener>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
