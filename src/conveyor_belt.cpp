// Gazebo Classic model plugin for a conveyor belt with a light curtain at the pick position.
//
// Every non-static object resting on the belt surface is driven along the belt direction.
// When an object reaches the light curtain the belt stops by itself, like a real
// conveyor with a photoelectric sensor; the robot restarts it after picking.
//   ros2 service call /conveyor/power std_srvs/srv/SetBool "{data: true}"   -> run
//   ros2 topic echo /conveyor/running                                      -> std_msgs/Bool
//   ros2 topic echo /conveyor/object_at_sensor                             -> std_msgs/Bool
//
// SDF parameters (see models/conveyor.sdf), all in the conveyor model frame:
//   <length> <width> <height>  belt size [m]; the belt runs along +y, top surface at z = height
//   <speed>                    [m/s]
//   <sensor_y>                 light curtain position along the belt
//   <exclude_model>            model that is never moved (the robot), may repeat

#include <atomic>
#include <memory>
#include <set>
#include <string>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>

namespace ur_pick_place
{

class ConveyorBelt : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    world_ = model->GetWorld();
    node_ = gazebo_ros::Node::Get(sdf);

    length_ = sdf->Get<double>("length", 1.4).first;
    width_ = sdf->Get<double>("width", 0.2).first;
    height_ = sdf->Get<double>("height", 0.2).first;
    speed_ = sdf->Get<double>("speed", 0.1).first;
    sensor_y_ = sdf->Get<double>("sensor_y", 0.0).first;
    for (auto e = sdf->GetElement("exclude_model"); e; e = e->GetNextElement("exclude_model")) {
      excluded_.insert(e->Get<std::string>());
    }

    running_pub_ = node_->create_publisher<std_msgs::msg::Bool>("conveyor/running", 10);
    sensor_pub_ = node_->create_publisher<std_msgs::msg::Bool>("conveyor/object_at_sensor", 10);
    power_srv_ = node_->create_service<std_srvs::srv::SetBool>(
      "conveyor/power",
      [this](const std_srvs::srv::SetBool::Request::SharedPtr req,
      std_srvs::srv::SetBool::Response::SharedPtr res) {
        if (req->data) {
          at_sensor_prev_ = false;  // an object still at the sensor stops the belt again
        }
        running_ = req->data;
        res->success = true;
        res->message = running_ ? "conveyor running" : "conveyor stopped";
      });

    update_conn_ = gazebo::event::Events::ConnectWorldUpdateBegin(
      std::bind(&ConveyorBelt::OnUpdate, this));
    RCLCPP_INFO(node_->get_logger(), "conveyor ready (%.2f m, %.2f m/s, sensor at y=%.2f)",
      length_, speed_, sensor_y_);
  }

private:
  void OnUpdate()
  {
    const auto belt_pose = model_->WorldPose();
    const auto dir = belt_pose.Rot().RotateVector(ignition::math::Vector3d::UnitY);
    bool at_sensor = false;

    for (const auto & m : world_->Models()) {
      if (m == model_ || m->IsStatic() || excluded_.count(m->GetName())) {
        continue;
      }
      // object position in the belt frame
      const auto p = belt_pose.Rot().RotateVectorReverse(m->WorldPose().Pos() - belt_pose.Pos());
      const bool on_belt = std::abs(p.X()) < width_ / 2 && std::abs(p.Y()) < length_ / 2 &&
        p.Z() > height_ - 0.005 && p.Z() < height_ + 0.08;
      if (!on_belt) {
        continue;
      }
      // the sensor beam covers a 2 cm band just upstream of sensor_y
      if (p.Y() > sensor_y_ - 0.02 && p.Y() < sensor_y_ + 0.05) {
        at_sensor = true;
      }
      // the belt carries the object: horizontal velocity = belt velocity, no spinning
      const double v = running_ ? speed_ : 0.0;
      const auto vel = m->WorldLinearVel();
      m->SetLinearVel(ignition::math::Vector3d(dir.X() * v, dir.Y() * v, std::min(vel.Z(), 0.0)));
      m->SetAngularVel(ignition::math::Vector3d::Zero);
    }

    if (at_sensor && !at_sensor_prev_) {
      running_ = false;  // light curtain interrupted -> stop the belt
      RCLCPP_INFO(node_->get_logger(), "object at sensor, conveyor stopped");
    }
    at_sensor_prev_ = at_sensor;

    // publish at ~10 Hz
    const auto now = world_->SimTime();
    if ((now - last_pub_).Double() >= 0.1) {
      last_pub_ = now;
      std_msgs::msg::Bool msg;
      msg.data = running_;
      running_pub_->publish(msg);
      msg.data = at_sensor;
      sensor_pub_->publish(msg);
    }
  }

  gazebo::physics::ModelPtr model_;
  gazebo::physics::WorldPtr world_;
  gazebo::event::ConnectionPtr update_conn_;
  gazebo_ros::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr running_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr sensor_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr power_srv_;
  std::set<std::string> excluded_;
  double length_{1.4}, width_{0.2}, height_{0.2}, speed_{0.1}, sensor_y_{0.0};
  // written by the service callback (ROS thread) and the physics update
  std::atomic<bool> running_{true};
  std::atomic<bool> at_sensor_prev_{false};
  gazebo::common::Time last_pub_;
};

GZ_REGISTER_MODEL_PLUGIN(ConveyorBelt)

}  // namespace ur_pick_place
