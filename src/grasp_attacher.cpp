// Gazebo Classic model plugin that decides whether a grasp succeeds and then holds the object.
//
// gazebo_ros2_control moves position-controlled joints by teleporting them, so a grasp that
// relies only on finger friction slips or explodes. This plugin is the simulated "physics truth"
// of a parallel gripper instead: when asked to grasp, it checks the object between the fingers
// with simple geometric rules and, if the grasp is possible, welds the object to the palm.
//
//   ros2 service call /grasp/attach std_srvs/srv/Trigger   -> try to grasp (message says why it failed)
//   ros2 service call /grasp/detach std_srvs/srv/Trigger   -> release
//   ros2 topic echo /grasp/holding                         -> std_msgs/Bool, the gripper's object sensor
//
// A grasp fails when (the robot is not told why; it must find out with its own sensors):
//   - no object centre is inside the pinch region between the fingers
//   - two or more objects are inside it (overlapping items)
//   - the object is wider than the gripper opening along the closing axis (rotated long item)
//   - random slip, probability slip_rate
// A held object is dropped during transport with probability drop_rate.
//
// SDF parameters (see urdf/parallel_gripper.xacro); slip_rate, drop_rate and seed are also
// ROS parameters of this node and can be changed at run time:
//   <palm_link>      link the object is welded to (fixed joints are merged, so use wrist_3_link)
//   <finger_link>    one per finger (two)
//   <max_opening>    [m] distance between the finger pads when fully open
//   <slip_rate> <drop_rate> <seed>

#include <cmath>
#include <memory>
#include <random>
#include <string>
#include <vector>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace ur_pick_place
{

using ignition::math::Vector3d;

// pinch region in the finger frame (z = finger direction, y = closing direction, x = finger width)
constexpr double kFingerLength = 0.06;  // fingers span z = 0 .. 0.06 from the finger joint
constexpr double kPinchHalfWidth = 0.02;  // half of the finger pad width plus margin

class GraspAttacher : public gazebo::ModelPlugin
{
public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    world_ = model->GetWorld();
    node_ = gazebo_ros::Node::Get(sdf);

    const auto palm_name = sdf->Get<std::string>("palm_link", "wrist_3_link").first;
    palm_ = model_->GetLink(palm_name);
    for (auto e = sdf->GetElement("finger_link"); e; e = e->GetNextElement("finger_link")) {
      if (auto link = model_->GetLink(e->Get<std::string>())) {
        fingers_.push_back(link);
      }
    }
    if (!palm_ || fingers_.size() != 2) {
      RCLCPP_ERROR(node_->get_logger(), "grasp attacher needs a palm link and two finger links");
      return;
    }
    max_opening_ = sdf->Get<double>("max_opening", 0.09).first;
    node_->declare_parameter("slip_rate", sdf->Get<double>("slip_rate", 0.0).first);
    node_->declare_parameter("drop_rate", sdf->Get<double>("drop_rate", 0.0).first);
    rng_.seed(static_cast<unsigned>(node_->declare_parameter("seed", sdf->Get<int>("seed", 1).first)));

    using Trigger = std_srvs::srv::Trigger;
    attach_srv_ = node_->create_service<Trigger>(
      "grasp/attach",
      [this](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res) { attach(*res); });
    detach_srv_ = node_->create_service<Trigger>(
      "grasp/detach",
      [this](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res) { detach(*res); });
    holding_pub_ = node_->create_publisher<std_msgs::msg::Bool>("grasp/holding", 10);
    update_conn_ = gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&GraspAttacher::OnUpdate, this));

    RCLCPP_INFO(node_->get_logger(), "grasp attacher ready (palm: %s, opening %.3f m)", palm_name.c_str(), max_opening_);
  }

private:
  double rate(const char * name) { return node_->get_parameter(name).as_double(); }
  bool chance(double p) { return std::uniform_real_distribution<double>(0.0, 1.0)(rng_) < p; }

  // width of a link along a world axis, from its collision shapes (box, cylinder, sphere)
  static double extentAlong(const gazebo::physics::LinkPtr & link, const Vector3d & axis)
  {
    double extent = 0.0;
    for (const auto & c : link->GetCollisions()) {
      const auto rot = c->WorldPose().Rot();
      const auto shape = c->GetShape();
      double e = 0.0;
      if (auto box = boost::dynamic_pointer_cast<gazebo::physics::BoxShape>(shape)) {
        const auto s = box->Size();
        e = std::abs(s.X() * rot.XAxis().Dot(axis)) + std::abs(s.Y() * rot.YAxis().Dot(axis)) +
          std::abs(s.Z() * rot.ZAxis().Dot(axis));
      } else if (auto cyl = boost::dynamic_pointer_cast<gazebo::physics::CylinderShape>(shape)) {
        const double c_ax = std::abs(rot.ZAxis().Dot(axis));
        e = 2 * cyl->GetRadius() * std::sqrt(std::max(0.0, 1 - c_ax * c_ax)) + cyl->GetLength() * c_ax;
      } else if (auto sph = boost::dynamic_pointer_cast<gazebo::physics::SphereShape>(shape)) {
        e = 2 * sph->GetRadius();
      }
      extent = std::max(extent, e);
    }
    return extent;
  }

  void attach(std_srvs::srv::Trigger::Response & res)
  {
    boost::recursive_mutex::scoped_lock lock(*world_->Physics()->GetPhysicsUpdateMutex());
    res.success = false;
    if (joint_) {
      res.message = "fail:already_holding";
      return;
    }

    // finger frame axes and the pinch region centre (between the finger joints, half way down)
    const auto left = fingers_[0]->WorldPose();
    const auto right = fingers_[1]->WorldPose();
    const Vector3d closing = (left.Pos() - right.Pos()).Normalized();
    const Vector3d along = left.Rot().ZAxis();
    const Vector3d across = along.Cross(closing).Normalized();
    const Vector3d centre = (left.Pos() + right.Pos()) / 2 + along * (kFingerLength / 2);

    std::vector<gazebo::physics::LinkPtr> inside;
    for (const auto & m : world_->Models()) {
      if (m == model_ || m->IsStatic()) {
        continue;
      }
      for (const auto & l : m->GetLinks()) {
        const Vector3d d = l->WorldCoGPose().Pos() - centre;
        if (std::abs(d.Dot(closing)) <= max_opening_ / 2 && std::abs(d.Dot(across)) <= kPinchHalfWidth &&
          std::abs(d.Dot(along)) <= kFingerLength / 2)
        {
          inside.push_back(l);
        }
      }
    }

    const double width = inside.empty() ? 0.0 : extentAlong(inside[0], closing);
    std::string reason;
    if (inside.empty()) {
      reason = "fail:nothing_between_fingers";
    } else if (inside.size() > 1) {
      reason = "fail:multiple_objects";
    } else if (width > max_opening_ - 0.002) {
      reason = "fail:too_wide";
    } else if (chance(rate("slip_rate"))) {
      reason = "fail:slipped";
    }
    if (!reason.empty()) {
      res.message = reason;
      RCLCPP_INFO(node_->get_logger(), "grasp %s (%zu object(s) in pinch region, width %.3f m)",
        reason.c_str(), inside.size(), width);
      return;
    }

    const auto & obj = inside[0];
    joint_ = world_->Physics()->CreateJoint("fixed", model_);
    joint_->SetName("grasp_fixed_joint");
    joint_->Load(palm_, obj, obj->WorldPose() - palm_->WorldPose());  // keeps the current offset
    joint_->Init();
    held_name_ = obj->GetScopedName();
    // a dropped object falls out some time into the transport move
    drop_at_ = chance(rate("drop_rate")) ?
      world_->SimTime().Double() + std::uniform_real_distribution<double>(0.8, 2.5)(rng_) : -1.0;
    res.success = true;
    res.message = "attached " + held_name_;
    RCLCPP_INFO(node_->get_logger(), "%s (width %.3f m, %zu collision(s))", res.message.c_str(), width,
      obj->GetCollisions().size());
  }

  void release()
  {
    joint_->Detach();
    joint_.reset();
    held_name_.clear();
    drop_at_ = -1.0;
  }

  void detach(std_srvs::srv::Trigger::Response & res)
  {
    boost::recursive_mutex::scoped_lock lock(*world_->Physics()->GetPhysicsUpdateMutex());
    if (!joint_) {
      res.success = false;
      res.message = "nothing attached";
      return;
    }
    res.success = true;
    res.message = "released " + held_name_;
    RCLCPP_INFO(node_->get_logger(), "%s", res.message.c_str());
    release();
  }

  void OnUpdate()
  {
    const double now = world_->SimTime().Double();
    if (joint_ && drop_at_ > 0 && now >= drop_at_) {
      RCLCPP_INFO(node_->get_logger(), "dropped %s during transport", held_name_.c_str());
      release();
    }
    if (now - last_pub_ >= 0.05) {  // object sensor at 20 Hz
      last_pub_ = now;
      std_msgs::msg::Bool msg;
      msg.data = static_cast<bool>(joint_);
      holding_pub_->publish(msg);
    }
  }

  gazebo::physics::ModelPtr model_;
  gazebo::physics::WorldPtr world_;
  gazebo::physics::LinkPtr palm_;
  std::vector<gazebo::physics::LinkPtr> fingers_;
  gazebo::physics::JointPtr joint_;
  std::string held_name_;
  double max_opening_{0.09};
  double drop_at_{-1.0};
  double last_pub_{0.0};
  std::mt19937 rng_;
  gazebo::event::ConnectionPtr update_conn_;
  gazebo_ros::Node::SharedPtr node_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr attach_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr detach_srv_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr holding_pub_;
};

GZ_REGISTER_MODEL_PLUGIN(GraspAttacher)

}  // namespace ur_pick_place
