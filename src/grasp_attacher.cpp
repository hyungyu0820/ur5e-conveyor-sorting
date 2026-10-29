// Gazebo Classic model plugin that makes grasps stable.
//
// gazebo_ros2_control moves position-controlled joints by teleporting them, so a grasp that
// relies only on finger friction slips or explodes. This plugin instead welds the object that is
// between the fingers to the palm link with a fixed joint:
//   ros2 service call /grasp/attach std_srvs/srv/Trigger   -> attach nearest object
//   ros2 service call /grasp/detach std_srvs/srv/Trigger   -> release it
//
// SDF parameters (see urdf/parallel_gripper.xacro):
//   <palm_link>      link the object is welded to (fixed joints are merged, so use wrist_3_link)
//   <finger_link>    one per finger; the object must be near the midpoint of these links
//   <max_distance>   [m] search radius around the finger midpoint

#include <memory>
#include <string>
#include <vector>

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

namespace ur_pick_place
{

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
    if (!palm_) {
      RCLCPP_ERROR(node_->get_logger(), "palm link '%s' not found", palm_name.c_str());
      return;
    }
    for (auto e = sdf->GetElement("finger_link"); e; e = e->GetNextElement("finger_link")) {
      auto link = model_->GetLink(e->Get<std::string>());
      if (!link) {
        RCLCPP_ERROR(node_->get_logger(), "finger link '%s' not found", e->Get<std::string>().c_str());
        return;
      }
      fingers_.push_back(link);
    }
    max_distance_ = sdf->Get<double>("max_distance", 0.05).first;

    using Trigger = std_srvs::srv::Trigger;
    attach_srv_ = node_->create_service<Trigger>(
      "grasp/attach",
      [this](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res) { attach(*res); });
    detach_srv_ = node_->create_service<Trigger>(
      "grasp/detach",
      [this](const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res) { detach(*res); });

    RCLCPP_INFO(node_->get_logger(), "grasp attacher ready (palm: %s, %zu fingers)",
      palm_name.c_str(), fingers_.size());
  }

private:
  void attach(std_srvs::srv::Trigger::Response & res)
  {
    boost::recursive_mutex::scoped_lock lock(*world_->Physics()->GetPhysicsUpdateMutex());
    if (joint_) {
      res.success = false;
      res.message = "already holding " + held_name_;
      return;
    }
    if (fingers_.empty()) {
      res.success = false;
      res.message = "no finger links configured";
      return;
    }

    ignition::math::Vector3d centre;
    for (const auto & f : fingers_) {
      centre += f->WorldCoGPose().Pos();
    }
    centre /= static_cast<double>(fingers_.size());

    gazebo::physics::LinkPtr best;
    double best_dist = max_distance_;
    for (const auto & m : world_->Models()) {
      if (m == model_ || m->IsStatic()) {
        continue;
      }
      for (const auto & l : m->GetLinks()) {
        const double d = l->WorldCoGPose().Pos().Distance(centre);
        if (d < best_dist) {
          best_dist = d;
          best = l;
        }
      }
    }
    if (!best) {
      res.success = false;
      res.message = "no object between the fingers";
      return;
    }

    joint_ = world_->Physics()->CreateJoint("fixed", model_);
    joint_->SetName("grasp_fixed_joint");
    // the joint keeps the current palm->object offset
    joint_->Load(palm_, best, best->WorldPose() - palm_->WorldPose());
    joint_->Init();

    held_name_ = best->GetScopedName();
    res.success = true;
    res.message = "attached " + held_name_;
    RCLCPP_INFO(node_->get_logger(), "%s (%.3f m from finger centre)", res.message.c_str(), best_dist);
  }

  void detach(std_srvs::srv::Trigger::Response & res)
  {
    boost::recursive_mutex::scoped_lock lock(*world_->Physics()->GetPhysicsUpdateMutex());
    if (!joint_) {
      res.success = false;
      res.message = "nothing attached";
      return;
    }
    joint_->Detach();
    joint_.reset();
    res.success = true;
    res.message = "released " + held_name_;
    RCLCPP_INFO(node_->get_logger(), "%s", res.message.c_str());
    held_name_.clear();
  }

  gazebo::physics::ModelPtr model_;
  gazebo::physics::WorldPtr world_;
  gazebo::physics::LinkPtr palm_;
  std::vector<gazebo::physics::LinkPtr> fingers_;
  gazebo::physics::JointPtr joint_;
  std::string held_name_;
  double max_distance_{0.05};
  gazebo_ros::Node::SharedPtr node_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr attach_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr detach_srv_;
};

GZ_REGISTER_MODEL_PLUGIN(GraspAttacher)

}  // namespace ur_pick_place
