// Gazebo Classic 11 world plugin used by the Galbot S1 demo (ROS 1).
//
// Topics (std_msgs/String, space separated fields):
//   /galbot/sim/attach          "<parent_model> <parent_link> <child_model> [child_link]"
//                               rigidly attaches the child to the parent link at the
//                               current relative pose (grasp)
//   /galbot/sim/detach          "<child_model>"  or "*" for all
//   /galbot/sim/set_model_pose  "<model> x y z roll pitch yaw"  (teleport, zero velocity)
// Published:
//   /galbot/sim/attached                 std_msgs/String (latched) "child:parent_link ..."
//   /galbot/objects/<model>/odom         nav_msgs/Odometry (world frame) for <publish_models>
//
// The attach joint is a revolute joint with zero limits, the same trick as
// pal-robotics/gazebo_ros_link_attacher, which works reliably with ODE.
//
// SDF parameters:
//   <publish_models>  space separated model names whose poses are published
//   <publish_rate>    Hz (default 20)

#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Pose3.hh>

#include <nav_msgs/Odometry.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

namespace galbot_s1_gazebo
{

class SimHelperPlugin : public gazebo::WorldPlugin
{
public:
  ~SimHelperPlugin() override
  {
    update_connection_.reset();
    if (nh_)
    {
      queue_.clear();
      queue_.disable();
      nh_->shutdown();
    }
    if (callback_thread_.joinable())
      callback_thread_.join();
  }

  void Load(gazebo::physics::WorldPtr world, sdf::ElementPtr sdf) override
  {
    world_ = world;
    if (sdf && sdf->HasElement("publish_models"))
    {
      std::istringstream ss(sdf->Get<std::string>("publish_models"));
      std::string name;
      while (ss >> name)
        publish_models_.push_back(name);
    }
    const double rate = (sdf && sdf->HasElement("publish_rate")) ? sdf->Get<double>("publish_rate") : 20.0;
    publish_period_ = rate > 0.0 ? 1.0 / rate : 0.05;

    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("[galbot_sim_helper] ROS is not initialized. Start Gazebo through gazebo_ros.");
      return;
    }
    nh_.reset(new ros::NodeHandle("galbot_sim_helper"));
    nh_->setCallbackQueue(&queue_);
    attach_sub_ = nh_->subscribe("/galbot/sim/attach", 10, &SimHelperPlugin::OnAttach, this);
    detach_sub_ = nh_->subscribe("/galbot/sim/detach", 10, &SimHelperPlugin::OnDetach, this);
    pose_sub_ = nh_->subscribe("/galbot/sim/set_model_pose", 10, &SimHelperPlugin::OnSetPose, this);
    state_pub_ = nh_->advertise<std_msgs::String>("/galbot/sim/attached", 1, true);
    for (const auto &name : publish_models_)
      odom_pubs_[name] = nh_->advertise<nav_msgs::Odometry>("/galbot/objects/" + name + "/odom", 1);
    callback_thread_ = std::thread([this]() {
      while (nh_->ok())
        queue_.callAvailable(ros::WallDuration(0.01));
    });
    PublishState();
    last_publish_ = world_->SimTime();
    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&SimHelperPlugin::OnUpdate, this, std::placeholders::_1));
    ROS_INFO_STREAM("[galbot_sim_helper] ready, publishing " << publish_models_.size() << " object poses");
  }

private:
  enum class Kind
  {
    Attach,
    Detach,
    SetPose
  };
  struct Request
  {
    Kind kind;
    std::string data;
  };

  void Queue(Kind kind, const std::string &data)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    requests_.push_back({kind, data});
  }
  void OnAttach(const std_msgs::String::ConstPtr &msg) { Queue(Kind::Attach, msg->data); }
  void OnDetach(const std_msgs::String::ConstPtr &msg) { Queue(Kind::Detach, msg->data); }
  void OnSetPose(const std_msgs::String::ConstPtr &msg) { Queue(Kind::SetPose, msg->data); }

  void OnUpdate(const gazebo::common::UpdateInfo &info)
  {
    std::vector<Request> requests;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      requests.swap(requests_);
    }
    bool changed = false;
    for (const auto &r : requests)
    {
      std::istringstream ss(r.data);
      if (r.kind == Kind::Attach)
      {
        std::string pm, pl, cm, cl;
        ss >> pm >> pl >> cm >> cl;
        changed |= Attach(pm, pl, cm, cl);
      }
      else if (r.kind == Kind::Detach)
      {
        std::string cm;
        ss >> cm;
        changed |= Detach(cm);
      }
      else
      {
        std::string name;
        double x = 0, y = 0, z = 0, roll = 0, pitch = 0, yaw = 0;
        ss >> name >> x >> y >> z >> roll >> pitch >> yaw;
        auto model = world_->ModelByName(name);
        if (!model)
        {
          ROS_WARN_STREAM("[galbot_sim_helper] set_model_pose: unknown model " << name);
          continue;
        }
        model->SetWorldPose(ignition::math::Pose3d(x, y, z, roll, pitch, yaw));
        model->SetLinearVel(ignition::math::Vector3d::Zero);
        model->SetAngularVel(ignition::math::Vector3d::Zero);
      }
    }
    if (changed)
      PublishState();

    if ((info.simTime - last_publish_).Double() >= publish_period_)
    {
      last_publish_ = info.simTime;
      PublishObjects(info);
    }
  }

  bool Attach(const std::string &pm, const std::string &pl, const std::string &cm, const std::string &cl)
  {
    if (joints_.count(cm))
    {
      ROS_WARN_STREAM("[galbot_sim_helper] " << cm << " is already attached");
      return false;
    }
    auto parent_model = world_->ModelByName(pm);
    auto child_model = world_->ModelByName(cm);
    if (!parent_model || !child_model)
    {
      ROS_WARN_STREAM("[galbot_sim_helper] attach: unknown model " << (parent_model ? cm : pm));
      return false;
    }
    auto parent_link = parent_model->GetLink(pl);
    auto child_link = cl.empty() ? child_model->GetLink() : child_model->GetLink(cl);
    if (!parent_link || !child_link)
    {
      ROS_WARN_STREAM("[galbot_sim_helper] attach: unknown link " << (parent_link ? cl : pl));
      return false;
    }
    gazebo::physics::JointPtr joint = world_->Physics()->CreateJoint("revolute", parent_model);
    joint->Attach(parent_link, child_link);
    joint->Load(parent_link, child_link, ignition::math::Pose3d());
    joint->SetModel(child_model);
    joint->SetUpperLimit(0, 0.0);
    joint->SetLowerLimit(0, 0.0);
    joint->Init();
    joints_[cm] = {joint, pl};
    ROS_INFO_STREAM("[galbot_sim_helper] attached " << cm << " to " << pm << "::" << pl);
    return true;
  }

  bool Detach(const std::string &cm)
  {
    bool changed = false;
    for (auto it = joints_.begin(); it != joints_.end();)
    {
      if (cm == "*" || cm.empty() || it->first == cm)
      {
        it->second.joint->Detach();
        ROS_INFO_STREAM("[galbot_sim_helper] detached " << it->first);
        it = joints_.erase(it);
        changed = true;
      }
      else
      {
        ++it;
      }
    }
    return changed;
  }

  void PublishState()
  {
    std_msgs::String msg;
    for (const auto &kv : joints_)
      msg.data += kv.first + ":" + kv.second.parent_link + " ";
    state_pub_.publish(msg);
  }

  void PublishObjects(const gazebo::common::UpdateInfo &info)
  {
    for (const auto &kv : odom_pubs_)
    {
      auto model = world_->ModelByName(kv.first);
      if (!model)
        continue;
      const auto pose = model->WorldPose();
      const auto v = model->WorldLinearVel();
      nav_msgs::Odometry odom;
      odom.header.stamp = ros::Time(info.simTime.sec, info.simTime.nsec);
      odom.header.frame_id = "odom";
      odom.child_frame_id = kv.first;
      odom.pose.pose.position.x = pose.Pos().X();
      odom.pose.pose.position.y = pose.Pos().Y();
      odom.pose.pose.position.z = pose.Pos().Z();
      odom.pose.pose.orientation.x = pose.Rot().X();
      odom.pose.pose.orientation.y = pose.Rot().Y();
      odom.pose.pose.orientation.z = pose.Rot().Z();
      odom.pose.pose.orientation.w = pose.Rot().W();
      odom.twist.twist.linear.x = v.X();
      odom.twist.twist.linear.y = v.Y();
      odom.twist.twist.linear.z = v.Z();
      kv.second.publish(odom);
    }
  }

  struct Attached
  {
    gazebo::physics::JointPtr joint;
    std::string parent_link;
  };

  gazebo::physics::WorldPtr world_;
  gazebo::event::ConnectionPtr update_connection_;
  std::unique_ptr<ros::NodeHandle> nh_;
  ros::CallbackQueue queue_;
  std::thread callback_thread_;
  ros::Subscriber attach_sub_, detach_sub_, pose_sub_;
  ros::Publisher state_pub_;
  std::map<std::string, ros::Publisher> odom_pubs_;
  std::vector<std::string> publish_models_;
  std::map<std::string, Attached> joints_;
  std::mutex mutex_;
  std::vector<Request> requests_;
  gazebo::common::Time last_publish_;
  double publish_period_{0.05};
};

GZ_REGISTER_WORLD_PLUGIN(SimHelperPlugin)

}  // namespace galbot_s1_gazebo
