// Gazebo Classic 11 model plugin: holonomic planar base for Galbot S1 (ROS 1).
//
// Subscribes geometry_msgs/Twist (body frame) and sets the linear / angular
// velocity of ONE link (the root link). gazebo_ros_planar_move sets the velocity
// of every link of the model, which would fight the arm joints - this plugin
// lets the arms move freely. The z velocity is left to physics (gravity +
// wheel contacts), roll / pitch rates are zeroed to keep the base upright.
//
// Publishes the ground truth pose of the link as nav_msgs/Odometry (frame
// <odometryFrame> == world).
//
// SDF parameters (all optional):
//   <link>             base_footprint
//   <commandTopic>     /galbot/base/cmd_vel
//   <odometryTopic>    /galbot/ground_truth/odom
//   <odometryFrame>    odom
//   <robotBaseFrame>   base_footprint
//   <odometryRate>     50
//   <commandTimeout>   0.5   [s, sim time]

#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <gazebo/common/Events.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>

namespace galbot_s1_gazebo
{

class PlanarBasePlugin : public gazebo::ModelPlugin
{
public:
  ~PlanarBasePlugin() override
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

  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    model_ = model;
    world_ = model->GetWorld();

    const std::string link_name = Param<std::string>(sdf, "link", "base_footprint");
    link_ = model_->GetLink(link_name);
    if (!link_)
    {
      ROS_WARN_STREAM("[galbot_planar_base] link '" << link_name << "' not found, using canonical link");
      link_ = model_->GetLink();
    }
    cmd_topic_ = Param<std::string>(sdf, "commandTopic", "/galbot/base/cmd_vel");
    odom_topic_ = Param<std::string>(sdf, "odometryTopic", "/galbot/ground_truth/odom");
    odom_frame_ = Param<std::string>(sdf, "odometryFrame", "odom");
    base_frame_ = Param<std::string>(sdf, "robotBaseFrame", "base_footprint");
    const double rate = Param<double>(sdf, "odometryRate", 50.0);
    odom_period_ = rate > 0.0 ? 1.0 / rate : 0.02;
    timeout_ = Param<double>(sdf, "commandTimeout", 0.5);

    if (!ros::isInitialized())
    {
      ROS_FATAL_STREAM("[galbot_planar_base] ROS is not initialized. Start Gazebo through gazebo_ros "
                       "(libgazebo_ros_api_plugin.so).");
      return;
    }
    nh_.reset(new ros::NodeHandle("galbot_planar_base"));
    nh_->setCallbackQueue(&queue_);
    sub_ = nh_->subscribe(cmd_topic_, 1, &PlanarBasePlugin::OnCommand, this);
    odom_pub_ = nh_->advertise<nav_msgs::Odometry>(odom_topic_, 10);
    callback_thread_ = std::thread([this]() {
      while (nh_->ok())
        queue_.callAvailable(ros::WallDuration(0.01));
    });

    model_->SetAutoDisable(false);
    last_cmd_time_ = world_->SimTime();
    last_odom_time_ = world_->SimTime();
    update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
        std::bind(&PlanarBasePlugin::OnUpdate, this, std::placeholders::_1));
    ROS_INFO_STREAM("[galbot_planar_base] link " << link_->GetName() << ", cmd " << cmd_topic_ << ", odom "
                                                 << odom_topic_);
  }

private:
  template <typename T>
  static T Param(const sdf::ElementPtr &sdf, const std::string &key, const T &def)
  {
    if (sdf && sdf->HasElement(key))
      return sdf->Get<T>(key);
    return def;
  }

  void OnCommand(const geometry_msgs::Twist::ConstPtr &msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    vx_ = msg->linear.x;
    vy_ = msg->linear.y;
    wz_ = msg->angular.z;
    last_cmd_time_ = world_->SimTime();
  }

  void OnUpdate(const gazebo::common::UpdateInfo &info)
  {
    double vx, vy, wz;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      vx = vx_;
      vy = vy_;
      wz = wz_;
      if ((info.simTime - last_cmd_time_).Double() > timeout_)
        vx = vy = wz = 0.0;
    }
    const ignition::math::Pose3d pose = link_->WorldPose();
    const double yaw = pose.Rot().Yaw();
    const double c = std::cos(yaw), s = std::sin(yaw);
    const ignition::math::Vector3d current = link_->WorldLinearVel();
    link_->SetEnabled(true);
    link_->SetLinearVel(ignition::math::Vector3d(c * vx - s * vy, s * vx + c * vy, current.Z()));
    link_->SetAngularVel(ignition::math::Vector3d(0.0, 0.0, wz));

    if ((info.simTime - last_odom_time_).Double() >= odom_period_)
    {
      last_odom_time_ = info.simTime;
      PublishOdometry(info, pose, c, s);
    }
  }

  void PublishOdometry(const gazebo::common::UpdateInfo &info, const ignition::math::Pose3d &pose, double c,
                       double s)
  {
    nav_msgs::Odometry odom;
    odom.header.stamp = ros::Time(info.simTime.sec, info.simTime.nsec);
    odom.header.frame_id = odom_frame_;
    odom.child_frame_id = base_frame_;
    odom.pose.pose.position.x = pose.Pos().X();
    odom.pose.pose.position.y = pose.Pos().Y();
    odom.pose.pose.position.z = pose.Pos().Z();
    odom.pose.pose.orientation.x = pose.Rot().X();
    odom.pose.pose.orientation.y = pose.Rot().Y();
    odom.pose.pose.orientation.z = pose.Rot().Z();
    odom.pose.pose.orientation.w = pose.Rot().W();
    const ignition::math::Vector3d v = link_->WorldLinearVel();
    const ignition::math::Vector3d w = link_->WorldAngularVel();
    odom.twist.twist.linear.x = c * v.X() + s * v.Y();
    odom.twist.twist.linear.y = -s * v.X() + c * v.Y();
    odom.twist.twist.linear.z = v.Z();
    odom.twist.twist.angular.z = w.Z();
    odom_pub_.publish(odom);
  }

  gazebo::physics::ModelPtr model_;
  gazebo::physics::WorldPtr world_;
  gazebo::physics::LinkPtr link_;
  gazebo::event::ConnectionPtr update_connection_;

  std::unique_ptr<ros::NodeHandle> nh_;
  ros::CallbackQueue queue_;
  std::thread callback_thread_;
  ros::Subscriber sub_;
  ros::Publisher odom_pub_;

  std::mutex mutex_;
  double vx_{0.0}, vy_{0.0}, wz_{0.0};
  gazebo::common::Time last_cmd_time_;
  gazebo::common::Time last_odom_time_;
  double odom_period_{0.02};
  double timeout_{0.5};
  std::string cmd_topic_, odom_topic_, odom_frame_, base_frame_;
};

GZ_REGISTER_MODEL_PLUGIN(PlanarBasePlugin)

}  // namespace galbot_s1_gazebo
