// Gazebo (gz-sim 8 / Harmonic) world system used by the Galbot S1 demo (ROS 2).
//
// gz-transport topics (gz.msgs.StringMsg, bridged from ROS std_msgs/String):
//   /galbot/sim/attach          "<parent_model> <parent_link> <child_model> [child_link]"
//   /galbot/sim/detach          "<child_model>"  or "*" for all
//   /galbot/sim/set_model_pose  "<model> x y z roll pitch yaw"
// Published:
//   /galbot/sim/attached        "child:parent_link ..."
//
// Attaching creates a DetachableJoint component (fixed joint at the current
// relative pose) exactly like gz::sim::systems::DetachableJoint does, but the
// objects start detached and any model can be attached at run time.

#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/DetachableJoint.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/PoseCmd.hh>
#include <gz/transport/Node.hh>

namespace galbot_s1_gazebo
{

class SimHelper : public gz::sim::System, public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate
{
public:
  void Configure(const gz::sim::Entity & /*entity*/, const std::shared_ptr<const sdf::Element> & /*sdf*/,
                 gz::sim::EntityComponentManager & /*ecm*/, gz::sim::EventManager & /*events*/) override
  {
    node_.Subscribe("/galbot/sim/attach", &SimHelper::OnAttach, this);
    node_.Subscribe("/galbot/sim/detach", &SimHelper::OnDetach, this);
    node_.Subscribe("/galbot/sim/set_model_pose", &SimHelper::OnSetPose, this);
    state_pub_ = node_.Advertise<gz::msgs::StringMsg>("/galbot/sim/attached");
    gzmsg << "[galbot_sim_helper] ready" << std::endl;
  }

  void PreUpdate(const gz::sim::UpdateInfo & /*info*/, gz::sim::EntityComponentManager &ecm) override
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
        changed |= Attach(ecm, pm, pl, cm, cl);
      }
      else if (r.kind == Kind::Detach)
      {
        std::string cm;
        ss >> cm;
        changed |= Detach(ecm, cm);
      }
      else
      {
        std::string name;
        double x = 0, y = 0, z = 0, roll = 0, pitch = 0, yaw = 0;
        ss >> name >> x >> y >> z >> roll >> pitch >> yaw;
        const auto entity = ModelEntity(ecm, name);
        if (entity == gz::sim::kNullEntity)
        {
          gzwarn << "[galbot_sim_helper] set_model_pose: unknown model " << name << std::endl;
          continue;
        }
        const gz::math::Pose3d pose(x, y, z, roll, pitch, yaw);
        auto *cmd = ecm.Component<gz::sim::components::WorldPoseCmd>(entity);
        if (!cmd)
        {
          ecm.CreateComponent(entity, gz::sim::components::WorldPoseCmd(pose));
        }
        else
        {
          cmd->Data() = pose;
          ecm.SetChanged(entity, gz::sim::components::WorldPoseCmd::typeId, gz::sim::ComponentState::OneTimeChange);
        }
      }
    }
    if (changed)
      PublishState();
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
  struct Attached
  {
    gz::sim::Entity joint;
    std::string parent_link;
  };

  void Queue(Kind kind, const std::string &data)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    requests_.push_back({kind, data});
  }
  void OnAttach(const gz::msgs::StringMsg &msg) { Queue(Kind::Attach, msg.data()); }
  void OnDetach(const gz::msgs::StringMsg &msg) { Queue(Kind::Detach, msg.data()); }
  void OnSetPose(const gz::msgs::StringMsg &msg) { Queue(Kind::SetPose, msg.data()); }

  static gz::sim::Entity ModelEntity(gz::sim::EntityComponentManager &ecm, const std::string &name)
  {
    return ecm.EntityByComponents(gz::sim::components::Model(), gz::sim::components::Name(name));
  }

  bool Attach(gz::sim::EntityComponentManager &ecm, const std::string &pm, const std::string &pl,
              const std::string &cm, const std::string &cl)
  {
    if (attached_.count(cm))
    {
      gzwarn << "[galbot_sim_helper] " << cm << " is already attached" << std::endl;
      return false;
    }
    const auto parent_model = ModelEntity(ecm, pm);
    const auto child_model = ModelEntity(ecm, cm);
    if (parent_model == gz::sim::kNullEntity || child_model == gz::sim::kNullEntity)
    {
      gzwarn << "[galbot_sim_helper] attach: unknown model " << pm << " / " << cm << std::endl;
      return false;
    }
    const auto parent_link = gz::sim::Model(parent_model).LinkByName(ecm, pl);
    gz::sim::Model child(child_model);
    const auto child_link = cl.empty() ? child.CanonicalLink(ecm) : child.LinkByName(ecm, cl);
    if (parent_link == gz::sim::kNullEntity || child_link == gz::sim::kNullEntity)
    {
      gzwarn << "[galbot_sim_helper] attach: unknown link " << pl << " / " << cl << std::endl;
      return false;
    }
    const auto joint = ecm.CreateEntity();
    ecm.CreateComponent(joint, gz::sim::components::DetachableJoint({parent_link, child_link, "fixed"}));
    attached_[cm] = {joint, pl};
    gzmsg << "[galbot_sim_helper] attached " << cm << " to " << pm << "::" << pl << std::endl;
    return true;
  }

  bool Detach(gz::sim::EntityComponentManager &ecm, const std::string &cm)
  {
    bool changed = false;
    for (auto it = attached_.begin(); it != attached_.end();)
    {
      if (cm == "*" || cm.empty() || it->first == cm)
      {
        ecm.RequestRemoveEntity(it->second.joint);
        gzmsg << "[galbot_sim_helper] detached " << it->first << std::endl;
        it = attached_.erase(it);
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
    gz::msgs::StringMsg msg;
    std::string data;
    for (const auto &kv : attached_)
      data += kv.first + ":" + kv.second.parent_link + " ";
    msg.set_data(data);
    state_pub_.Publish(msg);
  }

  gz::transport::Node node_;
  gz::transport::Node::Publisher state_pub_;
  std::mutex mutex_;
  std::vector<Request> requests_;
  std::map<std::string, Attached> attached_;
};

}  // namespace galbot_s1_gazebo

GZ_ADD_PLUGIN(galbot_s1_gazebo::SimHelper, gz::sim::System, galbot_s1_gazebo::SimHelper::ISystemConfigure,
              galbot_s1_gazebo::SimHelper::ISystemPreUpdate)
