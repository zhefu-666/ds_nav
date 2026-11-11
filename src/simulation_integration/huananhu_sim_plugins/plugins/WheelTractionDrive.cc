// Experimental contact-driven omni wheels. No chassis wrench or pose command.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/components/JointForceCmd.hh>
#include <ignition/gazebo/components/JointPosition.hh>
#include <ignition/gazebo/components/JointVelocity.hh>
#include <ignition/math/Matrix3.hh>
#include <ignition/msgs/Utility.hh>
#include <ignition/msgs/odometry.pb.h>
#include <ignition/msgs/twist.pb.h>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include <mutex>
#include <vector>

namespace kaohe {
using namespace ignition;
using namespace gazebo;
class WheelTractionDrive : public System,
                           public ISystemConfigure,
                           public ISystemPreUpdate,
                           public ISystemPostUpdate {
  Model model{kNullEntity};
  Entity chassis{kNullEntity};
  std::array<Entity, 4> joints;
  std::array<math::Vector3d, 4> jac;
  std::array<double, 4> integral{{0, 0, 0, 0}}, ramp{{0, 0, 0, 0}},
      previous{{0, 0, 0, 0}};
  math::Matrix3d normal = math::Matrix3d::Zero;
  transport::Node node;
  transport::Node::Publisher encPub, truthPub;
  std::mutex mutex;
  msgs::Twist command;
  double simNow = 0, commandTime = -100, lastSample = -1, x = 0, y = 0, yaw = 0;
  double maxTorque = 1.2, kp = .30, ki = 1.0;
  bool encoderReady = false;
  std::string robot;
  void OnCmd(const msgs::Twist &m) {
    std::lock_guard<std::mutex> lock(mutex);
    command = m;
    commandTime = simNow;
  }

public:
  void Configure(const Entity &e, const std::shared_ptr<const sdf::Element> &s,
                 EntityComponentManager &ecm, EventManager &) override {
    model = Model(e);
    robot = model.Name(ecm);
    chassis = model.LinkByName(ecm, "chassis");
    Link(chassis).EnableVelocityChecks(ecm, true);
    if (s->HasElement("max_torque"))
      maxTorque = s->Get<double>("max_torque");
    if (s->HasElement("proportional_gain"))
      kp = s->Get<double>("proportional_gain");
    if (s->HasElement("integral_gain"))
      ki = s->Get<double>("integral_gain");
    auto settings = s->Clone();
    auto w = settings->GetElement("wheel");
    for (size_t i = 0; i < 4; i++) {
      joints[i] = model.JointByName(ecm, w->Get<std::string>("joint"));
      jac[i] = w->Get<math::Vector3d>("jacobian");
      for (size_t a = 0; a < 3; a++)
        for (size_t b = 0; b < 3; b++)
          normal(a, b) += jac[i][a] * jac[i][b];
      ecm.CreateComponent(joints[i], components::JointVelocity());
      ecm.CreateComponent(joints[i], components::JointPosition());
      w = w->GetNextElement("wheel");
    }
    encPub = node.Advertise<msgs::Odometry>("/" + robot + "/wheel_odometry");
    truthPub = node.Advertise<msgs::Odometry>("/" + robot + "/odometry");
    node.Subscribe("/" + robot + "/cmd_vel", &WheelTractionDrive::OnCmd, this);
  }

public:
  void PreUpdate(const UpdateInfo &info, EntityComponentManager &ecm) override {
    if (info.paused)
      return;
    double dt = std::chrono::duration<double>(info.dt).count();
    if (dt <= 0)
      return;
    msgs::Twist cmd;
    {
      std::lock_guard<std::mutex> lock(mutex);
      simNow = std::chrono::duration<double>(info.simTime).count();
      if (simNow - commandTime < .5)
        cmd = command;
    }
    math::Vector3d velocity(cmd.linear().x(), cmd.linear().y(),
                            cmd.angular().z());
    const bool stopRequested = velocity.SquaredLength() <= 1e-12;
    // Clamp the integral TORQUE consistently when an optional gain is used.
    // ki=1 preserves the original limit and ordinary-navigation behavior.
    const double integralLimit = ki > 0.0 ? maxTorque / ki : 0.0;
    for (size_t i = 0; i < 4; i++) {
      auto v = ecm.Component<components::JointVelocity>(joints[i]);
      if (!v || v->Data().empty())
        continue;
      double target = jac[i].Dot(velocity);
      if (stopRequested) {
        // An explicit zero / command watchdog stop must not retain propulsion
        // from an earlier ramp or PI integral. Wheel velocity feedback below
        // still applies bounded proportional braking; no chassis wrench is used.
        ramp[i] = 0.0;
        integral[i] = 0.0;
      } else {
        ramp[i] += std::max(-10 * dt, std::min(10 * dt, target - ramp[i]));
      }
      double error = ramp[i] - v->Data()[0];
      if (!stopRequested) {
        integral[i] =
            std::max(-integralLimit, std::min(integralLimit, integral[i] + error * dt));
      }
      double torque = std::max(
          -maxTorque, std::min(maxTorque, kp * error + ki * integral[i]));
      auto f = ecm.Component<components::JointForceCmd>(joints[i]);
      if (!f)
        ecm.CreateComponent(joints[i], components::JointForceCmd({torque}));
      else
        f->Data() = {torque};
    }
  }

public:
  void PostUpdate(const UpdateInfo &info,
                  const EntityComponentManager &ecm) override {
    if (info.paused)
      return;
    double now = std::chrono::duration<double>(info.simTime).count();
    if (lastSample >= 0 && now - lastSample < .02)
      return;
    std::array<double, 4> position;
    for (size_t i = 0; i < 4; i++) {
      auto p = ecm.Component<components::JointPosition>(joints[i]);
      if (!p || p->Data().empty())
        return;
      position[i] = p->Data()[0];
    }
    if (!encoderReady) {
      previous = position;
      lastSample = now;
      encoderReady = true;
      return;
    }
    double dt = now - lastSample;
    math::Vector3d rhs(0, 0, 0);
    for (size_t i = 0; i < 4; i++) {
      double speed = (position[i] - previous[i]) / dt;
      rhs += jac[i] * speed;
    }
    auto v = normal.Inverse() * rhs;
    x += (cos(yaw) * v.X() - sin(yaw) * v.Y()) * dt;
    y += (sin(yaw) * v.X() + cos(yaw) * v.Y()) * dt;
    yaw += v.Z() * dt;
    msgs::Odometry odom;
    odom.mutable_header()->mutable_stamp()->set_sec(static_cast<int64_t>(now));
    odom.mutable_header()->mutable_stamp()->set_nsec(
        static_cast<int32_t>((now - floor(now)) * 1e9));
    auto frame = odom.mutable_header()->add_data();
    frame->set_key("frame_id");
    frame->add_value("odom");
    auto child = odom.mutable_header()->add_data();
    child->set_key("child_frame_id");
    child->add_value("base_footprint");
    msgs::Set(odom.mutable_pose()->mutable_position(),
              math::Vector3d(x, y, .02));
    msgs::Set(odom.mutable_pose()->mutable_orientation(),
              math::Quaterniond(0, 0, yaw));
    msgs::Set(odom.mutable_twist()->mutable_linear(),
              math::Vector3d(v.X(), v.Y(), 0));
    msgs::Set(odom.mutable_twist()->mutable_angular(),
              math::Vector3d(0, 0, v.Z()));
    encPub.Publish(odom);
    // World truth is published only for external assessment. It is never used
    // in motor feedback, wheel odometry, or the localization pipeline above.
    msgs::Odometry truth;
    truth.mutable_header()->mutable_stamp()->CopyFrom(odom.header().stamp());
    auto truthFrame = truth.mutable_header()->add_data();
    truthFrame->set_key("frame_id");
    truthFrame->add_value("world");
    auto truthChild = truth.mutable_header()->add_data();
    truthChild->set_key("child_frame_id");
    truthChild->add_value(robot + "/chassis");
    auto pose = worldPose(chassis, ecm);
    msgs::Set(truth.mutable_pose(), pose);
    truthPub.Publish(truth);
    previous = position;
    lastSample = now;
  }
};
} // namespace kaohe
IGNITION_ADD_PLUGIN(kaohe::WheelTractionDrive, ignition::gazebo::System,
                    kaohe::WheelTractionDrive::ISystemConfigure,
                    kaohe::WheelTractionDrive::ISystemPreUpdate,
                    kaohe::WheelTractionDrive::ISystemPostUpdate)
IGNITION_ADD_PLUGIN_ALIAS(kaohe::WheelTractionDrive,
                          "kaohe::WheelTractionDrive")
