// Assessment only: periodic complete body-contact evidence, including no contact.
// The existing event-only contact topic and actuator gates are unchanged.
#include <chrono>
#include <sstream>
#include <ignition/gazebo/System.hh>
#include <ignition/gazebo/components/Collision.hh>
#include <ignition/gazebo/components/ContactSensor.hh>
#include <ignition/gazebo/components/ContactSensorData.hh>
#include <ignition/gazebo/components/Name.hh>
#include <ignition/gazebo/components/ParentEntity.hh>
#include <ignition/msgs/stringmsg.pb.h>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>

namespace kaohe {
using namespace ignition;
using namespace gazebo;
class BodyContactEvidence : public System, public ISystemConfigure,
                            public ISystemPostUpdate {
  Entity model{kNullEntity};
  transport::Node node;
  transport::Node::Publisher publisher;
  uint64_t contacts = 0;
  double lastPublish = -1;
  size_t expectedSensors = 0;
  bool intervalComplete = true;

public:
  void Configure(const Entity &entity, const std::shared_ptr<const sdf::Element> &,
                 EntityComponentManager &, EventManager &) override {
    model = entity;
    publisher = node.Advertise<msgs::StringMsg>("/assessment/body_contact_status");
  }

  void PostUpdate(const UpdateInfo &info, const EntityComponentManager &ecm) override {
    if (info.paused) return;
    const double now = std::chrono::duration<double>(info.simTime).count();
    if (info.dt < std::chrono::steady_clock::duration::zero()) {
      contacts = 0;
      lastPublish = -1;
      return;
    }
    bool complete = true;
    size_t sensors = 0;
    // Resolve the actual configured body sensors, never all collision entities:
    // wheel-floor support must not become a body-contact failure.
    ecm.Each<components::ContactSensor, components::ParentEntity>(
        [&](const Entity &, const components::ContactSensor *sensor,
            const components::ParentEntity *parent) {
          auto linkParent = ecm.Component<components::ParentEntity>(parent->Data());
          if (!linkParent || linkParent->Data() != model) return true;
          auto contact = sensor->Data()->GetElement("contact");
          if (contact->Get<std::string>("topic", "").first != "/assessment/body_contacts")
            return true;
          ++sensors;
          if (!contact->HasElement("collision")) { complete = false; return true; }
          for (auto collision = contact->GetElement("collision"); collision;
               collision = collision->GetNextElement("collision")) {
            const auto entities = ecm.ChildrenByComponents(parent->Data(),
                components::Collision(), components::Name(collision->Get<std::string>()));
            if (entities.size() != 1) { complete = false; continue; }
            auto data = ecm.Component<components::ContactSensorData>(entities.front());
            if (!data) { complete = false; continue; }
            contacts += data->Data().contact_size();
          }
          return true;
        });
    if (expectedSensors == 0 && complete && sensors > 0) expectedSensors = sensors;
    intervalComplete = intervalComplete && complete && sensors > 0 && sensors == expectedSensors;
    // Accumulate every physics step, preserving impacts shorter than 50 ms.
    if (lastPublish >= 0 && now-lastPublish < .05) return;
    const auto ns = std::chrono::duration_cast<std::chrono::nanoseconds>(info.simTime).count();
    std::ostringstream payload;
    payload << "{\"stamp_ns\":" << ns << ",\"contacts\":" << contacts
            << ",\"complete\":" << (intervalComplete ? "true" : "false")
            << ",\"sensors\":" << sensors << ",\"expected_sensors\":" << expectedSensors << "}";
    msgs::StringMsg status;
    status.set_data(payload.str());
    publisher.Publish(status);
    contacts = 0;
    lastPublish = now;
    intervalComplete = true;
  }
};
}  // namespace kaohe
IGNITION_ADD_PLUGIN(kaohe::BodyContactEvidence, ignition::gazebo::System,
                    kaohe::BodyContactEvidence::ISystemConfigure,
                    kaohe::BodyContactEvidence::ISystemPostUpdate)
IGNITION_ADD_PLUGIN_ALIAS(kaohe::BodyContactEvidence, "kaohe::BodyContactEvidence")
