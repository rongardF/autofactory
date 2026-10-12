// Copyright (c) 2025 FZI Forschungszentrum Informatik
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: Ron Freimann

#ifndef RFID_SIM__RFIDREADER_HH_
#define RFID_SIM__RFIDREADER_HH_

#include <chrono>
#include <string>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/dataframe.pb.h>

#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

namespace autofactory
{
namespace rfid
{
/// \brief System plugin that marks the link it is attached to as an RFID
/// reader with a spherical omnidirectional detection zone.
///
/// Attach it under a `<model>` or `<link>` in SDF:
/// \code{.xml}
/// <plugin filename="rfid_sim_systems" name="autofactory::rfid::RfidReader">
///   <range>0.15</range>
///   <update_rate>30</update_rate>
///   <detected_topic>/rfid/reader0/detected</detected_topic>
///   <content_topic>/rfid/reader0/content</content_topic>
///   <reader_id>reader0</reader_id>
/// </plugin>
/// \endcode
///
/// On each throttled PostUpdate it reads its own world pose, finds the nearest
/// RfidTag within `<range>` and publishes a Boolean "detected" flag and a
/// Dataframe carrying the detected tag's payload (empty when none in range).
class RfidReader :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPostUpdate
{
  /// \brief Constructor.
  public: RfidReader() = default;

  /// \brief Destructor.
  public: ~RfidReader() override = default;

  // Documentation inherited.
  public: void Configure(
      const gz::sim::Entity & _entity,
      const std::shared_ptr<const sdf::Element> & _sdf,
      gz::sim::EntityComponentManager & _ecm,
      gz::sim::EventManager & _eventMgr) override;

  // Documentation inherited.
  public: void PostUpdate(
      const gz::sim::UpdateInfo & _info,
      const gz::sim::EntityComponentManager & _ecm) override;

  /// \brief The reader link entity this plugin is attached to.
  private: gz::sim::Entity entity{gz::sim::kNullEntity};

  /// \brief Detection sphere radius in metres.
  private: double range{0.1};

  /// \brief Identifier placed into Dataframe.dst_address.
  private: std::string readerId;

  /// \brief Transport node used for publishing.
  private: gz::transport::Node node;

  /// \brief Publisher for the boolean detection flag.
  private: gz::transport::Node::Publisher detectedPub;

  /// \brief Publisher for the detected tag content payload.
  private: gz::transport::Node::Publisher contentPub;

  /// \brief Publication period; zero means publish every step.
  private: std::chrono::steady_clock::duration updatePeriod{0};

  /// \brief Simulation time of the last publication.
  private: std::chrono::steady_clock::duration lastUpdateTime{0};

  /// \brief True once the first publication cycle has run.
  private: bool published{false};

  /// \brief True when transport publishers were set up successfully.
  private: bool valid{false};
};
}  // namespace rfid
}  // namespace autofactory

#endif  // RFID_SIM__RFIDREADER_HH_
