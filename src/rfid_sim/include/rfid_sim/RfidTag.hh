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

#ifndef RFID_SIM__RFIDTAG_HH_
#define RFID_SIM__RFIDTAG_HH_

#include <memory>

#include <gz/sim/System.hh>

namespace autofactory
{
namespace rfid
{
/// \brief System plugin that marks the link it is attached to as an RFID tag
/// holding a fixed data payload.
///
/// Attach it under a `<model>` or `<link>` in SDF:
/// \code{.xml}
/// <plugin filename="rfid_sim_systems" name="autofactory::rfid::RfidTag">
///   <id>tag-A17</id>
///   <data encoding="hex">DEADBEEF36</data>
/// </plugin>
/// \endcode
///
/// The plugin only implements ISystemConfigure: on load it decodes `<data>`
/// per the `encoding` attribute and attaches an `RfidTagData` component to its
/// entity. It performs no per-step work; readers discover it via the component.
class RfidTag :
  public gz::sim::System,
  public gz::sim::ISystemConfigure
{
  /// \brief Constructor.
  public: RfidTag() = default;

  /// \brief Destructor.
  public: ~RfidTag() override = default;

  // Documentation inherited.
  public: void Configure(
      const gz::sim::Entity & _entity,
      const std::shared_ptr<const sdf::Element> & _sdf,
      gz::sim::EntityComponentManager & _ecm,
      gz::sim::EventManager & _eventMgr) override;
};
}  // namespace rfid
}  // namespace autofactory

#endif  // RFID_SIM__RFIDTAG_HH_
