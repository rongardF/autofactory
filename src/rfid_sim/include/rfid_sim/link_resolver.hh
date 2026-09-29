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

#ifndef RFID_SIM__LINK_RESOLVER_HH_
#define RFID_SIM__LINK_RESOLVER_HH_

#include <memory>
#include <string>

#include <gz/sim/Entity.hh>
#include <gz/sim/EntityComponentManager.hh>

#include <sdf/Element.hh>

namespace autofactory
{
namespace rfid
{
/// \brief Resolve the reference link for a model-attached RFID plugin.
///
/// gz-sim only loads system plugins on model (and world/sensor/visual/actor)
/// entities, never on links directly, so the RFID plugins attach to a
/// `<model>` and pick a link inside it as their spatial reference:
///   * `<link>NAME</link>` selects a specific child link by name.
///   * otherwise the model's canonical link is used.
///
/// \param[in] _entity The entity the plugin is attached to (expected: a model).
/// \param[in] _sdf The plugin's SDF element.
/// \param[in] _ecm The entity-component manager.
/// \param[in] _pluginLabel Prefix used in log messages, e.g. "[RfidReader]".
/// \return The resolved link entity, or gz::sim::kNullEntity on failure.
gz::sim::Entity ResolveReferenceLink(
    const gz::sim::Entity & _entity,
    const std::shared_ptr<const sdf::Element> & _sdf,
    const gz::sim::EntityComponentManager & _ecm,
    const std::string & _pluginLabel);
}  // namespace rfid
}  // namespace autofactory

#endif  // RFID_SIM__LINK_RESOLVER_HH_
