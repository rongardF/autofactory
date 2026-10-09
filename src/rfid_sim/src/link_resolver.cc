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

#include "rfid_sim/link_resolver.hh"

#include <gz/common/Console.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/components/Name.hh>

namespace autofactory
{
namespace rfid
{
//////////////////////////////////////////////////
gz::sim::Entity ResolveReferenceLink(
    const gz::sim::Entity & _entity,
    const std::shared_ptr<const sdf::Element> & _sdf,
    const gz::sim::EntityComponentManager & _ecm,
    const std::string & _pluginLabel)
{
  const gz::sim::Model model(_entity);
  if (!model.Valid(_ecm))
  {
    // The plugin was not attached to a model (e.g. a sensor or visual);
    // use the attached entity itself as the reference frame.
    return _entity;
  }

  // Non-const copy so we can call the non-const sdf::Element getters.
  auto sdf = _sdf->Clone();

  if (sdf->HasElement("link"))
  {
    const std::string linkName = sdf->Get<std::string>("link");
    const gz::sim::Entity linkEntity = model.LinkByName(_ecm, linkName);
    if (linkEntity == gz::sim::kNullEntity)
    {
      gzerr << _pluginLabel << " model has no link named [" << linkName
            << "]." << std::endl;
    }
    return linkEntity;
  }

  const gz::sim::Entity canonical = model.CanonicalLink(_ecm);
  if (canonical == gz::sim::kNullEntity)
  {
    gzwarn << _pluginLabel << " model has no canonical link; using the model "
           << "origin as the reference frame." << std::endl;
    return _entity;
  }
  return canonical;
}
}  // namespace rfid
}  // namespace autofactory
