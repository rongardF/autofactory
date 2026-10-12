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

#include "rfid_sim/RfidTag.hh"

#include <string>
#include <vector>

#include <gz/common/Console.hh>
#include <gz/sim/components/Name.hh>

#include "rfid_sim/components/RfidTagData.hh"
#include "rfid_sim/encoding.hh"
#include "rfid_sim/link_resolver.hh"

using namespace autofactory::rfid;

//////////////////////////////////////////////////
void RfidTag::Configure(
    const gz::sim::Entity & _entity,
    const std::shared_ptr<const sdf::Element> & _sdf,
    gz::sim::EntityComponentManager & _ecm,
    gz::sim::EventManager & /*_eventMgr*/)
{
  // Resolve a default tag id from the entity name, if present.
  std::string defaultId;
  if (auto nameComp = _ecm.Component<gz::sim::components::Name>(_entity))
  {
    defaultId = nameComp->Data();
  }

  // Non-const copy so we can call the non-const sdf::Element getters.
  auto sdf = _sdf->Clone();

  const std::string id = sdf->Get<std::string>("id", defaultId).first;

  // Resolve the link this tag lives on (plugins attach to models in gz-sim).
  const gz::sim::Entity linkEntity =
      ResolveReferenceLink(_entity, _sdf, _ecm, "[RfidTag]");
  if (linkEntity == gz::sim::kNullEntity)
  {
    gzerr << "[RfidTag] tag [" << id << "] could not resolve a reference "
          << "link; tag will not be registered." << std::endl;
    return;
  }

  if (!sdf->HasElement("data"))
  {
    gzerr << "[RfidTag] entity [" << id << "] has no <data> element; "
          << "tag will hold an empty payload." << std::endl;
  }

  auto dataElem = sdf->GetElement("data");
  const std::string rawText = dataElem->Get<std::string>();
  const std::string encoding =
      dataElem->Get<std::string>("encoding", "plain").first;

  std::vector<uint8_t> bytes;
  std::string error;
  if (!DecodePayload(rawText, encoding, bytes, error))
  {
    gzerr << "[RfidTag] entity [" << id << "] failed to decode <data> with "
          << "encoding [" << encoding << "]: " << error
          << ". Tag will not be registered." << std::endl;
    return;
  }

  const std::size_t byteCount = bytes.size();

  RfidTagPayload payload;
  payload.id = id;
  payload.bytes = std::move(bytes);

  _ecm.CreateComponent(
      linkEntity, gz::sim::components::RfidTagData(std::move(payload)));

  gzmsg << "[RfidTag] registered tag [" << id << "] with "
        << byteCount << " payload byte(s) (encoding [" << encoding
        << "])." << std::endl;
}
