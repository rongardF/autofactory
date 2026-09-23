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

#ifndef RFID_SIM__COMPONENTS__RFIDTAGDATA_HH_
#define RFID_SIM__COMPONENTS__RFIDTAGDATA_HH_

#include <cstdint>
#include <string>
#include <vector>

#include <gz/sim/components/Component.hh>
#include <gz/sim/components/Factory.hh>
#include <gz/sim/config.hh>

namespace autofactory
{
namespace rfid
{
/// \brief Plain payload carried by an RFID tag entity.
///
/// Attached to a tag's link entity by the RfidTag system plugin and read back
/// by the RfidReader system plugin. Because both plugins live in the same
/// shared library the component type identity is shared between them.
struct RfidTagPayload
{
  /// \brief Tag identifier. Copied into Dataframe.src_address on read.
  std::string id;

  /// \brief Raw payload bytes. Copied into Dataframe.data on read.
  std::vector<uint8_t> bytes;
};
}  // namespace rfid
}  // namespace autofactory

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace components
{
/// \brief Custom ECS component that marks an entity as an RFID tag and stores
/// its payload. Not serialized; it is only consumed in-process by RfidReader.
using RfidTagData =
    Component<autofactory::rfid::RfidTagPayload, class RfidTagDataTag>;
GZ_SIM_REGISTER_COMPONENT("autofactory.rfid.RfidTagData", RfidTagData)
}  // namespace components
}
}  // namespace sim
}  // namespace gz

#endif  // RFID_SIM__COMPONENTS__RFIDTAGDATA_HH_
