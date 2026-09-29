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

#include "rfid_sim/RfidReader.hh"

#include <limits>
#include <string>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>

#include "rfid_sim/components/RfidTagData.hh"
#include "rfid_sim/link_resolver.hh"

using namespace autofactory::rfid;

//////////////////////////////////////////////////
void RfidReader::Configure(
    const gz::sim::Entity & _entity,
    const std::shared_ptr<const sdf::Element> & _sdf,
    gz::sim::EntityComponentManager & _ecm,
    gz::sim::EventManager & /*_eventMgr*/)
{
  // Resolve the reader link (plugins attach to models in gz-sim, never links).
  this->entity = ResolveReferenceLink(_entity, _sdf, _ecm, "[RfidReader]");
  if (this->entity == gz::sim::kNullEntity)
  {
    gzerr << "[RfidReader] could not resolve a reference link; reader will "
          << "be inactive." << std::endl;
    return;
  }

  // Non-const copy so we can call the non-const sdf::Element getters.
  auto sdf = _sdf->Clone();

  // Resolve a default id from the entity name, if present.
  std::string defaultId;
  if (auto nameComp = _ecm.Component<gz::sim::components::Name>(_entity))
  {
    defaultId = nameComp->Data();
  }
  this->readerId = sdf->Get<std::string>("reader_id", defaultId).first;

  this->range = sdf->Get<double>("range", 0.1).first;
  if (this->range <= 0.0)
  {
    gzerr << "[RfidReader] <range> must be > 0, got [" << this->range
          << "]; falling back to 0.1 m." << std::endl;
    this->range = 0.1;
  }

  // Detection scope prefix based on the reader's scoped name.
  const std::string scoped = gz::sim::scopedName(_entity, _ecm, "/", false);
  const std::string detectedTopic = sdf->Get<std::string>(
      "detected_topic", "/" + scoped + "/rfid/detected").first;
  const std::string contentTopic = sdf->Get<std::string>(
      "content_topic", "/" + scoped + "/rfid/content").first;

  const double updateRate = sdf->Get<double>("update_rate", 0.0).first;
  if (updateRate > 0.0)
  {
    std::chrono::duration<double> period{1.0 / updateRate};
    this->updatePeriod =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
  }

  this->detectedPub = this->node.Advertise<gz::msgs::Boolean>(detectedTopic);
  this->contentPub = this->node.Advertise<gz::msgs::Dataframe>(contentTopic);

  if (!this->detectedPub || !this->contentPub)
  {
    gzerr << "[RfidReader] reader [" << this->readerId << "] failed to "
          << "advertise topics [" << detectedTopic << "] / ["
          << contentTopic << "]." << std::endl;
    return;
  }

  this->valid = true;
  gzmsg << "[RfidReader] reader [" << this->readerId << "] range ["
        << this->range << " m] detected -> [" << detectedTopic
        << "] content -> [" << contentTopic << "]." << std::endl;
}

//////////////////////////////////////////////////
void RfidReader::PostUpdate(
    const gz::sim::UpdateInfo & _info,
    const gz::sim::EntityComponentManager & _ecm)
{
  if (!this->valid || _info.paused)
  {
    return;
  }

  // Throttle to the configured update rate.
  if (this->updatePeriod > std::chrono::steady_clock::duration::zero() &&
      this->published &&
      (_info.simTime - this->lastUpdateTime) < this->updatePeriod)
  {
    return;
  }
  this->lastUpdateTime = _info.simTime;
  this->published = true;

  const gz::math::Pose3d readerPose =
      gz::sim::worldPose(this->entity, _ecm);
  const gz::math::Vector3d readerPos = readerPose.Pos();

  const double rangeSq = this->range * this->range;
  double bestDistSq = std::numeric_limits<double>::max();
  const RfidTagPayload * bestTag{nullptr};

  _ecm.Each<gz::sim::components::RfidTagData, gz::sim::components::Pose>(
      [&](const gz::sim::Entity & _tagEntity,
          const gz::sim::components::RfidTagData * _tag,
          const gz::sim::components::Pose *) -> bool
      {
        const gz::math::Vector3d tagPos =
            gz::sim::worldPose(_tagEntity, _ecm).Pos();
        const double distSq = (readerPos - tagPos).SquaredLength();
        if (distSq <= rangeSq && distSq < bestDistSq)
        {
          bestDistSq = distSq;
          bestTag = &_tag->Data();
        }
        return true;
      });

  // Detection flag.
  gz::msgs::Boolean detectedMsg;
  detectedMsg.set_data(bestTag != nullptr);
  this->detectedPub.Publish(detectedMsg);

  // Content payload (empty Dataframe when no tag is in range).
  gz::msgs::Dataframe contentMsg;
  contentMsg.set_dst_address(this->readerId);
  if (bestTag != nullptr)
  {
    contentMsg.set_src_address(bestTag->id);
    contentMsg.set_data(bestTag->bytes.data(), bestTag->bytes.size());
  }
  this->contentPub.Publish(contentMsg);
}
