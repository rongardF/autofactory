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

#include "rfid_scanner/RfidReader.hh"

#include <algorithm>
#include <cctype>
#include <limits>
#include <sstream>
#include <string>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>

#include "rfid_scanner/components/RfidTagData.hh"
#include "rfid_scanner/link_resolver.hh"

using namespace autofactory::rfid;

namespace
{
/// \brief Parse a full decimal string into a uint32.
/// \return True on success.
bool ParseDecimalU32(const std::string & _text, uint32_t & _out)
{
  if (_text.empty() ||
      !std::all_of(_text.begin(), _text.end(),
        [](unsigned char _c) {return std::isdigit(_c) != 0;}))
  {
    return false;
  }
  try
  {
    std::size_t consumed{0};
    const unsigned long long value = std::stoull(_text, &consumed, 10);
    if (consumed != _text.size() ||
        value > std::numeric_limits<uint32_t>::max())
    {
      return false;
    }
    _out = static_cast<uint32_t>(value);
    return true;
  }
  catch (const std::exception &)
  {
    return false;
  }
}
}  // namespace

//////////////////////////////////////////////////
RfidReader::~RfidReader()
{
  {
    std::lock_guard<std::mutex> lock(this->writeMutex);
    this->shuttingDown = true;
  }
  this->writeCv.notify_all();
  if (this->writeEnabled)
  {
    this->node.UnadvertiseSrv(this->writeService);
  }
}

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

  // Optional write service.
  this->writeService = sdf->Get<std::string>("write_service", "").first;
  if (this->writeService.empty())
  {
    return;
  }

  const double timeout = sdf->Get<double>("write_timeout", 0.5).first;
  if (timeout <= 0.0)
  {
    gzerr << "[RfidReader] <write_timeout> must be > 0, got [" << timeout
          << "]; falling back to 0.5 s." << std::endl;
    this->writeTimeout = std::chrono::duration<double>(0.5);
  }
  else
  {
    this->writeTimeout = std::chrono::duration<double>(timeout);
  }

  if (!this->node.Advertise(
        this->writeService, &RfidReader::OnWriteRequest, this))
  {
    gzerr << "[RfidReader] reader [" << this->readerId << "] failed to "
          << "advertise write service [" << this->writeService << "]."
          << std::endl;
    return;
  }
  this->writeEnabled = true;
  gzmsg << "[RfidReader] reader [" << this->readerId << "] write service ["
        << this->writeService << "] timeout [" << this->writeTimeout.count()
        << " s]." << std::endl;
}

//////////////////////////////////////////////////
bool RfidReader::OnWriteRequest(
    const gz::msgs::Dataframe & _req, gz::msgs::StringMsg & _rep)
{
  uint32_t start{0};
  if (!ParseDecimalU32(_req.dst_address(), start))
  {
    _rep.set_data("invalid start address '" + _req.dst_address() + "'");
    return true;
  }
  if (_req.data().empty())
  {
    _rep.set_data("empty write");
    return true;
  }

  std::unique_lock<std::mutex> lock(this->writeMutex);
  if (this->shuttingDown)
  {
    _rep.set_data("reader shutting down");
    return true;
  }
  if (this->pendingWrite)
  {
    _rep.set_data("write already in progress");
    return true;
  }

  auto request = std::make_shared<PendingWrite>();
  request->tagId = _req.src_address();
  request->start = start;
  request->data.assign(_req.data().begin(), _req.data().end());
  this->pendingWrite = request;

  const bool finished = this->writeCv.wait_for(
      lock, this->writeTimeout,
      [&]() {return request->done || this->shuttingDown;});

  if (this->pendingWrite == request)
  {
    this->pendingWrite.reset();
  }

  if (!finished)
  {
    std::ostringstream err;
    err << "write not applied within " << this->writeTimeout.count()
        << " s (simulation paused?)";
    _rep.set_data(err.str());
    return true;
  }
  if (!request->done)
  {
    _rep.set_data("reader shutting down");
    return true;
  }
  _rep.set_data(request->error);
  return true;
}

//////////////////////////////////////////////////
void RfidReader::PreUpdate(
    const gz::sim::UpdateInfo & _info,
    gz::sim::EntityComponentManager & _ecm)
{
  if (_info.paused || !this->valid || !this->writeEnabled)
  {
    return;
  }

  std::lock_guard<std::mutex> lock(this->writeMutex);
  if (!this->pendingWrite || this->pendingWrite->done)
  {
    return;
  }
  PendingWrite & request = *this->pendingWrite;

  if (this->detectedTagEntity == gz::sim::kNullEntity)
  {
    request.error = "no tag detected";
  }
  else if (this->detectedTagId != request.tagId)
  {
    request.error = "detected tag '" + this->detectedTagId +
      "' does not match expected '" + request.tagId + "'";
  }
  else
  {
    auto * comp = _ecm.Component<gz::sim::components::RfidTagData>(
        this->detectedTagEntity);
    if (comp == nullptr)
    {
      request.error = "tag entity no longer exists";
    }
    else
    {
      auto & bytes = comp->Data().bytes;
      const uint64_t end =
        static_cast<uint64_t>(request.start) + request.data.size();
      if (end > bytes.size())
      {
        std::ostringstream err;
        err << "out of bounds: start " << request.start << " + size "
            << request.data.size() << " > tag memory size " << bytes.size();
        request.error = err.str();
      }
      else
      {
        std::copy(request.data.begin(), request.data.end(),
                  bytes.begin() + request.start);
        _ecm.SetChanged(this->detectedTagEntity,
                        gz::sim::components::RfidTagData::typeId,
                        gz::sim::ComponentState::OneTimeChange);
      }
    }
  }

  request.done = true;
  this->writeCv.notify_all();
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
  gz::sim::Entity bestEntity{gz::sim::kNullEntity};

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
          bestEntity = _tagEntity;
        }
        return true;
      });

  if (this->writeEnabled)
  {
    std::lock_guard<std::mutex> lock(this->writeMutex);
    this->detectedTagEntity = bestEntity;
    this->detectedTagId = bestTag != nullptr ? bestTag->id : std::string();
  }

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
