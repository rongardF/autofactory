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

#ifndef RFID_SIM__ENCODING_HH_
#define RFID_SIM__ENCODING_HH_

#include <cstdint>
#include <string>
#include <vector>

namespace autofactory
{
namespace rfid
{
/// \brief Decode an SDF `<data>` string into raw payload bytes.
///
/// \param[in] _text The raw text from the SDF `<data>` element.
/// \param[in] _encoding One of "plain", "hex" or "base64" (case-insensitive).
/// \param[out] _out Decoded bytes on success; left unchanged on failure.
/// \param[out] _error Human-readable error message when decoding fails.
/// \return True on success, false on malformed input or unknown encoding.
bool DecodePayload(
    const std::string & _text,
    const std::string & _encoding,
    std::vector<uint8_t> & _out,
    std::string & _error);
}  // namespace rfid
}  // namespace autofactory

#endif  // RFID_SIM__ENCODING_HH_
