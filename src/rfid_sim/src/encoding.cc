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

#include "rfid_sim/encoding.hh"

#include <algorithm>
#include <array>
#include <cctype>
#include <cstdint>
#include <string>
#include <vector>

namespace
{
/// \brief Lower-case a copy of a string (ASCII only).
std::string ToLower(std::string _s)
{
  std::transform(_s.begin(), _s.end(), _s.begin(),
      [](unsigned char _c) { return static_cast<char>(std::tolower(_c)); });
  return _s;
}

/// \brief Convert a single hex digit to its 0-15 value; -1 if invalid.
int HexValue(char _c)
{
  if (_c >= '0' && _c <= '9') return _c - '0';
  if (_c >= 'a' && _c <= 'f') return _c - 'a' + 10;
  if (_c >= 'A' && _c <= 'F') return _c - 'A' + 10;
  return -1;
}

/// \brief Decode a hex string (whitespace ignored) into bytes.
bool DecodeHex(
    const std::string & _text,
    std::vector<uint8_t> & _out,
    std::string & _error)
{
  std::string compact;
  compact.reserve(_text.size());
  for (const char c : _text)
  {
    if (std::isspace(static_cast<unsigned char>(c)))
    {
      continue;
    }
    compact.push_back(c);
  }

  if (compact.size() % 2 != 0)
  {
    _error = "hex string has an odd number of digits";
    return false;
  }

  std::vector<uint8_t> bytes;
  bytes.reserve(compact.size() / 2);
  for (std::size_t i = 0; i < compact.size(); i += 2)
  {
    const int hi = HexValue(compact[i]);
    const int lo = HexValue(compact[i + 1]);
    if (hi < 0 || lo < 0)
    {
      _error = "invalid hex digit";
      return false;
    }
    bytes.push_back(static_cast<uint8_t>((hi << 4) | lo));
  }

  _out = std::move(bytes);
  return true;
}

/// \brief Map a base64 character to its 0-63 value; -1 if invalid.
int Base64Value(char _c)
{
  if (_c >= 'A' && _c <= 'Z') return _c - 'A';
  if (_c >= 'a' && _c <= 'z') return _c - 'a' + 26;
  if (_c >= '0' && _c <= '9') return _c - '0' + 52;
  if (_c == '+') return 62;
  if (_c == '/') return 63;
  return -1;
}

/// \brief Decode a standard base64 string (whitespace ignored) into bytes.
bool DecodeBase64(
    const std::string & _text,
    std::vector<uint8_t> & _out,
    std::string & _error)
{
  std::string compact;
  compact.reserve(_text.size());
  for (const char c : _text)
  {
    if (std::isspace(static_cast<unsigned char>(c)))
    {
      continue;
    }
    compact.push_back(c);
  }

  // Count and strip trailing padding.
  std::size_t padding = 0;
  while (!compact.empty() && compact.back() == '=')
  {
    ++padding;
    compact.pop_back();
  }

  if (padding > 2)
  {
    _error = "too many base64 padding characters";
    return false;
  }

  std::vector<uint8_t> bytes;
  bytes.reserve((compact.size() * 3) / 4);

  uint32_t buffer = 0;
  int bitsCollected = 0;
  for (const char c : compact)
  {
    const int value = Base64Value(c);
    if (value < 0)
    {
      _error = "invalid base64 character";
      return false;
    }
    buffer = (buffer << 6) | static_cast<uint32_t>(value);
    bitsCollected += 6;
    if (bitsCollected >= 8)
    {
      bitsCollected -= 8;
      bytes.push_back(static_cast<uint8_t>((buffer >> bitsCollected) & 0xFF));
    }
  }

  _out = std::move(bytes);
  return true;
}
}  // namespace

namespace autofactory
{
namespace rfid
{
//////////////////////////////////////////////////
bool DecodePayload(
    const std::string & _text,
    const std::string & _encoding,
    std::vector<uint8_t> & _out,
    std::string & _error)
{
  const std::string encoding = ToLower(_encoding);

  if (encoding.empty() || encoding == "plain")
  {
    _out.assign(_text.begin(), _text.end());
    return true;
  }

  if (encoding == "hex")
  {
    return DecodeHex(_text, _out, _error);
  }

  if (encoding == "base64")
  {
    return DecodeBase64(_text, _out, _error);
  }

  _error = "unknown encoding [" + _encoding + "]";
  return false;
}
}  // namespace rfid
}  // namespace autofactory
