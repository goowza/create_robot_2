/**
Software License Agreement (BSD)

\file      data.h
\authors   Jacob Perron <jperron@sfu.ca>
\copyright Copyright (c) 2015, Autonomy Lab (Simon Fraser University), All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
 * Neither the name of Autonomy Lab nor the names of its contributors may
   be used to endorse or promote products derived from this software without
   specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef CREATE_DATA_H
#define CREATE_DATA_H

#include <map>
#include <memory>
#include <vector>

#include "libcreate/packet.h"
#include "libcreate/types.h"

namespace create {
  class Data {
    private:
      std::map<uint8_t, std::shared_ptr<Packet> > packets;
      uint32_t totalDataBytes;
      std::vector<uint8_t> ids;

    public:
      Data(ProtocolVersion version = V_3);
      ~Data();

      bool isValidPacketID(const uint8_t id) const;
      std::shared_ptr<Packet> getPacket(const uint8_t id);
      void validateAll();
      uint32_t getTotalDataBytes() const;
      uint8_t getNumPackets() const;
      std::vector<uint8_t> getPacketIDs();
  };
}  // namespace create

#endif // CREATE_DATA_H
