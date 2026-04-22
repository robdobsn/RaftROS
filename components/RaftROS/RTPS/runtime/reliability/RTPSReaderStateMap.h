#pragma once

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// RTPS Reader State Map - per-(remote participant, writer entity id) reader state container.
//
// Used by RTPS wrappers to plug into RTPSRxSubmessageRunnerCallbacks::resolveReaderWriterState.
// Creates-on-miss, returns-on-hit. Header-only to keep build wiring minimal.
//
// Rob Dobson 2026
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#include <array>
#include <cstdint>
#include <cstring>
#include <map>

#include "runtime/reliability/RTPSReaderRuntime.h"

namespace RaftRuntime::RTPS::Runtime::Reader
{

class RTPSReaderStateMap
{
public:
    // Key: 12-byte remote guidPrefix followed by 4-byte writer entity id.
    using Key = std::array<uint8_t, 16>;

    // Lookup or insert a reader state for the given (guidPrefix, writerEID).
    // Newly created entries are initialized with the supplied reliability kind.
    RTPSReaderWriterState* getOrCreate(const uint8_t* guidPrefix,
                                       const uint8_t* writerEID,
                                       RTPSReaderReliabilityKind defaultKind = RTPSReaderReliabilityKind::Reliable)
    {
        if (!guidPrefix || !writerEID)
            return nullptr;
        Key key{};
        std::memcpy(key.data(), guidPrefix, 12);
        std::memcpy(key.data() + 12, writerEID, 4);
        auto it = _states.find(key);
        if (it == _states.end())
        {
            RTPSReaderWriterState fresh{};
            fresh.reliabilityKind = defaultKind;
            it = _states.emplace(key, fresh).first;
        }
        return &it->second;
    }

    // Lookup only; returns nullptr if not present.
    RTPSReaderWriterState* find(const uint8_t* guidPrefix, const uint8_t* writerEID)
    {
        if (!guidPrefix || !writerEID)
            return nullptr;
        Key key{};
        std::memcpy(key.data(), guidPrefix, 12);
        std::memcpy(key.data() + 12, writerEID, 4);
        auto it = _states.find(key);
        return (it == _states.end()) ? nullptr : &it->second;
    }

    size_t size() const { return _states.size(); }
    void clear() { _states.clear(); }

private:
    std::map<Key, RTPSReaderWriterState> _states;
};

} // namespace RaftRuntime::RTPS::Runtime::Reader
