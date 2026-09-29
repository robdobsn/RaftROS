/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// AutoPubParameterStore.h
//
// The node's ROS 2 parameters and the six services that expose them:
// list / get / get_types / set / set_atomically / describe.  Each service
// handler takes the request CDR and writes the response CDR, which is what
// a Raw service in AutoPubServiceRegistry hands it.  Values live here; a
// parameter's `onSet` callback decides what a change means (apply it,
// persist it, or refuse it with a reason).
//
// Scalars only (bool, int64, double, string); a fixed table; no heap after
// declaration except the std::function callbacks.  RaftCore-free.
//
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include <cstdint>
#include <cstring>
#include <cstdio>
#include <functional>
#include "AutoPubParamCodec.h"

namespace RaftRuntime::AutoPub
{

/// @brief Called when a set has passed the store's checks (type, read-only,
/// size), before the value is stored.  Apply or persist the value here;
/// return false with `reason` set to refuse it instead.
using AutoPubParamSetCallback = std::function<bool(const AutoPubParamValue& newValue, const char*& reason)>;

template <uint8_t CAPACITY = 16>
class AutoPubParameterStore
{
public:
    static constexpr uint8_t INVALID_SLOT = 0xFF;
    static constexpr uint32_t MAX_PER_REQUEST = CAPACITY;   ///< Names or parameters read from one request

    /// @brief Declare a parameter.  Fails on a full table, a duplicate name,
    /// a name or string value that does not fit, or an array type.
    uint8_t declare(const char* name, const AutoPubParamValue& value, const char* description = "",
                    bool readOnly = false, AutoPubParamSetCallback onSet = {})
    {
        if (!name || !*name || std::strlen(name) >= AUTOPUB_PARAM_NAME_MAX || find(name) != INVALID_SLOT ||
            value.type == AutoPubParamType::NotSet || value.type > AutoPubParamType::String || value.stringTooLong)
            return INVALID_SLOT;
        for (uint8_t slot = 0; slot < CAPACITY; ++slot)
        {
            Param& p = _params[slot];
            if (p.inUse)
                continue;
            p.inUse = true;
            std::strcpy(p.name, name);
            p.value = value;
            p.description = description ? description : "";
            p.readOnly = readOnly;
            p.onSet = std::move(onSet);
            ++_count;
            return slot;
        }
        return INVALID_SLOT;
    }

    uint8_t find(const char* name) const
    {
        if (!name)
            return INVALID_SLOT;
        for (uint8_t slot = 0; slot < CAPACITY; ++slot)
            if (_params[slot].inUse && std::strcmp(_params[slot].name, name) == 0)
                return slot;
        return INVALID_SLOT;
    }
    const AutoPubParamValue* value(const char* name) const
    {
        const uint8_t slot = find(name);
        return slot == INVALID_SLOT ? nullptr : &_params[slot].value;
    }
    /// @brief Change a value from the application side (no callback, no checks beyond type)
    bool setLocal(const char* name, const AutoPubParamValue& value)
    {
        const uint8_t slot = find(name);
        if (slot == INVALID_SLOT || _params[slot].value.type != value.type)
            return false;
        _params[slot].value = value;
        return true;
    }
    uint8_t count() const { return _count; }

    // ---- The six services: request CDR in, response CDR out (0 = could not build) ----

    /// @brief ListParameters.  A name matches when there are no prefixes, or
    /// it equals a prefix, or begins with `prefix.`; depth 0 is unlimited,
    /// otherwise the levels ('.' separated) below the prefix are limited.
    uint32_t list(const uint8_t* request, uint32_t length, uint8_t* out, uint32_t capacity) const
    {
        char prefixes[4][AUTOPUB_PARAM_NAME_MAX];
        uint32_t prefixCount = 0;
        uint64_t depth = 0;
        if (!AutoPubParamCodec_readListRequest(request, length, prefixes, 4, prefixCount, depth))
            return 0;
        if (prefixCount > 4)
            prefixCount = 4;
        const char* names[CAPACITY];
        uint32_t nameCount = 0;
        for (uint8_t slot = 0; slot < CAPACITY; ++slot)
        {
            const Param& p = _params[slot];
            if (!p.inUse)
                continue;
            bool matched = prefixCount == 0 && (depth == 0 || levels(p.name) < depth);
            for (uint32_t index = 0; index < prefixCount && !matched; ++index)
            {
                const size_t prefixLen = std::strlen(prefixes[index]);
                if (prefixLen == 0 || std::strncmp(p.name, prefixes[index], prefixLen) != 0)
                    continue;
                if (p.name[prefixLen] == '\0')
                    matched = true;
                else if (p.name[prefixLen] == '.')
                    matched = depth == 0 || levels(p.name + prefixLen + 1) < depth;
            }
            if (matched)
                names[nameCount++] = p.name;
        }
        // Prefixes in the result: the part before the last '.' of each matched
        // name, once each (our names are flat, so normally none)
        const char* resultPrefixes[CAPACITY];
        char prefixStore[CAPACITY][AUTOPUB_PARAM_NAME_MAX];
        uint32_t resultPrefixCount = 0;
        for (uint32_t index = 0; index < nameCount; ++index)
        {
            const char* dot = std::strrchr(names[index], '.');
            if (!dot)
                continue;
            const size_t len = (size_t)(dot - names[index]);
            bool seen = false;
            for (uint32_t other = 0; other < resultPrefixCount && !seen; ++other)
                seen = std::strlen(resultPrefixes[other]) == len && std::strncmp(resultPrefixes[other], names[index], len) == 0;
            if (seen)
                continue;
            std::memcpy(prefixStore[resultPrefixCount], names[index], len);
            prefixStore[resultPrefixCount][len] = '\0';
            resultPrefixes[resultPrefixCount] = prefixStore[resultPrefixCount];
            ++resultPrefixCount;
        }
        return AutoPubParamCodec_writeListResult(out, capacity, names, nameCount, resultPrefixes, resultPrefixCount);
    }

    /// @brief GetParameters: a value per requested name; unknown names get NotSet, as rclcpp answers
    uint32_t get(const uint8_t* request, uint32_t length, uint8_t* out, uint32_t capacity) const
    {
        auto& names = _scratchNames;
        uint32_t count = 0;
        if (!AutoPubParamCodec_readNames(request, length, names, MAX_PER_REQUEST, count) || count > MAX_PER_REQUEST)
            return 0;
        auto& values = _scratchValues;
        for (uint32_t index = 0; index < MAX_PER_REQUEST; ++index)
            values[index] = AutoPubParamValue();
        for (uint32_t index = 0; index < count; ++index)
        {
            const uint8_t slot = find(names[index]);
            if (slot != INVALID_SLOT)
                values[index] = _params[slot].value;
        }
        return AutoPubParamCodec_writeValues(out, capacity, values, count);
    }

    /// @brief GetParameterTypes: a type per requested name; unknown names get NotSet
    uint32_t getTypes(const uint8_t* request, uint32_t length, uint8_t* out, uint32_t capacity) const
    {
        auto& names = _scratchNames;
        uint32_t count = 0;
        if (!AutoPubParamCodec_readNames(request, length, names, MAX_PER_REQUEST, count) || count > MAX_PER_REQUEST)
            return 0;
        auto& types = _scratchTypes;
        for (uint32_t index = 0; index < count; ++index)
        {
            const uint8_t slot = find(names[index]);
            types[index] = slot == INVALID_SLOT ? AutoPubParamType::NotSet : _params[slot].value.type;
        }
        return AutoPubParamCodec_writeTypes(out, capacity, types, count);
    }

    /// @brief DescribeParameters: a descriptor per requested name; unknown names are described NotSet
    uint32_t describe(const uint8_t* request, uint32_t length, uint8_t* out, uint32_t capacity) const
    {
        auto& names = _scratchNames;
        uint32_t count = 0;
        if (!AutoPubParamCodec_readNames(request, length, names, MAX_PER_REQUEST, count) || count > MAX_PER_REQUEST)
            return 0;
        auto& descriptors = _scratchDescriptors;
        for (uint32_t index = 0; index < MAX_PER_REQUEST; ++index)
            descriptors[index] = AutoPubParamDescriptor();
        for (uint32_t index = 0; index < count; ++index)
        {
            const uint8_t slot = find(names[index]);
            descriptors[index].name = names[index];
            if (slot == INVALID_SLOT)
                continue;
            descriptors[index].type = _params[slot].value.type;
            descriptors[index].description = _params[slot].description;
            descriptors[index].readOnly = _params[slot].readOnly;
        }
        return AutoPubParamCodec_writeDescriptors(out, capacity, descriptors, count);
    }

    /// @brief SetParameters: each parameter checked and applied on its own, a result each
    uint32_t set(const uint8_t* request, uint32_t length, uint8_t* out, uint32_t capacity)
    {
        auto& entries = _scratchEntries;
        uint32_t count = 0;
        if (!AutoPubParamCodec_readSetRequest(request, length, entries, MAX_PER_REQUEST, count) || count > MAX_PER_REQUEST)
            return 0;
        auto& results = _scratchResults;
        for (uint32_t index = 0; index < count; ++index)
            results[index] = AutoPubParamResult();      // scratch is reused across requests
        for (uint32_t index = 0; index < count; ++index)
            results[index].successful = check(entries[index], results[index].reason, _reasons[index]) &&
                                        apply(entries[index], results[index].reason);
        return AutoPubParamCodec_writeResults(out, capacity, results, count);
    }

    /// @brief SetParametersAtomically: every parameter passes the store's
    /// checks before any is applied.  Owners' callbacks then run in order; one
    /// that refuses stops the rest, but what earlier callbacks applied stays -
    /// the store cannot undo an owner's side effects.
    uint32_t setAtomically(const uint8_t* request, uint32_t length, uint8_t* out, uint32_t capacity)
    {
        auto& entries = _scratchEntries;
        uint32_t count = 0;
        if (!AutoPubParamCodec_readSetRequest(request, length, entries, MAX_PER_REQUEST, count) || count > MAX_PER_REQUEST)
            return 0;
        AutoPubParamResult result{true, ""};
        for (uint32_t index = 0; index < count && result.successful; ++index)
            result.successful = check(entries[index], result.reason, _reasons[0]);
        for (uint32_t index = 0; index < count && result.successful; ++index)
            result.successful = apply(entries[index], result.reason);
        return AutoPubParamCodec_writeSingleResult(out, capacity, result);
    }

    static const char* typeName(AutoPubParamType type)
    {
        switch (type)
        {
            case AutoPubParamType::NotSet:       return "NOT_SET";
            case AutoPubParamType::Bool:         return "BOOL";
            case AutoPubParamType::Integer:      return "INTEGER";
            case AutoPubParamType::Double:       return "DOUBLE";
            case AutoPubParamType::String:       return "STRING";
            case AutoPubParamType::ByteArray:    return "BYTE_ARRAY";
            case AutoPubParamType::BoolArray:    return "BOOL_ARRAY";
            case AutoPubParamType::IntegerArray: return "INTEGER_ARRAY";
            case AutoPubParamType::DoubleArray:  return "DOUBLE_ARRAY";
            case AutoPubParamType::StringArray:  return "STRING_ARRAY";
        }
        return "?";
    }

private:
    struct Param
    {
        bool inUse = false;
        char name[AUTOPUB_PARAM_NAME_MAX] = {};
        AutoPubParamValue value;
        const char* description = "";
        bool readOnly = false;
        AutoPubParamSetCallback onSet;
    };
    Param _params[CAPACITY];
    uint8_t _count = 0;
    char _reasons[MAX_PER_REQUEST][AUTOPUB_PARAM_TEXT_MAX] = {};   ///< A formatted refusal per parameter of one set request
    // Per-request working space, held here rather than on the caller's stack:
    // a 16-name dump needs ~2.3 kB, and the loop task's headroom is ~5 kB.
    // One request at a time (loop task only), so one set serves every service.
    mutable char _scratchNames[MAX_PER_REQUEST][AUTOPUB_PARAM_NAME_MAX] = {};
    mutable AutoPubParamValue _scratchValues[MAX_PER_REQUEST];
    mutable AutoPubParamType _scratchTypes[MAX_PER_REQUEST] = {};
    mutable AutoPubParamDescriptor _scratchDescriptors[MAX_PER_REQUEST];
    AutoPubParamEntry _scratchEntries[MAX_PER_REQUEST];
    AutoPubParamResult _scratchResults[MAX_PER_REQUEST];

    static uint64_t levels(const char* name)
    {
        uint64_t n = 0;
        for (; *name; ++name)
            n += *name == '.';
        return n;
    }

    /// @brief Can this parameter take this value?  Mirrors the reasons a
    /// Jazzy node gives, so the tools show familiar text.
    bool check(const AutoPubParamEntry& entry, const char*& reason, char* reasonBuf)
    {
        const uint8_t slot = entry.nameTooLong ? INVALID_SLOT : find(entry.name);
        if (slot == INVALID_SLOT)
        {
            std::snprintf(reasonBuf, AUTOPUB_PARAM_TEXT_MAX, "Invalid access to undeclared parameter(s): [%s]", entry.name);
            reason = reasonBuf;
            return false;
        }
        const Param& p = _params[slot];
        if (p.readOnly)
        {
            std::snprintf(reasonBuf, AUTOPUB_PARAM_TEXT_MAX, "Trying to set a read-only parameter: %s.", p.name);
            reason = reasonBuf;
            return false;
        }
        if (entry.value.type == AutoPubParamType::NotSet)
        {
            reason = "Parameters cannot be deleted on this node";
            return false;
        }
        if (entry.value.type != p.value.type)
        {
            std::snprintf(reasonBuf, AUTOPUB_PARAM_TEXT_MAX, "Wrong parameter type, expected 'Type.%s' got 'Type.%s'",
                          typeName(p.value.type), typeName(entry.value.type));
            reason = reasonBuf;
            return false;
        }
        if (entry.value.hasArrayData)
        {
            reason = "Array parameters are not supported on this node";
            return false;
        }
        if (entry.value.stringTooLong)
        {
            std::snprintf(reasonBuf, AUTOPUB_PARAM_TEXT_MAX, "String values are limited to %u characters",
                          (unsigned)(AUTOPUB_PARAM_STRING_MAX - 1));
            reason = reasonBuf;
            return false;
        }
        return true;
    }
    /// @brief Give the owner the value, then store it
    bool apply(const AutoPubParamEntry& entry, const char*& reason)
    {
        const uint8_t slot = find(entry.name);
        if (slot == INVALID_SLOT)
            return false;
        Param& p = _params[slot];
        if (p.onSet)
        {
            const char* callbackReason = nullptr;
            if (!p.onSet(entry.value, callbackReason))
            {
                reason = callbackReason ? callbackReason : "Refused by the parameter's owner";
                return false;
            }
        }
        p.value = entry.value;
        return true;
    }
};

} // namespace RaftRuntime::AutoPub
