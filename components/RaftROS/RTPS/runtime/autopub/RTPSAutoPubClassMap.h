#pragma once

#include "AutoPub/AutoPubClassMap.h"

namespace RaftRuntime::RTPS::Runtime::AutoPub
{

using RTPSAutoPubMsgKind = RaftRuntime::AutoPub::AutoPubMsgKind;
using RTPSAutoPubClassMapping = RaftRuntime::AutoPub::AutoPubClassMapping;

inline const char* RTPSAutoPubClassMap_typeName(RTPSAutoPubMsgKind kind)
{
    return RaftRuntime::AutoPub::AutoPubClassMap_typeName(kind);
}

inline bool RTPSAutoPubClassMap_hasClas(const char* const* clasArray, size_t clasCount, const char* code)
{
    return RaftRuntime::AutoPub::AutoPubClassMap_hasClas(clasArray, clasCount, code);
}

inline RTPSAutoPubClassMapping RTPSAutoPubClassMap_lookup(
    const char* const* clasArray, size_t clasCount, const char* deviceTypeName)
{
    return RaftRuntime::AutoPub::AutoPubClassMap_lookup(clasArray, clasCount, deviceTypeName);
}

}
