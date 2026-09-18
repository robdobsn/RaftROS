#pragma once

#include "AutoPub/AutoPubCDRSerializer.h"
#include "RTPSAutoPubClassMap.h"

namespace RaftRuntime::RTPS::Runtime::AutoPub
{

using RTPSAutoPubAttrType = RaftRuntime::AutoPub::AutoPubAttrType;
using RTPSAutoPubAttrFieldDesc = RaftRuntime::AutoPub::AutoPubAttrFieldDesc;
using RTPSAutoPubCDRContext = RaftRuntime::AutoPub::AutoPubCDRContext;

inline bool RTPSAutoPubCDRSerializer_serialize(
        RTPSAutoPubMsgKind kind,
        const RTPSAutoPubCDRContext& ctx,
        uint8_t* pOutBuf, uint32_t outBufLen,
        uint32_t& outBytesWritten)
{
    return RaftRuntime::AutoPub::AutoPubCDRSerializer_serialize(
        kind, ctx, pOutBuf, outBufLen, outBytesWritten);
}

inline bool RTPSAutoPubCDRSerializer_serializeString(
        const char* pText,
        uint8_t* pOutBuf, uint32_t outBufLen,
        uint32_t& outBytesWritten)
{
    return RaftRuntime::AutoPub::AutoPubCDRSerializer_serializeString(
        pText, pOutBuf, outBufLen, outBytesWritten);
}

inline bool RTPSAutoPubCDRSerializer_readFieldDouble(
        const RTPSAutoPubCDRContext& ctx,
        const char* name,
        double& out,
        bool applyScale = true)
{
    return RaftRuntime::AutoPub::AutoPubCDRSerializer_readFieldDouble(ctx, name, out, applyScale);
}

inline bool RTPSAutoPubCDRSerializer_readFieldInt32(
        const RTPSAutoPubCDRContext& ctx,
        const char* name,
        int32_t& out,
        bool applyScale = true)
{
    return RaftRuntime::AutoPub::AutoPubCDRSerializer_readFieldInt32(ctx, name, out, applyScale);
}

inline bool RTPSAutoPubCDRSerializer_readFieldBool(
        const RTPSAutoPubCDRContext& ctx,
        const char* name,
        bool& out)
{
    return RaftRuntime::AutoPub::AutoPubCDRSerializer_readFieldBool(ctx, name, out);
}

}
