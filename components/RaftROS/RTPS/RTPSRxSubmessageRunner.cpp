#include "RTPSRxSubmessageRunner.h"

#include "RTPSReliabilityPolicy.h"

bool RTPSRxSubmessageRunner_run(
    const uint8_t* packet,
    uint32_t packetLen,
    const struct sockaddr_in& fromAddr,
    RTPSRxChannel channel,
    uint32_t& acknackCount,
    const RTPSRxSubmessageRunnerCallbacks& callbacks,
    void* userCtx)
{
    if (!packet || packetLen == 0)
        return false;

    uint8_t srcGuidPrefix[12];
    uint32_t hdrLen = RTPSMessage::parseHeader(packet, packetLen, srcGuidPrefix);
    if (hdrLen == 0)
    {
        if (callbacks.onInvalidHeader)
            callbacks.onInvalidHeader(userCtx, channel);
        return false;
    }

    uint32_t offset = hdrLen;
    while (offset < packetLen)
    {
        RTPSSubmessageId submsgId;
        uint8_t flags;
        const uint8_t* pContent;
        uint32_t contentLen;
        uint32_t submsgSize = RTPSMessage::parseSubmessage(
            packet + offset, packetLen - offset, submsgId, flags, pContent, contentLen);
        if (submsgSize == 0)
            break;

        if ((submsgId == SUBMSG_HEARTBEAT) && (contentLen >= 28))
        {
            const uint8_t* fallbackReaderEID = pContent;
            const uint8_t* writerEID = pContent + 4;
            uint32_t lastSNHigh = RTPSMessage::readLE32(pContent + 20);
            uint32_t lastSNLow = RTPSMessage::readLE32(pContent + 24);

            bool responded = false;
            int sent = -1;
            if (RTPSReliability_shouldRespondToHeartbeat(flags) &&
                callbacks.getLocalGuidPrefix && callbacks.resolveReaderEID &&
                callbacks.resolveAckDest && callbacks.sendAck)
            {
                const uint8_t* ourReaderEID = callbacks.resolveReaderEID(
                    userCtx, channel, writerEID, fallbackReaderEID);
                struct sockaddr_in ackDest = {};
                if (ourReaderEID && callbacks.resolveAckDest(userCtx, channel, fromAddr, ackDest))
                {
                    uint8_t ackBuf[80];
                    uint32_t pos = 0;
                    pos += RTPSMessage::writeHeader(
                        ackBuf + pos, sizeof(ackBuf) - pos, callbacks.getLocalGuidPrefix(userCtx));
                    pos += RTPSMessage::writeInfoDST(
                        ackBuf + pos, sizeof(ackBuf) - pos, srcGuidPrefix);

                    uint32_t ackBaseLow = lastSNLow + 1;
                    uint32_t ackBaseHigh = lastSNHigh + (ackBaseLow == 0 ? 1 : 0);
                    acknackCount++;
                    pos += RTPSMessage::writeAcknack(
                        ackBuf + pos, sizeof(ackBuf) - pos,
                        ourReaderEID, writerEID,
                        (int32_t)ackBaseHigh, ackBaseLow, acknackCount);

                    sent = callbacks.sendAck(userCtx, ackBuf, pos, ackDest);
                    responded = true;
                }
            }

            if (callbacks.onHeartbeat)
                callbacks.onHeartbeat(userCtx, channel, writerEID, lastSNLow, responded, sent);
        }
        else if ((submsgId == SUBMSG_DATA) && (contentLen >= 24))
        {
            if (callbacks.onData)
                callbacks.onData(userCtx, channel, packet, packetLen, srcGuidPrefix, fromAddr, pContent, contentLen);
        }
        else if ((submsgId == SUBMSG_ACKNACK) && (contentLen >= 24))
        {
            if (callbacks.onAckNack)
                callbacks.onAckNack(userCtx, channel, srcGuidPrefix, pContent, contentLen, fromAddr);
        }
        else
        {
            if (callbacks.onOther)
                callbacks.onOther(userCtx, channel, submsgId, flags, contentLen);
        }

        offset += submsgSize;
    }

    return true;
}
