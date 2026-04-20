#include "RTPSAckNackRunner.h"
#include "RTPSReliabilityPolicy.h"

void RTPSAckNackRunner_run(
    const uint8_t* srcGuidPrefix,
    const uint8_t* pContent,
    uint32_t contentLen,
    const RTPSAckNackRunnerOptions& options,
    const RTPSAckNackRunnerCallbacks& callbacks,
    void* userCtx)
{
    RTPSAckNackFields fields;
    if (!RTPSAckNack_parse(pContent, contentLen, fields))
        return;

    const RTPSAckNackWriterKind writerKind = RTPSAckNack_classifyWriter(fields.writerEID);
    if (callbacks.logParsed)
        callbacks.logParsed(userCtx, fields, writerKind);

    if (!callbacks.resolveRemote || !callbacks.resolveRemote(userCtx, srcGuidPrefix))
    {
        if (callbacks.unknownRemote)
            callbacks.unknownRemote(userCtx);
        return;
    }

    if (!callbacks.executeAction)
        return;

    switch (writerKind)
    {
        case RTPSAckNackWriterKind::SedpPublications:
        {
            const bool gatePass = !options.requirePublicationSeq2GateForRetransmit ||
                                  RTPSReliability_acknackRequestsSeq(fields.bitmapBaseLow, 2);
            if (!gatePass)
                return;

            if (RTPSReliability_acknackRequestsSeq(fields.bitmapBaseLow, 1))
            {
                callbacks.executeAction(
                    userCtx,
                    RTPSAckNackRunnerAction::RetransmitSedpRosDiscoveryPublication,
                    fields,
                    writerKind);
            }

            if (options.publicationsIncludesChatterAnnouncement &&
                RTPSReliability_acknackRequestsSeq(fields.bitmapBaseLow, 2))
            {
                callbacks.executeAction(
                    userCtx,
                    RTPSAckNackRunnerAction::RetransmitSedpChatterPublication,
                    fields,
                    writerKind);
            }
            return;
        }

        case RTPSAckNackWriterKind::SedpSubscriptions:
            if (RTPSReliability_acknackRequestsSeq(fields.bitmapBaseLow, 1))
            {
                callbacks.executeAction(
                    userCtx,
                    RTPSAckNackRunnerAction::RetransmitSedpRosDiscoverySubscription,
                    fields,
                    writerKind);
            }
            return;

        case RTPSAckNackWriterKind::RosDiscoveryInfo:
            if (RTPSReliability_acknackRequestsSeq(fields.bitmapBaseLow, 1))
            {
                callbacks.executeAction(
                    userCtx,
                    RTPSAckNackRunnerAction::RetransmitRosDiscoveryInfo,
                    fields,
                    writerKind);
            }
            return;

        case RTPSAckNackWriterKind::Chatter:
        {
            const uint64_t chatterSeq = callbacks.getChatterSeq ? callbacks.getChatterSeq(userCtx) : 0;
            if ((chatterSeq > 0) && RTPSReliability_acknackRequestsSeq(fields.bitmapBaseLow, chatterSeq))
            {
                callbacks.executeAction(
                    userCtx,
                    RTPSAckNackRunnerAction::RetransmitChatterData,
                    fields,
                    writerKind);
            }
            return;
        }

        default:
            return;
    }
}
