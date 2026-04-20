#include "RTPSBuiltinEndpointMap.h"
#include "RTPSTypes.h"

#include <string.h>

const uint8_t* RTPSBuiltinEndpointMap_localReaderForRemoteWriter(const uint8_t* remoteWriterEID,
                                                                 const uint8_t* fallbackReaderEID)
{
    if (!remoteWriterEID)
        return fallbackReaderEID ? fallbackReaderEID : ENTITYID_UNKNOWN;

    if (memcmp(remoteWriterEID, ENTITYID_SEDP_BUILTIN_PUBLICATIONS_WRITER, 4) == 0)
        return ENTITYID_SEDP_BUILTIN_PUBLICATIONS_READER;
    if (memcmp(remoteWriterEID, ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_WRITER, 4) == 0)
        return ENTITYID_SEDP_BUILTIN_SUBSCRIPTIONS_READER;
    if (memcmp(remoteWriterEID, ENTITYID_SPDP_BUILTIN_PARTICIPANT_WRITER, 4) == 0)
        return ENTITYID_SPDP_BUILTIN_PARTICIPANT_READER;
    if (memcmp(remoteWriterEID, ENTITYID_P2P_BUILTIN_PARTICIPANT_MESSAGE_WRITER, 4) == 0)
        return ENTITYID_P2P_BUILTIN_PARTICIPANT_MESSAGE_READER;

    return fallbackReaderEID ? fallbackReaderEID : ENTITYID_UNKNOWN;
}
