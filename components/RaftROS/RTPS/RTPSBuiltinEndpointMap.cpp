#include "RTPSBuiltinEndpointMap.h"

#include "runtime/reliability/RTPSReliabilityAndWriterStateRuntime.h"

const uint8_t* RTPSBuiltinEndpointMap_localReaderForRemoteWriter(const uint8_t* remoteWriterEID,
                                                                 const uint8_t* fallbackReaderEID)
{
    return RaftROS::RTPS::Runtime::ReliabilityAndWriterState::localReaderForRemoteWriter(
        remoteWriterEID,
        fallbackReaderEID);
}
