#pragma once

#include <stdint.h>

// Resolve the local reader endpoint for a remote writer endpoint.
// If no known mapping exists, returns fallbackReaderEID when provided, otherwise ENTITYID_UNKNOWN.
const uint8_t* RTPSBuiltinEndpointMap_localReaderForRemoteWriter(const uint8_t* remoteWriterEID,
                                                                 const uint8_t* fallbackReaderEID);
