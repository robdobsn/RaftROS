# Exactly one ROS 2 transport backend is compiled into an image - see
# components/RaftROS/RaftROSBackendSelect.h.  The RTPS runtime and the Zenoh
# SysMod are never linked together; the auto-publish pipeline below the backend
# contract (CDR, device mapping, publisher pool) is shared and always built.
if(CONFIG_RAFTROS_BACKEND_RTPS)
    set(RAFTROS_BACKEND_SRCS
      "components/RaftROS/RTPS/RaftROSRTPS.cpp"
      "components/RaftROS/RTPS/runtime/core/RTPSParticipant.cpp"
      "components/RaftROS/RTPS/runtime/discovery/SPDPHandler.cpp"
      "components/RaftROS/RTPS/runtime/announce/SEDPHandler.cpp"
      "components/RaftROS/RTPS/runtime/schedule/RTPSRuntimeSchedule.cpp"
      "components/RaftROS/RTPS/runtime/announce/RTPSInitialAnnouncePlan.cpp"
      "components/RaftROS/RTPS/runtime/announce/RTPSInitialAnnounceRunner.cpp"
      "components/RaftROS/RTPS/runtime/announce/RTPSWriterHeartbeatRunner.cpp"
      "components/RaftROS/RTPS/runtime/receive/RTPSRxSubmessageRunner.cpp"
      "components/RaftROS/RTPS/runtime/receive/RTPSRunnerAdapterHelpers.cpp"
      "components/RaftROS/RTPS/runtime/discovery/RTPSDiscoveryRuntime.cpp"
      "components/RaftROS/RTPS/runtime/reliability/RTPSReliabilityAndWriterStateRuntime.cpp"
      "components/RaftROS/RTPS/runtime/reliability/RTPSReaderRuntime.cpp"
      "components/RaftROS/RTPS/runtime/reliability/RTPSReaderRunner.cpp"
      "components/RaftROS/RTPS/runtime/wire/RTPSMessage.cpp"
    )
else()
    set(RAFTROS_BACKEND_SRCS
      "components/RaftROS/Zenoh/RaftROSZenoh.cpp"
    )
endif()

# Register the component
idf_component_register(
    NAME
      RaftROS
    SRCS
      ${RAFTROS_BACKEND_SRCS}
      "components/RaftROS/AutoPub/AutoPubCDRSerializer.cpp"
      "components/RaftROS/CDR/CDREncoder.cpp"
      "components/RaftROS/CDR/CDRDecoder.cpp"
    INCLUDE_DIRS
      "components/RaftROS"
      "components/RaftROS/RTPS"
      "components/RaftROS/CDR"
    REQUIRES
      RaftCore
      esp_hw_support
      esp_netif
      esp_timer
      lwip
    )
