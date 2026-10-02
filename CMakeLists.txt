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
elseif(CONFIG_RAFTROS_BACKEND_ZENOH_PICO)
    # Experimental: the Zenoh ROS layer over Eclipse zenoh-pico (EPL-2.0 OR
    # Apache-2.0), a third-party library fetched by tools/fetch_zenoh_pico.sh into
    # the git-ignored third_party/ - never part of the default or RTPS builds.
    set(ZENOH_PICO_DIR "${CMAKE_CURRENT_LIST_DIR}/third_party/zenoh-pico")
    if(NOT EXISTS "${ZENOH_PICO_DIR}/include/zenoh-pico.h")
        message(FATAL_ERROR "CONFIG_RAFTROS_BACKEND_ZENOH_PICO needs zenoh-pico: run tools/fetch_zenoh_pico.sh in ${CMAKE_CURRENT_LIST_DIR}")
    endif()

    # zenoh-pico's sources, minus every platform layer except ESP-IDF's (the
    # selection its own PlatformIO build makes for the espidf framework)
    file(GLOB_RECURSE ZENOH_PICO_SRCS "${ZENOH_PICO_DIR}/src/*.c")
    list(FILTER ZENOH_PICO_SRCS EXCLUDE REGEX "/src/system/")
    list(APPEND ZENOH_PICO_SRCS
      "${ZENOH_PICO_DIR}/src/system/common/platform.c"
      "${ZENOH_PICO_DIR}/src/system/socket/esp32.c"
      "${ZENOH_PICO_DIR}/src/system/espidf/system.c"
    )

    # Build configuration: a client of one router over TCP, with only the
    # features RaftROS uses, so the footprint compared is the one it would ship
    set(FRAG_MAX_SIZE 4096)                     # zenoh-pico's embedded defaults
    set(BATCH_UNICAST_SIZE 2048)
    set(BATCH_MULTICAST_SIZE 2048)
    set(Z_CONFIG_SOCKET_TIMEOUT 100)
    set(Z_TRANSPORT_LEASE 10000)
    set(Z_TRANSPORT_LEASE_EXPIRE_FACTOR 3)
    set(Z_RUNTIME_MAX_TASKS 64)
    set(Z_RUNTIME_IDLE_READ_TASK_SLEEP 0)
    set(Z_TRANSPORT_ACCEPT_TIMEOUT 1000)
    set(Z_TRANSPORT_CONNECT_TIMEOUT 10000)
    set(Z_FEATURE_MULTI_THREAD 1)               # its own FreeRTOS task reads and keeps the lease
    set(Z_FEATURE_PUBLICATION 1)
    set(Z_FEATURE_SUBSCRIPTION 1)
    set(Z_FEATURE_QUERY 0)                      # the device serves queries, it never sends them
    set(Z_FEATURE_QUERYABLE 1)
    set(Z_FEATURE_LIVELINESS 1)
    set(Z_FEATURE_INTEREST 1)
    set(Z_FEATURE_FRAGMENTATION 1)
    set(Z_FEATURE_ENCODING_VALUES 1)
    set(Z_FEATURE_LINK_TCP 1)
    set(Z_FEATURE_UNICAST_TRANSPORT 1)
    set(Z_FEATURE_TCP_NODELAY 1)
    set(Z_FEATURE_SESSION_CHECK 1)
    set(Z_FEATURE_BATCHING 1)
    set(Z_FEATURE_AUTO_RECONNECT 1)
    foreach(off ADMIN_SPACE ADVANCED_PUBLICATION ADVANCED_SUBSCRIPTION BATCH_PEER_MUTEX BATCH_TX_MUTEX
                CONNECTIVITY LINK_BLUETOOTH LINK_SERIAL LINK_SERIAL_USB LINK_TLS LINK_UDP_MULTICAST
                LINK_UDP_UNICAST LINK_WS LOCAL_QUERYABLE LOCAL_SUBSCRIBER MATCHING MULTICAST_DECLARATIONS
                MULTICAST_TRANSPORT RAWETH_TRANSPORT RX_CACHE SCOUTING UNICAST_PEER)
        set(Z_FEATURE_${off} 0)
    endforeach()
    if(CONFIG_RAFTROS_ZENOH_PICO_TUNED)
        set(BATCH_UNICAST_SIZE 1024)            # larger messages are fragmented
        set(FRAG_MAX_SIZE 2048)                 # largest message reassembled: requests are <= 512 B
        set(Z_RUNTIME_MAX_TASKS 8)              # sizes static executor containers inside each session
    endif()
    set(ZENOH_PICO_GEN_DIR "${CMAKE_CURRENT_BINARY_DIR}/zenoh-pico-include")
    configure_file("${ZENOH_PICO_DIR}/include/zenoh-pico/config.h.in"
                   "${ZENOH_PICO_GEN_DIR}/zenoh-pico/config.h")

    set(RAFTROS_BACKEND_SRCS
      "components/RaftROS/ZenohPico/RaftROSZenohPico.cpp"
      ${ZENOH_PICO_SRCS}
    )
    # The generated config.h must win over the template's directory
    set(RAFTROS_BACKEND_INCLUDES "${ZENOH_PICO_GEN_DIR}" "${ZENOH_PICO_DIR}/include")
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
      ${RAFTROS_BACKEND_INCLUDES}
    REQUIRES
      RaftCore
      esp_hw_support
      esp_netif
      esp_timer
      lwip
    )

if(CONFIG_RAFTROS_BACKEND_ZENOH_PICO)
    # zenoh-pico selects its ESP-IDF platform layer from this define; our SysMod
    # includes zenoh-pico.h too, so it applies to the whole component
    target_compile_definitions(${COMPONENT_LIB} PRIVATE ZENOH_ESPIDF)
    # Third-party C: built as its authors build it, without our warnings-as-errors
    set_source_files_properties(${ZENOH_PICO_SRCS} PROPERTIES COMPILE_OPTIONS "-Wno-error;-w")
    # zenoh-pico's ESP-IDF platform header includes driver/uart.h even with
    # serial links off (it then uses nothing from it).  A component dependency
    # cannot be made conditional on Kconfig - ESP-IDF resolves those before
    # the configuration is loaded - so the header paths are added here, for
    # this build only, rather than making every build depend on the driver.
    foreach(uartComponent esp_driver_uart esp_hal_uart)
        idf_component_get_property(uartDir ${uartComponent} COMPONENT_DIR)
        target_include_directories(${COMPONENT_LIB} PRIVATE "${uartDir}/include")
    endforeach()
endif()
