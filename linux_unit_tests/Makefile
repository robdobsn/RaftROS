# Compiler
CC = g++
SIZE ?= size

# Build artifact directories
BUILD_DIR = build
OBJ_DIR = $(BUILD_DIR)/obj

# Compiler flags
CFLAGS = -Wall -std=c++20 -lc -g -DRAFT_CORE -DRAFTROS_ACK_HEX_DUMP_ENABLE=1 -DRAFTROS_ACK_VERBOSE_LOG_LABELS_ENABLE=1

# Include paths
INCLUDES = -I../components/RaftROS -I../components/RaftROS/RTPS -I../components/RaftROS/CDR -I./RaftCore/components/core/Utils -I./RaftCore/components/core/ArduinoUtils -I.

# Source files
SOURCES = main.cpp utils.cpp \
	../components/RaftROS/CDR/CDREncoder.cpp \
	../components/RaftROS/CDR/CDRDecoder.cpp \
	../components/RaftROS/AutoPub/AutoPubCDRSerializer.cpp \
	../components/RaftROS/RTPS/runtime/wire/RTPSMessage.cpp \
	../components/RaftROS/RTPS/runtime/core/RTPSParticipant.cpp \
	../components/RaftROS/RTPS/runtime/discovery/SPDPHandler.cpp \
	../components/RaftROS/RTPS/runtime/schedule/RTPSRuntimeSchedule.cpp \
	../components/RaftROS/RTPS/runtime/announce/RTPSInitialAnnouncePlan.cpp \
	../components/RaftROS/RTPS/runtime/announce/RTPSInitialAnnounceRunner.cpp \
	../components/RaftROS/RTPS/runtime/announce/RTPSWriterHeartbeatRunner.cpp \
	../components/RaftROS/RTPS/runtime/receive/RTPSRxSubmessageRunner.cpp \
	../components/RaftROS/RTPS/runtime/receive/RTPSRunnerAdapterHelpers.cpp \
	../components/RaftROS/RTPS/runtime/discovery/RTPSDiscoveryRuntime.cpp \
	../components/RaftROS/RTPS/runtime/reliability/RTPSReliabilityAndWriterStateRuntime.cpp \
	../components/RaftROS/RTPS/runtime/reliability/RTPSReaderRuntime.cpp \
	../components/RaftROS/RTPS/runtime/reliability/RTPSReaderRunner.cpp \
	../components/RaftROS/RTPS/runtime/announce/SEDPHandler.cpp

# Map source file path to object file under $(OBJ_DIR)
define src_to_obj
$(OBJ_DIR)/$(patsubst ../%,%,$(patsubst %.cpp,%.o,$(1)))
endef

# Object files
OBJECTS = $(foreach src,$(SOURCES),$(call src_to_obj,$(src)))

# Output binary
OUTPUT = linux_unit_tests

# Standalone binary
STANDALONE_SRC = raftros_standalone.cpp
STANDALONE_OBJ = $(call src_to_obj,$(STANDALONE_SRC))
STANDALONE_OUT = raftros_linux

# Shared protocol sources (no main)
PROTO_SOURCES = utils.cpp \
	../components/RaftROS/CDR/CDREncoder.cpp \
	../components/RaftROS/CDR/CDRDecoder.cpp \
	../components/RaftROS/AutoPub/AutoPubCDRSerializer.cpp \
	../components/RaftROS/RTPS/runtime/wire/RTPSMessage.cpp \
	../components/RaftROS/RTPS/runtime/core/RTPSParticipant.cpp \
	../components/RaftROS/RTPS/runtime/discovery/SPDPHandler.cpp \
	../components/RaftROS/RTPS/runtime/schedule/RTPSRuntimeSchedule.cpp \
	../components/RaftROS/RTPS/runtime/announce/RTPSInitialAnnouncePlan.cpp \
	../components/RaftROS/RTPS/runtime/announce/RTPSInitialAnnounceRunner.cpp \
	../components/RaftROS/RTPS/runtime/announce/RTPSWriterHeartbeatRunner.cpp \
	../components/RaftROS/RTPS/runtime/receive/RTPSRxSubmessageRunner.cpp \
	../components/RaftROS/RTPS/runtime/receive/RTPSRunnerAdapterHelpers.cpp \
	../components/RaftROS/RTPS/runtime/discovery/RTPSDiscoveryRuntime.cpp \
	../components/RaftROS/RTPS/runtime/reliability/RTPSReliabilityAndWriterStateRuntime.cpp \
	../components/RaftROS/RTPS/runtime/reliability/RTPSReaderRuntime.cpp \
	../components/RaftROS/RTPS/runtime/reliability/RTPSReaderRunner.cpp \
	../components/RaftROS/RTPS/runtime/announce/SEDPHandler.cpp
PROTO_OBJECTS = $(foreach src,$(PROTO_SOURCES),$(call src_to_obj,$(src)))

# RaftCore
REPO_URL=https://github.com/robdobsn/RaftCore
DEST_DIR=./RaftCore
# GIT_TAG=v1.0  # Optional: Specify a tag if needed

# Ensure raft_core is executed every time by making it .PHONY
.PHONY: raft_core clean all standalone zenoh-test zenoh-session-test zenoh-session-probe zenoh-wire-test zenoh-storage-test resource-baseline

all: $(OUTPUT)

standalone: $(STANDALONE_OUT)

ZENOH_TEST_OUT = $(BUILD_DIR)/zenoh/zenoh_codec_tests
ZENOH_TEST_CFLAGS = -std=c++17 -Wall -Wextra -Werror -pedantic -g

$(ZENOH_TEST_OUT): zenoh_codec_tests.cpp ../components/RaftROS/Zenoh/ZenohROSCodec.h ../components/RaftROS/Zenoh/ZenohROSIdentity.h
	@mkdir -p $(dir $@)
	$(CC) $(ZENOH_TEST_CFLAGS) -I../components/RaftROS $< -o $@

zenoh-test: $(ZENOH_TEST_OUT)
	./$(ZENOH_TEST_OUT)

ZENOH_SESSION_TEST_OUT = $(BUILD_DIR)/zenoh/zenoh_session_tests

$(ZENOH_SESSION_TEST_OUT): zenoh_session_tests.cpp ../components/RaftROS/Zenoh/ZenohStreamFramer.h ../components/RaftROS/Zenoh/ZenohTCPSession.h ../components/RaftROS/Zenoh/ZenohNetworkMessage.h
	@mkdir -p $(dir $@)
	$(CC) $(ZENOH_TEST_CFLAGS) -I../components/RaftROS $< -o $@

zenoh-session-test: $(ZENOH_SESSION_TEST_OUT)
	./$(ZENOH_SESSION_TEST_OUT)

ZENOH_SESSION_PROBE_OUT = $(BUILD_DIR)/zenoh/zenoh_session_probe
ZENOH_PROBE_LDFLAGS ?=
ZENOH_SENSOR_DIR = ../components/RaftROS/AutoPub
ZENOH_PROBE_SERIALIZERS = ../components/RaftROS/CDR/CDREncoder.cpp $(ZENOH_SENSOR_DIR)/AutoPubCDRSerializer.cpp

$(ZENOH_SESSION_PROBE_OUT): zenoh_session_probe.cpp ../components/RaftROS/Zenoh/ZenohStreamFramer.h ../components/RaftROS/Zenoh/ZenohTCPSession.h ../components/RaftROS/Zenoh/ZenohNetworkMessage.h ../components/RaftROS/Zenoh/ZenohROSCodec.h ../components/RaftROS/Zenoh/ZenohROSIdentity.h ../components/RaftROS/CDR/CDREncoder.h $(ZENOH_PROBE_SERIALIZERS) $(ZENOH_SENSOR_DIR)/AutoPubCDRSerializer.h $(ZENOH_SENSOR_DIR)/AutoPubClassMap.h $(ZENOH_SENSOR_DIR)/AutoPubSampleRunner.h
	@mkdir -p $(dir $@)
	$(CC) $(ZENOH_TEST_CFLAGS) -I../components/RaftROS -I../components/RaftROS/CDR $< $(ZENOH_PROBE_SERIALIZERS) $(ZENOH_PROBE_LDFLAGS) -o $@

zenoh-session-probe: $(ZENOH_SESSION_PROBE_OUT)

ZENOH_ALLOCATION_FAILURE_OUT = $(BUILD_DIR)/zenoh/zenoh_probe_allocation_failure

$(ZENOH_ALLOCATION_FAILURE_OUT): $(ZENOH_SESSION_PROBE_OUT) zenoh_probe_allocation_failure.cpp
	$(CC) $(ZENOH_TEST_CFLAGS) -I../components/RaftROS -I../components/RaftROS/CDR zenoh_session_probe.cpp $(ZENOH_PROBE_SERIALIZERS) zenoh_probe_allocation_failure.cpp -o $@

zenoh-storage-test: $(ZENOH_SESSION_PROBE_OUT) $(ZENOH_ALLOCATION_FAILURE_OUT)
	python3 zenoh_wire_interop.py --probe $(ZENOH_SESSION_PROBE_OUT) --allocation-failure-probe $(ZENOH_ALLOCATION_FAILURE_OUT) --allocation-failure-only

zenoh-wire-test: $(ZENOH_SESSION_PROBE_OUT)
	python3 zenoh_wire_interop.py --probe $(ZENOH_SESSION_PROBE_OUT)

RESOURCE_DIR = $(BUILD_DIR)/resources
RESOURCE_CFLAGS = -std=c++20 -Wall -Os -DNDEBUG -ffunction-sections -fdata-sections -fstack-usage -DRAFT_CORE -DRAFTROS_ACK_HEX_DUMP_ENABLE=0 -DRAFTROS_ACK_VERBOSE_LOG_LABELS_ENABLE=0

resource-baseline: raft_core
	@mkdir -p $(RESOURCE_DIR)
	find $(RESOURCE_DIR) -name '*.su' -type f -delete
	$(MAKE) -B -o raft_core BUILD_DIR=$(RESOURCE_DIR)/rtps STANDALONE_OUT=$(RESOURCE_DIR)/rtps/raftros_linux CFLAGS="$(RESOURCE_CFLAGS)" LDFLAGS="-Wl,--gc-sections,-Map=$(RESOURCE_DIR)/rtps/raftros_linux.map" standalone
	$(MAKE) -B BUILD_DIR=$(RESOURCE_DIR)/zenoh ZENOH_TEST_CFLAGS="$(RESOURCE_CFLAGS)" ZENOH_PROBE_LDFLAGS="-Wl,--gc-sections,-Map=$(RESOURCE_DIR)/zenoh/zenoh/zenoh_session_probe.map" zenoh-session-probe
	$(SIZE) -B $(RESOURCE_DIR)/rtps/raftros_linux $(RESOURCE_DIR)/zenoh/zenoh/zenoh_session_probe
	python3 resource_report.py --root $(RESOURCE_DIR) --compiler "$(CC)" --size-tool "$(SIZE)" --compile-flags="$(RESOURCE_CFLAGS)" > $(RESOURCE_DIR)/report.json
	@echo "Host resource report: $(RESOURCE_DIR)/report.json"

raft_core:
	@echo "Fetching RaftCore repository..."
	python3 ../scripts/FetchGitRepo.py $(REPO_URL) $(DEST_DIR) $(GIT_TAG)

# Compile object files into $(OBJ_DIR), mirroring source tree layout.
define make_obj_rule
$(call src_to_obj,$(1)): $(1)
	@echo "Compiling $(1)..."
	@mkdir -p $$(dir $$@)
	$$(CC) $$(CFLAGS) $$(INCLUDES) -c $$< -o $$@
endef

$(foreach src,$(sort $(SOURCES) $(PROTO_SOURCES) $(STANDALONE_SRC)),$(eval $(call make_obj_rule,$(src))))

# Ensure $(OUTPUT) depends on raft_core
$(OUTPUT): $(OBJECTS) raft_core
	@echo "Building $(OUTPUT)..."
	$(CC) $(CFLAGS) $(INCLUDES) -o $(OUTPUT) $(OBJECTS) $(LDFLAGS)

# Standalone ROS 2 participant
$(STANDALONE_OUT): $(STANDALONE_OBJ) $(PROTO_OBJECTS) raft_core
	@echo "Building $(STANDALONE_OUT)..."
	$(CC) $(CFLAGS) $(INCLUDES) -o $(STANDALONE_OUT) $(STANDALONE_OBJ) $(PROTO_OBJECTS) $(LDFLAGS)

clean:
	@echo "Cleaning up..."
	rm -f $(OUTPUT) $(STANDALONE_OUT)
	rm -rf $(BUILD_DIR)
