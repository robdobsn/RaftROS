# Compiler
CC = g++

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
	../components/RaftROS/RTPS/runtime/announce/SEDPHandler.cpp
PROTO_OBJECTS = $(foreach src,$(PROTO_SOURCES),$(call src_to_obj,$(src)))

# RaftCore
REPO_URL=https://github.com/robdobsn/RaftCore
DEST_DIR=./RaftCore
# GIT_TAG=v1.0  # Optional: Specify a tag if needed

# Ensure raft_core is executed every time by making it .PHONY
.PHONY: raft_core clean all standalone

all: $(OUTPUT)

standalone: $(STANDALONE_OUT)

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
	$(CC) $(CFLAGS) $(INCLUDES) -o $(OUTPUT) $(OBJECTS)

# Standalone ROS 2 participant
$(STANDALONE_OUT): $(STANDALONE_OBJ) $(PROTO_OBJECTS) raft_core
	@echo "Building $(STANDALONE_OUT)..."
	$(CC) $(CFLAGS) $(INCLUDES) -o $(STANDALONE_OUT) $(STANDALONE_OBJ) $(PROTO_OBJECTS)

clean:
	@echo "Cleaning up..."
	rm -f $(OUTPUT) $(STANDALONE_OUT)
	rm -rf $(BUILD_DIR)
