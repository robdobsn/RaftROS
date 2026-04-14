# Compiler
CC = g++

# Compiler flags
CFLAGS = -Wall -std=c++20 -lc -g -DRAFT_CORE

# Include paths
INCLUDES = -I../components/RaftROS -I../components/RaftROS/RTPS -I../components/RaftROS/CDR -I./RaftCore/components/core/Utils -I./RaftCore/components/core/ArduinoUtils -I.

# Source files
SOURCES = main.cpp utils.cpp ../components/RaftROS/CDR/CDREncoder.cpp ../components/RaftROS/CDR/CDRDecoder.cpp ../components/RaftROS/RTPS/RTPSMessage.cpp

# Object files
OBJECTS = $(SOURCES:.cpp=.o)

# Output binary
OUTPUT = linux_unit_tests

# RaftCore
REPO_URL=https://github.com/robdobsn/RaftCore
DEST_DIR=./RaftCore
# GIT_TAG=v1.0  # Optional: Specify a tag if needed

# Ensure raft_core is executed every time by making it .PHONY
.PHONY: raft_core clean all

all: $(OUTPUT)

raft_core:
	@echo "Fetching RaftCore repository..."
	python3 ../scripts/FetchGitRepo.py $(REPO_URL) $(DEST_DIR) $(GIT_TAG)

# Compile object files
%.o: %.cpp
	@echo "Compiling $<..."
	$(CC) $(CFLAGS) $(INCLUDES) -c $< -o $@

# Ensure $(OUTPUT) depends on raft_core
$(OUTPUT): $(OBJECTS) raft_core
	@echo "Building $(OUTPUT)..."
	$(CC) $(CFLAGS) $(INCLUDES) -o $(OUTPUT) $(OBJECTS)

clean:
	@echo "Cleaning up..."
	rm -f $(OUTPUT) $(OBJECTS)
