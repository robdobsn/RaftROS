# Set the target Espressif chip
set(IDF_TARGET "esp32s3")

# ESP-IDF version used to build (for both local and Docker builds using "raft build")
# This is the default for all SysTypes - it can be overridden in systypes/<SysType>/features.cmake
# It must be a literal version on one line as it is also read by the raft command line tool
set(ESP_IDF_VERSION "6.1")

# Raft components
set(RAFT_COMPONENTS
    RaftCore@main
    RaftSysMods@main
    RaftWebServer@main
    RaftI2C@main
)

# RaftROS component (local path - parent directory contains RaftROS CMakeLists.txt)
list(APPEND EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../../..")

# File system
set(FS_TYPE "littlefs")
set(FS_IMAGE_PATH "../Common/FSImage")

# Web UI

# Uncomment the "set" line below if you want to use the web UI
# This assumes an app is built using npm run build
# it also assumes that the web app is built into a folder called "dist" in the UI_SOURCE_PATH
# set(UI_SOURCE_PATH "../Common/WebUI")

# Uncomment the following line if you do NOT want to gzip the web UI
# set(WEB_UI_GEN_FLAGS ${WEB_UI_GEN_FLAGS} --nogzip)

# Uncomment the following line to include a source map for the web UI - this will increase the size of the web UI
# set(WEB_UI_GEN_FLAGS ${WEB_UI_GEN_FLAGS} --incmap)
