# Unexpected Maker FeatherS3.  Board specifics (I2C pins, the LDO2 enable that
# powers the second QWIIC connector) are in SysTypes.json.
include("${BUILD_CONFIG_DIR}/../Common/features.cmake")

# Device types: RaftCore's records, plus this SysType's DevTypes.json
# (APDS9960, MPU6050), which the Raft build merges automatically
set(DEV_TYPE_JSON_FILES /devtypes/DeviceTypeRecords.json)
