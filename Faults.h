#pragma once //ensures file is only included once

//list of status' the system can start in
enum StartupStatus {
  STARTUP_OK,
  STARTUP_ERROR,
  STARTUP_CRITICAL
};

//list of faults in startup
enum SystemFault : uint16_t {
  FAULT_NONE        = 0,
  FAULT_SD_CARD     = 1 << 0,
  FAULT_IMU         = 1 << 1,
  FAULT_BMP         = 1 << 2,
  FAULT_DEBUGLOG     = 1 << 3,
  FAULT_SERVOLOG     = 1 << 4,
  FAULT_SENSORLOG     = 1 << 5
};

uint16_t systemFaults = FAULT_NONE;

