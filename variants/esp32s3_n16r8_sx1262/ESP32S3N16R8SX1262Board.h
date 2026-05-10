#pragma once

#include <helpers/ESP32Board.h>

#include <stddef.h>

class ESP32S3N16R8SX1262Board : public ESP32Board {
  bool camera_online = false;
  bool sd_online = false;
  bool sd_mounted = false;

  bool mountSD();
  void unmountSD();

public:
  void begin();
  bool captureToSD(char* path_buffer, size_t path_buffer_size, size_t* bytes_written = nullptr);

  bool isCameraOnline() const { return camera_online; }
  bool isSDOnline() const { return sd_online; }

  const char* getManufacturerName() const {
    return "ESP32-S3 N16R8 SX1262";
  }
};