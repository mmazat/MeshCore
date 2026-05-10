#pragma once

#include <helpers/ESP32Board.h>

#include <stddef.h>

class ESP32S3N16R8SX1262Board : public ESP32Board {
  bool camera_online = false;
  bool sd_online = false;
  bool sd_mounted = false;

  bool mountSD();
  // SD card is always mounted after boot; unmountSD is a no-op
  void unmountSD();

public:
  void begin();
  bool captureToSD(char* path_buffer, size_t path_buffer_size, size_t* bytes_written = nullptr,
                   uint16_t* out_width = nullptr, uint16_t* out_height = nullptr);
  bool getSDFileSize(const char* path, size_t* file_size);
  bool computeSDFileCRC32(const char* path, uint32_t* crc32_out);
  bool readSDFileChunk(const char* path, size_t offset, uint8_t* buffer, size_t buffer_size, size_t* bytes_read);

  bool isCameraOnline() const { return camera_online; }
  bool isSDOnline() const { return sd_online; }

  const char* getManufacturerName() const {
    return "ESP32-S3 N16R8 SX1262";
  }
};