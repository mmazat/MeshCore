#pragma once

#include <Arduino.h>

#include "DataStore.h"

class MyMesh;

class MeshcoreImageTransfer {
public:
  explicit MeshcoreImageTransfer(FILESYSTEM& state_fs) : state_fs_(&state_fs) {}

  void begin();
  bool start(const char* file_path, uint32_t job_seed, const char* direct_target_name = nullptr);
  bool abort(MyMesh& mesh);
  void loop(MyMesh& mesh);
  bool handleDirectMessage(MyMesh& mesh, const char* text, const char* sender_name,
                           uint32_t response_timestamp, char* reply_text, size_t reply_text_len);
  bool handleProtocolMessage(const char* text, const char* sender_name = nullptr);
  bool formatBleProgressMessage(const char* text, char* out, size_t out_len) const;
  bool isActive() const { return image_buffer_ != nullptr; }

  void handleCaptureChunkCommand(MyMesh& mesh, int chunk_id, const char* sender_name);

private:
  // RAM-caching buffer for /imgtx/current.jpg image data
  uint8_t* image_buffer_ = nullptr;
  size_t image_buffer_size_ = 0;
  uint32_t total_chunks_ = 0;

  void freeImageBuffer();
  bool captureCurrentJpgIfMissing();
  bool loadCurrentJpgToBuffer();
  bool ensureImageReady();
  bool sendChunkByIndex(MyMesh& mesh, int chunk_id, const char* sender_name);
  bool copyFileOnSd(const char* src_path, const char* dst_path);

  // kept for constructor compatibility and potential future filesystem operations
  FILESYSTEM* state_fs_;
};