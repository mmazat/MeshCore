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
  bool handleProtocolMessage(const char* text, const char* sender_name = nullptr);
  bool formatBleProgressMessage(const char* text, char* out, size_t out_len) const;
  bool isActive() const { return state_.active != 0; }

private:
  struct PersistedState {
    uint32_t magic;
    uint16_t version;
    uint8_t active;
    uint8_t start_acked;
    uint32_t crc32;
    uint32_t file_size;
    uint32_t total_chunks;
    uint32_t last_acked_chunk;
    uint32_t chunk_size;
    uint32_t last_attempt_millis;
    char job_id[17];
    char file_path[96];
    char file_name[32];
    char direct_target_name[32];
  };

  bool loadState();
  bool saveState();
  void clearState();
  void appendLog(const char* fmt, ...);
  bool sendStart(MyMesh& mesh);
  bool sendChunk(MyMesh& mesh);
  bool readChunk(uint32_t chunk_idx, uint8_t* buffer, size_t* bytes_read) const;
  bool computeCRC32(const char* file_path, uint32_t* crc32_out) const;
  void resetState();

  FILESYSTEM* state_fs_;
  PersistedState state_{};
};