#include "MeshcoreImageTransfer.h"

#include <stdarg.h>

#include <target.h>

#include <helpers/TxtDataHelpers.h>

#include "MyMesh.h"

namespace {

#if defined(ESP32_S3_N16R8_SX1262)
constexpr bool kLocalImageCaptureSupported = true;
#else
constexpr bool kLocalImageCaptureSupported = false;
#endif

constexpr uint32_t kStateMagic = 0x31524654; // TFR1
constexpr uint16_t kStateVersion = 5;
constexpr uint32_t kNoChunkAcked = 0xFFFFFFFFu;
constexpr size_t kRawChunkBytes = 96;
constexpr unsigned long kRetryIntervalMs = 5000;
constexpr char kTransferDir[] = "/imgtx";
constexpr char kStatePath[] = "/imgtx/state.bin";
constexpr char kStateTmpPath[] = "/imgtx/state.bin.tmp";
constexpr char kLogPath[] = "/imgtx/tx.log";
constexpr char kProtocolPrefix[] = "@img1|";

File openReadFile(FILESYSTEM* fs, const char* path) {
#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
  return fs->open(path, FILE_O_READ);
#elif defined(RP2040_PLATFORM)
  return fs->open(path, "r");
#else
  return fs->open(path, "r", false);
#endif
}

File openWriteFile(FILESYSTEM* fs, const char* path) {
#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
  fs->remove(path);
  return fs->open(path, FILE_O_WRITE);
#elif defined(RP2040_PLATFORM)
  return fs->open(path, "w");
#else
  return fs->open(path, "w", true);
#endif
}

File openAppendFile(FILESYSTEM* fs, const char* path) {
#if defined(NRF52_PLATFORM) || defined(STM32_PLATFORM)
  return fs->open(path, FILE_O_WRITE | FILE_O_APPEND);
#elif defined(RP2040_PLATFORM)
  return fs->open(path, "a");
#else
  return fs->open(path, "a", true);
#endif
}

size_t encodeBase64(const uint8_t* input, size_t input_len, char* output, size_t output_size) {
  static constexpr char table[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

  size_t output_len = 4 * ((input_len + 2) / 3);
  if (output_size <= output_len) {
    return 0;
  }

  size_t in_index = 0;
  size_t out_index = 0;
  while (in_index < input_len) {
    uint32_t octet_a = in_index < input_len ? input[in_index++] : 0;
    uint32_t octet_b = in_index < input_len ? input[in_index++] : 0;
    uint32_t octet_c = in_index < input_len ? input[in_index++] : 0;
    uint32_t triple = (octet_a << 16) | (octet_b << 8) | octet_c;

    output[out_index++] = table[(triple >> 18) & 0x3F];
    output[out_index++] = table[(triple >> 12) & 0x3F];
    output[out_index++] = table[(triple >> 6) & 0x3F];
    output[out_index++] = table[triple & 0x3F];
  }

  size_t mod = input_len % 3;
  if (mod > 0) {
    output[output_len - 1] = '=';
    if (mod == 1) {
      output[output_len - 2] = '=';
    }
  }

  output[output_len] = 0;
  return output_len;
}

const char* baseName(const char* path) {
  const char* slash = strrchr(path, '/');
  return slash == nullptr ? path : slash + 1;
}

bool startsWith(const char* text, const char* prefix) {
  return strncmp(text, prefix, strlen(prefix)) == 0;
}

ContactInfo* findContactBySenderName(MyMesh& mesh, const char* sender_name) {
  if (sender_name == nullptr || sender_name[0] == 0) {
    return nullptr;
  }

  ContactInfo* exact_match = mesh.searchContactsByPrefix(sender_name);
  if (exact_match != nullptr) {
    return exact_match;
  }

  ContactInfo best_match;
  bool have_match = false;
  size_t best_len = 0;
  const size_t sender_len = strlen(sender_name);
  for (int idx = 0; idx < mesh.getNumContacts(); idx++) {
    ContactInfo contact;
    if (!mesh.getContactByIdx(idx, contact) || contact.name[0] == 0) {
      continue;
    }

    size_t contact_len = strlen(contact.name);
    if (contact_len == 0 || contact_len > sender_len) {
      continue;
    }

    if (strncmp(sender_name, contact.name, contact_len) == 0 && contact_len > best_len) {
      best_match = contact;
      best_len = contact_len;
      have_match = true;
    }
  }

  if (!have_match) {
    return nullptr;
  }

  return mesh.searchContactsByPrefix(best_match.name);
}

ContactInfo* findTargetContact(MyMesh& mesh, const char* target_name) {
  if (target_name == nullptr || target_name[0] == 0) {
    return nullptr;
  }

  ContactInfo* c = mesh.searchContactsByPrefix(target_name);
  if (c != nullptr) return c;

  // Try prefix match for truncated names
  ContactInfo best_match;
  bool have_match = false;
  size_t best_len = 0;
  const size_t target_len = strlen(target_name);
  for (int idx = 0; idx < mesh.getNumContacts(); idx++) {
    ContactInfo contact;
    if (!mesh.getContactByIdx(idx, contact) || contact.name[0] == 0) {
      continue;
    }
    size_t contact_len = strlen(contact.name);
    if (contact_len == 0 || contact_len > target_len) {
      continue;
    }
    if (strncmp(target_name, contact.name, contact_len) == 0 && contact_len > best_len) {
      best_match = contact;
      best_len = contact_len;
      have_match = true;
    }
  }

  if (!have_match) {
    return nullptr;
  }
  return mesh.searchContactsByPrefix(best_match.name);
}

void copyName(char* dest, size_t dest_size, const char* src) {
  if (dest_size == 0) {
    return;
  }

  if (src == nullptr) {
    dest[0] = 0;
    return;
  }

  strncpy(dest, src, dest_size - 1);
  dest[dest_size - 1] = 0;
}

} // namespace

void MeshcoreImageTransfer::begin() {
  state_fs_->mkdir(kTransferDir);
  if (loadState() && state_.active != 0) {
    appendLog("resume job=%s path=%s ack=%lu/%lu",
              state_.job_id,
              state_.file_path,
              state_.last_acked_chunk == kNoChunkAcked ? 0ul : state_.last_acked_chunk + 1,
              state_.total_chunks);
  }
}

bool MeshcoreImageTransfer::start(const char* file_path, uint32_t job_seed, const char* direct_target_name) {
  if (file_path == nullptr || file_path[0] == 0) {
    return false;
  }

  if (state_.active != 0) {
    appendLog("replace-active job=%s path=%s", state_.job_id, state_.file_path);
    clearState();
  }

  size_t file_size = 0;
  if (!board_get_sd_file_size(file_path, &file_size) || file_size == 0) {
    return false;
  }

  uint32_t crc32 = 0;
  if (!computeCRC32(file_path, &crc32)) {
    return false;
  }

  resetState();
  state_.magic = kStateMagic;
  state_.version = kStateVersion;
  state_.active = 1;
  state_.start_acked = 0;
  state_.crc32 = crc32;
  state_.file_size = static_cast<uint32_t>(file_size);
  state_.chunk_size = static_cast<uint32_t>(kRawChunkBytes);
  state_.total_chunks = static_cast<uint32_t>((file_size + kRawChunkBytes - 1) / kRawChunkBytes);
  state_.last_acked_chunk = kNoChunkAcked;
  state_.last_attempt_millis = 0;
  snprintf(state_.job_id, sizeof(state_.job_id), "%08lx%08lx",
           static_cast<unsigned long>(job_seed),
           static_cast<unsigned long>(file_size));
  strncpy(state_.file_path, file_path, sizeof(state_.file_path) - 1);
  strncpy(state_.file_name, baseName(file_path), sizeof(state_.file_name) - 1);
  if (direct_target_name != nullptr && direct_target_name[0] != 0) {
    copyName(state_.direct_target_name, sizeof(state_.direct_target_name), direct_target_name);
  } else {
    state_.direct_target_name[0] = 0;
  }

  appendLog("start job=%s path=%s size=%lu chunks=%lu crc=%08lx target=%s",
            state_.job_id,
            state_.file_path,
            static_cast<unsigned long>(state_.file_size),
            static_cast<unsigned long>(state_.total_chunks),
            static_cast<unsigned long>(state_.crc32),
            state_.direct_target_name);
  return saveState();
}

void MeshcoreImageTransfer::loop(MyMesh& mesh) {
  if (state_.active == 0) {
    return;
  }

  unsigned long now = millis();
  if (state_.last_attempt_millis != 0 && (now - state_.last_attempt_millis) < kRetryIntervalMs) {
    return;
  }

  bool sent = state_.start_acked == 0 ? sendStart(mesh) : sendChunk(mesh);
  if (sent) {
    state_.last_attempt_millis = now;
    saveState();
  }
}

bool MeshcoreImageTransfer::handleProtocolMessage(const char* text, const char* sender_name) {
  if (text == nullptr || !startsWith(text, kProtocolPrefix)) {
    return false;
  }

  char buffer[192];
  strncpy(buffer, text, sizeof(buffer) - 1);
  buffer[sizeof(buffer) - 1] = 0;

  char* save_ptr = nullptr;
  char* prefix = strtok_r(buffer, "|", &save_ptr);
  char* verb = strtok_r(nullptr, "|", &save_ptr);
  char* job_id = strtok_r(nullptr, "|", &save_ptr);

  if (prefix == nullptr || verb == nullptr || job_id == nullptr || strcmp(job_id, state_.job_id) != 0) {
    return startsWith(text, kProtocolPrefix);
  }

  if (strcmp(verb, "as") == 0) {
    if (sender_name != nullptr && sender_name[0] != 0) {
      copyName(state_.direct_target_name, sizeof(state_.direct_target_name), sender_name);
    }
    state_.start_acked = 1;
    state_.last_attempt_millis = 0;
    appendLog("ack-start job=%s target=%s", state_.job_id,
              state_.direct_target_name[0] != 0 ? state_.direct_target_name : "(none)");
    saveState();
    return true;
  }

  if (strcmp(verb, "ac") == 0) {
    char* chunk_idx_text = strtok_r(nullptr, "|", &save_ptr);
    if (chunk_idx_text == nullptr) {
      return true;
    }

    uint32_t chunk_idx = static_cast<uint32_t>(strtoul(chunk_idx_text, nullptr, 10));
    uint32_t expected = state_.last_acked_chunk == kNoChunkAcked ? 0 : state_.last_acked_chunk + 1;
    if (chunk_idx == expected) {
      if (sender_name != nullptr && sender_name[0] != 0) {
        copyName(state_.direct_target_name, sizeof(state_.direct_target_name), sender_name);
      }
      state_.last_acked_chunk = chunk_idx;
      state_.last_attempt_millis = 0;
      appendLog("ack-chunk job=%s idx=%lu/%lu", state_.job_id,
                static_cast<unsigned long>(chunk_idx),
                static_cast<unsigned long>(state_.total_chunks));
      if (state_.last_acked_chunk + 1 >= state_.total_chunks) {
        appendLog("complete job=%s path=%s", state_.job_id, state_.file_path);
        clearState();
      } else {
        saveState();
      }
    }
    return true;
  }

  if (strcmp(verb, "x") == 0) {
    appendLog("abort job=%s reason=%s", state_.job_id, save_ptr == nullptr ? "unspecified" : save_ptr);
    clearState();
    return true;
  }

  return true;
}

bool MeshcoreImageTransfer::formatBleProgressMessage(const char* text, char* out, size_t out_len) const {
  if (out == nullptr || out_len == 0) {
    return false;
  }

  out[0] = 0;
  if (text == nullptr || !startsWith(text, kProtocolPrefix)) {
    return false;
  }

  char buffer[192];
  strncpy(buffer, text, sizeof(buffer) - 1);
  buffer[sizeof(buffer) - 1] = 0;

  char* save_ptr = nullptr;
  char* prefix = strtok_r(buffer, "|", &save_ptr);
  char* verb = strtok_r(nullptr, "|", &save_ptr);
  char* job_id = strtok_r(nullptr, "|", &save_ptr);

  if (prefix == nullptr || verb == nullptr || job_id == nullptr || strcmp(job_id, state_.job_id) != 0) {
    return false;
  }

  if (strcmp(verb, "as") == 0) {
    snprintf(out, out_len, "image transfer started job=%s 0/%lu",
             state_.job_id,
             static_cast<unsigned long>(state_.total_chunks));
    return true;
  }

  if (strcmp(verb, "ac") == 0) {
    char* chunk_idx_text = strtok_r(nullptr, "|", &save_ptr);
    if (chunk_idx_text == nullptr || state_.total_chunks == 0) {
      return false;
    }

    uint32_t chunk_idx = static_cast<uint32_t>(strtoul(chunk_idx_text, nullptr, 10));
    uint32_t completed = chunk_idx + 1;
    bool is_final = completed >= state_.total_chunks;
    bool is_first = completed == 1;
    bool is_milestone = (completed % 10u) == 0u || (completed == state_.total_chunks);
    uint32_t percent = static_cast<uint32_t>((static_cast<uint64_t>(completed) * 100u) / state_.total_chunks);

    if (!is_first && !is_milestone && !is_final) {
      return false;
    }

    snprintf(out, out_len, "image transfer progress %lu/%lu (%lu%%)",
             static_cast<unsigned long>(completed),
             static_cast<unsigned long>(state_.total_chunks),
             static_cast<unsigned long>(percent));
    return true;
  }

  if (strcmp(verb, "x") == 0) {
    snprintf(out, out_len, "image transfer aborted: %s",
             save_ptr == nullptr || save_ptr[0] == 0 ? "unspecified" : save_ptr);
    return true;
  }

  return false;
}

bool MeshcoreImageTransfer::loadState() {
  resetState();
  File state_file = openReadFile(state_fs_, kStatePath);
  if (!state_file) {
    state_file = openReadFile(state_fs_, kStateTmpPath);
  }
  if (!state_file) {
    return false;
  }

  size_t bytes_read = state_file.read(reinterpret_cast<uint8_t*>(&state_), sizeof(state_));
  state_file.close();
  if (bytes_read != sizeof(state_) || state_.magic != kStateMagic || state_.version != kStateVersion) {
    resetState();
    return false;
  }
  return true;
}

bool MeshcoreImageTransfer::saveState() {
  File state_file = openWriteFile(state_fs_, kStateTmpPath);
  if (!state_file) {
    return false;
  }

  size_t bytes_written = state_file.write(reinterpret_cast<const uint8_t*>(&state_), sizeof(state_));
  state_file.flush();
  state_file.close();
  if (bytes_written != sizeof(state_)) {
    state_fs_->remove(kStateTmpPath);
    return false;
  }

  state_fs_->remove(kStatePath);
  if (!state_fs_->rename(kStateTmpPath, kStatePath)) {
    state_fs_->remove(kStateTmpPath);
    return false;
  }
  return true;
}

void MeshcoreImageTransfer::clearState() {
  resetState();
  state_fs_->remove(kStatePath);
  state_fs_->remove(kStateTmpPath);
}

void MeshcoreImageTransfer::appendLog(const char* fmt, ...) {
  char line[192];
  va_list args;
  va_start(args, fmt);
  vsnprintf(line, sizeof(line), fmt, args);
  va_end(args);

  File log_file = openAppendFile(state_fs_, kLogPath);
  if (!log_file) {
    return;
  }

  log_file.printf("%lu %s\n", millis(), line);
  log_file.flush();
  log_file.close();
}

bool MeshcoreImageTransfer::sendStart(MyMesh& mesh) {
  char message[160];
  snprintf(message, sizeof(message), "@img1|s|%s|%lu|%lu|%lu|%08lx|%s",
           state_.job_id,
           static_cast<unsigned long>(state_.file_size),
           static_cast<unsigned long>(state_.total_chunks),
           static_cast<unsigned long>(state_.chunk_size),
           static_cast<unsigned long>(state_.crc32),
           state_.file_name);

  const char* target_name = state_.direct_target_name[0] != 0 ? state_.direct_target_name : nullptr;
  ContactInfo* recipient = findTargetContact(mesh, target_name);
  if (recipient == nullptr) {
    appendLog("missing-contact job=%s want=%s", state_.job_id,
              target_name != nullptr ? target_name : "(none)");
    return false;
  }

  uint32_t expected_ack = 0;
  uint32_t est_timeout = 0;
  bool sent = mesh.sendMessage(
      *recipient,
      mesh.getRTCClock()->getCurrentTime(),
      0,
      message,
      expected_ack,
      est_timeout) != MSG_SEND_FAILED;
  if (sent) {
    appendLog("send-start-direct job=%s via=%s", state_.job_id, recipient->name);
  } else {
    appendLog("send-start-direct-failed job=%s via=%s", state_.job_id, recipient->name);
  }
  return sent;
}

bool MeshcoreImageTransfer::sendChunk(MyMesh& mesh) {
  uint32_t next_chunk = state_.last_acked_chunk == kNoChunkAcked ? 0 : state_.last_acked_chunk + 1;
  if (next_chunk >= state_.total_chunks) {
    clearState();
    return false;
  }

  if (state_.chunk_size > kRawChunkBytes) {
    appendLog("chunk-size-invalid job=%s size=%lu max=%u", state_.job_id,
              static_cast<unsigned long>(state_.chunk_size),
              static_cast<unsigned>(kRawChunkBytes));
    return false;
  }

  uint8_t raw[kRawChunkBytes];
  size_t bytes_read = 0;
  if (!readChunk(next_chunk, raw, &bytes_read) || bytes_read == 0) {
    appendLog("chunk-read-failed job=%s idx=%lu", state_.job_id, static_cast<unsigned long>(next_chunk));
    return false;
  }

  // Encode chunk as text: @img1|d|<job_id>|<chunk_idx>|<base64_data>
  char b64_buf[132]; // ceil(96*4/3)+1
  size_t b64_len = encodeBase64(raw, bytes_read, b64_buf, sizeof(b64_buf));
  if (b64_len == 0) {
    appendLog("chunk-encode-failed job=%s idx=%lu", state_.job_id, static_cast<unsigned long>(next_chunk));
    return false;
  }

  char message[160];
  int msg_len = snprintf(message, sizeof(message), "@img1|d|%s|%lu|%s",
                         state_.job_id,
                         static_cast<unsigned long>(next_chunk),
                         b64_buf);
  if (msg_len < 0 || static_cast<size_t>(msg_len) >= sizeof(message)) {
    appendLog("chunk-msg-too-long job=%s idx=%lu len=%d", state_.job_id,
              static_cast<unsigned long>(next_chunk), msg_len);
    return false;
  }

  const char* target_name = state_.direct_target_name[0] != 0 ? state_.direct_target_name : nullptr;
  ContactInfo* recipient = findTargetContact(mesh, target_name);
  if (recipient == nullptr) {
    appendLog("missing-contact job=%s want=%s", state_.job_id,
              target_name != nullptr ? target_name : "(none)");
    return false;
  }

  uint32_t expected_ack = 0;
  uint32_t est_timeout = 0;
  bool sent = mesh.sendMessage(
      *recipient,
      mesh.getRTCClock()->getCurrentTime(),
      0,
      message,
      expected_ack,
      est_timeout) != MSG_SEND_FAILED;
  if (sent) {
    appendLog("send-chunk job=%s idx=%lu bytes=%u via=%s", state_.job_id,
              static_cast<unsigned long>(next_chunk),
              static_cast<unsigned>(bytes_read),
              recipient->name);
  } else {
    appendLog("send-chunk-failed job=%s idx=%lu via=%s", state_.job_id,
              static_cast<unsigned long>(next_chunk),
              recipient->name);
  }
  return sent;
}

bool MeshcoreImageTransfer::readChunk(uint32_t chunk_idx, uint8_t* buffer, size_t* bytes_read) const {
  if (bytes_read == nullptr) {
    return false;
  }

  *bytes_read = 0;

  if (!kLocalImageCaptureSupported) {
    (void)chunk_idx;
    (void)buffer;
    return false;
  }

  size_t offset = static_cast<size_t>(chunk_idx) * state_.chunk_size;
  return board_read_sd_file_chunk(state_.file_path, offset, buffer, state_.chunk_size, bytes_read);
}

bool MeshcoreImageTransfer::abort(MyMesh& mesh) {
  if (state_.active == 0) {
    return false;
  }

  char message[64];
  snprintf(message, sizeof(message), "@img1|x|%s|user-abort", state_.job_id);

  const char* target_name = state_.direct_target_name[0] != 0 ? state_.direct_target_name : nullptr;
  ContactInfo* recipient = findTargetContact(mesh, target_name);
  if (recipient != nullptr) {
    uint32_t expected_ack = 0;
    uint32_t est_timeout = 0;
    mesh.sendMessage(*recipient, mesh.getRTCClock()->getCurrentTime(), 0, message, expected_ack, est_timeout);
  }
  appendLog("user-abort job=%s", state_.job_id);
  clearState();
  return true;
}

bool MeshcoreImageTransfer::computeCRC32(const char* file_path, uint32_t* crc32_out) const {
  if (!kLocalImageCaptureSupported) {
    (void)file_path;
    (void)crc32_out;
    return false;
  }

  return board_compute_sd_file_crc32(file_path, crc32_out);
}

void MeshcoreImageTransfer::resetState() {
  memset(&state_, 0, sizeof(state_));
  state_.magic = kStateMagic;
  state_.version = kStateVersion;
  state_.last_acked_chunk = kNoChunkAcked;
  state_.chunk_size = static_cast<uint32_t>(kRawChunkBytes);
}