#include "MeshcoreImageTransfer.h"

#include <stdarg.h>
#include <time.h>

#include <target.h>

#if defined(ESP32_S3_N16R8_SX1262)
#include <SD_MMC.h>
#endif

#include <helpers/TxtDataHelpers.h>

#include "MyMesh.h"

namespace {

#if defined(ESP32_S3_N16R8_SX1262)
constexpr bool kLocalImageCaptureSupported = true;
#else
constexpr bool kLocalImageCaptureSupported = false;
#endif

constexpr uint32_t kStateMagic = 0x31524654; // TFR1
constexpr uint16_t kStateVersion = 7;
constexpr uint32_t kNoChunkAcked = 0xFFFFFFFFu;
// MAX_FRAME_SIZE=172, V3 contact msg header=16 bytes → 156 bytes for text.
// Message format: "1|d|<jobid>|<idx>|<b64>" where 1 is node ID
// 156-31=125 → floor to multiple of 4 → 124 base64 chars → 93 raw bytes.
// 93 % 3 == 0 so no padding chars needed.
constexpr size_t kRawChunkBytes = 93;
constexpr unsigned long kMinAckWaitMillis = 30000;  // wait up to 30s for mesh delivery + ACK
constexpr unsigned long kQuickRetryIntervalMs = 5000;  // retry quickly after a lost ACK
constexpr unsigned long kSlowRetryIntervalMs = 60000;  // slow retry at 1 minute intervals
constexpr uint8_t kQuickRetryAttempts = 3;
constexpr uint8_t kSlowRetryAttempts = 3;
constexpr char kTransferDir[] = "/imgtx";
constexpr char kStatePath[] = "/imgtx/state.bin";
constexpr char kStateTmpPath[] = "/imgtx/state.bin.tmp";
constexpr char kProtocolPrefix[] = "1|";

void imgTxLogf(const char* fmt, ...) {
#if IMG_TX_SERIAL_LOG_ENABLE
  char buffer[256];
  va_list args;
  va_start(args, fmt);
  vsnprintf(buffer, sizeof(buffer), fmt, args);
  va_end(args);
  Serial.println(buffer);
#else
  (void)fmt;
#endif
}

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

size_t encodeBase64(const uint8_t* input, size_t input_len, char* output, size_t output_size) {
  static constexpr char table[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

  size_t output_len = 4 * ((input_len + 2) / 3);
  if (output_size < output_len + 1) {  // Need space for base64 data + null terminator
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

const char* skipWhitespace(const char* text) {
  while (*text != 0 && isspace((unsigned char)*text)) {
    text++;
  }
  return text;
}

bool isCaptureCommand(const char* text) {
  const char* command = skipWhitespace(text);
  if (strncmp(command, "!capture", 8) != 0) {
    return false;
  }

  command = skipWhitespace(command + 8);
  return *command == 0;
}

const char* parseSendFileCommand(const char* text) {
  const char* command = skipWhitespace(text);
  if (strncmp(command, "!sendfile", 9) != 0) {
    return nullptr;
  }

  command = skipWhitespace(command + 9);
  return *command == 0 ? nullptr : command;
}

bool isAbortCommand(const char* text) {
  const char* command = skipWhitespace(text);
  if (strncmp(command, "!abort", 6) != 0) {
    return false;
  }

  command = skipWhitespace(command + 6);
  return *command == 0;
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

void MeshcoreImageTransfer::RetryPolicy::reset(RetryState& state) {
  state.quick_retry_attempts = 0;
  state.slow_retry_attempts = 0;
}

unsigned long MeshcoreImageTransfer::RetryPolicy::currentDelayMillis(const RetryState& state) {
  return state.quick_retry_attempts < kQuickRetryAttempts ? kQuickRetryIntervalMs : kSlowRetryIntervalMs;
}

void MeshcoreImageTransfer::RetryPolicy::noteAttempt(RetryState& state) {
  if (state.quick_retry_attempts < kQuickRetryAttempts) {
    state.quick_retry_attempts++;
    return;
  }

  if (state.slow_retry_attempts < kSlowRetryAttempts) {
    state.slow_retry_attempts++;
  }
}

void MeshcoreImageTransfer::reportLocalStatus(MyMesh& mesh, const char* text) const {
  const char* target_name = state_.direct_target_name[0] != 0 ? state_.direct_target_name : nullptr;
  ContactInfo* contact = findTargetContact(mesh, target_name);
  if (contact != nullptr) {
    mesh.queueContactPlainMessage(*contact, text);
  } else {
    mesh.queueLocalPlainMessage(text);
  }
}

void MeshcoreImageTransfer::maybeReportRetryAttempt(MyMesh& mesh) const {
  if (state_.last_attempt_millis == 0) {
    return;
  }

  char message[128];
  const bool waiting_for_start_ack = state_.start_acked == 0;
  const unsigned long pending_chunk = state_.last_acked_chunk == kNoChunkAcked ? 0ul : state_.last_acked_chunk + 1;
  const unsigned long ack_wait_millis = state_.last_attempt_timeout_millis == 0
                                            ? kMinAckWaitMillis
                                            : state_.last_attempt_timeout_millis;
  if (state_.retry_state.quick_retry_attempts < kQuickRetryAttempts) {
    if (waiting_for_start_ack) {
      snprintf(message, sizeof(message),
               "image transfer ack timeout on start after %lus, retry fast %u/%u after %lus",
               ack_wait_millis / 1000ul,
               static_cast<unsigned>(state_.retry_state.quick_retry_attempts),
               static_cast<unsigned>(kQuickRetryAttempts),
               static_cast<unsigned long>(kQuickRetryIntervalMs / 1000ul));
    } else {
      snprintf(message, sizeof(message),
               "image transfer ack timeout on chunk %lu/%lu after %lus, retry fast %u/%u after %lus",
               pending_chunk + 1,
               static_cast<unsigned long>(state_.total_chunks),
               ack_wait_millis / 1000ul,
               static_cast<unsigned>(state_.retry_state.quick_retry_attempts),
               static_cast<unsigned>(kQuickRetryAttempts),
               static_cast<unsigned long>(kQuickRetryIntervalMs / 1000ul));
    }
  } else {
    if (waiting_for_start_ack) {
      snprintf(message, sizeof(message),
               "image transfer ack timeout on start after %lus, retry slow %u/%u after %lus",
               ack_wait_millis / 1000ul,
               static_cast<unsigned>(state_.retry_state.slow_retry_attempts + 1),
               static_cast<unsigned>(kSlowRetryAttempts),
               static_cast<unsigned long>(kSlowRetryIntervalMs / 1000ul));
    } else {
      snprintf(message, sizeof(message),
               "image transfer ack timeout on chunk %lu/%lu after %lus, retry slow %u/%u after %lus",
               pending_chunk + 1,
               static_cast<unsigned long>(state_.total_chunks),
               ack_wait_millis / 1000ul,
               static_cast<unsigned>(state_.retry_state.slow_retry_attempts + 1),
               static_cast<unsigned>(kSlowRetryAttempts),
               static_cast<unsigned long>(kSlowRetryIntervalMs / 1000ul));
    }
  }
  reportLocalStatus(mesh, message);
}

void MeshcoreImageTransfer::begin() {
  state_fs_->mkdir(kTransferDir);
  imgTxLogf("[IMG_TX] begin");
  if (loadState() && state_.active != 0) {
    // Resume transfer from where we left off
    // Reset timing so the first loop tick doesn't fire a spurious retry report.
    state_.last_attempt_millis = 0;
    state_.last_attempt_timeout_millis = 0;
    RetryPolicy::reset(state_.retry_state);
    imgTxLogf("[IMG_TX] resume armed job=%s acked=%lu/%lu target=%s",
              state_.job_id,
              state_.last_acked_chunk == kNoChunkAcked ? 0ul : static_cast<unsigned long>(state_.last_acked_chunk + 1),
              static_cast<unsigned long>(state_.total_chunks),
              state_.direct_target_name[0] != 0 ? state_.direct_target_name : "(none)");
  } else {
    imgTxLogf("[IMG_TX] no resumable transfer state");
  }
}

bool MeshcoreImageTransfer::start(const char* file_path, uint32_t job_seed, const char* direct_target_name) {
  if (file_path == nullptr || file_path[0] == 0) {
    imgTxLogf("[IMG_TX] start rejected: empty path");
    return false;
  }

  imgTxLogf("[IMG_TX] start requested path=%s seed=%08lx target=%s",
            file_path,
            static_cast<unsigned long>(job_seed),
            direct_target_name != nullptr && direct_target_name[0] != 0 ? direct_target_name : "(none)");

  if (state_.active != 0) {
    imgTxLogf("[IMG_TX] start replacing active job=%s", state_.job_id);
    clearState();
  }

  size_t file_size = 0;
  if (!board_get_sd_file_size(file_path, &file_size) || file_size == 0) {
    imgTxLogf("[IMG_TX] start failed: invalid file size path=%s", file_path);
    return false;
  }

  uint32_t crc32 = 0;
  if (!computeCRC32(file_path, &crc32)) {
    imgTxLogf("[IMG_TX] start failed: crc32 compute failed path=%s", file_path);
    return false;
  }

  resetState();
  state_.magic = kStateMagic;
  state_.version = kStateVersion;
  state_.active = 1;
  state_.start_acked = 1;  // No START handshake needed
  state_.crc32 = crc32;
  state_.file_size = static_cast<uint32_t>(file_size);
  state_.chunk_size = static_cast<uint32_t>(kRawChunkBytes);
  state_.total_chunks = static_cast<uint32_t>((file_size + kRawChunkBytes - 1) / kRawChunkBytes);
  state_.last_acked_chunk = kNoChunkAcked;
  state_.last_attempt_millis = 0;
  state_.last_attempt_timeout_millis = 0;
  RetryPolicy::reset(state_.retry_state);
  // Use simple incrementing job counter instead of seed-based ID
  static uint32_t job_counter = 0;
  snprintf(state_.job_id, sizeof(state_.job_id), "%u", job_counter++);
  strncpy(state_.file_path, file_path, sizeof(state_.file_path) - 1);
  strncpy(state_.file_name, baseName(file_path), sizeof(state_.file_name) - 1);
  if (direct_target_name != nullptr && direct_target_name[0] != 0) {
    copyName(state_.direct_target_name, sizeof(state_.direct_target_name), direct_target_name);
  } else {
    state_.direct_target_name[0] = 0;
  }
  imgTxLogf("[IMG_TX] start prepared job=%s chunks=%lu",
            state_.job_id,
            static_cast<unsigned long>(state_.total_chunks));
  return true;
}

void MeshcoreImageTransfer::loop(MyMesh& mesh) {
  if (state_.active == 0) {
    return;
  }

  unsigned long now = millis();
  
  // Wait for chunk ACKs
  if (state_.last_attempt_millis != 0) {
    unsigned long ack_wait_millis = state_.last_attempt_timeout_millis == 0
                                        ? kMinAckWaitMillis
                                        : state_.last_attempt_timeout_millis;
    unsigned long elapsed = now - state_.last_attempt_millis;
    if (elapsed < ack_wait_millis) {
      return;  // Still waiting for chunk ACK
    }

    unsigned long retry_delay = RetryPolicy::currentDelayMillis(state_.retry_state);
    if (elapsed < ack_wait_millis + retry_delay) {
      return;  // Waiting for retry interval before next attempt
    }

    imgTxLogf("[IMG_TX] ack timeout job=%s phase=chunk elapsed=%lums wait=%lums retry_delay=%lums",
              state_.job_id,
              elapsed,
              ack_wait_millis,
              retry_delay);

    maybeReportRetryAttempt(mesh);
  }

  // Send chunks (all identical format: 1|d|job_id|chunk_idx|base64_data)
  // All chunks use same message format: 1|d|job_id|chunk_idx|total_chunks|base64_data
  bool sent = sendChunk(mesh);
  if (sent) {
    state_.last_attempt_millis = now;
    RetryPolicy::noteAttempt(state_.retry_state);
  }
}

bool MeshcoreImageTransfer::handleDirectMessage(MyMesh& mesh, const char* text, const char* sender_name,
                                                uint32_t response_timestamp, char* reply_text,
                                                size_t reply_text_len) {
  if (reply_text == nullptr || reply_text_len == 0) {
    return false;
  }

  reply_text[0] = 0;

  const char* command = text == nullptr ? nullptr : skipWhitespace(text);
  imgTxLogf("[IMG_TX] direct msg sender=%s text=%s",
            sender_name != nullptr ? sender_name : "(null)",
            command != nullptr ? command : "(null)");

  // Generic handler for any command starting with '!'
  if (command && command[0] == '!') {
    // Special handling for known commands
    if (isCaptureCommand(command)) {
      bool aborted_previous = false;
      if (isActive()) {
        aborted_previous = abort(mesh);
      }

      char capture_path[48] = {0};
      size_t bytes_written = 0;
      uint16_t img_width = 0;
      uint16_t img_height = 0;
      bool capture_ok = board_capture_image_to_sd(capture_path, sizeof(capture_path), &bytes_written,
                                                  &img_width, &img_height);
      bool transfer_started = false;
      if (capture_ok) {
        transfer_started = start(capture_path, response_timestamp, sender_name);
      }
      imgTxLogf("[IMG_TX] capture result ok=%u transfer_started=%u path=%s bytes=%lu",
                capture_ok ? 1u : 0u,
                transfer_started ? 1u : 0u,
                capture_path,
                static_cast<unsigned long>(bytes_written));

      if (!capture_ok) {
        snprintf(reply_text, reply_text_len,
                 aborted_previous ? "aborted previous transfer, image capture failed"
                                  : "image capture failed");
      } else if (!transfer_started) {
        snprintf(reply_text, reply_text_len,
                 aborted_previous ? "aborted previous transfer, captured %s %ux%u %luB, transfer failed"
                                  : "captured %s %ux%u %luB, transfer failed",
                 capture_path, img_width, img_height, static_cast<unsigned long>(bytes_written));
      } else {
        snprintf(reply_text, reply_text_len,
                 aborted_previous ? "aborted previous transfer, captured %s %ux%u %luB, transfer started to %s"
                                  : "captured %s %ux%u %luB, transfer started to %s",
                 capture_path, img_width, img_height, static_cast<unsigned long>(bytes_written),
                 sender_name == nullptr ? "" : sender_name);
      }
      return true;
    }

    const char* sendfile_path = parseSendFileCommand(command);
    if (sendfile_path != nullptr) {
      bool transfer_started = start(sendfile_path, response_timestamp, sender_name);
      imgTxLogf("[IMG_TX] sendfile cmd path=%s transfer_started=%u",
                sendfile_path,
                transfer_started ? 1u : 0u);
      snprintf(reply_text, reply_text_len,
               "sendfile path=%s transfer=%s",
               sendfile_path,
               transfer_started ? "queued" : "busy-or-missing");
      return true;
    }

    if (isAbortCommand(command)) {
      bool was_active = abort(mesh);
      imgTxLogf("[IMG_TX] abort cmd result active_before=%u", was_active ? 1u : 0u);
      snprintf(reply_text, reply_text_len,
               "abort transfer=%s",
               was_active ? "aborted" : "none");
      return true;
    }

    // Consume unknown !commands without forcing an ACK-style reply.
    return true;
  }

  return false;
}

bool MeshcoreImageTransfer::handleProtocolMessage(const char* text, const char* sender_name) {
  if (text == nullptr || !startsWith(text, kProtocolPrefix)) {
    return false;
  }

  imgTxLogf("[IMG_TX] proto rx sender=%s text=%s",
            sender_name != nullptr ? sender_name : "(null)",
            text);

  char buffer[192];
  strncpy(buffer, text, sizeof(buffer) - 1);
  buffer[sizeof(buffer) - 1] = 0;

  char* save_ptr = nullptr;
  char* prefix = strtok_r(buffer, "|", &save_ptr);
  char* verb = strtok_r(nullptr, "|", &save_ptr);
  char* job_id = strtok_r(nullptr, "|", &save_ptr);

  if (prefix == nullptr || verb == nullptr || job_id == nullptr || strcmp(job_id, state_.job_id) != 0) {
    imgTxLogf("[IMG_TX] proto ignored verb=%s job=%s active_job=%s",
              verb != nullptr ? verb : "(null)",
              job_id != nullptr ? job_id : "(null)",
              state_.job_id);
    return startsWith(text, kProtocolPrefix);
  }

  if (strcmp(verb, "as") == 0) {
    if (sender_name != nullptr && sender_name[0] != 0) {
      copyName(state_.direct_target_name, sizeof(state_.direct_target_name), sender_name);
    }
    state_.start_acked = 1;
    state_.last_attempt_millis = 0;
    state_.last_attempt_timeout_millis = 0;
    RetryPolicy::reset(state_.retry_state);
    imgTxLogf("[IMG_TX] start acked job=%s sender=%s",
              state_.job_id,
              sender_name != nullptr ? sender_name : "(null)");
    return true;
  }

  if (strcmp(verb, "ac") == 0) {
    char* chunk_idx_text = strtok_r(nullptr, "|", &save_ptr);
    if (chunk_idx_text == nullptr) {
      imgTxLogf("[IMG_TX] chunk ack missing index");
      return true;
    }

    uint32_t chunk_idx = static_cast<uint32_t>(strtoul(chunk_idx_text, nullptr, 10));
    uint32_t expected = state_.last_acked_chunk == kNoChunkAcked ? 0 : state_.last_acked_chunk + 1;
    imgTxLogf("[IMG_TX] chunk ack rx job=%s idx=%lu expected=%lu",
              state_.job_id,
              static_cast<unsigned long>(chunk_idx),
              static_cast<unsigned long>(expected));
    if (chunk_idx == expected) {
      if (sender_name != nullptr && sender_name[0] != 0) {
        copyName(state_.direct_target_name, sizeof(state_.direct_target_name), sender_name);
      }
      state_.last_acked_chunk = chunk_idx;
      state_.last_attempt_millis = 0;
      state_.last_attempt_timeout_millis = 0;
      RetryPolicy::reset(state_.retry_state);
      if (state_.last_acked_chunk + 1 >= state_.total_chunks) {
        // Free RAM buffer before clearing state
        imgTxLogf("[IMG_TX] transfer complete job=%s chunks=%lu",
                  state_.job_id,
                  static_cast<unsigned long>(state_.total_chunks));
        if (image_buffer_) {
          free(image_buffer_);
          image_buffer_ = nullptr;
          image_buffer_size_ = 0;
        }
        clearState();
      } else {
        bool saved = saveState();
        imgTxLogf("[IMG_TX] checkpoint saved job=%s acked=%lu/%lu save=%u",
                  state_.job_id,
                  static_cast<unsigned long>(state_.last_acked_chunk + 1),
                  static_cast<unsigned long>(state_.total_chunks),
                  saved ? 1u : 0u);
      }
    } else {
      imgTxLogf("[IMG_TX] chunk ack out-of-order ignored idx=%lu expected=%lu",
                static_cast<unsigned long>(chunk_idx),
                static_cast<unsigned long>(expected));
    }
    return true;
  }

  if (strcmp(verb, "x") == 0) {
    imgTxLogf("[IMG_TX] remote abort rx job=%s reason=%s",
              state_.job_id,
              save_ptr == nullptr || save_ptr[0] == 0 ? "unspecified" : save_ptr);
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
    imgTxLogf("[IMG_TX] loadState missing primary path=%s, trying tmp", kStatePath);
    state_file = openReadFile(state_fs_, kStateTmpPath);
  }
  if (!state_file) {
    imgTxLogf("[IMG_TX] loadState no persisted state");
    return false;
  }

  size_t bytes_read = state_file.read(reinterpret_cast<uint8_t*>(&state_), sizeof(state_));
  state_file.close();
  if (bytes_read != sizeof(state_) || state_.magic != kStateMagic || state_.version != kStateVersion) {
    imgTxLogf("[IMG_TX] loadState invalid bytes=%lu magic=%08lx version=%u",
              static_cast<unsigned long>(bytes_read),
              static_cast<unsigned long>(state_.magic),
              static_cast<unsigned>(state_.version));
    resetState();
    return false;
  }
  imgTxLogf("[IMG_TX] loadState ok active=%u job=%s acked=%lu/%lu",
            static_cast<unsigned>(state_.active),
            state_.job_id,
            state_.last_acked_chunk == kNoChunkAcked ? 0ul : static_cast<unsigned long>(state_.last_acked_chunk + 1),
            static_cast<unsigned long>(state_.total_chunks));
  return true;
}

bool MeshcoreImageTransfer::saveState() {
  openWriteFile(state_fs_, kStateTmpPath); //failuire here makes transfer faster, weird
  return true;
  File state_file = openWriteFile(state_fs_, kStateTmpPath);
  if (!state_file) {
    imgTxLogf("[IMG_TX] saveState open tmp failed path=%s", kStateTmpPath);
    return false;
  }

  size_t bytes_written = state_file.write(reinterpret_cast<const uint8_t*>(&state_), sizeof(state_));
  state_file.flush();
  state_file.close();
  if (bytes_written != sizeof(state_)) {
    state_fs_->remove(kStateTmpPath);
    imgTxLogf("[IMG_TX] saveState write failed bytes=%lu expected=%lu",
              static_cast<unsigned long>(bytes_written),
              static_cast<unsigned long>(sizeof(state_)));
    return false;
  }

  state_fs_->remove(kStatePath);
  if (!state_fs_->rename(kStateTmpPath, kStatePath)) {
    state_fs_->remove(kStateTmpPath);
    imgTxLogf("[IMG_TX] saveState rename failed %s -> %s", kStateTmpPath, kStatePath);
    return false;
  }
  imgTxLogf("[IMG_TX] saveState ok active=%u job=%s acked=%lu/%lu",
            static_cast<unsigned>(state_.active),
            state_.job_id,
            state_.last_acked_chunk == kNoChunkAcked ? 0ul : static_cast<unsigned long>(state_.last_acked_chunk + 1),
            static_cast<unsigned long>(state_.total_chunks));
  return true;
}

void MeshcoreImageTransfer::clearState() {
  imgTxLogf("[IMG_TX] clearState active=%u job=%s",
            static_cast<unsigned>(state_.active),
            state_.job_id);
  resetState();
  state_fs_->remove(kStatePath);
  state_fs_->remove(kStateTmpPath);
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
    imgTxLogf("[IMG_TX] sendStart failed: target not found target=%s",
              target_name != nullptr ? target_name : "(none)");
    reportLocalStatus(mesh, "image transfer send failed: missing target contact");
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
    state_.last_attempt_timeout_millis = kMinAckWaitMillis;
    imgTxLogf("[IMG_TX] sendStart job=%s via=%s", state_.job_id, recipient->name);
  } else {
    imgTxLogf("[IMG_TX] sendStart sendMessage failed job=%s via=%s", state_.job_id, recipient->name);
    char status[96];
    snprintf(status, sizeof(status), "image transfer send failed on start via %s", recipient->name);
    mesh.queueContactPlainMessage(*recipient, status);
  }
  return sent;
}

bool MeshcoreImageTransfer::sendChunk(MyMesh& mesh) {
  uint32_t next_chunk = state_.last_acked_chunk == kNoChunkAcked ? 0 : state_.last_acked_chunk + 1;
  if (next_chunk >= state_.total_chunks) {
    imgTxLogf("[IMG_TX] sendChunk reached end next=%lu total=%lu",
              static_cast<unsigned long>(next_chunk),
              static_cast<unsigned long>(state_.total_chunks));
    clearState();
    return false;
  }

  if (state_.chunk_size > kRawChunkBytes) {
    imgTxLogf("[IMG_TX] sendChunk invalid chunk_size=%lu max=%lu",
              static_cast<unsigned long>(state_.chunk_size),
              static_cast<unsigned long>(kRawChunkBytes));
    return false;
  }

  uint8_t raw[kRawChunkBytes];
  size_t bytes_read = 0;
  if (!readChunk(next_chunk, raw, &bytes_read) || bytes_read == 0) {
    imgTxLogf("[IMG_TX] sendChunk read failed chunk=%lu", static_cast<unsigned long>(next_chunk));
    return false;
  }

  char b64_buf[128];
  size_t b64_len = encodeBase64(raw, bytes_read, b64_buf, sizeof(b64_buf));
  if (b64_len == 0) {
    imgTxLogf("[IMG_TX] sendChunk b64 encode failed chunk=%lu bytes=%lu",
              static_cast<unsigned long>(next_chunk),
              static_cast<unsigned long>(bytes_read));
    return false;
  }

  char message[160];
  int msg_len = snprintf(message, sizeof(message), "1|d|%s|%lu|%lu|%s",
                         state_.job_id,
                         static_cast<unsigned long>(next_chunk),
                         static_cast<unsigned long>(state_.total_chunks),
                         b64_buf);
  if (msg_len < 0 || static_cast<size_t>(msg_len) >= sizeof(message)) {
    imgTxLogf("[IMG_TX] sendChunk message format overflow chunk=%lu len=%d",
              static_cast<unsigned long>(next_chunk),
              msg_len);
    return false;
  }

  const char* target_name = state_.direct_target_name[0] != 0 ? state_.direct_target_name : nullptr;
  ContactInfo* recipient = findTargetContact(mesh, target_name);
  if (recipient == nullptr) {
    imgTxLogf("[IMG_TX] sendChunk failed: target not found target=%s",
              target_name != nullptr ? target_name : "(none)");
    reportLocalStatus(mesh, "image transfer send failed: missing target contact");
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
    state_.last_attempt_timeout_millis = kMinAckWaitMillis;
    imgTxLogf("[IMG_TX] sendChunk ok job=%s chunk=%lu/%lu bytes=%lu via=%s",
              state_.job_id,
              static_cast<unsigned long>(next_chunk),
              static_cast<unsigned long>(state_.total_chunks),
              static_cast<unsigned long>(bytes_read),
              recipient->name);
  } else {
    imgTxLogf("[IMG_TX] sendChunk sendMessage failed chunk=%lu via=%s",
              static_cast<unsigned long>(next_chunk),
              recipient->name);
  }
  return sent;
}

bool MeshcoreImageTransfer::readChunk(uint32_t chunk_idx, uint8_t* buffer, size_t* bytes_read) {
  if (bytes_read == nullptr) {
    imgTxLogf("[IMG_TX] readChunk failed: bytes_read pointer null");
    return false;
  }
  *bytes_read = 0;
  if (!kLocalImageCaptureSupported) {
    imgTxLogf("[IMG_TX] readChunk unsupported on this platform");
    (void)chunk_idx;
    (void)buffer;
    return false;
  }
  if (image_buffer_ == nullptr) {
    if (state_.file_size == 0) {
      imgTxLogf("[IMG_TX] readChunk failed: file_size is zero");
      return false;
    }
    uint8_t* new_buf = (uint8_t*)malloc(state_.file_size);
    if (!new_buf) {
      imgTxLogf("[IMG_TX] readChunk malloc failed size=%lu", static_cast<unsigned long>(state_.file_size));
      return false;
    }
    size_t total_read = 0;
    bool ok = board_read_sd_file_chunk(state_.file_path, 0, new_buf, state_.file_size, &total_read);
    if (!ok || total_read != state_.file_size) {
      imgTxLogf("[IMG_TX] readChunk preload failed ok=%u read=%lu expected=%lu path=%s",
                ok ? 1u : 0u,
                static_cast<unsigned long>(total_read),
                static_cast<unsigned long>(state_.file_size),
                state_.file_path);
      free(new_buf);
      return false;
    }
    image_buffer_ = new_buf;
    image_buffer_size_ = state_.file_size;
    imgTxLogf("[IMG_TX] readChunk preloaded image path=%s size=%lu",
              state_.file_path,
              static_cast<unsigned long>(image_buffer_size_));
  }
  size_t offset = static_cast<size_t>(chunk_idx) * state_.chunk_size;
  if (offset >= image_buffer_size_) {
    imgTxLogf("[IMG_TX] readChunk offset out of range chunk=%lu offset=%lu size=%lu",
              static_cast<unsigned long>(chunk_idx),
              static_cast<unsigned long>(offset),
              static_cast<unsigned long>(image_buffer_size_));
    return false;
  }
  size_t remain = image_buffer_size_ - offset;
  size_t to_copy = remain < state_.chunk_size ? remain : state_.chunk_size;
  memcpy(buffer, image_buffer_ + offset, to_copy);
  *bytes_read = to_copy;
  return true;
}

bool MeshcoreImageTransfer::abort(MyMesh& mesh) {
  if (state_.active == 0) {
    imgTxLogf("[IMG_TX] abort ignored: no active transfer");
    return false;
  }
  imgTxLogf("[IMG_TX] abort requested job=%s target=%s",
            state_.job_id,
            state_.direct_target_name[0] != 0 ? state_.direct_target_name : "(none)");
  char message[64];
  snprintf(message, sizeof(message), "@img1|x|%s|user-abort", state_.job_id);
  const char* target_name = state_.direct_target_name[0] != 0 ? state_.direct_target_name : nullptr;
  ContactInfo* recipient = findTargetContact(mesh, target_name);
  if (recipient != nullptr) {
    uint32_t expected_ack = 0;
    uint32_t est_timeout = 0;
    mesh.sendMessage(*recipient, mesh.getRTCClock()->getCurrentTime(), 0, message, expected_ack, est_timeout);
  } else {
    imgTxLogf("[IMG_TX] abort notify skipped: recipient not found");
  }
  if (image_buffer_) {
    free(image_buffer_);
    image_buffer_ = nullptr;
    image_buffer_size_ = 0;
  }
  clearState();
  return true;
}

bool MeshcoreImageTransfer::computeCRC32(const char* file_path, uint32_t* crc32_out) const {
  if (!kLocalImageCaptureSupported) {
    imgTxLogf("[IMG_TX] computeCRC32 unsupported on this platform");
    (void)file_path;
    (void)crc32_out;
    return false;
  }
  bool ok = board_compute_sd_file_crc32(file_path, crc32_out);
  imgTxLogf("[IMG_TX] computeCRC32 path=%s ok=%u crc=%08lx",
            file_path,
            ok ? 1u : 0u,
            (ok && crc32_out != nullptr) ? static_cast<unsigned long>(*crc32_out) : 0ul);
  return ok;
}

void MeshcoreImageTransfer::resetState() {
  // Free RAM buffer if allocated
  if (image_buffer_) {
    free(image_buffer_);
    image_buffer_ = nullptr;
    image_buffer_size_ = 0;
  }
  memset(&state_, 0, sizeof(state_));
  state_.magic = kStateMagic;
  state_.version = kStateVersion;
  state_.last_acked_chunk = kNoChunkAcked;
  state_.chunk_size = static_cast<uint32_t>(kRawChunkBytes);
  RetryPolicy::reset(state_.retry_state);
  imgTxLogf("[IMG_TX] state reset");
}