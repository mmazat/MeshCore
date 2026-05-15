#include "MeshcoreImageTransfer.h"

#include <stdarg.h>

#include <target.h>

#if defined(ESP32_S3_N16R8_SX1262)
#include <SD_MMC.h>
#endif

#include "MyMesh.h"

namespace {

constexpr size_t kRawChunkBytes = 93;
constexpr char kCurrentImagePath[] = "/imgtx/current.jpg";

uint32_t calcCrc32(const uint8_t* data, size_t len) {
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; bit++) {
      uint32_t mask = -(crc & 1u);
      crc = (crc >> 1) ^ (0xEDB88320u & mask);
    }
  }
  return ~crc;
}

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

size_t encodeBase64(const uint8_t* input, size_t input_len, char* output, size_t output_size) {
  static constexpr char table[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

  size_t output_len = 4 * ((input_len + 2) / 3);
  if (output_size < output_len + 1) {
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

const char* skipWhitespace(const char* text) {
  while (*text != 0 && isspace((unsigned char)*text)) {
    text++;
  }
  return text;
}

ContactInfo* findTargetContact(MyMesh& mesh, const char* target_name) {
  if (target_name == nullptr || target_name[0] == 0) {
    return nullptr;
  }

  ContactInfo* c = mesh.searchContactsByPrefix(target_name);
  if (c != nullptr) {
    return c;
  }

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

}  // namespace

void MeshcoreImageTransfer::begin() {
  imgTxLogf("[IMG_TX] begin");
}

bool MeshcoreImageTransfer::start(const char* file_path, uint32_t job_seed, const char* direct_target_name) {
  (void)job_seed;
  (void)direct_target_name;

  if (file_path == nullptr || file_path[0] == 0) {
    return false;
  }

  // Optional helper path for manual sendfile workflows: copy to canonical current.jpg
  bool copied = copyFileOnSd(file_path, kCurrentImagePath);
  if (!copied) {
    imgTxLogf("[IMG_TX] start copy failed src=%s dst=%s", file_path, kCurrentImagePath);
    return false;
  }

  freeImageBuffer();
  return loadCurrentJpgToBuffer();
}

void MeshcoreImageTransfer::loop(MyMesh& mesh) {
  (void)mesh;
  // Stateless mode: no auto-send, no retries, no timeouts.
}

bool MeshcoreImageTransfer::handleProtocolMessage(const char* text, const char* sender_name) {
  (void)text;
  (void)sender_name;
  // Stateless mode does not use protocol ACK/control frames.
  return false;
}

bool MeshcoreImageTransfer::formatBleProgressMessage(const char* text, char* out, size_t out_len) const {
  (void)text;
  if (out != nullptr && out_len > 0) {
    out[0] = 0;
  }
  return false;
}

void MeshcoreImageTransfer::freeImageBuffer() {
  if (image_buffer_ != nullptr) {
    free(image_buffer_);
    image_buffer_ = nullptr;
    image_buffer_size_ = 0;
    total_chunks_ = 0;
  }
}

bool MeshcoreImageTransfer::copyFileOnSd(const char* src_path, const char* dst_path) {
#if !defined(ESP32_S3_N16R8_SX1262)
  (void)src_path;
  (void)dst_path;
  return false;
#else
  File src = SD_MMC.open(src_path, "r");
  if (!src) {
    return false;
  }

  SD_MMC.mkdir("/imgtx");
  SD_MMC.remove(dst_path);

  File dst = SD_MMC.open(dst_path, "w");
  if (!dst) {
    src.close();
    return false;
  }

  uint8_t buf[1024];
  while (true) {
    int n = src.read(buf, sizeof(buf));
    if (n <= 0) {
      break;
    }
    if (dst.write(buf, n) != (size_t)n) {
      dst.close();
      src.close();
      return false;
    }
  }

  dst.close();
  src.close();
  return true;
#endif
}

bool MeshcoreImageTransfer::captureCurrentJpgIfMissing() {
  size_t file_size = 0;
  if (board_get_sd_file_size(kCurrentImagePath, &file_size) && file_size > 0) {
    return true;
  }

  // If no current.jpg exists, capture and normalize to /imgtx/current.jpg
  char captured_path[64] = {0};
  size_t bytes_written = 0;
  uint16_t img_width = 0;
  uint16_t img_height = 0;
  bool capture_ok = board_capture_image_to_sd(captured_path, sizeof(captured_path), &bytes_written,
                                              &img_width, &img_height);
  if (!capture_ok || bytes_written == 0) {
    imgTxLogf("[IMG_TX] capture failed");
    return false;
  }

  if (strcmp(captured_path, kCurrentImagePath) != 0) {
    if (!copyFileOnSd(captured_path, kCurrentImagePath)) {
      imgTxLogf("[IMG_TX] capture copy failed src=%s", captured_path);
      return false;
    }
  }

  imgTxLogf("[IMG_TX] capture ready path=%s bytes=%lu", kCurrentImagePath, (unsigned long)bytes_written);
  return true;
}

bool MeshcoreImageTransfer::loadCurrentJpgToBuffer() {
  size_t file_size = 0;
  if (!board_get_sd_file_size(kCurrentImagePath, &file_size) || file_size == 0) {
    return false;
  }

  uint8_t* new_buf = (uint8_t*)malloc(file_size);
  if (new_buf == nullptr) {
    return false;
  }

  size_t total_read = 0;
  bool ok = board_read_sd_file_chunk(kCurrentImagePath, 0, new_buf, file_size, &total_read);
  if (!ok || total_read != file_size) {
    free(new_buf);
    return false;
  }

  freeImageBuffer();
  image_buffer_ = new_buf;
  image_buffer_size_ = file_size;
  total_chunks_ = (uint32_t)((file_size + kRawChunkBytes - 1) / kRawChunkBytes);
  imgTxLogf("[IMG_TX] loaded current.jpg bytes=%lu chunks=%lu",
            (unsigned long)image_buffer_size_,
            (unsigned long)total_chunks_);
  return true;
}

bool MeshcoreImageTransfer::ensureImageReady() {
  if (image_buffer_ != nullptr && image_buffer_size_ > 0 && total_chunks_ > 0) {
    return true;
  }

  if (!captureCurrentJpgIfMissing()) {
    return false;
  }

  return loadCurrentJpgToBuffer();
}

bool MeshcoreImageTransfer::sendChunkByIndex(MyMesh& mesh, int chunk_id, const char* sender_name) {
  if (chunk_id < 0) {
    return false;
  }

  if (!ensureImageReady()) {
    return false;
  }

  size_t offset = (size_t)chunk_id * kRawChunkBytes;
  if (offset >= image_buffer_size_) {
    return false;
  }

  size_t bytes_read = image_buffer_size_ - offset;
  if (bytes_read > kRawChunkBytes) {
    bytes_read = kRawChunkBytes;
  }

  char b64_buf[128];
  size_t b64_len = encodeBase64(image_buffer_ + offset, bytes_read, b64_buf, sizeof(b64_buf));
  if (b64_len == 0) {
    return false;
  }

  uint32_t chunk_crc32 = calcCrc32(image_buffer_ + offset, bytes_read);

  char message[160];
  // Packet format: chunk_id|total_chunks|crc32_hex|Base64_data
  int msg_len = snprintf(message, sizeof(message), "%d|%lu|%08lx|%s",
                         chunk_id,
                         (unsigned long)total_chunks_,
                         (unsigned long)chunk_crc32,
                         b64_buf);
  if (msg_len <= 0 || (size_t)msg_len >= sizeof(message)) {
    return false;
  }

  ContactInfo* recipient = findTargetContact(mesh, sender_name);
  if (recipient == nullptr) {
    return false;
  }

  // Fire-and-forget send. No ACK tracking, no retries.
  uint32_t expected_ack = 0;
  uint32_t est_timeout = 0;
  bool sent = mesh.sendMessage(*recipient,
                               mesh.getRTCClock()->getCurrentTime(),
                               0,
                               message,
                               expected_ack,
                               est_timeout) != MSG_SEND_FAILED;
  if (sent) {
    imgTxLogf("[IMG_TX] sent chunk=%d/%lu", chunk_id, (unsigned long)total_chunks_);
  }
  return sent;
}

void MeshcoreImageTransfer::handleCaptureChunkCommand(MyMesh& mesh, int chunk_id, const char* sender_name) {
  imgTxLogf("[IMG_TX] request chunk=%d from=%s",
            chunk_id,
            sender_name != nullptr && sender_name[0] != 0 ? sender_name : "(unknown)");
  (void)sendChunkByIndex(mesh, chunk_id, sender_name);
}

bool MeshcoreImageTransfer::handleDirectMessage(MyMesh& mesh, const char* text, const char* sender_name,
                                                uint32_t response_timestamp, char* reply_text,
                                                size_t reply_text_len) {
  (void)response_timestamp;

  if (reply_text == nullptr || reply_text_len == 0) {
    return false;
  }
  reply_text[0] = 0;

  const char* command = text == nullptr ? nullptr : skipWhitespace(text);
  if (command == nullptr || command[0] != '!') {
    return false;
  }

  if (strncmp(command, "!capture|", 9) == 0) {
    const char* idx_text = command + 9;
    while (*idx_text != 0 && isspace((unsigned char)*idx_text)) {
      idx_text++;
    }
    if (*idx_text == 0) {
      snprintf(reply_text, reply_text_len, "capture request missing chunk id");
      return true;
    }

    long chunk_id = strtol(idx_text, nullptr, 10);
    imgTxLogf("[IMG_TX] request chunk=%ld from=%s",
              chunk_id,
              sender_name != nullptr && sender_name[0] != 0 ? sender_name : "(unknown)");
    bool sent = sendChunkByIndex(mesh, (int)chunk_id, sender_name);
    if (!sent) {
      snprintf(reply_text, reply_text_len, "capture chunk %ld failed", chunk_id);
    }
    return true;
  }

  if (strcmp(command, "!capture") == 0) {
    imgTxLogf("[IMG_TX] request chunk=0 from=%s",
              sender_name != nullptr && sender_name[0] != 0 ? sender_name : "(unknown)");
    bool sent = sendChunkByIndex(mesh, 0, sender_name);
    if (!sent) {
      snprintf(reply_text, reply_text_len, "capture chunk 0 failed");
    }
    return true;
  }

  if (strcmp(command, "!abort") == 0) {
    freeImageBuffer();
#if defined(ESP32_S3_N16R8_SX1262)
    SD_MMC.remove(kCurrentImagePath);
#endif
    snprintf(reply_text, reply_text_len, "abort done");
    imgTxLogf("[IMG_TX] abort: cleared buffer and deleted %s", kCurrentImagePath);
    return true;
  }

  return true;
}

bool MeshcoreImageTransfer::abort(MyMesh& mesh) {
  (void)mesh;
  freeImageBuffer();
#if defined(ESP32_S3_N16R8_SX1262)
  SD_MMC.remove(kCurrentImagePath);
#endif
  imgTxLogf("[IMG_TX] abort api: cleared buffer and deleted %s", kCurrentImagePath);
  return true;
}
