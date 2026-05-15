#include "MeshcoreImageTransfer.h"

#include <stdarg.h>

#include <target.h>

#if defined(ESP32_S3_N16R8_SX1262)
#include <SD_MMC.h>
#endif

#include "MyMesh.h"

namespace {

constexpr size_t kRawChunkBytes = 93;

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

int findNextJobIdOnSd() {
  int max_job_id = -1;
#if defined(ESP32_S3_N16R8_SX1262)
  File dir = SD_MMC.open("/imgtx");
  if (dir) {
    File entry = dir.openNextFile();
    while (entry) {
      const char* fname = entry.name();
      const char* ext = strrchr(fname, '.');
      if (ext && strcmp(ext, ".jpg") == 0) {
        const char* base = strrchr(fname, '/');
        const char* num = base ? base + 1 : fname;
        char numbuf[16] = {0};
        strncpy(numbuf, num, sizeof(numbuf) - 1);
        char* dot = strrchr(numbuf, '.');
        if (dot) *dot = '\0';
        bool all_digits = true;
        for (size_t i = 0; numbuf[i]; i++) {
          if (!isdigit((unsigned char)numbuf[i])) {
            all_digits = false;
            break;
          }
        }
        if (all_digits && numbuf[0] != '\0') {
          int jid = atoi(numbuf);
          if (jid > max_job_id) max_job_id = jid;
        }
      }
      entry = dir.openNextFile();
    }
    dir.close();
  }
#endif
  return max_job_id + 1;
}

}  // namespace

void MeshcoreImageTransfer::begin() {
  imgTxLogf("[IMG_TX] begin");
}

bool MeshcoreImageTransfer::start(const char* file_path, uint32_t job_seed, const char* direct_target_name) {
  (void)file_path;
  (void)job_seed;
  (void)direct_target_name;
  // Start is intentionally inert. A new capture is created only when !capture
  // is received with job_id=-1.
  imgTxLogf("[IMG_TX] start: no-op (capture deferred to !capture)");
  return true;
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
  image_job_id_ = -1;
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

bool MeshcoreImageTransfer::captureImageToJobFile(const char* job_img_path) {
  if (job_img_path == nullptr || job_img_path[0] == 0) {
    return false;
  }

#if !defined(ESP32_S3_N16R8_SX1262)
  (void)job_img_path;
  return false;
#else
  SD_MMC.mkdir("/imgtx");

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

  if (strcmp(captured_path, job_img_path) != 0) {
    if (!copyFileOnSd(captured_path, job_img_path)) {
      imgTxLogf("[IMG_TX] capture copy failed src=%s dst=%s", captured_path, job_img_path);
      return false;
    }
  }

  imgTxLogf("[IMG_TX] capture saved path=%s bytes=%lu", job_img_path, (unsigned long)bytes_written);
  return true;
#endif
}

bool MeshcoreImageTransfer::loadJobJpgToBuffer(const char* job_img_path) {
  if (job_img_path == nullptr || job_img_path[0] == 0) {
    return false;
  }

  size_t file_size = 0;
  if (!board_get_sd_file_size(job_img_path, &file_size) || file_size == 0) {
    return false;
  }

  uint8_t* new_buf = (uint8_t*)malloc(file_size);
  if (new_buf == nullptr) {
    return false;
  }

  size_t total_read = 0;
  bool ok = board_read_sd_file_chunk(job_img_path, 0, new_buf, file_size, &total_read);
  if (!ok || total_read != file_size) {
    free(new_buf);
    return false;
  }

  freeImageBuffer();
  image_buffer_ = new_buf;
  image_buffer_size_ = file_size;
  total_chunks_ = (uint32_t)((file_size + kRawChunkBytes - 1) / kRawChunkBytes);
  imgTxLogf("[IMG_TX] loaded %s bytes=%lu chunks=%lu",
            job_img_path,
            (unsigned long)image_buffer_size_,
            (unsigned long)total_chunks_);
  return true;
}

bool MeshcoreImageTransfer::sendChunkByIndex(MyMesh& mesh, int chunk_id, const char* sender_name) {
  if (chunk_id < 0) {
    return false;
  }

  // Buffer/job must already be prepared by command routing.
  if (image_buffer_ == nullptr || image_buffer_size_ == 0 || total_chunks_ == 0 || image_job_id_ < 0) {
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

  char message[512];
  // Packet format: job_id|chunk_id|total_chunks|crc32_hex|Base64_data
  int msg_len = snprintf(message, sizeof(message), "%d|%d|%lu|%08lx|%s",
                         (image_job_id_ >= 0 ? image_job_id_ : 0),
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
    imgTxLogf("[IMG_TX] sent chunk job=%d chunk=%d/%lu", (image_job_id_ >= 0 ? image_job_id_ : 0), chunk_id, (unsigned long)total_chunks_);
  }
  return sent;
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
    // Format: !capture|<job_id>|<chunk_id>
    const char* args = command + 9;
    while (*args != 0 && isspace((unsigned char)*args)) args++;
    const char* sep = strchr(args, '|');
    if (sep == nullptr) {
      snprintf(reply_text, reply_text_len, "bad request");
      return true;
    }
    char jidbuf[16] = {0};
    size_t jidlen = sep - args;
    if (jidlen == 0 || jidlen >= sizeof(jidbuf)) {
      snprintf(reply_text, reply_text_len, "bad request");
      return true;
    }
    strncpy(jidbuf, args, jidlen);
    int req_job_id = atoi(jidbuf);
    int chunk_id = atoi(sep + 1);

    int effective_job_id = req_job_id;
    if (req_job_id < 0) {
      // job_id=-1 means create a new capture/job and send first chunk.
      int new_job_id = findNextJobIdOnSd();
      char job_img_path[64];
      snprintf(job_img_path, sizeof(job_img_path), "/imgtx/%d.jpg", new_job_id);
      if (!captureImageToJobFile(job_img_path)) {
        snprintf(reply_text, reply_text_len, "capture failed");
        return true;
      }
      if (!loadJobJpgToBuffer(job_img_path)) {
        snprintf(reply_text, reply_text_len, "load failed");
        return true;
      }
      image_job_id_ = new_job_id;
      effective_job_id = new_job_id;
      chunk_id = 0;
      imgTxLogf("[IMG_TX] new capture job_id=%d", effective_job_id);
    } else {
      if (last_aborted_job_id_ >= 0 && req_job_id == last_aborted_job_id_) {
        imgTxLogf("[IMG_TX] ignoring chunk request for aborted job_id=%d", req_job_id);
        snprintf(reply_text, reply_text_len, "job aborted");
        return true;
      }
      if (image_job_id_ != req_job_id) {
        char job_img_path[64];
        snprintf(job_img_path, sizeof(job_img_path), "/imgtx/%d.jpg", req_job_id);
        if (!loadJobJpgToBuffer(job_img_path)) {
          snprintf(reply_text, reply_text_len, "job not found");
          return true;
        }
        image_job_id_ = req_job_id;
      }
      effective_job_id = req_job_id;
    }

    if (last_aborted_job_id_ >= 0 && effective_job_id == last_aborted_job_id_) {
      imgTxLogf("[IMG_TX] ignoring chunk request for aborted job_id=%d", effective_job_id);
      snprintf(reply_text, reply_text_len, "job aborted");
      return true;
    }
    imgTxLogf("[IMG_TX] request chunk=%d job_id=%d from=%s", chunk_id, effective_job_id,
              sender_name != nullptr && sender_name[0] != 0 ? sender_name : "(unknown)");
    bool sent = sendChunkByIndex(mesh, chunk_id, sender_name);
    if (!sent) {
      snprintf(reply_text, reply_text_len, "capture chunk %d failed", chunk_id);
    }
    return true;
  }

  if (strcmp(command, "!capture") == 0) {
    // Bare !capture behaves as !capture|-1|0 (new capture).
    int new_job_id = findNextJobIdOnSd();
    char job_img_path[64];
    snprintf(job_img_path, sizeof(job_img_path), "/imgtx/%d.jpg", new_job_id);
    if (!captureImageToJobFile(job_img_path)) {
      snprintf(reply_text, reply_text_len, "capture failed");
      return true;
    }
    if (!loadJobJpgToBuffer(job_img_path)) {
      snprintf(reply_text, reply_text_len, "load failed");
      return true;
    }
    image_job_id_ = new_job_id;
    imgTxLogf("[IMG_TX] bootstrap !capture new job_id=%d from=%s", image_job_id_,
              sender_name != nullptr && sender_name[0] != 0 ? sender_name : "(unknown)");
    bool sent = sendChunkByIndex(mesh, 0, sender_name);
    if (!sent) {
      snprintf(reply_text, reply_text_len, "capture chunk 0 failed");
    }
    return true;
  }

  if (strncmp(command, "!abort", 6) == 0) {
    // Abort always targets the current active job on companion.
    int aborted_job_id = image_job_id_;
    freeImageBuffer();
    if (aborted_job_id >= 0) {
      last_aborted_job_id_ = aborted_job_id;
      imgTxLogf("[IMG_TX] abort: set last_aborted_job_id_=%d", last_aborted_job_id_);
    } else {
      imgTxLogf("[IMG_TX] abort: no active job, last_aborted_job_id_ unchanged (%d)", last_aborted_job_id_);
    }
    // Do not remove any images on abort; preserve all job_id.jpg files.
    snprintf(reply_text, reply_text_len, "abort done");
    imgTxLogf("[IMG_TX] abort: cleared buffer, all images preserved");
    return true;
  }

  return true;
}

bool MeshcoreImageTransfer::abort(MyMesh& mesh) {
  (void)mesh;
  // Mark current job as aborted so future chunk requests are ignored.
  if (image_job_id_ >= 0) {
    last_aborted_job_id_ = image_job_id_;
    imgTxLogf("[IMG_TX] abort api: set last_aborted_job_id_=%d", last_aborted_job_id_);
  } else {
    imgTxLogf("[IMG_TX] abort api: no active job, last_aborted_job_id_ unchanged");
  }
  freeImageBuffer();
  // Do not remove any images on abort; preserve all job_id.jpg files.
  imgTxLogf("[IMG_TX] abort api: cleared buffer, all images preserved");
  return true;
}
