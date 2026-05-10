#include "ESP32S3N16R8SX1262Board.h"

#include <Arduino.h>
#include <FS.h>
#include <SPI.h>
#include <SD_MMC.h>
#include <esp_camera.h>

namespace {

framesize_t preferred_sensor_framesize(sensor_t* sensor) {
  if (sensor == nullptr) {
    return FRAMESIZE_UXGA;
  }

  const camera_sensor_info_t* info = esp_camera_sensor_get_info(&sensor->id);
  if (info == nullptr) {
    return FRAMESIZE_UXGA;
  }

  return info->max_size;
}

bool init_camera() {
  camera_config_t config = {};
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = CAM_PIN_Y2;
  config.pin_d1 = CAM_PIN_Y3;
  config.pin_d2 = CAM_PIN_Y4;
  config.pin_d3 = CAM_PIN_Y5;
  config.pin_d4 = CAM_PIN_Y6;
  config.pin_d5 = CAM_PIN_Y7;
  config.pin_d6 = CAM_PIN_Y8;
  config.pin_d7 = CAM_PIN_Y9;
  config.pin_xclk = CAM_PIN_XCLK;
  config.pin_pclk = CAM_PIN_PCLK;
  config.pin_vsync = CAM_PIN_VSYNC;
  config.pin_href = CAM_PIN_HREF;
  config.pin_sccb_sda = CAM_PIN_SIOD;
  config.pin_sccb_scl = CAM_PIN_SIOC;
  config.pin_pwdn = CAM_PIN_PWDN;
  config.pin_reset = CAM_PIN_RESET;
  config.xclk_freq_hz = CAM_XCLK_FREQ_HZ;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_UXGA;
  config.jpeg_quality = CAM_JPEG_QUALITY;
  config.fb_count = CAM_FB_COUNT;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = psramFound() ? CAMERA_FB_IN_PSRAM : CAMERA_FB_IN_DRAM;

  if (psramFound()) {
    config.jpeg_quality = CAM_JPEG_QUALITY <= 10 ? CAM_JPEG_QUALITY : 10;
    config.fb_count = CAM_FB_COUNT >= 2 ? CAM_FB_COUNT : 2;
    config.grab_mode = CAMERA_GRAB_LATEST;
  }

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("camera init failed: 0x%x\n", err);
    return false;
  }

  sensor_t* sensor = esp_camera_sensor_get();
  framesize_t target_framesize = preferred_sensor_framesize(sensor);
  if (sensor != nullptr && sensor->set_framesize(sensor, target_framesize) != ESP_OK) {
    Serial.println("camera framesize switch failed");
  }

  return true;
}

bool init_sd() {
#if defined(BOARD_USE_SD_MMC)
  if (!SD_MMC.setPins(SDMMC_CLK_PIN, SDMMC_CMD_PIN, SDMMC_D0_PIN)) {
    Serial.println("sd pin assign failed");
    return false;
  }

  bool mounted = SD_MMC.begin("/sdcard", true);
  if (!mounted) {
    Serial.println("sd init failed");
    return false;
  }

  uint8_t card_type = SD_MMC.cardType();
  if (card_type == CARD_NONE) {
    Serial.println("sd card missing");
    return false;
  }

  return true;
#else
  return false;
#endif
}

} // namespace

bool ESP32S3N16R8SX1262Board::mountSD() {
  if (sd_mounted) {
    return true;
  }

  sd_online = init_sd();
  sd_mounted = sd_online;
  return sd_online;
}

void ESP32S3N16R8SX1262Board::unmountSD() {
  if (!sd_mounted) {
    return;
  }

  SD_MMC.end();
  sd_mounted = false;
}

void ESP32S3N16R8SX1262Board::begin() {
  ESP32Board::begin();

  camera_online = init_camera();
  sd_online = mountSD();
  unmountSD();
}

bool ESP32S3N16R8SX1262Board::captureToSD(char* path_buffer, size_t path_buffer_size,
                                         size_t* bytes_written) {
  if (bytes_written != nullptr) {
    *bytes_written = 0;
  }

  if (!camera_online || path_buffer == nullptr || path_buffer_size == 0) {
    return false;
  }

  if (!mountSD()) {
    return false;
  }

  camera_fb_t* frame = esp_camera_fb_get();
  if (frame == nullptr) {
    Serial.println("camera capture failed");
    unmountSD();
    return false;
  }

  unsigned long capture_id = millis();
  snprintf(path_buffer, path_buffer_size, "/capture_%010lu.jpg", capture_id);

  File image = SD_MMC.open(path_buffer, FILE_WRITE);
  if (!image) {
    Serial.printf("capture open failed: %s\n", path_buffer);
    esp_camera_fb_return(frame);
    path_buffer[0] = 0;
    unmountSD();
    return false;
  }

  size_t written = image.write(frame->buf, frame->len);
  image.flush();
  image.close();
  esp_camera_fb_return(frame);

  if (bytes_written != nullptr) {
    *bytes_written = written;
  }

  if (written != frame->len) {
    Serial.printf("capture write failed: %s (%u/%u)\n", path_buffer,
                  static_cast<unsigned>(written), static_cast<unsigned>(frame->len));
    SD_MMC.remove(path_buffer);
    path_buffer[0] = 0;
    unmountSD();
    return false;
  }

  Serial.printf("capture saved: %s (%u bytes)\n", path_buffer, static_cast<unsigned>(written));
  unmountSD();
  return true;
}