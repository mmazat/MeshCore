#include <Arduino.h>
#include "target.h"

ESP32S3N16R8SX1262Board board;

static SPIClass spi;
RADIO_CLASS radio = new Module(P_LORA_NSS, P_LORA_DIO_1, P_LORA_RESET, P_LORA_BUSY, spi);
WRAPPER_CLASS radio_driver(radio, board);

ESP32RTCClock fallback_clock;
AutoDiscoverRTCClock rtc_clock(fallback_clock);
SensorManager sensors;

#ifndef LORA_CR
  #define LORA_CR 5
#endif

bool radio_init() {
  fallback_clock.begin();
  rtc_clock.begin(Wire);

  return radio.std_init(&spi);
}

uint32_t radio_get_rng_seed() {
  return radio.random(0x7FFFFFFF);
}

bool board_capture_image_to_sd(char* path_buffer, size_t path_buffer_size, size_t* bytes_written,
                              uint16_t* out_width, uint16_t* out_height) {
  return board.captureToSD(path_buffer, path_buffer_size, bytes_written, out_width, out_height);
}

bool board_get_sd_file_size(const char* path, size_t* file_size) {
  return board.getSDFileSize(path, file_size);
}

bool board_compute_sd_file_crc32(const char* path, uint32_t* crc32_out) {
  return board.computeSDFileCRC32(path, crc32_out);
}

bool board_read_sd_file_chunk(const char* path, size_t offset, uint8_t* buffer, size_t buffer_size, size_t* bytes_read) {
  return board.readSDFileChunk(path, offset, buffer, buffer_size, bytes_read);
}

void radio_set_params(float freq, float bw, uint8_t sf, uint8_t cr) {
  radio.setFrequency(freq);
  radio.setSpreadingFactor(sf);
  radio.setBandwidth(bw);
  radio.setCodingRate(cr);
}

void radio_set_tx_power(int8_t dbm) {
  radio.setOutputPower(dbm);
}

mesh::LocalIdentity radio_new_identity() {
  RadioNoiseListener rng(radio);
  return mesh::LocalIdentity(&rng);
}