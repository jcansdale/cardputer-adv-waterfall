# SPDX-License-Identifier: GPL-3.0-or-later

cmake_minimum_required(VERSION 3.20.0)

# The Cardputer-Adv is a StampS3 with this repo's shield: plain "-b m5stack_stamps3" suffices.
if(NOT DEFINED SHIELD AND NOT DEFINED CACHE{SHIELD} AND "${BOARD}" MATCHES "^m5stack_stamps3")
  set(SHIELD m5stack_cardputer_adv)
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(esp_sdr_waterfall)

target_sources(app PRIVATE src/main.c)
# The FFTs and per-sample loops: Zephyr builds for size, and its ISO C mode turns contraction
# off (fused multiply-adds halve the FPU work).
set_source_files_properties(src/main.c PROPERTIES COMPILE_OPTIONS "-O3;-ffp-contract=fast")

# esp-dsp's ESP32-S3 float FFT kernel alone (west project esp-dsp), as apps/link takes it.
set(ESP_DSP_FFT "${WEST_TOPDIR}/modules/lib/esp-dsp/modules/fft/float/dsps_fft2r_fc32_aes3_.S")
if(NOT EXISTS "${ESP_DSP_FFT}")
  message(FATAL_ERROR "esp-dsp not found: ${ESP_DSP_FFT} (west project esp-dsp)")
endif()
target_sources(app PRIVATE "${ESP_DSP_FFT}")
set_source_files_properties(${ESP_DSP_FFT} PROPERTIES
  COMPILE_DEFINITIONS CONFIG_IDF_TARGET_ESP32S3=1
  INCLUDE_DIRECTORIES "${WEST_TOPDIR}/modules/lib/esp-dsp/modules/fft/include")
