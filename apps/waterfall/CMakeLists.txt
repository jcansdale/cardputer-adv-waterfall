# SPDX-License-Identifier: GPL-3.0-or-later

cmake_minimum_required(VERSION 3.20.0)

# The Cardputer-Adv is a StampS3 with this repo's shield: plain "-b m5stack_stamps3" suffices.
if(NOT DEFINED SHIELD AND NOT DEFINED CACHE{SHIELD} AND "${BOARD}" MATCHES "^m5stack_stamps3")
  set(SHIELD m5stack_cardputer_adv)
endif()

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(esp_sdr_waterfall)

target_sources(app PRIVATE src/main.c)
