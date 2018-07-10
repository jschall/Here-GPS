#pragma once

#include <stdint.h>
#include <modules/platform_stm32f302x8/platform_stm32f302x8.h>

#define BOARD_CONFIG_HW_NAME "org.hex.here+"
#define BOARD_CONFIG_HW_MAJOR_VER 2
#define BOARD_CONFIG_HW_MINOR_VER 0

#define BOARD_CONFIG_HW_INFO_STRUCTURE { \
    .hw_name = BOARD_CONFIG_HW_NAME, \
    .hw_major_version = BOARD_CONFIG_HW_MAJOR_VER, \
    .hw_minor_version = BOARD_CONFIG_HW_MINOR_VER, \
    .board_desc_fmt = SHARED_HW_INFO_BOARD_DESC_FMT_NONE, \
    .board_desc = 0, \
}
