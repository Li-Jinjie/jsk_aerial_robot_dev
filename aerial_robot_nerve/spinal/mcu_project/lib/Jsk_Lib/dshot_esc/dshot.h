//
// Created by li-jinjie on 24-1-21.
// ref: https://github.com/mokhwasomssi/stm32_hal_dshot/tree/main
//

#ifndef __cplusplus
#error "Please define __cplusplus, because this is a c++ based file "
#endif

#ifndef DSHOT_H
#define DSHOT_H

#include <stm32h7xx_hal.h>
#include <stdbool.h>
#include <cstdint>
#include <math.h>  // lrintf
#include "esc_telem.h"

/* User Configuration */
// Timer Clock
#define TIMER_CLOCK 200000000  // TIM1 clock on stm32H7 spinal (Tim1OutputFreq_Value in spinal.ioc), verified by oscilloscope

/* Definition */
#define MHZ_TO_HZ(x) ((x) * 1000000)

#define DSHOT600_HZ MHZ_TO_HZ(12)
#define DSHOT300_HZ MHZ_TO_HZ(6)
#define DSHOT150_HZ MHZ_TO_HZ(3)

#define MOTOR_BIT_0 7
#define MOTOR_BIT_1 14
#define MOTOR_BITLENGTH 20

#define DSHOT_FRAME_SIZE 16
#define DSHOT_DMA_BUFFER_SIZE 18 /* resolution + frame reset (2us) */

#define DSHOT_DISARM_THROTTLE 0
#define DSHOT_MIN_THROTTLE 48
#define DSHOT_MAX_THROTTLE 2047
#define DSHOT_RANGE (DSHOT_MAX_THROTTLE - DSHOT_MIN_THROTTLE)

#define DSHOT_CMD_SPIN_DIRECTION_1 7
#define DSHOT_CMD_SPIN_DIRECTION_2 8

// Motor setting
// TODO: should be set by onboard PC
#define NUM_MOTOR_MAG_POLE 14
#define IS_TELEMETRY true

// ESC Telemetry setting
// One telemetry slot every ESC_TELEM_REQUEST_DIVIDER control cycles (1kHz). Each slot first parses the reply requested
// in the previous slot, then requests the ESC of the current slot. Slot 0 sends no request (idle), slots 1-4 request
// ESC 1-4, so the idle slot lowers the per-ESC rate from 250Hz to 200Hz.
// Per-ESC freq = 1kHz / ESC_TELEM_REQUEST_DIVIDER / ESC_TELEM_SLOT_COUNT, i.e., 200Hz with the default values.
// Keep each reply (10 bytes) shorter than one slot: ~0.22ms at 460800 baud.
#define ESC_TELEM_REQUEST_DIVIDER 1
#define ESC_TELEM_SLOT_COUNT 5  // slot 0: idle, slots 1-4: ESC 1-4, slots >= 5: idle. Must be >= 5

namespace
{
#ifdef STM32H7
uint32_t motor1_dmabuffer_[DSHOT_DMA_BUFFER_SIZE] __attribute__((section(".DShotBufferSection1")));
uint32_t motor2_dmabuffer_[DSHOT_DMA_BUFFER_SIZE] __attribute__((section(".DShotBufferSection2")));
uint32_t motor3_dmabuffer_[DSHOT_DMA_BUFFER_SIZE] __attribute__((section(".DShotBufferSection3")));
uint32_t motor4_dmabuffer_[DSHOT_DMA_BUFFER_SIZE] __attribute__((section(".DShotBufferSection4")));
#else
uint32_t motor1_dmabuffer_[DSHOT_DMA_BUFFER_SIZE];
uint32_t motor2_dmabuffer_[DSHOT_DMA_BUFFER_SIZE];
uint32_t motor3_dmabuffer_[DSHOT_DMA_BUFFER_SIZE];
uint32_t motor4_dmabuffer_[DSHOT_DMA_BUFFER_SIZE];
#endif
}  // namespace

/* Enumeration */
typedef enum
{
  DSHOT150,
  DSHOT300,
  DSHOT600
} dshot_type_e;

class DShot
{
public:
  DShot() {};
  ~DShot() {};

  void init(dshot_type_e dshot_type, TIM_HandleTypeDef* htim_motor_1, uint32_t channel_motor_1,
            TIM_HandleTypeDef* htim_motor_2, uint32_t channel_motor_2, TIM_HandleTypeDef* htim_motor_3,
            uint32_t channel_motor_3, TIM_HandleTypeDef* htim_motor_4, uint32_t channel_motor_4);
  void initTelemetry(UART_HandleTypeDef* huart);

  void write(uint16_t* motor_value_array, bool is_telemetry);

  /* dshot telemtry */
  bool is_telemetry_ = false;
  int id_telem_ = 0;
  int id_telem_prev_ = -1;

  ESCReader esc_reader_;

private:
  TIM_HandleTypeDef* htim_motor_1_;
  uint32_t channel_motor_1_;
  TIM_HandleTypeDef* htim_motor_2_;
  uint32_t channel_motor_2_;
  TIM_HandleTypeDef* htim_motor_3_;
  uint32_t channel_motor_3_;
  TIM_HandleTypeDef* htim_motor_4_;
  uint32_t channel_motor_4_;

  /* Static functions */
  // dshot init
  uint32_t dshot_choose_type(dshot_type_e dshot_type);
  void dshot_set_timer(dshot_type_e dshot_type);
  static void dshot_dma_tc_callback(DMA_HandleTypeDef* hdma);
  void dshot_put_tc_callback_function();
  void dshot_start_pwm();

  // dshot write
  uint16_t dshot_prepare_packet(uint16_t value, bool dshot_telemetry);
  void dshot_prepare_dmabuffer(uint32_t* motor_dmabuffer, uint16_t value, bool is_telemetry);
  void dshot_prepare_dmabuffer_all(uint16_t* motor_value, bool* is_telemetry);
  void dshot_dma_start();
  void dshot_enable_dma_request();

  int num_freq_divide = 0;
};

#endif  // DSHOT_H
