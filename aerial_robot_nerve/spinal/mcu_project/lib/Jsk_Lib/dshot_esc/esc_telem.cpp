//
// Created by jinjie on 24/01/18.
//

#include "esc_telem.h"

void ESCReader::init(UART_HandleTypeDef* huart)
{
  huart_ = huart;

  // use DMA for UART RX
  __HAL_UART_DISABLE_IT(huart, UART_IT_PE);
  __HAL_UART_DISABLE_IT(huart, UART_IT_ERR);
  HAL_UART_Receive_DMA(huart, esc_telem_rx_buf_, ESC_BUFFER_SIZE);

  memset(esc_telem_rx_buf_, 0, ESC_BUFFER_SIZE);

  __HAL_UART_CLEAR_IDLEFLAG(huart_);  // discard any stale idle event from before DMA start
}

void ESCReader::update(spinal::ESCTelemetry& esc_msg)
{
  // Byte 0: Temperature
  // Byte 1: Voltage high byte
  // Byte 2: Voltage low byte
  // Byte 3: Current high byte
  // Byte 4: Current low byte
  // Byte 5: Consumption high byte
  // Byte 6: Consumption low byte
  // Byte 7: Rpm high byte
  // Byte 8: Rpm low byte
  // Byte 9: 8-bit CRC

  /* handle RX Overrun Error */
  if (__HAL_UART_GET_FLAG(huart_, UART_FLAG_ORE))
  {
    __HAL_UART_CLEAR_FLAG(huart_, UART_CLEAR_NEF | UART_CLEAR_OREF | UART_FLAG_RXNE | UART_FLAG_ORE);
    HAL_UART_Receive_DMA(huart_, esc_telem_rx_buf_, ESC_BUFFER_SIZE);  // restart
    esc_telem_rd_ptr_ = 0;  // DMA write pointer restarts from 0; re-sync read pointer to match
    __HAL_UART_CLEAR_IDLEFLAG(huart_);
    return;
  }

  // A KISS ESC telemetry reply is a single isolated UART burst: the line goes idle right after
  // it. Only trust the buffer once that idle gap has actually been observed, so we never parse
  // a frame that is still mid-transmission or spliced together from two separate replies.
  if (!__HAL_UART_GET_FLAG(huart_, UART_FLAG_IDLE)) return;
  __HAL_UART_CLEAR_IDLEFLAG(huart_);

  uint32_t dma_write_ptr = (ESC_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart_->hdmarx)) % (ESC_BUFFER_SIZE);
  uint32_t new_bytes = (dma_write_ptr + ESC_BUFFER_SIZE - esc_telem_rd_ptr_) % ESC_BUFFER_SIZE;

  if (new_bytes != 10)
  {
    // wrong length -> missed/extra bytes, not a valid single frame: drop and re-sync instead of
    // guessing at the contents
    esc_telem_rd_ptr_ = dma_write_ptr;
    esc_msg.crc_error = -1;  // indicate that the frame was dropped due to length mismatch
    return;
  }

  uint8_t buffer[10];  // buffer for KISS esc telemetry data
  for (int i = 0; i < 10; i++) buffer[i] = esc_telem_rx_buf_[(esc_telem_rd_ptr_ + i) % ESC_BUFFER_SIZE];
  esc_telem_rd_ptr_ = dma_write_ptr;

  /* check crc */
  uint8_t crc = get_crc8(buffer, 9);
  if (crc == buffer[9])  // crc matches -> no error
  {
    /* save data in esc_msg_1_ */
    esc_msg.temperature = buffer[0];
    esc_msg.voltage = buffer[1] << 8 | buffer[2];
    esc_msg.current = buffer[3] << 8 | buffer[4];
    esc_msg.consumption = buffer[5] << 8 | buffer[6];
    uint16_t erpm = buffer[7] << 8 | buffer[8];
    esc_msg.rpm = erpm * 100 / (num_motor_mag_pole_ / 2);
  }
  esc_msg.crc_error = crc - buffer[9];
}

uint8_t update_crc8(uint8_t crc, uint8_t crc_seed){
  uint8_t crc_u, i;
  crc_u = crc;
  crc_u ^= crc_seed;
  for ( i=0; i<8; i++) crc_u = ( crc_u & 0x80 ) ? 0x7 ^ ( crc_u << 1 ) : ( crc_u << 1 );
  return (crc_u);
}

uint8_t get_crc8(uint8_t *Buf, uint8_t BufLen){
  uint8_t crc = 0, i;
  for( i=0; i<BufLen; i++) crc = update_crc8(Buf[i], crc);
  return (crc);
}
