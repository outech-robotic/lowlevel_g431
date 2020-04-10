#ifndef __fdcan_H
#define __fdcan_H

#include "stm32g4xx_hal.h"
#include "stm32g4xx_hal_fdcan.h"

extern FDCAN_HandleTypeDef hcan;

typedef union{
  uint8_t u8[8];
  uint16_t u16[4];
  int32_t d32[2];
}byte_union;

// Struct used to store a received packet's information
typedef struct{
  FDCAN_RxHeaderTypeDef header;
  byte_union data;
} can_rx_msg;

// Struct used to store a pending transmit packet's information
typedef struct{
  FDCAN_TxHeaderTypeDef header;
  byte_union data;
} can_tx_msg;

enum CAN_ERROR_STATUS{
  PACKET_TX_ERROR = -5,
  NON_STD_NOT_SUPPORTED = -4,
  DATA_NPE = -3,
  PACKET_ID_TOO_LARGE = -2,
  PACKET_DLC_TOO_LARGE = -1,
  CAN_PKT_OK=0
};

int CAN_send_packet(uint16_t std_id, const uint8_t* data=nullptr, uint8_t size=0, bool remote = false);
int CAN_send_packet(can_tx_msg* msg);
int CAN_receive_packet(can_rx_msg* msg);

void MX_FDCAN1_Init(void);


#endif /*__ fdcan_H */
