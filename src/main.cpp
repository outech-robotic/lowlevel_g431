#include "peripherals/fdcan.h"
#include "peripherals/tim.h"
#include "utility/timing.h"
#include "com/serial.hpp"
#include "motion/MotionController.h"
#include "utility/Metro.hpp"
#include "config.h"

#include "com/isotp/isotp.h"
#include "com/proto/pb_encode.h"
#include "com/proto/pb_decode.h"
#include "com/proto/proto.pb.h"

#define STR(x)  #x
#define XSTR(x) STR(x)

/**********************************************************************
 *                             GLOBAL OBJECTS
 **********************************************************************/
Metro can_timer(10);
Metro heartbeat_timer(10);

Serial serial;
MotionController mcs;

static constexpr size_t C_RX_ID1 = 0x7E1;
static constexpr size_t C_RX_ID2 = 0x7F1;
static constexpr size_t C_TX_ID1 = 0x7E2;
static constexpr size_t C_TX_ID2 = 0x7F2;
static constexpr size_t C_ISOTP_BUFF_SIZE = 4096;
static IsoTpLink g_link1;
static IsoTpLink g_link2;
static uint8_t g_isotpRecvBuf1[C_ISOTP_BUFF_SIZE];
static uint8_t g_isotpRecvBuf2[C_ISOTP_BUFF_SIZE];
static uint8_t g_isotpSendBuf1[C_ISOTP_BUFF_SIZE];
static uint8_t g_isotpSendBuf2[C_ISOTP_BUFF_SIZE];


/**********************************************************************
 *                             ISO-TP FUNCTIONS
 **********************************************************************/

/* required, this must send a single CAN message with the given arbitration
   * ID (i.e. the CAN message ID) and data. The size will never be more than 8
   * bytes. */
  int  isotp_user_send_can(const uint32_t arbitration_id,
                           const uint8_t* data, const uint8_t size) {
      return CAN_send_packet(arbitration_id, data, size, false);
  }

  /* required, return system tick, unit is millisecond */
  uint32_t isotp_user_get_ms(void) {
      return millis();
  }

      /* optional, provide to receive debugging log messages */
  void isotp_user_debug(const char* message, ...) {

  }

  bool isotp_is_tx_available(IsoTpLink *link){
    return link->send_status == ISOTP_SEND_STATUS_IDLE;
  }


int main(void)
{
  int ret;
  bool status;

  can_rx_msg can_raw_msg;
  const size_t payload_rx_max = 128;
  uint8_t payload_rx1[payload_rx_max];
  uint8_t payload_rx2[payload_rx_max];
  uint16_t payload_rx1_size;
  uint16_t payload_rx2_size;
  uint8_t payload_tx1[64];
  uint8_t payload_tx2[64];

  isotp_init_link(&g_link1, C_TX_ID1, g_isotpSendBuf1, sizeof(g_isotpSendBuf1), g_isotpRecvBuf1, sizeof(g_isotpRecvBuf1));
  isotp_init_link(&g_link2, C_TX_ID2, g_isotpSendBuf2, sizeof(g_isotpSendBuf2), g_isotpRecvBuf2, sizeof(g_isotpRecvBuf2));

  pb_ostream_t ostream1 = pb_ostream_from_buffer(payload_tx1, sizeof(payload_tx1));
  pb_ostream_t ostream2 = pb_ostream_from_buffer(payload_tx2, sizeof(payload_tx2));
  pb_istream_t istream1;
  pb_istream_t istream2;

  BusMessage msg_rx1 = BusMessage_init_zero;
  BusMessage msg_rx2 = BusMessage_init_zero;
  BusMessage msg_tx1 = BusMessage_init_zero;
  BusMessage msg_tx2 = BusMessage_init_zero;

  msg_tx1.message.encoderPosition = EncoderPositionMsg_init_zero;
  msg_tx2.message.encoderPosition = EncoderPositionMsg_init_zero;

  msg_tx1.message.encoderPosition.left_tick  = 1;
  msg_tx2.message.encoderPosition.left_tick  = 1;
  msg_tx1.message.encoderPosition.right_tick = -1;
  msg_tx2.message.encoderPosition.right_tick = -1;
  msg_tx1.which_message = BusMessage_encoderPosition_tag;
  msg_tx2.which_message = BusMessage_encoderPosition_tag;

  MX_FDCAN1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
 
  pinMode(PIN_LED, PinDirection::OUTPUT);
  setPin(PIN_LED);
  PWM_write(PA10,  0);
  PWM_write(PA8,  0);

  serial.init(115200);
  serial.print("Setup done \r\n");

  while (1)
  {
    if(CAN_receive_packet(&can_raw_msg) == HAL_OK){
      if(can_raw_msg.header.Identifier == C_RX_ID1){
        isotp_on_can_message(&g_link1,  can_raw_msg.data.u8, can_raw_msg.header.DataLength);
      }
      else if(can_raw_msg.header.Identifier == C_RX_ID2){
        isotp_on_can_message(&g_link2,  can_raw_msg.data.u8, can_raw_msg.header.DataLength);
      }
    }

    isotp_poll(&g_link1);
    isotp_poll(&g_link2);

    ret = isotp_receive(&g_link1, payload_rx1, payload_rx_max, &payload_rx1_size);
    if(ret == ISOTP_RET_OK){
      istream1 = pb_istream_from_buffer(payload_rx1, payload_rx1_size);
      status = pb_decode(&istream1, BusMessage_fields, &msg_rx1);
      if(!status){
        serial.print("ERROR: pb_decode failed\r\n");
      }
      serial.print("ISOTP1 : ");
      switch(msg_rx1.which_message){
        case BusMessage_stopMoving_tag:
          serial.println("Packet Received: StopMoving");

          break;

        case BusMessage_translate_tag:
          serial.println("Packet Received: Translate");

          break;

        case BusMessage_rotate_tag:
          serial.println("Packet Received: Rotate");

          break;

        case BusMessage_moveWheelAtSpeed_tag:
          serial.println("Packet Received: MoveWheelAtSpeed");

          break;

        case BusMessage_setMotionControlMode_tag:
          serial.println("Packet Received: SetMotionControlMode");

          break;

        case BusMessage_setPID_tag:
          serial.println("Packet Received: SetPID");

          break;

        default:
          break;
      }
    }

    ret = isotp_receive(&g_link2, payload_rx2, payload_rx_max, &payload_rx2_size);
    if(ret == ISOTP_RET_OK){
      istream2 = pb_istream_from_buffer(payload_rx2, payload_rx2_size);
      status = pb_decode(&istream2, BusMessage_fields, &msg_rx2);
      if(!status){
        serial.print("ERROR: pb_decode failed\r\n");
      }
      serial.print("ISOTP2 : ");
      switch(msg_rx2.which_message){
        case BusMessage_stopMoving_tag:
          serial.println("Packet Received: StopMoving");

          break;

        case BusMessage_translate_tag:
          serial.println("Packet Received: Translate");

          break;

        case BusMessage_rotate_tag:
          serial.println("Packet Received: Rotate");

          break;

        case BusMessage_moveWheelAtSpeed_tag:
          serial.println("Packet Received: MoveWheelAtSpeed");

          break;

        case BusMessage_setMotionControlMode_tag:
          serial.println("Packet Received: SetMotionControlMode");

          break;

        case BusMessage_setPID_tag:
          serial.println("Packet Received: SetPID");

          break;

        default:
          break;
      }
    }

    if(can_timer.check()){
      if(isotp_is_tx_available(&g_link1)){
        togglePin(PIN_LED);
        msg_tx1.message.encoderPosition.left_tick++;
        msg_tx1.message.encoderPosition.right_tick--;
        ostream1 = pb_ostream_from_buffer(payload_tx1, sizeof(msg_tx1));
        status = pb_encode(&ostream1, BusMessage_fields, &msg_tx1);
        if(!status){
          serial.print("ERROR: pb_encode failed\r\n");
        }
        ret = isotp_send(&g_link1, payload_tx1, ostream1.bytes_written);
        if(ret != CAN_PKT_OK){
          asm volatile("nop");
        }
      }

      if(isotp_is_tx_available(&g_link2)){
        togglePin(PIN_LED);
        msg_tx2.message.encoderPosition.left_tick++;
        msg_tx2.message.encoderPosition.right_tick--;
        ostream2 = pb_ostream_from_buffer(payload_tx2, sizeof(msg_tx2));
        status = pb_encode(&ostream2, BusMessage_fields, &msg_tx2);
        if(!status){
          serial.print("ERROR: pb_encode failed\r\n");
        }
        ret = isotp_send(&g_link2, payload_tx2, ostream2.bytes_written);
        if(ret != CAN_PKT_OK){
          asm volatile("nop");
        }
      }
    }
  }
}
