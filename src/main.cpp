#include <can_pb.hpp>
#include "peripherals/tim.h"
#include "utility/timing.h"
#include "com/serial.hpp"
#include "motion/MotionController.h"
#include "utility/Metro.hpp"
#include "config.h"


#define STR(x)  #x
#define XSTR(x) STR(x)

/**********************************************************************
 *                             GLOBAL OBJECTS
 **********************************************************************/
Metro can_timer(10);
Metro heartbeat_timer(10);

Serial serial;
MotionController mcs;

int main(void)
{
  int ret;
  MotionController::STOP_STATUS mcs_stop_status;
  Can_PB canpb(CONST_CAN_RX_ID, CONST_CAN_TX_ID);
  Can_PB canpb2(0x10, 0x11);

  can_rx_msg can_raw_msg;

  BusMessage msg_rx = BusMessage_init_zero;
  BusMessage msg_tx = BusMessage_init_zero;

  msg_tx.message.encoderPosition = EncoderPositionMsg_init_zero;

  msg_tx.message.encoderPosition.left_tick  = 1;
  msg_tx.message.encoderPosition.right_tick = -1;
  msg_tx.which_message = BusMessage_encoderPosition_tag;

  MX_FDCAN1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();

  mcs.init();

  pinMode(PB3, PinDirection::OUTPUT);
  pinMode(PIN_LED, PinDirection::OUTPUT);
  setPin(PB3);
  setPin(PIN_LED);
  PWM_write(PA10,  0);
  PWM_write(PA8,  0);

  serial.init(115200);
  serial.print("Setup done \r\n");

  while (1)
  {
    if(CAN_receive_packet(&can_raw_msg) == HAL_OK){
      if(canpb.match_id(can_raw_msg.header.Identifier)){
        canpb.update_rx_msg(can_raw_msg);
      }
      if(canpb2.match_id(can_raw_msg.header.Identifier)){
        canpb2.update_rx_msg(can_raw_msg);
      }
    }

    canpb.update();
    canpb2.update();

    if(canpb.is_rx_available()){
      ret = canpb.receive_msg(msg_rx);
      if(ret == Can_PB::CAN_PB_RET_OK){
        switch(msg_rx.which_message){
          case BusMessage_stopMoving_tag:
            serial.println("Packet Received: StopMoving");
            mcs.stop(false);
            break;

          case BusMessage_translate_tag:
            serial.println("Packet Received: Translate");
            mcs.translate_ticks(msg_rx.message.translate.ticks);
            break;

          case BusMessage_rotate_tag:
            serial.println("Packet Received: Rotate");
            mcs.rotate_ticks(msg_rx.message.rotate.ticks);
            break;

          case BusMessage_moveWheelAtSpeed_tag:
            serial.println("Packet Received: MoveWheelAtSpeed");
            mcs.set_target_speed(Motor::Side::LEFT, msg_rx.message.moveWheelAtSpeed.left_tick_per_sec);
            mcs.set_target_speed(Motor::Side::RIGHT, msg_rx.message.moveWheelAtSpeed.right_tick_per_sec);
            break;

          case BusMessage_setMotionControlMode_tag:
            serial.println("Packet Received: SetMotionControlMode");
            mcs.set_control(msg_rx.message.setMotionControlMode.speed, msg_rx.message.setMotionControlMode.translation, msg_rx.message.setMotionControlMode.rotation);
            break;

          case BusMessage_setPID_tag:
            serial.println("Packet Received: SetPID");
            mcs.set_kp(msg_rx.message.setPID.pid_id ,msg_rx.message.setPID.kp);
            mcs.set_ki(msg_rx.message.setPID.pid_id ,msg_rx.message.setPID.ki);
            mcs.set_kd(msg_rx.message.setPID.pid_id ,msg_rx.message.setPID.kd);
            break;

          default:
            break;
        }
      }
    }


    mcs_stop_status = mcs.has_stopped();
    //If the robot has stopped or cannot move normally
    if(mcs_stop_status != MotionController::STOP_STATUS::NONE){
      msg_tx.which_message = BusMessage_movementEnded_tag;
      msg_tx.message.movementEnded = MovementEndedMsg_init_zero;
      msg_tx.message.movementEnded.blocked = (mcs_stop_status == MotionController::STOP_STATUS::BLOCKED);
      if(canpb.send_msg(msg_tx) != Can_PB::CAN_PB_RET_OK){
        serial.print("ERROR: SENDING MOVEMENT END\r\n");
      }
    }


    if(can_timer.check()){

      if(canpb.is_tx_available()){
        msg_tx.message.encoderPosition.left_tick++;
        msg_tx.message.encoderPosition.right_tick--;
        ret = canpb.send_msg(msg_tx);
        if(ret == Can_PB::CAN_PB_RET_OK){
          togglePin(PB3);
          togglePin(PIN_LED);
        }
      }
      if(canpb2.is_tx_available()){
        msg_tx.message.encoderPosition.left_tick++;
        msg_tx.message.encoderPosition.right_tick--;
        ret = canpb2.send_msg(msg_tx);
        if(ret == Can_PB::CAN_PB_RET_OK){
        }
      }
    }
  }
}


#ifdef __cplusplus
extern "C"{
#endif
void TIM1_UP_TIM16_IRQHandler(void){
  static uint8_t i = 0;
  // Control loop
  if(LL_TIM_IsActiveFlag_UPDATE(TIM16)){
    LL_TIM_ClearFlag_UPDATE(TIM16);
    mcs.update_position();
    mcs.control_motion();
    if((++i) == 10){ // Evry 10ms
      mcs.detect_stop();
      i = 0;
    }
  }
}
#ifdef __cplusplus
}
#endif
