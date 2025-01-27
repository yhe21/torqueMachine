#include <SPI.h>
#include"cybergear_driver_defs.h"
#define CAN_2515
const int SPI_CS_PIN = 9;
const int CAN_INT_PIN = 2;
#ifdef CAN_2515
#include "mcp2515_can.h"
mcp2515_can CAN(SPI_CS_PIN); // Set CS pin
#endif
// setup master can id and motor can id (default cybergear can id is 0x7F)
uint8_t master_can_id_ = 0x00;
uint8_t target_can_id_ = 0x01;
uint32_t id;
uint8_t  type=0x01; // bit0: ext, bit1: rtr
unsigned len=8;
byte cdata[8] = {0};



void setup()
{
  SERIAL_PORT_MONITOR.begin(115200);
    while(!Serial){};

    while (CAN_OK != CAN.begin(CAN_1000KBPS)) {             // init can bus : baudrate = 500k
        SERIAL_PORT_MONITOR.println("CAN init fail, retry...");
        delay(100);
    }
    SERIAL_PORT_MONITOR.println("CAN init ok!");

  
}

void loop()
{
delay(100);
set_run_mode(MODE_POSITION);
enable_motor();
set_limit_speed(2.0);
set_position_ref(0.1);
delay(1000);
set_position_ref(2);
delay(1000);
reset_motor();
delay(5000);
}
void send_command(
  uint8_t can_id, uint8_t cmd_id, uint16_t option, uint8_t len, uint8_t * data)
{
  uint32_t id = cmd_id << 24 | option << 8 | can_id;
  send_message(id, data, len, true);
  delay(1);

}
bool send_message(
  uint32_t id, const uint8_t * data, uint8_t len, bool ext)
{
  CAN.sendMsgBuf(id,true,false,len,data);
}
int float_to_uint(float x, float x_min, float x_max, int bits)
{
  float span = x_max - x_min;
  float offset = x_min;
  if (x > x_max)
    x = x_max;
  else if (x < x_min)
    x = x_min;
  return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
void write_float_data(
  uint8_t can_id, uint16_t addr, float value, float min, float max)
{
  uint8_t data[8] = {0x00};
  data[0] = addr & 0x00FF;
  data[1] = addr >> 8;

  float val = (max < value) ? max : value;
  val = (min > value) ? min : value;
  memcpy(&data[4], &value, 4);
  send_command(can_id, CMD_RAM_WRITE, master_can_id_, 8, data);
}
void enable_motor()
{
  uint8_t data[8] = {0x00};
  send_command(target_can_id_, CMD_ENABLE, master_can_id_, 8, data);
}
void reset_motor()
{
  uint8_t data[8] = {0x00};
  send_command(target_can_id_, CMD_RESET, master_can_id_, 8, data);
}

void set_run_mode(uint8_t run_mode)
{

  uint8_t data[8] = {0x00};
  data[0] = ADDR_RUN_MODE & 0x00FF;
  data[1] = ADDR_RUN_MODE >> 8;
  data[4] = run_mode;
  send_command(target_can_id_, CMD_RAM_WRITE, master_can_id_, 8, data);
}
void set_limit_speed(float speed)
{
  write_float_data(target_can_id_, ADDR_LIMIT_SPEED, speed, 0.0f, V_MAX);
}

void set_limit_current(float current)
{
  write_float_data(target_can_id_, ADDR_LIMIT_CURRENT, current, 0.0f, IQ_MAX);
}

void set_current_kp(float kp)
{
  write_float_data(target_can_id_, ADDR_CURRENT_KP, kp, 0.0f, KP_MAX);
}

void set_current_ki(float ki)
{
  // Currently not implemented
}

void set_current_filter_gain(float gain)
{
  write_float_data(
    target_can_id_, ADDR_CURRENT_FILTER_GAIN, gain, CURRENT_FILTER_GAIN_MIN,
    CURRENT_FILTER_GAIN_MAX);
}

void set_limit_torque(float torque)
{
  write_float_data(target_can_id_, ADDR_LIMIT_TORQUE, torque, 0.0f, T_MAX);
}
void set_position_ref(float position)
{
  write_float_data(target_can_id_, ADDR_LOC_REF, position, P_MIN, P_MAX);
}
void set_position_kp(float kp)
{
  write_float_data(target_can_id_, ADDR_LOC_KP, kp, 0.0f, 200.0f);
}

void set_velocity_kp(float kp)
{
  write_float_data(target_can_id_, ADDR_SPD_KP, kp, 0.0f, 200.0f);
}

void set_velocity_ki(float ki)
{
  write_float_data(target_can_id_, ADDR_SPD_KI, ki, 0.0f, 200.0f);
}