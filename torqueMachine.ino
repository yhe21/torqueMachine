#include <SPI.h>
#include"cybergear_driver_defs.h"
#define CAN_2515
#define DELAY_TIME 3
#define RUN_TIME 1200
#define STARTP -1.2
//-1.7 for 34 close;-1.2 for 20 open
#define ENDP -3.0

#define CURRENT 6.0
//7.4 for 34 close;6.0 for 20 open
#define RCURRENT 1.0
#define LIMIT_SPEED 7.0
#define BTN_INPUT 2
#define SOL_OUTPUT 3
#define SAFE_INPUT 4
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
unsigned long switchTimer=0;
byte len=8;
byte data[8] = {0};
byte cdata[8] = {0};
int iValue=0;
int sensorSwitch=LOW;
int sensorSafe;
bool inLock=false;
bool outLock=false;
float fValue=0.0;
void setup()
{
  //SERIAL_PORT_MONITOR.begin(115200);
  //  while(!Serial){};
    while (CAN_OK != CAN.begin(CAN_1000KBPS)) {             // init can bus : baudrate = 500k
        SERIAL_PORT_MONITOR.println("CAN init fail, retry...");
        delay(100);
    }
    //SERIAL_PORT_MONITOR.println("CAN init ok!");
    iValue=read_int_data(CMD_RESPONSE);
  pinMode(SOL_OUTPUT, OUTPUT);
  pinMode(BTN_INPUT, INPUT_PULLUP);
  pinMode(SAFE_INPUT, INPUT_PULLUP);
}
void loop()
{
  sensorSwitch = digitalRead(BTN_INPUT);
  sensorSafe = digitalRead(SAFE_INPUT);
  if(sensorSafe==LOW){switchTimer=millis();}
  if (sensorSafe==HIGH&&sensorSwitch==LOW){
    if((millis()-switchTimer>250)&&inLock==false){
      runCyc();
      inLock=true;
    }
  }
  else{
    switchTimer=millis();
    inLock=false;
  }
}
void runCyc(){
delay(10);
//clear read data
iValue=read_int_data(CMD_RESPONSE);
//init setting:set limit current,set run mode to MODE_POSITION,Enable motor
set_limit_current(CURRENT);
read_ram_data(ADDR_LIMIT_CURRENT);
fValue=read_float_data(CMD_RAM_READ);
set_run_mode(MODE_POSITION);
enable_motor();
//set limit_speed
set_limit_speed(LIMIT_SPEED);
//go to start position
set_position_ref(STARTP);
read_ram_data(ADDR_MECH_POS);
fValue=read_float_data(CMD_RAM_READ);
unsigned long startTime = millis();
//check if at start pos
while ((abs(fValue-STARTP)>0.05)&&(millis() - startTime < 2000)){
  //Serial.println("Check start pos");
  //Serial.println(abs(fValue-STARTP));
  read_ram_data(ADDR_MECH_POS);
  fValue=read_float_data(CMD_RAM_READ);
  delay(10);
}

close_clamp();
//go to end position
set_position_ref(ENDP);
//read_ram_data(ADDR_MECH_POS);
//fValue=read_float_data(CMD_RAM_READ);
delay(RUN_TIME);
read_ram_data(ADDR_MECH_POS);
fValue=read_float_data(CMD_RAM_READ);
//release stress
set_limit_current(1);
delay(100);
reset_motor();
delay(100);
open_clamp();
//test delay
delay(100);

if(abs(fValue-ENDP)<0.1){
  set_limit_current(4.0);
read_ram_data(ADDR_LIMIT_CURRENT);
fValue=read_float_data(CMD_RAM_READ);
set_run_mode(MODE_POSITION);
enable_motor();
//set limit_speed
set_limit_speed(5.0);
//go to start position
set_position_ref(STARTP);
read_ram_data(ADDR_MECH_POS);
fValue=read_float_data(CMD_RAM_READ);
delay(600);
reset_motor();
}
}
void open_clamp(){
  digitalWrite(SOL_OUTPUT, LOW);
  delay(150);
}
void close_clamp(){
  digitalWrite(SOL_OUTPUT,HIGH);
  delay(150);
}
void send_command(
  uint32_t can_id, uint32_t cmd_id, uint32_t option, uint8_t len, uint8_t * data)
{
  uint32_t id = cmd_id << 24 | option << 8 | can_id;
  //uint32_t id2 =id; 
  //Serial.print("0x");  // 打印前缀
  //for (int i = 28; i >= 0; i -= 4) {
  //  uint8_t nibble = (id2 >> i) & 0xF;  // 提取4位（一个十六进制位）
  //  Serial.print(nibble, HEX);  // 打印当前十六进制位
  //}
  //Serial.println("");
  send_message(id, data, len, true);


}
bool send_message(
  uint32_t id, const uint8_t * data, uint8_t len, bool ext)
{
  CAN.sendMsgBuf(id,ext,false,len,data);
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
float uint_to_float(uint16_t x, float x_min, float x_max)
{
  uint16_t type_max = 0xFFFF;
  float span = x_max - x_min;
  return (float)x / type_max * span + x_min;
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
  iValue=read_int_data(CMD_RESPONSE);
}
void reset_motor()
{
  uint8_t data[8] = {0x00};
  send_command(target_can_id_, CMD_RESET, master_can_id_, 8, data);
  iValue=read_int_data(CMD_RESPONSE);
}

void set_run_mode(uint8_t run_mode)
{

  uint8_t data[8] = {0x00};
  data[0] = ADDR_RUN_MODE & 0x00FF;
  data[1] = ADDR_RUN_MODE >> 8;
  data[4] = run_mode;
  send_command(target_can_id_, CMD_RAM_WRITE, master_can_id_, 8, data);
  iValue=read_int_data(CMD_RESPONSE);
}
void set_limit_speed(float speed)
{
  write_float_data(target_can_id_, ADDR_LIMIT_SPEED, speed, 0.0f, V_MAX);
  iValue=read_int_data(CMD_RESPONSE);
}

void set_limit_current(float current)
{
  write_float_data(target_can_id_, ADDR_LIMIT_CURRENT, current, 0.0f, IQ_MAX);
  iValue=read_int_data(CMD_RESPONSE);
}

void set_current_kp(float kp)
{
  write_float_data(target_can_id_, ADDR_CURRENT_KP, kp, 0.0f, KP_MAX);
  iValue=read_int_data(CMD_RESPONSE);
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
    iValue=read_int_data(CMD_RESPONSE);
}

void set_limit_torque(float torque)
{
  write_float_data(target_can_id_, ADDR_LIMIT_TORQUE, torque, 0.0f, T_MAX);
  iValue=read_int_data(CMD_RESPONSE);
}
void set_position_ref(float position)
{
  write_float_data(target_can_id_, ADDR_LOC_REF, position, P_MIN, P_MAX);
  iValue=read_int_data(CMD_RESPONSE);
}
void set_position_kp(float kp)
{
  write_float_data(target_can_id_, ADDR_LOC_KP, kp, 0.0f, 200.0f);
  iValue=read_int_data(CMD_RESPONSE);
}

void set_velocity_kp(float kp)
{
  write_float_data(target_can_id_, ADDR_SPD_KP, kp, 0.0f, 200.0f);
  iValue=read_int_data(CMD_RESPONSE);
}
void read_ram_data(uint16_t index)
{
  uint8_t data[8] = {0x00};
  memcpy(&data[0], &index, 2);
  send_command(target_can_id_, CMD_RAM_READ, master_can_id_, 8, data);
  
}
float read_float_data(uint16_t index)
{
  delay(DELAY_TIME);
  if (CAN_MSGAVAIL != CAN.checkReceive()) {
    return -1;
    }
  else{
    CAN.readMsgBuf(&len, cdata);
  }
  float value = 0.0;
  memcpy(&value, &cdata[4], sizeof(float));
id = CAN.getCanId();
if((int)(id>>24)==(int)index)
{
  Serial.println("index matched");
  
}
else
{
  Serial.println("index not matched");
  Serial.println(index,HEX);Serial.println(id>>24,HEX);
  Serial.print("0x    ");  // 打印前缀
  for (int i = 28; i >= 0; i -= 4) {
    uint8_t nibble = (id >> i) & 0xF;  // 提取4位（一个十六进制位）
    Serial.print(nibble, HEX);  // 打印当前十六进制位
  }
}
  Serial.print("DATA:    ");
  Serial.print(value);
  Serial.println("");
  return value;
}
int read_int_data(uint16_t index)
{
  delay(DELAY_TIME);
  if (CAN_MSGAVAIL != CAN.checkReceive()) {
    return -1;
    }
  else{
    CAN.readMsgBuf(&len, cdata);
  }
  int value = 0;
  memcpy(&value, &cdata[4], sizeof(int));
id = CAN.getCanId();
if((int)(id>>24)==(int)index)
{
  Serial.println("index matched");
}
else
{
  Serial.println("index not matched");
  Serial.println(index,HEX);Serial.println(id>>24,HEX);
  Serial.print("0x    ");  // 打印前缀
  for (int i = 28; i >= 0; i -= 4) {
    uint8_t nibble = (id >> i) & 0xF;  // 提取4位（一个十六进制位）
    Serial.print(nibble, HEX);  // 打印当前十六进制位
  }
}
  Serial.print("DATA:    ");
  Serial.print(value);
  Serial.println("");
  return value;
}
void set_velocity_ki(float ki)
{
  write_float_data(target_can_id_, ADDR_SPD_KI, ki, 0.0f, 200.0f);
  iValue=read_int_data(CMD_RESPONSE);
}