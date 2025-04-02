#include <SPI.h>
#include "cybergear_driver_defs.h"
#define CAN_2515
#define DELAY_TIME 3

//running setting:
#define RUN_TIME_1 5000
#define RUN_TIME_2 5000
//runtime in ms
#define STARTP -0.15
#define ENDP -3.0
#define CURRENT_1 4.0
#define CURRENT_2 6.3
//6.3 for 30 open;4.0 for 20 open
#define RCURRENT 1.0
//resting current, prevent spring back.
#define LIMIT_SPEED 7.0

//pin setup:
#define HIGH_INPUT 6
#define BTN_INPUT 5
#define SOL_OUTPUT 3
#define SAFE_INPUT 4
#define KP
const int SPI_CS_PIN = 9;
const int CAN_INT_PIN = 2;
#ifdef CAN_2515
#include "mcp2515_can.h"
mcp2515_can CAN(SPI_CS_PIN);  // Set CS pin
#endif
// setup master can id and motor can id (default cybergear can id is 0x7F)
uint8_t master_can_id_ = 0x00;
uint8_t target_can_id_ = 0x01;
uint32_t id;
uint8_t type = 0x01;  // bit0: ext, bit1: rtr
unsigned long switchTimer = 0;
unsigned long offTimer = 0;
byte len = 8;
byte data[8] = { 0 };
byte cdata[8] = { 0 };
int runtime=1000;
float current=1.0;
int iValue = 0;
int sensorSwitch = LOW;
int sensorSafe = HIGH;
int highSwitch=HIGH;
bool inLock = false;
float fValue = 0.0;
void setup() {
  Serial.begin(115200);
  //  while(!Serial){};
  while (CAN_OK != CAN.begin(CAN_1000KBPS)) {  // init can bus : baudrate = 500k
    Serial.println("CAN init fail, retry...");
    delay(100);
  }
  //SERIAL_PORT_MONITOR.println("CAN init ok!");
  iValue = read_int_data(CMD_RESPONSE);
  pinMode(SOL_OUTPUT, OUTPUT);
  pinMode(BTN_INPUT, INPUT_PULLUP);
  pinMode(SAFE_INPUT, INPUT_PULLUP);
  pinMode(HIGH_INPUT, INPUT_PULLUP);
  init_calib();
}
void loop() {
  sensorSwitch = digitalRead(BTN_INPUT);
  sensorSafe = digitalRead(SAFE_INPUT);
  highSwitch = digitalRead(HIGH_INPUT);
  if (sensorSafe == HIGH) { switchTimer = millis(); }
  if (sensorSafe == LOW && sensorSwitch == LOW) {
    if ((millis() - switchTimer > 250) && inLock == false) {
      if(highSwitch == HIGH){runtime=RUN_TIME_1; current=CURRENT_1; }
      else {runtime=RUN_TIME_2; current=CURRENT_2;}
      runCyc();
      inLock = true;
      offTimer = millis();
    }
  } else {
    switchTimer = millis();
  }
  if (sensorSwitch == LOW && inLock == true) { offTimer = millis(); }     //lock release timer reset
  if (inLock == true && (millis() - offTimer > 400)) { inLock = false; }  //lock release
  
  delay(5000);
  runtime=RUN_TIME_1;
  runCyc();
}
void runCyc() {
  delay(10);
  //clear read data
  iValue = read_int_data(CMD_RESPONSE);
  //init setting:set limit current,set run mode to MODE_POSITION,Enable motor
  set_limit_current(current);
  read_ram_data(ADDR_LIMIT_CURRENT);
  fValue = read_float_data(CMD_RAM_READ);
  set_run_mode(MODE_POSITION);
  enable_motor();
  //set limit_speed
  set_limit_speed(LIMIT_SPEED);
  //go to start position
  set_position_ref(STARTP);
  read_ram_data(ADDR_MECH_POS);
  fValue = read_float_data(CMD_RAM_READ);
  unsigned long startTime = millis();
  //check if at start pos
  while ((abs(fValue - STARTP) > 0.05) && (millis() - startTime < 2000)) {
    //Serial.println("Check start pos");
    //Serial.println(abs(fValue-STARTP));
    read_ram_data(ADDR_MECH_POS);
    fValue = read_float_data(CMD_RAM_READ);
    delay(10);
  }

  close_clamp();
  //go to end position
  set_position_ref(ENDP);
  //read_ram_data(ADDR_MECH_POS);
  //fValue=read_float_data(CMD_RAM_READ);
  delay(runtime);
  read_ram_data(ADDR_MECH_POS);
  fValue = read_float_data(CMD_RAM_READ);
  //release stress
  set_limit_current(RCURRENT);
  delay(100);
  reset_motor();
  delay(100);
  open_clamp();

  if (abs(fValue - ENDP) < 0.1) {
    set_limit_current(4.0);
    read_ram_data(ADDR_LIMIT_CURRENT);
    fValue = read_float_data(CMD_RAM_READ);
    set_run_mode(MODE_POSITION);
    enable_motor();
    //set limit_speed
    set_limit_speed(5.0);
    //go to start position
    set_position_ref(STARTP);
    read_ram_data(ADDR_MECH_POS);
    fValue = read_float_data(CMD_RAM_READ);
    delay(600);
    reset_motor();
  }
}
void init_calib(){
  unsigned long zero_timer=0;
  double init_pos=0.0;
  double init_speed=0.0;
  fValue = read_float_data(CMD_RAM_READ);
  read_ram_data(ADDR_MECH_POS);
  fValue = read_float_data(CMD_RAM_READ);
  init_pos=fValue+6.2;
  set_limit_current(1.0);
  read_ram_data(ADDR_LIMIT_CURRENT);
  fValue = read_float_data(CMD_RAM_READ);
  set_run_mode(MODE_POSITION);
  enable_motor();
  set_limit_speed(1.0);
  set_position_ref(init_pos);
  delay(100);
  read_ram_data(ADDR_MECH_VEL);
  init_speed = read_float_data(CMD_RAM_READ);
  zero_timer=millis();
  while((millis()-zero_timer)<500){
    read_ram_data(ADDR_MECH_VEL);
    init_speed = read_float_data(CMD_RAM_READ);
    if(abs(init_speed)>0.2){zero_timer=millis();
    Serial.print("Variable_1:");
    Serial.println(init_speed);
    }
  }
  reset_motor();
  set_mech_zero();
}
void open_clamp() {
  digitalWrite(SOL_OUTPUT, LOW);
  delay(150);
}
void close_clamp() {
  digitalWrite(SOL_OUTPUT, HIGH);
  delay(150);
}
void send_command(
  uint32_t can_id, uint32_t cmd_id, uint32_t option, uint8_t len, uint8_t* data) {
    //cmd_id+option+can_id  data
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
  uint32_t id, const uint8_t* data, uint8_t len, bool ext) {
  CAN.sendMsgBuf(id, ext, false, len, data);
}
uint16_t float_to_uint(float x, float x_min, float x_max, int bits) {
  float span = x_max - x_min;
  float offset = x_min;
  if (x > x_max)
    x = x_max;
  else if (x < x_min)
    x = x_min;
  return (uint16_t)((x - offset) * ((float)((1 << bits) - 1)) / span);
}
float uint_to_float(uint16_t x, float x_min, float x_max) {
  uint16_t type_max = 0xFFFF;
  float span = x_max - x_min;
  return (float)x / type_max * span + x_min;
}
void motion_command(
  float torque,float targetAngle,float targetSpeed,float Kp,float Kd){
    uint8_t data[8]={0x00};
    uint16_t iTorque,iAngle,iSpeed,iKp,iKd;
    iTorque=float_to_uint(torque,T_MIN , T_MAX,16);
    iAngle=float_to_uint(targetAngle,P_MIN,P_MAX,16);
    iSpeed=float_to_uint(targetSpeed,V_MIN,V_MAX,16);
    iKp=float_to_uint(Kp,KP_MIN,KP_MAX,16);
    iKd=float_to_uint(Kd,KD_MIN,KD_MAX,16);
    data[0] = (iAngle  >> 8) & 0xFF;  
    data[1] = (iAngle  >> 0) & 0xFF;  
    data[2] = (iSpeed  >> 8) & 0xFF;  
    data[3] = (iSpeed  >> 0) & 0xFF;  
    data[4] = (iKp     >> 8) & 0xFF;  
    data[5] = (iKp     >> 0) & 0xFF;  
    data[6] = (iKd     >> 8) & 0xFF;  
    data[7] = (iKd     >> 0) & 0xFF;  
    send_command(target_can_id_, CMD_MOTION, iTorque, 8, data);
  }
void write_float_data(
  uint8_t can_id, uint16_t addr, float value, float min, float max) {
  uint8_t data[8] = { 0x00 };
  data[0] = addr & 0x00FF;
  data[1] = addr >> 8;

  float val = (max < value) ? max : value;
  val = (min > value) ? min : value;
  memcpy(&data[4], &value, 4);
  send_command(can_id, CMD_RAM_WRITE, master_can_id_, 8, data);
}
void enable_motor() {
  uint8_t data[8] = { 0x00 };
  send_command(target_can_id_, CMD_ENABLE, master_can_id_, 8, data);
  iValue = read_int_data(CMD_RESPONSE);
}
void reset_motor() {
  uint8_t data[8] = { 0x00 };
  send_command(target_can_id_, CMD_RESET, master_can_id_, 8, data);
  iValue = read_int_data(CMD_RESPONSE);
}
void set_mech_zero() {
  uint8_t data[8] = { 0x00 };
  data[0]=0x01;
  send_command(target_can_id_, CMD_SET_MECH_POSITION_TO_ZERO, master_can_id_, 8, data);
  iValue = read_int_data(CMD_RESPONSE);
}
void set_run_mode(uint8_t run_mode) {

  uint8_t data[8] = { 0x00 };
  data[0] = ADDR_RUN_MODE & 0x00FF;
  data[1] = ADDR_RUN_MODE >> 8;
  data[4] = run_mode;
  send_command(target_can_id_, CMD_RAM_WRITE, master_can_id_, 8, data);
  iValue = read_int_data(CMD_RESPONSE);
}
void set_limit_speed(float speed) {
  write_float_data(target_can_id_, ADDR_LIMIT_SPEED, speed, 0.0f, V_MAX);
  iValue = read_int_data(CMD_RESPONSE);
}

void set_limit_current(float current) {
  write_float_data(target_can_id_, ADDR_LIMIT_CURRENT, current, 0.0f, IQ_MAX);
  iValue = read_int_data(CMD_RESPONSE);
}

void set_current_kp(float kp) {
  write_float_data(target_can_id_, ADDR_CURRENT_KP, kp, 0.0f, KP_MAX);
  iValue = read_int_data(CMD_RESPONSE);
}

void set_current_ki(float ki) {
  // Currently not implemented
}

void set_current_filter_gain(float gain) {
  write_float_data(
    target_can_id_, ADDR_CURRENT_FILTER_GAIN, gain, CURRENT_FILTER_GAIN_MIN,
    CURRENT_FILTER_GAIN_MAX);
  iValue = read_int_data(CMD_RESPONSE);
}

void set_limit_torque(float torque) {
  write_float_data(target_can_id_, ADDR_LIMIT_TORQUE, torque, 0.0f, T_MAX);
  iValue = read_int_data(CMD_RESPONSE);
}
void set_position_ref(float position) {
  write_float_data(target_can_id_, ADDR_LOC_REF, position, P_MIN, P_MAX);
  iValue = read_int_data(CMD_RESPONSE);
}
void set_position_kp(float kp) {
  write_float_data(target_can_id_, ADDR_LOC_KP, kp, 0.0f, 200.0f);
  iValue = read_int_data(CMD_RESPONSE);
}

void set_velocity_kp(float kp) {
  write_float_data(target_can_id_, ADDR_SPD_KP, kp, 0.0f, 200.0f);
  iValue = read_int_data(CMD_RESPONSE);
}
void read_ram_data(uint16_t index) {
  uint8_t data[8] = { 0x00 };
  memcpy(&data[0], &index, 2);
  send_command(target_can_id_, CMD_RAM_READ, master_can_id_, 8, data);
}
float read_float_data(uint16_t index) {
  delay(DELAY_TIME);
  if (CAN_MSGAVAIL != CAN.checkReceive()) {
    return -1;
  } else {
    CAN.readMsgBuf(&len, cdata);
  }
  float value = 0.0;
  memcpy(&value, &cdata[4], sizeof(float));
  id = CAN.getCanId();
/*  if ((int)(id >> 24) == (int)index) {
    Serial.println("index matched");

  } else {
    Serial.println("index not matched");
    Serial.println(index, HEX);
    Serial.println(id >> 24, HEX);
    Serial.print("0x    ");  // 打印前缀
    for (int i = 28; i >= 0; i -= 4) {
      uint8_t nibble = (id >> i) & 0xF;  // 提取4位（一个十六进制位）
      Serial.print(nibble, HEX);         // 打印当前十六进制位
    }
  }
  Serial.print("DATA:    ");
  Serial.print(value);
  Serial.println("");
  */
  return value;
}
int read_int_data(uint16_t index) {
  delay(DELAY_TIME);
  if (CAN_MSGAVAIL != CAN.checkReceive()) {
    return -1;
  } else {
    CAN.readMsgBuf(&len, cdata);
  }
  int value = 0;
  memcpy(&value, &cdata[4], sizeof(int));
  id = CAN.getCanId();
  /*if ((int)(id >> 24) == (int)index) {
    Serial.println("index matched");
  } else {
    Serial.println("index not matched");
    Serial.println(index, HEX);
    Serial.println(id >> 24, HEX);
    Serial.print("0x    ");  // 打印前缀
    for (int i = 28; i >= 0; i -= 4) {
      uint8_t nibble = (id >> i) & 0xF;  // 提取4位（一个十六进制位）
      Serial.print(nibble, HEX);         // 打印当前十六进制位
    }
  }
  Serial.print("DATA:    ");
  Serial.print(value);
  Serial.println("");
  */
  return value;
}
void set_velocity_ki(float ki) {
  write_float_data(target_can_id_, ADDR_SPD_KI, ki, 0.0f, 200.0f);
  iValue = read_int_data(CMD_RESPONSE);
}