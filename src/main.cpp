#include <Arduino.h> // 确保包含Arduino核心库
#include <EEPROM.h> //引入EEPROM库以便保存角度数据
#include <cmath>

// 原作者：BI6OPR
// 此版本作者：BG5CVT
// 在原来的基础上优化了控制代码，修复了自动转向异常的问题
// Version Code：v0.6 Pre2
// 发布时间：2025/02/09
// 本程序用于控制步进电机，实现Pelco-D控制

// Version 0.5更新日志：
// 更改信号输出PIN口，以适配v0.5控制底板

// 立春天，风渐暖，伊人一去不复返
// 献给可爱的小音

// 业余卫星软件云台参数如下
// 水平参数：55.5
// 垂直参数：155.5

// 信号灯输出口
const int LED_MSG_PIN = 13;

// 定义了板上的控制端，26作为水平方位角方向，27作为俯仰角方向
const int AZ_DIRECTION_PIN = 26;
const int EL_DIRECTION_PIN = 27;
// const int AZ_DIRECTION_PIN = 4;
// const int EL_DIRECTION_PIN = 16;

// 定义了PWM引脚，需要将这个脚接入PUL端，25作为方位角脉冲，14作为俯仰角脉冲
const int AZ_SPEED_PUL_PIN = 25;
const int EL_SPEED_PUL_PIN = 14;
// const int AZ_SPEED_PUL_PIN = 2;
// const int EL_SPEED_PUL_PIN = 15;

// 定义串口接受数据的全局变量
const int BUFFER_SIZE = 16;
char buf[BUFFER_SIZE];

// 步进电机的转动方向
bool az_stepper_direction = false;
bool el_stepper_direction = false;

// 步进电机速度,wait ms（间隔时间越小,速度越快）
int stepperSpeed = 500; // 初始速度设置为500微秒

// 步进电机控制
bool is_azcontrol_stepper = false;
bool is_elcontrol_stepper = false;

// 当前指令标志
int currentCommand = 0;

// 定义PWM初始变量
const int PWM_FREQ = 10000;       // PWM 频率（单位：Hz）
const int PWM_RESOLUTION = 8;    // 分辨率（8 位时，占空比范围 0-255）


// 定时器变量
unsigned long previousMillis = 0; // 上一次打印状态的时间
const long interval = 3000; // 打印状态的时间间隔（3秒）

// 声明函数
// void handleCommand(char command);
void stepMotor(int pin, bool direction, int speed);
void printStatus();

// 定义角度初始变量
int az_angle = 000;
int el_angle = 000;
unsigned long azLastCalcTime = 0; // 上次计算时间
const long azCalcInterval = 100;  // 计算间隔：100ms
unsigned long elLastCalcTime = 0; // 上次计算时间
const long elCalcInterval = 100;  // 计算间隔：100ms
float azCurrentAngle = 0.0;
float elCurrentAngle = 0.0;
void setup()
{
  // 配置串口:
  Serial.begin(9600); // 设置波特率
  // 设置IO输入输出
  pinMode(LED_MSG_PIN, OUTPUT);
  pinMode(AZ_DIRECTION_PIN, OUTPUT);
  pinMode(EL_DIRECTION_PIN, OUTPUT);
  pinMode(AZ_SPEED_PUL_PIN, OUTPUT);
  pinMode(EL_SPEED_PUL_PIN, OUTPUT);

  // 初始化信号灯
  digitalWrite(LED_MSG_PIN, LOW);

  // 初始化PWM模块
  ledcSetup(0 , PWM_FREQ, 8);
  ledcAttachPin(AZ_SPEED_PUL_PIN, 0);

  ledcSetup(1 , PWM_FREQ, 8);
  ledcAttachPin(EL_SPEED_PUL_PIN, 1);
}
void handleStringCommand(String cmd) {
  if (cmd == "S") {
      digitalWrite(LED_MSG_PIN, LOW);
      is_azcontrol_stepper = false;
      is_elcontrol_stepper = false;
      Serial.println("0000.");
  } else if (cmd == "A") {
      is_azcontrol_stepper = false;
  } else if (cmd == "E") {
      is_elcontrol_stepper = false;
  } else if (cmd == "R") {
      az_stepper_direction = true;
      digitalWrite(AZ_DIRECTION_PIN, HIGH);
      stepperSpeed = 10;
      is_azcontrol_stepper = true;
      Serial.println("0002.");
  } else if (cmd == "L") {
      az_stepper_direction = false;
      digitalWrite(AZ_DIRECTION_PIN, LOW);
      stepperSpeed = 10;
      is_azcontrol_stepper = true;
      Serial.println("0004.");
  } else if (cmd == "U") {
      el_stepper_direction = true;
      digitalWrite(EL_DIRECTION_PIN, HIGH);
      stepperSpeed = 10;
      is_elcontrol_stepper = true;
      Serial.println("0008.");
  } else if (cmd == "D") {
      el_stepper_direction = false;
      digitalWrite(EL_DIRECTION_PIN, LOW);
      stepperSpeed = 10;
      is_elcontrol_stepper = true;
      Serial.println("0016.");
  } else if (cmd == "C") {
      int az_angle = round(azCurrentAngle);
      Serial.print("AZ=");
      Serial.println(az_angle);
  } else if (cmd == "B") {
      int el_angle = round(elCurrentAngle);
      Serial.print("EL=");
      Serial.println(el_angle);
  } else if (cmd == "C2") { // ✅ 支持多字符命令
      int az_angle = round(azCurrentAngle);
      int el_angle = round(elCurrentAngle);
      Serial.print("AZ=");
      Serial.println(az_angle);
      Serial.print("EL=");
      Serial.println(el_angle);
  } else {
      Serial.println("Unknown command.");
  }
}  
void readSerialData()
{
  if (Serial.available() > 0) {
      Serial.setTimeout(100);
      String command = Serial.readStringUntil('\n'); // 读取直到换行符
      command.trim(); // 去除前后空格
      handleStringCommand(command);
  }
}


void calculateAngleAZ()
{
  // 每调用一次，假设增加一定角度（例如每个脉冲对应 0.1 度）
  float anglePerStep = 0.1; // 根据你的步进电机 + 减速比 + 编码器设置调整
  if (az_stepper_direction == true)
  {
    azCurrentAngle += anglePerStep;
  }
  else
  {
    azCurrentAngle -= anglePerStep;
  }

  // 角度归一化到 [0, 360)
  if (azCurrentAngle >= 360.0) azCurrentAngle -= 360.0;
  if (azCurrentAngle < 0.0) azCurrentAngle += 360.0;

}

void calculateAngleEL()
{
  // 每调用一次，假设增加一定角度（例如每个脉冲对应 0.1 度）
  float anglePerStep = 0.1; // 根据你的步进电机 + 减速比 + 编码器设置调整
  if (el_stepper_direction == true)
  {
    elCurrentAngle += anglePerStep;
  }
  else
  {
    elCurrentAngle -= anglePerStep;
  }

  // 角度归一化到 [0, 360)
  if (elCurrentAngle >= 90.0) elCurrentAngle -= 90.0;
  if (elCurrentAngle < 90.0) elCurrentAngle += 90.0;

}

void loop()
{
  unsigned long currentMillis = millis();

  // 读取串口数据
  readSerialData();

  // 控制步进电机
  if (is_azcontrol_stepper)
  {
    ledcWrite(0, 128);
    if (currentMillis - azLastCalcTime >= azCalcInterval)
    {
      azLastCalcTime = currentMillis; // 更新上次计算时间
      calculateAngleAZ();              // 执行角度计算函数
    }
  
  }

  if (is_azcontrol_stepper == false)
  {
    ledcWrite(0, 0);
  
  }

  if (is_elcontrol_stepper)
  {
    ledcWrite(1, 128);
    if (currentMillis - elLastCalcTime >= elCalcInterval)
    {
      elLastCalcTime = currentMillis; // 更新上次计算时间
      calculateAngleEL();              // 执行角度计算函数
    }
  }

  if (is_elcontrol_stepper == false)
  {
    ledcWrite(1, 0);
  }
  
}

void printStatus()
{
  Serial.print("Current Status - AZ Direction: ");
  Serial.print(az_stepper_direction ? "Forward" : "Backward");
  Serial.print(", EL Direction: ");
  Serial.print(el_stepper_direction ? "Forward" : "Backward");
  Serial.print(", AZ Control: ");
  Serial.print(is_azcontrol_stepper ? "On" : "Off");
  Serial.print(", EL Control: ");
  Serial.println(is_elcontrol_stepper ? "On" : "Off");
}

void set_zero()
{
  if(!EEPROM.begin(4096)) //之后替换为串口接收指令
  {
    EEPROM.begin(4096);
    EEPROM.write(4096, (az_angle >> 8) & 0xFF); // 高位字节
    EEPROM.write(4097, az_angle & 0xFF);       // 低位字节
    EEPROM.write(4096, (el_angle >> 8) & 0xFF); // 高位字节
    EEPROM.write(4097, el_angle & 0xFF);       // 低位字节
  }
}

// read:
// int readAngle = (EEPROM.read(4096) << 8) | EEPROM.read(4097);

