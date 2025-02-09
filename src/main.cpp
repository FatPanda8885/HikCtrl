#include <Arduino.h> // 确保包含Arduino核心库

// 原作者：BI6OPR
// 此版本作者：BG5CVT
// 在原来的基础上优化了控制代码，修复了自动转向异常的问题
// Version Code：v0.5
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

// 定时器变量
unsigned long previousMillis = 0; // 上一次打印状态的时间
const long interval = 3000; // 打印状态的时间间隔（3秒）

// 声明函数
void handlePelcoDCommand(int command);
void stepMotor(int pin, bool direction, int speed);
void printStatus();

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
}

void readSerialData()
{
  if (Serial.available() >= 7)
  {
    int rlen = Serial.readBytes(buf, 7); // 读取7个字节
    if (rlen == 7 && buf[0] == 0xFF && buf[1] == 0x01) // 校验Pelco-D命令格式
    {
      int command = buf[6];
      if (command != currentCommand) // 只有在接收到新的命令时才处理
      {
        handlePelcoDCommand(command);
        currentCommand = command; // 更新当前命令
      }

      // 打印接收到的数据
      Serial.print("Received command (HEX): ");
      for (int i = 0; i < rlen; i++)
      {
        Serial.print(buf[i], HEX);
        if (i < rlen - 1)
        {
          Serial.print(" ");
        }
      }
      Serial.println();
    }
    else
    {
      Serial.println("Invalid command format.");
    }
  }
}

void handlePelcoDCommand(int command)
{
  switch (command)
  {
    case 1: // 停止
      digitalWrite(LED_MSG_PIN, LOW);
      is_azcontrol_stepper = false;
      is_elcontrol_stepper = false;
      Serial.println("0000.");
      break;
    case 2: // 右转
      az_stepper_direction = true; // 水平步进电机正转
      digitalWrite(AZ_DIRECTION_PIN, HIGH);
      stepperSpeed = 10; // 步进电机速度
      is_azcontrol_stepper = true;
      Serial.println("0002.");
      break;
    case 4: // 左转
      az_stepper_direction = false; // 水平步进电机反转
      digitalWrite(AZ_DIRECTION_PIN, LOW);
      stepperSpeed = 10; // 步进电机速度
      is_azcontrol_stepper = true;
      Serial.println("0004.");
      break;
    case 8: // 向上
      el_stepper_direction = true; // 俯仰步进电机正转
      digitalWrite(EL_DIRECTION_PIN, HIGH);
      stepperSpeed = 10; // 步进电机速度
      is_elcontrol_stepper = true;
      Serial.println("0003.");
      break;
    case 16: // 向下
      el_stepper_direction = false; // 俯仰步进电机反转
      digitalWrite(EL_DIRECTION_PIN, LOW);
      stepperSpeed = 10; // 步进电机速度
      is_elcontrol_stepper = true;
      Serial.println("0003.");
      break;
    default:
      Serial.println("Unknown command.");
      break;
  }
}

void loop()
{
  // 读取串口数据
  readSerialData();

  // 控制步进电机
  if (is_azcontrol_stepper)
  {
    stepMotor(AZ_SPEED_PUL_PIN, az_stepper_direction, stepperSpeed);
  }

  if (is_elcontrol_stepper)
  {
    stepMotor(EL_SPEED_PUL_PIN, el_stepper_direction, stepperSpeed);
  }

  // 打印当前状态
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;
    printStatus();
  }
}

void stepMotor(int pin, bool direction, int speed)
{
  digitalWrite(pin, HIGH);
  delayMicroseconds(speed / 2); // 脉冲宽度
  digitalWrite(pin, LOW);
  delayMicroseconds(speed / 2); // 间隔时间
  // Serial.print("Step on pin "); Serial.print(pin); Serial.print(" direction "); Serial.print(direction); Serial.print(" speed "); Serial.println(speed);
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
