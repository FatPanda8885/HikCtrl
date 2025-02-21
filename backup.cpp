#include <Arduino.h>
#include <WiFi.h>

// WiFi 网络配置
const char* ssid = "HAMZJ";
const char* password = "zyyb833833833";

// TCP 服务器端口
const int serverPort = 80;

// 信号灯输出口
const int LED_MSG_PIN = 13;

// 定义了板上的控制端，26作为水平方位角方向，27作为俯仰角方向
const int AZ_DIRECTION_PIN = 26;
const int EL_DIRECTION_PIN = 27;

// 定义了PWM引脚，需要将这个脚接入PUL端，25作为方位角脉冲，14作为俯仰角脉冲
const int AZ_SPEED_PUL_PIN = 25;
const int EL_SPEED_PUL_PIN = 14;

// 定义串口接受数据的全局变量
const int BUFFER_SIZE = 16;
char buf[BUFFER_SIZE];

// 步进电机的转动方向
bool az_stepper_direction = false;
bool el_stepper_direction = false;
// 步进电机速度,wait ms（间隔时间越小,速度越快）
int stepperSpeed = 5; // 初始速度设置为500微秒
// 步进电机控制
bool is_azcontrol_stepper = false;
bool is_elcontrol_stepper = false;

bool last_azcontrol_stepper = false;
bool last_elcontrol_stepper = false;

// 当前指令标志
int currentCommand = 0;

// 定时器变量
unsigned long previousMillis = 0; // 上一次打印状态的时间
const long interval = 3000; // 打印状态的时间间隔（3秒）

// 声明函数
void handlePelcoDCommand(int command);
void stepMotor(int pin, bool direction, int speed);
void printStatus();

bool prev_az_control = false; // 记录 AZ 电机上一次的控制状态
bool prev_el_control = false; // 记录 EL 电机上一次的控制状态

// TCP 服务器
WiFiServer server(serverPort);

WiFiClient client;
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

  // 连接到 WiFi 网络
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // 启动 TCP 服务器
  server.begin();
  Serial.println("TCP Server started");
}

void readWifiData()
{
  // 处理客户端连接（非阻塞模式）
  if (!client) { // 如果没有活跃的客户端
    client = server.available(); // 检查新连接
    if (client) {
      Serial.println("New client connected");
    }
  } else { // 处理已连接的客户端
    if (client.connected() && client.available() >= 7) {
      int rlen = client.readBytes(buf, 7);
      if (rlen == 7 && buf[0] == 0xFF && buf[1] == 0x01 && buf[3] != 0x53 && buf[3] != 0x51) {
        int command = buf[3];
        if (command != currentCommand) {
          handlePelcoDCommand(command);
          currentCommand = command;
        }
        // 打印接收到的数据（保持原有逻辑）
        Serial.print("Received command (HEX): ");
        for (int i = 0; i < rlen; i++) {
          Serial.print(buf[i], HEX);
          if (i < rlen - 1) Serial.print(" ");
        }
        Serial.println();
      } else {
        Serial.println("Invalid command format.");
      }
    }
    
    // 检查客户端是否断开
    if (!client.connected()) {
      client.stop();
      Serial.println("Client disconnected");
    }
  }
}
void loop()
{
  readWifiData();
  // 控制步进电机（优先执行）
  if (is_azcontrol_stepper)
  {
    stepMotor(AZ_SPEED_PUL_PIN, az_stepper_direction, stepperSpeed);
  }

  if (is_elcontrol_stepper)
  {
    stepMotor(EL_SPEED_PUL_PIN, el_stepper_direction, stepperSpeed);
  }
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;
    printStatus();
  }
}

void handlePelcoDCommand(int command)
{
  switch (command)
  {
    case 0: // 停止
      digitalWrite(LED_MSG_PIN, LOW);
      is_azcontrol_stepper = false;
      is_elcontrol_stepper = false;
      Serial.println("0000.");
      break;
    case 2: // 右转
      digitalWrite(LED_MSG_PIN, HIGH);
      az_stepper_direction = true; // 水平步进电机正转
      digitalWrite(AZ_DIRECTION_PIN, HIGH);
      stepperSpeed = 5; // 步进电机速度
      is_azcontrol_stepper = true;
      Serial.println("0002.");
      break;
    case 4: // 左转
      digitalWrite(LED_MSG_PIN, HIGH);
      az_stepper_direction = false; // 水平步进电机反转
      digitalWrite(AZ_DIRECTION_PIN, LOW);
      stepperSpeed = 5; // 步进电机速度
      is_azcontrol_stepper = true;
      Serial.println("0004.");
      break;
    case 8: // 向上
      digitalWrite(LED_MSG_PIN, HIGH);
      el_stepper_direction = true; // 俯仰步进电机正转
      digitalWrite(EL_DIRECTION_PIN, HIGH);
      stepperSpeed = 5; // 步进电机速度
      is_elcontrol_stepper = true;
      Serial.println("0008.");
      break;
    case 16: // 向下
      digitalWrite(LED_MSG_PIN, HIGH);
      el_stepper_direction = false; // 俯仰步进电机反转
      digitalWrite(EL_DIRECTION_PIN, LOW);
      stepperSpeed = 5; // 步进电机速度
      is_elcontrol_stepper = true;
      Serial.println("0016.");
      break;
    case 0x0C: // 左上
      digitalWrite(LED_MSG_PIN, HIGH);
      az_stepper_direction = false; // 水平步进电机反转
      digitalWrite(AZ_DIRECTION_PIN, LOW);
      el_stepper_direction = true; // 俯仰步进电机正转
      digitalWrite(EL_DIRECTION_PIN, HIGH);
      stepperSpeed = 5; // 步进电机速度
      is_azcontrol_stepper = true;
      is_elcontrol_stepper = true;
      Serial.println("000C.");
      break;
    case 0x0A: // 右上
      digitalWrite(LED_MSG_PIN, HIGH);
      az_stepper_direction = true; // 水平步进电机正转
      digitalWrite(AZ_DIRECTION_PIN, HIGH);
      el_stepper_direction = true; // 俯仰步进电机正转
      digitalWrite(EL_DIRECTION_PIN, HIGH);
      stepperSpeed = 5; // 步进电机速度
      is_azcontrol_stepper = true;
      is_elcontrol_stepper = true;
      Serial.println("000A.");
      break;
    case 0x14: // 左下
      digitalWrite(LED_MSG_PIN, HIGH);
      az_stepper_direction = false; // 水平步进电机反转
      digitalWrite(AZ_DIRECTION_PIN, LOW);
      el_stepper_direction = false; // 俯仰步进电机反转
      digitalWrite(EL_DIRECTION_PIN, LOW);
      stepperSpeed = 5; // 步进电机速度
      is_azcontrol_stepper = true;
      is_elcontrol_stepper = true;
      Serial.println("0014.");
      break;
    case 0x12: // 右下
      digitalWrite(LED_MSG_PIN, HIGH);
      az_stepper_direction = true; // 水平步进电机正转
      digitalWrite(AZ_DIRECTION_PIN, HIGH);
      el_stepper_direction = false; // 俯仰步进电机反转
      digitalWrite(EL_DIRECTION_PIN, LOW);
      stepperSpeed = 5;
      is_azcontrol_stepper = true;
      is_elcontrol_stepper = true;
      Serial.println("0012.");
      break;
    default:
      Serial.println("Unknown command.");
      break;
  }
}

void stepMotor(int pin, bool direction, int speed)
{
  digitalWrite(pin, HIGH);
  delayMicroseconds(speed / 2); // 脉冲宽度
  digitalWrite(pin, LOW);
  delayMicroseconds(speed / 2); // 间隔时间
  // Serial.print("Step on pin "); Serial.print(pin); Serial.print(" direction "); Serial.print(direction ? "Forward" : "Backward"); Serial.print(" speed "); Serial.println(speed); // 调试信息
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