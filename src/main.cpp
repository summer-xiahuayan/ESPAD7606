#include "AD7606.cpp"
#include <WiFi.h>
#include <lwip/sockets.h> // 添加原始套接字支持
#include <math.h>         // 用于正弦波生成

#define MISO 10
#define CONVSTB 9
#define CONVSTA 8
#define CS 7
#define SCK 2
#define RESET 0
#define BUSY 6
#define OS0 5
#define OS1 4
#define OS2 12
#define RANGE 1

// WiFi和UDP服务器配置
const char *ssid = "309";
const char *password = "12345678";
const char *serverIP = "192.168.3.51"; // 服务器IP地址
const int serverPort = 8081;            // 服务器端口号
const int localPort = 50000;            // 自定义本地端口号

AD7606_SPI AD(MISO, SCK, CS, CONVSTA, CONVSTB, BUSY, RESET);
int sock = -1; // 套接字描述符

// 正弦波参数
const float AMPLITUDE = 5.0;      // 幅度5V
const float PHASE_STEP = PI / 4;  // 相位步长45°(π/4弧度)
const int SAMPLES_PER_CYCLE = 10;  // 每个周期8个采样点
unsigned long lastUpdateTime = 0; // 上次更新时间
int cycleCount = 0;               // 周期计数

// 定义电压转换函数
float adcToVoltage(int16_t adcValue)
{
  return adcValue * 5.0 / 32768;
}

// WiFi连接函数
void connectWiFi()
{
  Serial.print("连接到WiFi: ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.println("WiFi连接成功");
  Serial.print("IP地址: ");
  Serial.println(WiFi.localIP());
}

// 初始化UDP套接字并绑定本地端口
bool initUDPSocket()
{
  Serial.print("初始化UDP套接字，本地端口: ");
  Serial.println(localPort);

  // 关闭现有套接字（如果有）
  if (sock != -1)
  {
    close(sock);
    sock = -1;
  }

  // 创建UDP套接字
  sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (sock < 0)
  {
    Serial.println("创建UDP套接字失败");
    return false;
  }

  // 设置套接字选项
  int opt = 1;
  if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0)
  {
    Serial.println("设置套接字选项失败");
    close(sock);
    return false;
  }

  // 设置套接字为非阻塞模式
  int flags = fcntl(sock, F_GETFL, 0);
  fcntl(sock, F_SETFL, flags | O_NONBLOCK);

  // 绑定本地端口
  struct sockaddr_in localAddr;
  localAddr.sin_family = AF_INET;
  localAddr.sin_addr.s_addr = INADDR_ANY;
  localAddr.sin_port = htons(localPort);

  if (bind(sock, (struct sockaddr *)&localAddr, sizeof(localAddr)) < 0)
  {
    Serial.println("绑定本地端口失败");
    close(sock);
    return false;
  }

  Serial.println("UDP套接字初始化成功");
  return true;
}

// 发送UDP数据
bool sendUDPData(const char *data, int len)
{
  if (sock < 0)
    return false;

  // 设置服务器地址
  struct sockaddr_in serverAddr;
  serverAddr.sin_family = AF_INET;
  serverAddr.sin_port = htons(serverPort);
  serverAddr.sin_addr.s_addr = inet_addr(serverIP);

  // 发送UDP数据
  int sent = sendto(sock, data, len, 0, (struct sockaddr *)&serverAddr, sizeof(serverAddr));

  if (sent < 0)
  {
    Serial.println("发送UDP数据失败");
    return false;
  }

  return (sent == len);
}

// 生成8个相位差为45°的正弦波数据
void generateSineWaveData(int16_t *data)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    // 计算当前相位
    float phase = PHASE_STEP * i + 2 * PI * cycleCount / SAMPLES_PER_CYCLE;
    // 生成正弦波电压值
    float voltage = AMPLITUDE * sin(phase);
    // 转换为ADC值
    data[i] = (int16_t)(voltage * 32768 / 5.0);
  }

  cycleCount++; // 周期计数加1
}

void setup()
{
  Serial.begin(115200);

  // 配置控制引脚
  pinMode(OS0, OUTPUT);
  pinMode(OS1, OUTPUT);
  pinMode(OS2, OUTPUT);
  pinMode(RANGE, OUTPUT);

  // 设置为±10V量程，关闭过采样
  digitalWrite(RANGE, true);
  digitalWrite(OS0, true);
  digitalWrite(OS1, true);
  digitalWrite(OS2, false);

  // 复位AD7606
  pinMode(RESET, OUTPUT);
  digitalWrite(RESET, HIGH);
  delay(1);
  digitalWrite(RESET, LOW);
  delay(10);

  Serial.println("AD7606初始化完成");

  // 连接WiFi
  connectWiFi();

  // 初始化UDP套接字
  initUDPSocket();
}

void loop()
{
  int16_t Data[8];
  String dataString = "";
  unsigned long currentTime = millis();

  // 每秒更新一次数据
  if (currentTime - lastUpdateTime >= 1000)
  {
    lastUpdateTime = currentTime;

    // 生成正弦波数据
    generateSineWaveData(Data);

    // 构建要发送的数据字符串
    for (uint8_t i = 0; i < 8; i++)
    {
      float voltage = adcToVoltage(Data[i]);
      // 添加到数据字符串
      dataString += String(voltage, 3); // 保留3位小数
      if (i < 7)
      {
        dataString += ",";
      }
    }

    // 发送UDP数据
    if (sendUDPData(dataString.c_str(), dataString.length()))
    {
      Serial.println("UDP数据已发送: " + dataString);
    }
    else
    {
      Serial.println("UDP数据发送失败，重新初始化套接字...");
      initUDPSocket();
    }

    Serial.println("-------------------");
  }

  delay(10); // 小延迟，避免CPU占用过高
}