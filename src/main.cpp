

#include "AD7606.cpp"

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

AD7606_SPI AD(MISO, SCK, CS, CONVSTA, CONVSTB, BUSY, RESET);

// 定义电压转换函数
float adcToVoltage(int16_t adcValue)
{
  float tt;
  tt = adcValue;
  return tt * 5.0 / 32768;
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
}

void loop()
{
  int16_t Data[8];

  // 读取所有8个通道
  AD.read(Data, 8);

  // 打印每个通道的电压值
  for (uint8_t i = 0; i < 8; i++)
  {
    float voltage = adcToVoltage(Data[i]);
    // if(voltage<0){
    //   voltage = -voltage;
    // }
    Serial.print("通道 ");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(voltage, 3); // 保留3位小数
    Serial.println(" V");
  }

  Serial.println("-------------------");
  delay(1000); // 每秒采样一次
}