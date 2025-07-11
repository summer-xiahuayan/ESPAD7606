#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <nvs_flash.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <SPI.h>
#include <Wire.h>
#include <lwip/sockets.h>
#include <math.h>
#include "AD7606.cpp"
#include <driver/timer.h>

// SH1106显示定义
#include <SH1106Wire.h>
#define SDA 4
#define SCL 5
SH1106Wire display(0x3c, SDA, SCL, GEOMETRY_128_64, I2C_ONE, 1700000);
const int localPort = 50000; // 自定义本地端口号

// 按键相关变量
#define BOOT_PIN 9
#define DEBOUNCE_DELAY 50 // 按键去抖延迟(毫秒)

// 继电器控制引脚定义
#define RELAY1_PIN 12 // 第一个继电器控制引脚
#define RELAY2_PIN 13 // 第二个继电器控制引脚

// 频率范围定义（降低分辨率后，最小频率相应调整）
#define MIN_FREQ 0.01f
#define MAX_FREQ 50.0f

// 定时器相关定义（降低分辨率：100微秒）
hw_timer_t *pwmTimer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

#define PWM_RESOLUTION_US 100 // 100微秒分辨率

// 软件PWM相关变量
volatile float pwmFrequency1 = 1.0f;
volatile float pwmFrequency2 = 1.0f;
volatile float pwmDutyCycle1 = 0.0f;
volatile float pwmDutyCycle2 = 0.0f;
volatile float pwmPhase1 = 0.0f;
volatile float pwmPhase2 = 180.0f;
volatile unsigned long pwmPeriod1 = 10000; // 1Hz周期(100微秒单位)，即10000 * 100μs = 1s
volatile unsigned long pwmHighTime1 = 0;   // 高电平时间(100微秒单位)
volatile unsigned long pwmPeriod2 = 10000; // 1Hz周期(100微秒单位)
volatile unsigned long pwmHighTime2 = 0;   // 高电平时间(100微秒单位)
volatile bool pwmState1 = false;
volatile bool pwmState2 = false;
volatile unsigned long pwmTimer1 = 0;
volatile unsigned long pwmTimer2 = 0;
volatile unsigned long pwmGlobalTimer = 0; // 全局定时器计数器

int lastButtonState = HIGH;         // 上次按键状态
int buttonState = HIGH;             // 当前稳定按键状态
unsigned long lastDebounceTime = 0; // 上次状态变化时间

// AD7606定义
#define MISO 10
#define CONVSTB 21
#define CONVSTA 20
#define CS 7
#define SCK 2
#define RESET 0
#define BUSY 6
#define OS0 8
#define OS1 9
#define OS2 3
#define RANGE 1
#define SWITCH1 11

// 配置结构体 - 已更新为两路PWM独立控制
typedef struct
{
  char ssid[32];
  char password[64];
  char serverIp[16];
  int serverPort;
  bool isConfigured;

  // 新增PWM配置参数（两路独立控制）
  float pwmFrequency1; // 继电器1 PWM频率(Hz)
  float pwmDutyCycle1; // 继电器1占空比(0-100%)
  float pwmPhase1;     // 继电器1相位(度)

  float pwmFrequency2; // 继电器2 PWM频率(Hz)
  float pwmDutyCycle2; // 继电器2占空比(0-100%)
  float pwmPhase2;     // 继电器2相位(度)

  // 新增UDP配置参数
  float udpFrequency; // UDP上传频率(Hz)，0.01-50Hz
} ConfigData;

ConfigData config;
WebServer server(80);
WiFiClient client;
int sock = -1;

// AD7606实例
AD7606_SPI AD(MISO, SCK, CS, CONVSTA, CONVSTB, BUSY, RESET);

// 正弦波参数
const float AMPLITUDE = 5.0;
const float PHASE_STEP = PI / 4;
const int SAMPLES_PER_CYCLE = 9;
unsigned long lastUpdateTime = 0;
int cycleCount = 0;

// 电压转换函数
float adcToVoltage(int16_t adcValue)
{
  return adcValue * 5.0 / 32768;
}

// 限制频率在有效范围内
float constrainFrequency(float freq)
{
  if (freq < MIN_FREQ)
    return MIN_FREQ;
  if (freq > MAX_FREQ)
    return MAX_FREQ;
  return freq;
}

// 加载配置 - 已更新
bool loadConfig()
{
  Preferences preferences;
  preferences.begin("esp_config", true);

  if (!preferences.isKey("isConfigured"))
  {
    Serial.println("Configuration not found, using defaults");

    // 设置默认配置
    strcpy(config.ssid, "");
    strcpy(config.password, "");
    strcpy(config.serverIp, "192.168.1.1");
    config.serverPort = 8080;
    config.isConfigured = false;

    // 设置PWM默认参数（两路独立），限制在有效频率范围
    config.pwmFrequency1 = 1.0; // 默认1Hz
    config.pwmDutyCycle1 = 0.0; // 默认0%占空比(关闭)
    config.pwmPhase1 = 0.0;     // 默认0度相位

    config.pwmFrequency2 = 1.0; // 默认1Hz
    config.pwmDutyCycle2 = 0.0; // 默认0%占空比(关闭)
    config.pwmPhase2 = 180.0;   // 默认180度相位(与通道1反相)

    // 设置UDP默认参数，限制在有效频率范围
    config.udpFrequency = 1.0; // 默认1Hz

    preferences.end();
    return false;
  }

  config.isConfigured = preferences.getBool("isConfigured", false);
  preferences.getString("ssid", config.ssid, 32);
  preferences.getString("password", config.password, 64);
  preferences.getString("serverIp", config.serverIp, 16);
  config.serverPort = preferences.getInt("serverPort", 0);

  // 加载新增的PWM配置（两路独立），并限制频率范围
  config.pwmFrequency1 = constrainFrequency(preferences.getFloat("pwmFrequency1", 1.0));
  config.pwmDutyCycle1 = preferences.getFloat("pwmDutyCycle1", 0.0);
  config.pwmPhase1 = preferences.getFloat("pwmPhase1", 0.0);

  config.pwmFrequency2 = constrainFrequency(preferences.getFloat("pwmFrequency2", 1.0));
  config.pwmDutyCycle2 = preferences.getFloat("pwmDutyCycle2", 0.0);
  config.pwmPhase2 = preferences.getFloat("pwmPhase2", 180.0);

  // 加载新增的UDP配置，限制频率范围
  config.udpFrequency = constrainFrequency(preferences.getFloat("udpFrequency", 1.0));

  preferences.end();
  Serial.println("Configuration loaded successfully");
  return true;
}

// 保存配置 - 已更新
bool saveConfig()
{
  Preferences preferences;
  preferences.begin("esp_config", false);

  preferences.putBool("isConfigured", config.isConfigured);
  preferences.putString("ssid", config.ssid);
  preferences.putString("password", config.password);
  preferences.putString("serverIp", config.serverIp);
  preferences.putInt("serverPort", config.serverPort);

  // 保存新增的PWM配置（两路独立）
  preferences.putFloat("pwmFrequency1", config.pwmFrequency1);
  preferences.putFloat("pwmDutyCycle1", config.pwmDutyCycle1);
  preferences.putFloat("pwmPhase1", config.pwmPhase1);

  preferences.putFloat("pwmFrequency2", config.pwmFrequency2);
  preferences.putFloat("pwmDutyCycle2", config.pwmDutyCycle2);
  preferences.putFloat("pwmPhase2", config.pwmPhase2);

  // 保存新增的UDP配置
  preferences.putFloat("udpFrequency", config.udpFrequency);

  preferences.end();
  Serial.println("Configuration saved successfully");
  return true;
}

// 生成配置页面HTML - 已更新（限制频率输入范围）
// 其他代码保持不变...

// 生成配置页面HTML - 无外部依赖版本
String getConfigPage()
{
  String html = R"(<!DOCTYPE html>
<html lang="zh-CN">

<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ESP_AD7606配置中心</title>
    <style>
        /* 基础样式 */
        * {
            box-sizing: border-box;
            margin: 0;
            padding: 0;
        }

        body {
            font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Helvetica Neue", Arial, sans-serif;
            background-color: #f8fafc;
            color: #1e293b;
            min-height: 100vh;
            display: flex;
            flex-direction: column;
        }

        /* 头部样式 */
        header {
            
            background-color: #4a6fa5;
            font-size: 25px;
            color: white;
            padding: 8px;
            height: 60px;
            border-radius: 10px;
            margin-bottom: 1px;
            display: flex;
            justify-content: center;
            align-items: center;
        }


        .header-content {
            display: flex;
            justify-content: space-between;
            align-items: center;
            max-width: 1200px;
            margin: 0 auto;
          
        }

        .header-title {
            font-size:2rem;
            font-weight: bold;
            display: flex;
            align-items: center;
          
        }

        .header-icon {
            margin-right: 0.5rem;
        }

        .status-info {
            display: flex;
            align-items: center;
            font-size: 0.875rem;
        }

        .status-item {
            margin-left: 1.5rem;
            display: flex;
            align-items: center;
        }

        .status-icon {
            margin-right: 2.5rem;
            
        }

        /* 主内容样式 */
        main {
            flex-grow: 1;
            max-width: 1200px;
            margin: 0 auto;
            padding: 1.5rem;
            width: 100%;
        }

        /* 状态卡片 */
        .status-cards {
            display: grid;
            grid-template-columns: 1fr;
            gap: 1rem;
            margin-bottom: 1.5rem;
        }

        @media (min-width: 768px) {
            .status-cards {
                grid-template-columns: repeat(3, 1fr);
            }
        }

        .status-card {
            background-color: white;
            border-radius: 0.75rem;
            padding: 1rem;
            box-shadow: 0 2px 8px rgba(0, 0, 0, 0.08);
            display: flex;
            align-items: center;
            transition: transform 0.2s, box-shadow 0.2s;
        }

        .status-card:hover {
            transform: translateY(-2px);
            box-shadow: 0 4px 12px rgba(0, 0, 0, 0.12);
        }

        .card-icon {
            padding: 0.75rem;
            border-radius: 0.5rem;
            margin-right: 1rem;
        }

        .card-icon-blue {
            background-color: white;
            color: white;
        }

        .card-icon-green {
            background-color: rgba(16, 185, 129, 0.1);
            color: #10b981;
        }

        .card-icon-indigo {
            background-color: rgba(99, 102, 241, 0.1);
            color: #6366f1;
        }

        .card-text {
            flex-grow: 1;
        }

        .card-label {
            font-size: 1rem;
            color: #64748b;
        }

        .card-value {
            font-weight: 600;
            font-size: 14px;
        }

        /* 表单样式 */
        .config-form {
            display: grid;
            grid-template-columns: 1fr;
            gap: 1.5rem;
        }

        .form-section {
            background-color: white;
            border-radius: 0.75rem;
            padding: 1.5rem;
            box-shadow: 0 2px 8px rgba(0, 0, 0, 0.08);
            transition: opacity 0.3s;
        }

        .section-header {
            display: flex;
            align-items: center;
            margin-bottom: 1.25rem;
        }

        .section-icon {
            padding: 0.5rem;
            border-radius: 50%;
            margin-right: 0.75rem;
        }

        .section-title {
            font-size: 1.125rem;
            font-weight: 600;
        }

        .form-grid {
            display: grid;
            grid-template-columns: 1fr;
            gap: 1rem;
        }

        @media (min-width: 768px) {
            .form-grid {
                grid-template-columns: repeat(2, 1fr);
            }

            .form-grid-3 {
                grid-template-columns: repeat(3, 1fr);
            }
        }

        .form-group {
            margin-bottom: 1rem;
        }

        .form-label {
            display: block;
            font-size: 0.875rem;
            font-weight: 500;
            margin-bottom: 0.375rem;
        }

        .form-input {
            width: 100%;
            padding: 0.625rem 0.75rem;
            border: 1px solid #cbd5e1;
            border-radius: 0.375rem;
            font-size: 0.875rem;
            transition: border-color 0.2s, box-shadow 0.2s;
        }

        .form-input:focus {
            outline: none;
            border-color: #3b82f6;
            box-shadow: 0 0 0 2px rgba(59, 130, 246, 0.2);
        }

        .input-container {
  position: relative; /* 确保子元素绝对定位相对于此容器 */
  width: 100%;
}

.input-icon {
  position: absolute;
  left: 0.75rem;
  top: 50%;
  transform: translateY(-50%);
  width: 1.25rem; /* 固定图标宽度 */
  height: 1.25rem; /* 固定图标高度 */
  color: #94a3b8; /* 图标颜色 */
  pointer-events: none; /* 防止图标阻挡输入框点击 */
}

.input-with-icon {
  padding-left: 2.5rem; /* 为图标腾出空间 */
}

        .help-text {
            font-size: 0.75rem;
            color: #64748b;
            margin-top: 0.25rem;
            display: flex;
            align-items: center;
        }

       

        /* PWM特定样式 */
        .pwm-status {
            padding: 0.25rem 0.5rem;
            border-radius: 0.25rem;
            font-size: 0.75rem;
            font-weight: 500;
        }

        .pwm-disabled {
            background-color: #f1f5f9;
            color: #334155;
        }

        .pwm-active {
            background-color: #dbeafe;
            color: #1e40af;
        }

        .pwm-full {
            background-color: #dcfce7;
            color: #166534;
        }

        .duty-cycle-bar {
            height: 0.5rem;
            border-radius: 0.25rem;
            margin-top: 0.25rem;
            transition: width 0.3s;
        }

        .duty-cycle-bar-blue {
            background-color: #3b82f6;
        }

        .duty-cycle-bar-purple {
            background-color: #8b5cf6;
        }

        .phase-marks {
            display: flex;
            justify-content: space-between;
            font-size: 0.75rem;
            color: #64748b;
            margin-top: 0.25rem;
        }

        /* 按钮样式 */
        .submit-button {
            background:  #4a6fa5;
            color: white;
            padding: 0.75rem 2rem;
            border: none;
            border-radius: 0.5rem;
            font-size: 1rem;
            font-weight: 500;
            cursor: pointer;
            transition: transform 0.2s, box-shadow 0.2s;
            display: flex;
            align-items: center;
            justify-content: center;
            margin: 2rem auto 0;
        }

        .submit-button:hover {
            transform: translateY(-1px);
            box-shadow: 0 4px 6px -1px rgba(0, 0, 0, 0.1);
        }

        .button-icon {
            margin-right: 0.5rem;
        }

        /* 页脚样式 */
        footer {
            background-color:  #4a6fa5;
            color: white;
            padding: 1rem 1.5rem;
            text-align: center;
            font-size: 0.875rem;
            margin-top: 2rem;
        }

        .footer-version {
            color:  #4a6fa5;
            margin-top: 0.25rem;
        }

        /* 动画效果 */
        .fade-in {
            animation: fadeIn 0.5s ease-in-out;
        }

        @keyframes fadeIn {
            from {
                opacity: 0;
                transform: translateY(10px);
            }

            to {
                opacity: 1;
                transform: translateY(0);
            }
        }

        /* 自定义图标样式 */
        .icon {
            display: inline-block;
            width: 1em;
            height: 1em;
            stroke-width: 0;
            stroke: currentColor;
            fill: currentColor;
        }

        /* 模拟Font Awesome图标 */
        .fa {
            display: inline-block;
            font: normal normal normal 14px/1 FontAwesome;
            font-size: inherit;
            text-rendering: auto;
            -webkit-font-smoothing: antialiased;
            -moz-osx-font-smoothing: grayscale;
        }

        .fa-wifi:before {
            content: "📶";
        }

        .fa-clock-o:before {
            content: "🕒";
        }

        .fa-server:before {
            content: "🖥️";
        }

        .fa-cog:before {
            content: "⚙️";
        }

        .fa-signal:before {
            content: "📶";
        }

        .fa-lock:before {
            content: "🔒";
        }

        .fa-globe:before {
            content: "🌍";
        }

        .fa-plug:before {
            content: "🔌";
        }

        .fa-refresh:before {
            content: "🔄";
        }

        .fa-tachometer:before {
            content: "🚗";
        }

        .fa-sliders:before {
            content: "🎚️";
        }

        .fa-circle-o-notch:before {
            content: "🔄";
        }

        .fa-save:before {
            content: "💾";
        }

        .fa-info-circle:before {
            content: "ℹ️";
        }

        .fa-power-off:before {
            content: "🔌";
        }

        .fa-microchip:before {
            content: "💻";
        }
        footer {
  background-color: #4a6fa5; /* 蓝色背景 */
  color: white; /* 文字白色 */
  padding: 1rem 1.5rem;
  text-align: center;
  font-size: 0.875rem;
  margin-top: 2rem;
}

.footer-version {
  color: rgba(255, 255, 255, 0.8); /* 版本号文字稍浅，增加层次感 */
}
    </style>
</head>

<body>
    <header>
        <div class="header-content">
            <div class="header-title">
                ESP_AD7606配置中心
            </div>
            </div>
        </div>
    </header>

    <main>
        <div class="status-cards">
            <div class="status-card">
                <div class="card-icon card-icon-blue">
                    <span class="status-icon">
                        <svg xmlns="http://www.w3.org/2000/svg"
                            viewBox="0 0 512 512"><!--!Font Awesome Free 6.7.2 by @fontawesome - https://fontawesome.com License - https://fontawesome.com/license/free Copyright 2025 Fonticons, Inc.-->
                            <path
                                d="M176 24c0-13.3-10.7-24-24-24s-24 10.7-24 24l0 40c-35.3 0-64 28.7-64 64l-40 0c-13.3 0-24 10.7-24 24s10.7 24 24 24l40 0 0 56-40 0c-13.3 0-24 10.7-24 24s10.7 24 24 24l40 0 0 56-40 0c-13.3 0-24 10.7-24 24s10.7 24 24 24l40 0c0 35.3 28.7 64 64 64l0 40c0 13.3 10.7 24 24 24s24-10.7 24-24l0-40 56 0 0 40c0 13.3 10.7 24 24 24s24-10.7 24-24l0-40 56 0 0 40c0 13.3 10.7 24 24 24s24-10.7 24-24l0-40c35.3 0 64-28.7 64-64l40 0c13.3 0 24-10.7 24-24s-10.7-24-24-24l-40 0 0-56 40 0c13.3 0 24-10.7 24-24s-10.7-24-24-24l-40 0 0-56 40 0c13.3 0 24-10.7 24-24s-10.7-24-24-24l-40 0c0-35.3-28.7-64-64-64l0-40c0-13.3-10.7-24-24-24s-24 10.7-24 24l0 40-56 0 0-40c0-13.3-10.7-24-24-24s-24 10.7-24 24l0 40-56 0 0-40zM160 128l192 0c17.7 0 32 14.3 32 32l0 192c0 17.7-14.3 32-32 32l-192 0c-17.7 0-32-14.3-32-32l0-192c0-17.7 14.3-32 32-32zm192 32l-192 0 0 192 192 0 0-192z" />
                        </svg>
                    </span>
                </div>
                <div class="card-text">
                    <div class="card-label">ESP状态</div>
                    <div class="card-value" id="wifi-ssid">未连接</div>
                </div>
            </div>

            <div class="status-card">
                <div class="card-icon card-icon-blue">
                    <span class="status-icon">
                        <svg xmlns="http://www.w3.org/2000/svg"
                            viewBox="0 0 512 512"><!--!Font Awesome Free 6.7.2 by @fontawesome - https://fontawesome.com License - https://fontawesome.com/license/free Copyright 2025 Fonticons, Inc.-->
                            <path
                                d="M64 32C28.7 32 0 60.7 0 96l0 64c0 35.3 28.7 64 64 64l384 0c35.3 0 64-28.7 64-64l0-64c0-35.3-28.7-64-64-64L64 32zm280 72a24 24 0 1 1 0 48 24 24 0 1 1 0-48zm48 24a24 24 0 1 1 48 0 24 24 0 1 1 -48 0zM64 288c-35.3 0-64 28.7-64 64l0 64c0 35.3 28.7 64 64 64l384 0c35.3 0 64-28.7 64-64l0-64c0-35.3-28.7-64-64-64L64 288zm280 72a24 24 0 1 1 0 48 24 24 0 1 1 0-48zm56 24a24 24 0 1 1 48 0 24 24 0 1 1 -48 0z" />
                        </svg>
                    </span>
                </div>
                <div class="card-text">
                    <div class="card-label">服务器</div>
                    <div class="card-value" id="server-status">未配置</div>
                </div>
            </div>

            <div class="status-card">
                <div class="card-icon card-icon-blue">
                    <span class="status-icon">
                        <svg xmlns="http://www.w3.org/2000/svg"
                            viewBox="0 0 640 512"><!--!Font Awesome Free 6.7.2 by @fontawesome - https://fontawesome.com License - https://fontawesome.com/license/free Copyright 2025 Fonticons, Inc.-->
                            <path
                                d="M128 64c0-17.7 14.3-32 32-32l160 0c17.7 0 32 14.3 32 32l0 352 96 0 0-160c0-17.7 14.3-32 32-32l128 0c17.7 0 32 14.3 32 32s-14.3 32-32 32l-96 0 0 160c0 17.7-14.3 32-32 32l-160 0c-17.7 0-32-14.3-32-32l0-352-96 0 0 160c0 17.7-14.3 32-32 32L32 288c-17.7 0-32-14.3-32-32s14.3-32 32-32l96 0 0-160z" />
                        </svg>
                    </span>
                </div>
                <div class="card-text">
                    <div class="card-label">PWM通道</div>
                    <div class="card-value" id="pwm-status">0/2 激活</div>
                </div>
            </div>
        </div>

        <form action='/save' method='post' class="config-form">
            <div class="form-section fade-in">
                <div class="section-header">
                    <div class="card-icon card-icon-blue">
                        <span class="status-icon">
                            <svg xmlns="http://www.w3.org/2000/svg"
                                viewBox="0 0 640 512"><!--!Font Awesome Free 6.7.2 by @fontawesome - https://fontawesome.com License - https://fontawesome.com/license/free Copyright 2025 Fonticons, Inc.-->
                                <path
                                    d="M54.2 202.9C123.2 136.7 216.8 96 320 96s196.8 40.7 265.8 106.9c12.8 12.2 33 11.8 45.2-.9s11.8-33-.9-45.2C549.7 79.5 440.4 32 320 32S90.3 79.5 9.8 156.7C-2.9 169-3.3 189.2 8.9 202s32.5 13.2 45.2 .9zM320 256c56.8 0 108.6 21.1 148.2 56c13.3 11.7 33.5 10.4 45.2-2.8s10.4-33.5-2.8-45.2C459.8 219.2 393 192 320 192s-139.8 27.2-190.5 72c-13.3 11.7-14.5 31.9-2.8 45.2s31.9 14.5 45.2 2.8c39.5-34.9 91.3-56 148.2-56zm64 160a64 64 0 1 0 -128 0 64 64 0 1 0 128 0z" />
                            </svg>
                        </span>
                        </div>
                    <h2 class="section-title">WiFi配置</h2>
                </div>

                <div class="form-grid">
                    <div class="form-group">
                        <label for="ssid" class="form-label">WiFi名称 (SSID)</label>  
                        <div class="input-container">
                            <span class="input-icon">
                                <svg xmlns="http://www.w3.org/2000/svg"
                                    viewBox="0 0 576 512"><!--!Font Awesome Free 6.7.2 by @fontawesome - https://fontawesome.com License - https://fontawesome.com/license/free Copyright 2025 Fonticons, Inc.-->
                                    <path
                                        d="M64 0C28.7 0 0 28.7 0 64L0 448c0 35.3 28.7 64 64 64l256 0c35.3 0 64-28.7 64-64l0-19.3c-2.7 1.1-5.4 2-8.2 2.7l-60.1 15c-3 .7-6 1.2-9 1.4c-.9 .1-1.8 .2-2.7 .2l-64 0c-6.1 0-11.6-3.4-14.3-8.8l-8.8-17.7c-1.7-3.4-5.1-5.5-8.8-5.5s-7.2 2.1-8.8 5.5l-8.8 17.7c-2.9 5.9-9.2 9.4-15.7 8.8s-12.1-5.1-13.9-11.3L144 381l-9.8 32.8c-6.1 20.3-24.8 34.2-46 34.2L80 448c-8.8 0-16-7.2-16-16s7.2-16 16-16l8.2 0c7.1 0 13.3-4.6 15.3-11.4l14.9-49.5c3.4-11.3 13.8-19.1 25.6-19.1s22.2 7.8 25.6 19.1l11.6 38.6c7.4-6.2 16.8-9.7 26.8-9.7c15.9 0 30.4 9 37.5 23.2l4.4 8.8 8.9 0c-3.1-8.8-3.7-18.4-1.4-27.8l15-60.1c2.8-11.3 8.6-21.5 16.8-29.7L384 203.6l0-43.6-128 0c-17.7 0-32-14.3-32-32L224 0 64 0zM256 0l0 128 128 0L256 0zM549.8 139.7c-15.6-15.6-40.9-15.6-56.6 0l-29.4 29.4 71 71 29.4-29.4c15.6-15.6 15.6-40.9 0-56.6l-14.4-14.4zM311.9 321c-4.1 4.1-7 9.2-8.4 14.9l-15 60.1c-1.4 5.5 .2 11.2 4.2 15.2s9.7 5.6 15.2 4.2l60.1-15c5.6-1.4 10.8-4.3 14.9-8.4L512.1 262.7l-71-71L311.9 321z" />
                                </svg>
                            </span>
                            <input type="text" id="ssid" name="ssid" required class="form-input input-with-icon"
                                placeholder="输入WiFi名称">
                        </div>
                    </div>

                    <div class="form-group">
                        <label for="password" class="form-label">WiFi密码</label>
                        <div class="input-container">
                            <span class="input-icon">
                                <svg xmlns="http://www.w3.org/2000/svg"
                                    viewBox="0 0 448 512"><!--!Font Awesome Free 6.7.2 by @fontawesome - https://fontawesome.com License - https://fontawesome.com/license/free Copyright 2025 Fonticons, Inc.-->
                                    <path
                                        d="M144 144l0 48 160 0 0-48c0-44.2-35.8-80-80-80s-80 35.8-80 80zM80 192l0-48C80 64.5 144.5 0 224 0s144 64.5 144 144l0 48 16 0c35.3 0 64 28.7 64 64l0 192c0 35.3-28.7 64-64 64L64 512c-35.3 0-64-28.7-64-64L0 256c0-35.3 28.7-64 64-64l16 0z" />
                                </svg>
                            </span>
                            <input type="password" id="password" name="password" class="form-input input-with-icon"
                                placeholder="输入WiFi密码">
                        </div>
                    </div>
                </div>
            </div>

            <div class="form-section fade-in">
                <div class="section-header">
                    <div class="card-icon card-icon-blue">
                        <span class="status-icon">
                            <svg xmlns="http://www.w3.org/2000/svg"
                                viewBox="0 0 512 512"><!--!Font Awesome Free 6.7.2 by @fontawesome - https://fontawesome.com License - https://fontawesome.com/license/free Copyright 2025 Fonticons, Inc.-->
                                <path
                                    d="M78.6 5C69.1-2.4 55.6-1.5 47 7L7 47c-8.5 8.5-9.4 22-2.1 31.6l80 104c4.5 5.9 11.6 9.4 19 9.4l54.1 0 109 109c-14.7 29-10 65.4 14.3 89.6l112 112c12.5 12.5 32.8 12.5 45.3 0l64-64c12.5-12.5 12.5-32.8 0-45.3l-112-112c-24.2-24.2-60.6-29-89.6-14.3l-109-109 0-54.1c0-7.5-3.5-14.5-9.4-19L78.6 5zM19.9 396.1C7.2 408.8 0 426.1 0 444.1C0 481.6 30.4 512 67.9 512c18 0 35.3-7.2 48-19.9L233.7 374.3c-7.8-20.9-9-43.6-3.6-65.1l-61.7-61.7L19.9 396.1zM512 144c0-10.5-1.1-20.7-3.2-30.5c-2.4-11.2-16.1-14.1-24.2-6l-63.9 63.9c-3 3-7.1 4.7-11.3 4.7L352 176c-8.8 0-16-7.2-16-16l0-57.4c0-4.2 1.7-8.3 4.7-11.3l63.9-63.9c8.1-8.1 5.2-21.8-6-24.2C388.7 1.1 378.5 0 368 0C288.5 0 224 64.5 224 144l0 .8 85.3 85.3c36-9.1 75.8 .5 104 28.7L429 274.5c49-23 83-72.8 83-130.5zM56 432a24 24 0 1 1 48 0 24 24 0 1 1 -48 0z" />
                            </svg>
                        </span>
                    </div>
                    <h2 class="section-title">服务器配置</h2>
                </div>

                <div class="form-grid">
                    <div class="form-group">
                        <label for="serverIp" class="form-label">服务器IP地址</label>
                        <div class="input-container">
                            <span class="input-icon">
                                <svg xmlns="http://www.w3.org/2000/svg"
                                    viewBox="0 0 512 512"><!--!Font Awesome Free 6.7.2 by @fontawesome - https://fontawesome.com License - https://fontawesome.com/license/free Copyright 2025 Fonticons, Inc.-->
                                    <path
                                        d="M64 32C28.7 32 0 60.7 0 96l0 64c0 35.3 28.7 64 64 64l384 0c35.3 0 64-28.7 64-64l0-64c0-35.3-28.7-64-64-64L64 32zm280 72a24 24 0 1 1 0 48 24 24 0 1 1 0-48zm48 24a24 24 0 1 1 48 0 24 24 0 1 1 -48 0zM64 288c-35.3 0-64 28.7-64 64l0 64c0 35.3 28.7 64 64 64l384 0c35.3 0 64-28.7 64-64l0-64c0-35.3-28.7-64-64-64L64 288zm280 72a24 24 0 1 1 0 48 24 24 0 1 1 0-48zm56 24a24 24 0 1 1 48 0 24 24 0 1 1 -48 0z" />
                                </svg>
                            </span>
                            <input type="text" id="serverIp" name="serverIp" required class="form-input input-with-icon"
                                placeholder="例如: 192.168.1.100">
                        </div>
                    </div>

                    <div class="form-group">
                        <label for="serverPort" class="form-label">服务器端口</label>
                        <div class="input-container">
                            <span class="input-icon">
                                <svg xmlns="http://www.w3.org/2000/svg"
                                    viewBox="0 0 640 512"><!--!Font Awesome Free 6.7.2 by @fontawesome - https://fontawesome.com License - https://fontawesome.com/license/free Copyright 2025 Fonticons, Inc.-->
                                    <path
                                        d="M256 64l128 0 0 64-128 0 0-64zM240 0c-26.5 0-48 21.5-48 48l0 96c0 26.5 21.5 48 48 48l48 0 0 32L32 224c-17.7 0-32 14.3-32 32s14.3 32 32 32l96 0 0 32-48 0c-26.5 0-48 21.5-48 48l0 96c0 26.5 21.5 48 48 48l160 0c26.5 0 48-21.5 48-48l0-96c0-26.5-21.5-48-48-48l-48 0 0-32 256 0 0 32-48 0c-26.5 0-48 21.5-48 48l0 96c0 26.5 21.5 48 48 48l160 0c26.5 0 48-21.5 48-48l0-96c0-26.5-21.5-48-48-48l-48 0 0-32 96 0c17.7 0 32-14.3 32-32s-14.3-32-32-32l-256 0 0-32 48 0c26.5 0 48-21.5 48-48l0-96c0-26.5-21.5-48-48-48L240 0zM96 448l0-64 128 0 0 64L96 448zm320-64l128 0 0 64-128 0 0-64z" />
                                </svg>
                            </span>
                            <input type="number" id="serverPort" name="serverPort" required
                                class="form-input input-with-icon" placeholder="例如: 8080" min="1" max="65535">
                        </div>
                    </div>

                    <div class="form-group md:col-span-2">
                        <label for="udpFrequency" class="form-label">UDP上传频率 (Hz)</label>
                        <div class="input-container">
                            <span class="input-icon">
                                <svg xmlns="http://www.w3.org/2000/svg"
                                    viewBox="0 0 512 512"><!--!Font Awesome Free 6.7.2 by @fontawesome - https://fontawesome.com License - https://fontawesome.com/license/free Copyright 2025 Fonticons, Inc.-->
                                    <path
                                        d="M288 109.3L288 352c0 17.7-14.3 32-32 32s-32-14.3-32-32l0-242.7-73.4 73.4c-12.5 12.5-32.8 12.5-45.3 0s-12.5-32.8 0-45.3l128-128c12.5-12.5 32.8-12.5 45.3 0l128 128c12.5 12.5 12.5 32.8 0 45.3s-32.8 12.5-45.3 0L288 109.3zM64 352l128 0c0 35.3 28.7 64 64 64s64-28.7 64-64l128 0c35.3 0 64 28.7 64 64l0 32c0 35.3-28.7 64-64 64L64 512c-35.3 0-64-28.7-64-64l0-32c0-35.3 28.7-64 64-64zM432 456a24 24 0 1 0 0-48 24 24 0 1 0 0 48z" />
                                </svg>
                            </span>
                            <input type="number" id="udpFrequency" name="udpFrequency" min="0.01" max="50" step="0.01"
                                value="1.0" required class="form-input input-with-icon">

                        </div>
                        <div class="help-text flex items-center gap-2 text-sm text-gray-700">
                            <span>频率范围: 0.01-50Hz</span>           
                        </div>
                    </div>
                </div>
            </div>

            <div class="space-y-4">
                <div class="form-section fade-in">
                    <div class="section-header">
                        <div class="card-icon card-icon-blue">
                            <span class="status-icon">
                                <svg xmlns="http://www.w3.org/2000/svg"
                                    viewBox="0 0 640 512"><!--!Font Awesome Free 6.7.2 by @fontawesome - https://fontawesome.com License - https://fontawesome.com/license/free Copyright 2025 Fonticons, Inc.-->
                                    <path
                                        d="M128 64c0-17.7 14.3-32 32-32l160 0c17.7 0 32 14.3 32 32l0 352 96 0 0-160c0-17.7 14.3-32 32-32l128 0c17.7 0 32 14.3 32 32s-14.3 32-32 32l-96 0 0 160c0 17.7-14.3 32-32 32l-160 0c-17.7 0-32-14.3-32-32l0-352-96 0 0 160c0 17.7-14.3 32-32 32L32 288c-17.7 0-32-14.3-32-32s14.3-32 32-32l96 0 0-160z" />
                                </svg>
                            </span>
                            </div>
                        <div class="flex justify-between w-full">
                            <h2 class="section-title">PWM通道 1 (GPIO<span class="text-primary">)" +
                String(RELAY1_PIN) +
                R"(</span>)</h2>
                            <div class="pwm-status pwm-disabled" id="pwm1-status">已禁用</div>
                        </div>
                    </div>

                    <div class="form-grid form-grid-3">
                        <div class="form-group">
                            <label for="pwmFrequency1" class="form-label">频率 (Hz)</label>
                            <div class="input-container">
                                <span class="input-icon">
                                    <svg xmlns="http://www.w3.org/2000/svg"
                                        viewBox="0 0 512 512"><!--!Font Awesome Free 6.7.2 by @fontawesome - https://fontawesome.com License - https://fontawesome.com/license/free Copyright 2025 Fonticons, Inc.-->
                                        <path
                                            d="M0 224c0 17.7 14.3 32 32 32s32-14.3 32-32c0-53 43-96 96-96l160 0 0 32c0 12.9 7.8 24.6 19.8 29.6s25.7 2.2 34.9-6.9l64-64c12.5-12.5 12.5-32.8 0-45.3l-64-64c-9.2-9.2-22.9-11.9-34.9-6.9S320 19.1 320 32l0 32L160 64C71.6 64 0 135.6 0 224zm512 64c0-17.7-14.3-32-32-32s-32 14.3-32 32c0 53-43 96-96 96l-160 0 0-32c0-12.9-7.8-24.6-19.8-29.6s-25.7-2.2-34.9 6.9l-64 64c-12.5 12.5-12.5 32.8 0 45.3l64 64c9.2 9.2 22.9 11.9 34.9 6.9s19.8-16.6 19.8-29.6l0-32 160 0c88.4 0 160-71.6 160-160z" />
                                    </svg>
                                    </span>
                                <input type="number" id="pwmFrequency1" name="pwmFrequency1" min="0.01" max="50"
                                    step="0.01" value="1.0" required class="form-input input-with-icon">
                                    
                            </div>
                            <div class="help-text">
                                <span>频率范围: 0.01-50Hz</span>
                            </div>
                        </div>

                        <div class="form-group">
                            <label for="pwmDutyCycle1" class="form-label">占空比 (%)</label>
                            <div class="input-container">
                                <span class="input-icon">
                                    <svg xmlns="http://www.w3.org/2000/svg"
                                        viewBox="0 0 384 512"><!--!Font Awesome Free 6.7.2 by @fontawesome - https://fontawesome.com License - https://fontawesome.com/license/free Copyright 2025 Fonticons, Inc.-->
                                        <path
                                            d="M374.6 118.6c12.5-12.5 12.5-32.8 0-45.3s-32.8-12.5-45.3 0l-320 320c-12.5 12.5-12.5 32.8 0 45.3s32.8 12.5 45.3 0l320-320zM128 128A64 64 0 1 0 0 128a64 64 0 1 0 128 0zM384 384a64 64 0 1 0 -128 0 64 64 0 1 0 128 0z" />
                                    </svg>
                                    </span>
                                <input type="number" id="pwmDutyCycle1" name="pwmDutyCycle1" min="0" max="100"
                                    step="0.1" value="0.0" required class="form-input input-with-icon">
                            </div>
                            <div class="duty-cycle-bar duty-cycle-bar-blue" id="duty-cycle-bar1" style="width: 0%">
                            </div>
                        </div>

                        <div class="form-group">
                            <label for="pwmPhase1" class="form-label">相位 (度)</label>
                            <div class="input-container">
                                <span class="input-icon">
                                    <svg xmlns="http://www.w3.org/2000/svg"
                                        viewBox="0 0 512 512"><!--!Font Awesome Free 6.7.2 by @fontawesome - https://fontawesome.com License - https://fontawesome.com/license/free Copyright 2025 Fonticons, Inc.-->
                                        <path
                                            d="M169.7 .9c-22.8-1.6-41.9 14-47.5 34.7L110.4 80c.5 0 1.1 0 1.6 0c176.7 0 320 143.3 320 320c0 .5 0 1.1 0 1.6l44.4-11.8c20.8-5.5 36.3-24.7 34.7-47.5C498.5 159.5 352.5 13.5 169.7 .9zM399.8 410.2c.1-3.4 .2-6.8 .2-10.2c0-159.1-128.9-288-288-288c-3.4 0-6.8 .1-10.2 .2L.5 491.9c-1.5 5.5 .1 11.4 4.1 15.4s9.9 5.6 15.4 4.1L399.8 410.2zM176 208a32 32 0 1 1 0 64 32 32 0 1 1 0-64zm64 128a32 32 0 1 1 64 0 32 32 0 1 1 -64 0zM96 384a32 32 0 1 1 64 0 32 32 0 1 1 -64 0z" />
                                    </svg>
                                    </span>
                                <input type="number" id="pwmPhase1" name="pwmPhase1" min="0" max="360" step="1"
                                    value="0.0" required class="form-input input-with-icon">
                            </div>
                            <div class="phase-marks">
                                <span>0°</span>
                                <span>180°</span>
                                <span>360°</span>
                            </div>
                        </div>
                    </div>
                </div>

                <div class="form-section fade-in">
                    <div class="section-header">
                        <div class="card-icon card-icon-blue">
                            <span class="status-icon">
                                <svg xmlns="http://www.w3.org/2000/svg"
                                    viewBox="0 0 640 512"><!--!Font Awesome Free 6.7.2 by @fontawesome - https://fontawesome.com License - https://fontawesome.com/license/free Copyright 2025 Fonticons, Inc.-->
                                    <path
                                        d="M128 64c0-17.7 14.3-32 32-32l160 0c17.7 0 32 14.3 32 32l0 352 96 0 0-160c0-17.7 14.3-32 32-32l128 0c17.7 0 32 14.3 32 32s-14.3 32-32 32l-96 0 0 160c0 17.7-14.3 32-32 32l-160 0c-17.7 0-32-14.3-32-32l0-352-96 0 0 160c0 17.7-14.3 32-32 32L32 288c-17.7 0-32-14.3-32-32s14.3-32 32-32l96 0 0-160z" />
                                </svg>
                            </span>
                        </div>
                        <div class="flex justify-between w-full">
                            <h2 class="section-title">PWM通道 2 (GPIO<span class="text-primary">)" +
                String(RELAY2_PIN) +
                R"(</span>)</h2>
                            <div class="pwm-status pwm-disabled" id="pwm2-status">已禁用</div>
                        </div>
                    </div>

                    <div class="form-grid form-grid-3">
                        <div class="form-group">
                            <label for="pwmFrequency2" class="form-label">频率 (Hz)</label>
                            <div class="input-container">
                                <span class="input-icon">
                                    <svg xmlns="http://www.w3.org/2000/svg"
                                        viewBox="0 0 512 512"><!--!Font Awesome Free 6.7.2 by @fontawesome - https://fontawesome.com License - https://fontawesome.com/license/free Copyright 2025 Fonticons, Inc.-->
                                        <path
                                            d="M0 224c0 17.7 14.3 32 32 32s32-14.3 32-32c0-53 43-96 96-96l160 0 0 32c0 12.9 7.8 24.6 19.8 29.6s25.7 2.2 34.9-6.9l64-64c12.5-12.5 12.5-32.8 0-45.3l-64-64c-9.2-9.2-22.9-11.9-34.9-6.9S320 19.1 320 32l0 32L160 64C71.6 64 0 135.6 0 224zm512 64c0-17.7-14.3-32-32-32s-32 14.3-32 32c0 53-43 96-96 96l-160 0 0-32c0-12.9-7.8-24.6-19.8-29.6s-25.7-2.2-34.9 6.9l-64 64c-12.5 12.5-12.5 32.8 0 45.3l64 64c9.2 9.2 22.9 11.9 34.9 6.9s19.8-16.6 19.8-29.6l0-32 160 0c88.4 0 160-71.6 160-160z" />
                                    </svg>
                                    </span>
                                <input type="number" id="pwmFrequency2" name="pwmFrequency2" min="0.01" max="50"
                                    step="0.01" value="1.0" required class="form-input input-with-icon">
                            </div>
                            <div class="help-text">
                                <span>频率范围: 0.01-50Hz</span>
                            </div>
                        </div>

                        <div class="form-group">
                            <label for="pwmDutyCycle2" class="form-label">占空比 (%)</label>
                            <div class="input-container">
                                <span class="input-icon">
                                    <svg xmlns="http://www.w3.org/2000/svg"
                                        viewBox="0 0 384 512"><!--!Font Awesome Free 6.7.2 by @fontawesome - https://fontawesome.com License - https://fontawesome.com/license/free Copyright 2025 Fonticons, Inc.-->
                                        <path
                                            d="M374.6 118.6c12.5-12.5 12.5-32.8 0-45.3s-32.8-12.5-45.3 0l-320 320c-12.5 12.5-12.5 32.8 0 45.3s32.8 12.5 45.3 0l320-320zM128 128A64 64 0 1 0 0 128a64 64 0 1 0 128 0zM384 384a64 64 0 1 0 -128 0 64 64 0 1 0 128 0z" />
                                    </svg>
                                    </span>
                                <input type="number" id="pwmDutyCycle2" name="pwmDutyCycle2" min="0" max="100"
                                    step="0.1" value="0.0" required class="form-input input-with-icon">
                            </div>
                            <div class="duty-cycle-bar duty-cycle-bar-purple" id="duty-cycle-bar2" style="width: 0%">
                            </div>
                        </div>

                        <div class="form-group">
                            <label for="pwmPhase2" class="form-label">相位 (度)</label>
                            <div class="input-container">
                                <span class="input-icon">
                                    <svg xmlns="http://www.w3.org/2000/svg"
                                        viewBox="0 0 512 512"><!--!Font Awesome Free 6.7.2 by @fontawesome - https://fontawesome.com License - https://fontawesome.com/license/free Copyright 2025 Fonticons, Inc.-->
                                        <path
                                            d="M169.7 .9c-22.8-1.6-41.9 14-47.5 34.7L110.4 80c.5 0 1.1 0 1.6 0c176.7 0 320 143.3 320 320c0 .5 0 1.1 0 1.6l44.4-11.8c20.8-5.5 36.3-24.7 34.7-47.5C498.5 159.5 352.5 13.5 169.7 .9zM399.8 410.2c.1-3.4 .2-6.8 .2-10.2c0-159.1-128.9-288-288-288c-3.4 0-6.8 .1-10.2 .2L.5 491.9c-1.5 5.5 .1 11.4 4.1 15.4s9.9 5.6 15.4 4.1L399.8 410.2zM176 208a32 32 0 1 1 0 64 32 32 0 1 1 0-64zm64 128a32 32 0 1 1 64 0 32 32 0 1 1 -64 0zM96 384a32 32 0 1 1 64 0 32 32 0 1 1 -64 0z" />
                                    </svg>
                                    </span>
                                <input type="number" id="pwmPhase2" name="pwmPhase2" min="0" max="360" step="1"
                                    value="180.0" required class="form-input input-with-icon">
                            </div>
                            <div class="phase-marks">
                                <span>0°</span>
                                <span>180°</span>
                                <span>360°</span>
                            </div>
                        </div>
                    </div>
                </div>
            </div>

            <button type="submit" class="submit-button">
                保存配置
            </button>
        </form>
    </main>

    <footer>
        <p>ESP_AD7606数据采集系统 &copy; 2025</p>
        <p class="footer-version" >Version:v1.0.0       Copyright@SUMMER</p>
    </footer>

    <script>
        // 更新占空比进度条
        document.getElementById('pwmDutyCycle1').addEventListener('input', function () {
            document.getElementById('duty-cycle-bar1').style.width = this.value + '%';
            updatePwmStatus(1);
        });

        document.getElementById('pwmDutyCycle2').addEventListener('input', function () {
            document.getElementById('duty-cycle-bar2').style.width = this.value + '%';
            updatePwmStatus(2);
        });

        // 更新PWM状态显示
        function updatePwmStatus(channel) {
            const dutyCycle = parseFloat(document.getElementById(`pwmDutyCycle${channel}`).value);
            const statusElement = document.getElementById(`pwm${channel}-status`);

            if (dutyCycle > 0 && dutyCycle < 100) {
                statusElement.textContent = 'PWM模式';
                statusElement.className = 'pwm-status pwm-active';
            } else if (dutyCycle >= 100) {
                statusElement.textContent = '持续开启';
                statusElement.className = 'pwm-status pwm-full';
            } else {
                statusElement.textContent = '已禁用';
                statusElement.className = 'pwm-status pwm-disabled';
            }

            // 更新总PWM状态
            updateOverallPwmStatus();
        }

        // 更新总PWM状态
        function updateOverallPwmStatus() {
            const dutyCycle1 = parseFloat(document.getElementById('pwmDutyCycle1').value);
            const dutyCycle2 = parseFloat(document.getElementById('pwmDutyCycle2').value);
            const activeChannels = (dutyCycle1 > 0 ? 1 : 0) + (dutyCycle2 > 0 ? 1 : 0);

            document.getElementById('pwm-status').textContent = `${activeChannels}/2 激活`;
        }

        // 页面加载时初始化
        window.addEventListener('load', function () {
            setSvgFavicon();
            // 初始化占空比进度条
            document.getElementById('duty-cycle-bar1').style.width = document.getElementById('pwmDutyCycle1').value + '%';
            document.getElementById('duty-cycle-bar2').style.width = document.getElementById('pwmDutyCycle2').value + '%';

            // 初始化PWM状态
            updatePwmStatus(1);
            updatePwmStatus(2);

            // 模拟WiFi状态
            setTimeout(function () {
                document.getElementById('wifi-status').textContent = '已连接';
                document.getElementById('wifi-ssid').textContent = 'ESP32-AP';
                document.getElementById('server-status').textContent = '已配置';
            }, 1000);

            // 更新时间
            function updateTime() {
                const now = new Date();
                const hours = now.getHours().toString().padStart(2, '0');
                const minutes = now.getMinutes().toString().padStart(2, '0');
                const seconds = now.getSeconds().toString().padStart(2, '0');
                document.getElementById('current-time').textContent = `${hours}:${minutes}:${seconds}`;
            }

            updateTime();
            setInterval(updateTime, 1000);
             // 使用内嵌SVG图标作为favicon
            

            // 添加淡入动画
            setTimeout(function () {
                const sections = document.querySelectorAll('.fade-in');
                sections.forEach((section, index) => {
                    setTimeout(() => {
                        section.style.opacity = '1';
                    }, 100 * index);
                });
            }, 100);
        });

        function setSvgFavicon() {
                // 创建与Font Awesome芯片图标等效的SVG代码
                const svgCode =`
                <svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 512 512"><!--!Font Awesome Free 6.7.2 by @fontawesome - https://fontawesome.com License - https://fontawesome.com/license/free Copyright 2025 Fonticons, Inc.--><path d="M495.9 166.6c3.2 8.7 .5 18.4-6.4 24.6l-43.3 39.4c1.1 8.3 1.7 16.8 1.7 25.4s-.6 17.1-1.7 25.4l43.3 39.4c6.9 6.2 9.6 15.9 6.4 24.6c-4.4 11.9-9.7 23.3-15.8 34.3l-4.7 8.1c-6.6 11-14 21.4-22.1 31.2c-5.9 7.2-15.7 9.6-24.5 6.8l-55.7-17.7c-13.4 10.3-28.2 18.9-44 25.4l-12.5 57.1c-2 9.1-9 16.3-18.2 17.8c-13.8 2.3-28 3.5-42.5 3.5s-28.7-1.2-42.5-3.5c-9.2-1.5-16.2-8.7-18.2-17.8l-12.5-57.1c-15.8-6.5-30.6-15.1-44-25.4L83.1 425.9c-8.8 2.8-18.6 .3-24.5-6.8c-8.1-9.8-15.5-20.2-22.1-31.2l-4.7-8.1c-6.1-11-11.4-22.4-15.8-34.3c-3.2-8.7-.5-18.4 6.4-24.6l43.3-39.4C64.6 273.1 64 264.6 64 256s.6-17.1 1.7-25.4L22.4 191.2c-6.9-6.2-9.6-15.9-6.4-24.6c4.4-11.9 9.7-23.3 15.8-34.3l4.7-8.1c6.6-11 14-21.4 22.1-31.2c5.9-7.2 15.7-9.6 24.5-6.8l55.7 17.7c13.4-10.3 28.2-18.9 44-25.4l12.5-57.1c2-9.1 9-16.3 18.2-17.8C227.3 1.2 241.5 0 256 0s28.7 1.2 42.5 3.5c9.2 1.5 16.2 8.7 18.2 17.8l12.5 57.1c15.8 6.5 30.6 15.1 44 25.4l55.7-17.7c8.8-2.8 18.6-.3 24.5 6.8c8.1 9.8 15.5 20.2 22.1 31.2l4.7 8.1c6.1 11 11.4 22.4 15.8 34.3zM256 336a80 80 0 1 0 0-160 80 80 0 1 0 0 160z"/></svg>
            `;

                // 将SVG转换为Data URL格式
                const svgDataUrl = 'data:image/svg+xml;base64,' + btoa(unescape(encodeURIComponent(svgCode)));

                // 设置为favicon
                let link = document.querySelector('link[rel="icon"]');
                if (!link) {
                    link = document.createElement('link');
                    link.rel = 'icon';
                    document.head.appendChild(link);
                }
                link.href = svgDataUrl;
            };
    </script>
</body>

</html>)";
  return html;
}

// 其他代码保持不变...

// 处理根路径请求
void handleRoot()
{
  server.sendHeader("Content-Type", "text/html; charset=utf-8");
  server.send(200, "text/html; charset=utf-8", getConfigPage());
}

// 更新PWM参数
void updatePwmParameters()
{
  // 禁用中断以安全更新共享变量
  portENTER_CRITICAL(&timerMux);

  // 通道1参数计算（单位：100微秒）
  pwmPeriod1 = (unsigned long)(1000000.0 / (pwmFrequency1 * PWM_RESOLUTION_US));

  // 优化占空比0和100的情况
  if (pwmDutyCycle1 >= 99.0f)
  {
    pwmHighTime1 = pwmPeriod1;
  }
  else if (pwmDutyCycle1 <= 1.0f)
  {
    pwmHighTime1 = 0;
  }
  else
  {
    pwmHighTime1 = (unsigned long)(pwmPeriod1 * pwmDutyCycle1 / 100.0);
  }

  // 通道2参数计算（单位：100微秒）
  pwmPeriod2 = (unsigned long)(1000000.0 / (pwmFrequency2 * PWM_RESOLUTION_US));

  // 优化占空比0和100的情况
  if (pwmDutyCycle2 >= 99.0f)
  {
    pwmHighTime2 = pwmPeriod2;
  }
  else if (pwmDutyCycle2 <= 1.0f)
  {
    pwmHighTime2 = 0;
  }
  else
  {
    pwmHighTime2 = (unsigned long)(pwmPeriod2 * pwmDutyCycle2 / 100.0);
  }

  // 重置计时器以应用新参数
  pwmGlobalTimer = 0;
  pwmTimer1 = (unsigned long)(pwmPeriod1 * pwmPhase1 / 360.0);
  pwmTimer2 = (unsigned long)(pwmPeriod2 * pwmPhase2 / 360.0);

  // 立即应用占空比0和100的状态
  if (pwmHighTime1 == 0)
  {
    digitalWrite(RELAY1_PIN, LOW);
    pwmState1 = false;
  }
  else if (pwmHighTime1 == pwmPeriod1)
  {
    digitalWrite(RELAY1_PIN, HIGH);
    pwmState1 = true;
  }

  if (pwmHighTime2 == 0)
  {
    digitalWrite(RELAY2_PIN, LOW);
    pwmState2 = false;
  }
  else if (pwmHighTime2 == pwmPeriod2)
  {
    digitalWrite(RELAY2_PIN, HIGH);
    pwmState2 = true;
  }

  // 重新启用中断
  portEXIT_CRITICAL(&timerMux);

  Serial.println("PWM参数更新:");
  Serial.print("通道1 - 频率: ");
  Serial.print(pwmFrequency1);
  Serial.print("Hz, 占空比: ");
  Serial.print(pwmDutyCycle1);
  Serial.print("%, 相位: ");
  Serial.println(pwmPhase1);

  Serial.print("通道2 - 频率: ");
  Serial.print(pwmFrequency2);
  Serial.print("Hz, 占空比: ");
  Serial.print(pwmDutyCycle2);
  Serial.print("%, 相位: ");
  Serial.println(pwmPhase2);
}

// 处理保存配置请求 - 已更新（增加频率验证）
void handleSave()
{
  if (server.hasArg("ssid") && server.hasArg("serverIp") && server.hasArg("serverPort") &&
      server.hasArg("pwmFrequency1") && server.hasArg("pwmDutyCycle1") && server.hasArg("pwmPhase1") &&
      server.hasArg("pwmFrequency2") && server.hasArg("pwmDutyCycle2") && server.hasArg("pwmPhase2") &&
      server.hasArg("udpFrequency"))
  {
    // 保存WiFi和服务器配置
    server.arg("ssid").toCharArray(config.ssid, 32);
    server.arg("password").toCharArray(config.password, 64);
    server.arg("serverIp").toCharArray(config.serverIp, 16);
    config.serverPort = server.arg("serverPort").toInt();
    config.isConfigured = true;

    // 保存PWM配置（两路独立），并验证频率范围
    float freq1 = server.arg("pwmFrequency1").toFloat();
    float freq2 = server.arg("pwmFrequency2").toFloat();
    config.pwmFrequency1 = constrainFrequency(freq1);
    config.pwmFrequency2 = constrainFrequency(freq2);

    config.pwmDutyCycle1 = server.arg("pwmDutyCycle1").toFloat();
    config.pwmPhase1 = server.arg("pwmPhase1").toFloat();

    config.pwmDutyCycle2 = server.arg("pwmDutyCycle2").toFloat();
    config.pwmPhase2 = server.arg("pwmPhase2").toFloat();

    // 保存UDP配置，验证频率范围
    config.udpFrequency = constrainFrequency(server.arg("udpFrequency").toFloat());

    if (saveConfig())
    {
      // 更新PWM参数
      portENTER_CRITICAL(&timerMux);
      pwmFrequency1 = config.pwmFrequency1;
      pwmFrequency2 = config.pwmFrequency2;
      pwmDutyCycle1 = config.pwmDutyCycle1;
      pwmDutyCycle2 = config.pwmDutyCycle2;
      pwmPhase1 = config.pwmPhase1;
      pwmPhase2 = config.pwmPhase2;
      portEXIT_CRITICAL(&timerMux);

      updatePwmParameters();

      server.sendHeader("Content-Type", "text/html; charset=utf-8");
      server.send(200, "text/html; charset=utf-8",
                  "<html><body><h1>Configuration Saved!</h1><p>Device will reboot in 5 seconds...</p><script>setTimeout(function(){window.location.href='/'},5000);</script></body></html>");

      delay(5000);
      ESP.restart();
    }
    else
    {
      server.sendHeader("Content-Type", "text/plain; charset=utf-8");
      server.send(500, "text/plain; charset=utf-8", "Failed to save configuration");
    }
  }
  else
  {
    server.sendHeader("Content-Type", "text/plain; charset=utf-8");
    server.send(400, "text/plain; charset=utf-8", "Incomplete parameters");
  }
}

// 初始化NVS
bool initNVS()
{
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
  {
    Serial.println("Erasing NVS partition...");
    err = nvs_flash_erase();
    if (err != ESP_OK)
    {
      Serial.printf("NVS erase failed: %d\n", err);
      return false;
    }
    err = nvs_flash_init();
  }
  if (err != ESP_OK)
  {
    Serial.printf("NVS initialization failed: %d\n", err);
    return false;
  }
  Serial.println("NVS initialized successfully");
  return true;
}

// 屏幕显示相关函数
void displaySystemInfo(const String &title, const String &info1 = "", const String &info2 = "",
                       const String &info3 = "", const String &info4 = "")
{
  display.clear();
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, title);
  if (!info1.isEmpty())
    display.drawString(0, 10, info1);
  if (!info2.isEmpty())
    display.drawString(0, 20, info2);
  if (!info3.isEmpty())
    display.drawString(0, 30, info3);
  if (!info4.isEmpty())
    display.drawString(0, 40, info4);
  display.display();
}

// 初始化AP模式
void initAPMode()
{
  WiFi.mode(WIFI_AP);
  WiFi.softAP("ESP32-C3-Config", "12345678");

  IPAddress myIP(192, 168, 4, 1);
  WiFi.softAPConfig(myIP, myIP, IPAddress(255, 255, 255, 0));

  Serial.print("AP mode started, IP address: ");
  Serial.println(WiFi.softAPIP());

  displaySystemInfo("AP MODE ACTIVE",
                    "SSID: ESP32-C3-Config",
                    "Password: 12345678",
                    "IP: " + WiFi.softAPIP().toString(),
                    "Visit: 192.168.4.1");

  server.on("/", HTTP_GET, handleRoot);
  server.on("/save", HTTP_POST, handleSave);
  server.begin();
  Serial.println("Web server started");
}

// 连接WiFi
bool connectWiFi()
{
  if (!config.isConfigured)
  {
    return false;
  }

  Serial.print("Connecting to WiFi: ");
  Serial.println(config.ssid);

  displaySystemInfo("CONNECTING TO WIFI",
                    "SSID: " + String(config.ssid),
                    "Please wait...");

  WiFi.mode(WIFI_STA);
  WiFi.begin(config.ssid, config.password);

  int timeout = 0;
  while (WiFi.status() != WL_CONNECTED && timeout < 20)
  {
    delay(500);
    Serial.print(".");
    timeout++;

    displaySystemInfo("CONNECTING TO WIFI",
                      "SSID: " + String(config.ssid),
                      "Attempt: " + String(timeout) + "/20");
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

    displaySystemInfo("WIFI CONNECTION SUCCESS",
                      "SSID: " + String(config.ssid),
                      "IP: " + WiFi.localIP().toString(),
                      "RSSI: " + String(WiFi.RSSI()) + " dBm");
    return true;
  }
  else
  {
    Serial.println("");
    Serial.println("WiFi connection failed, entering AP mode");

    displaySystemInfo("WIFI CONNECTION FAILED",
                      "SSID: " + String(config.ssid),
                      "Entering AP mode...");
    return false;
  }
}

// 初始化UDP套接字
bool initUDPSocket()
{
  Serial.print("Initializing UDP socket, Server: ");
  Serial.print(config.serverIp);
  Serial.print(":");
  Serial.println(config.serverPort);

  displaySystemInfo("INITIALIZING UDP",
                    "Server: " + String(config.serverIp) + ":" + String(config.serverPort),
                    "Please wait...");

  if (sock != -1)
  {
    close(sock);
    sock = -1;
  }

  sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (sock < 0)
  {
    Serial.println("UDP socket creation failed");

    displaySystemInfo("UDP INIT FAILED",
                      "Socket creation error",
                      "Check server settings");
    return false;
  }

  int opt = 1;
  if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0)
  {
    Serial.println("Socket option setup failed");
    close(sock);

    displaySystemInfo("UDP INIT FAILED",
                      "Option setup error",
                      "Check server settings");
    return false;
  }

  struct sockaddr_in localAddr;
  localAddr.sin_family = AF_INET;
  localAddr.sin_addr.s_addr = INADDR_ANY;
  localAddr.sin_port = htons(50000);

  if (bind(sock, (struct sockaddr *)&localAddr, sizeof(localAddr)) < 0)
  {
    Serial.println("Local port binding failed");
    close(sock);

    displaySystemInfo("UDP INIT FAILED",
                      "Port binding error",
                      "Port: 50000");
    return false;
  }

  Serial.println("UDP socket initialized successfully");

  displaySystemInfo("UDP INIT SUCCESS",
                    "Local Port: 50000",
                    "Server: " + String(config.serverIp) + ":" + String(config.serverPort),
                    "Data transmission ready");
  return true;
}

// 发送UDP数据
bool sendUDPData(const char *data, int len)
{
  if (sock < 0)
    return false;

  struct sockaddr_in serverAddr;
  serverAddr.sin_family = AF_INET;
  serverAddr.sin_port = htons(config.serverPort);
  serverAddr.sin_addr.s_addr = inet_addr(config.serverIp);

  int sent = sendto(sock, data, len, 0, (struct sockaddr *)&serverAddr, sizeof(serverAddr));

  if (sent < 0)
  {
    Serial.println("UDP data transmission failed");
    return false;
  }

  return (sent == len);
}

// 生成正弦波数据
void generateSineWaveData(int16_t *data)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    float phase = PHASE_STEP * i + 2 * PI * cycleCount / SAMPLES_PER_CYCLE;
    float voltage = AMPLITUDE * sin(phase);
    data[i] = (int16_t)(voltage * 32768 / 5.0);
  }
  cycleCount++;
}

// 检测BOOT键是否按下
void checkBootKey()
{
  int reading = digitalRead(BOOT_PIN);

  // 按键去抖处理
  if (reading != lastButtonState)
  {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > DEBOUNCE_DELAY)
  {
    // 确认按键状态发生了变化
    if (reading != buttonState)
    {
      buttonState = reading;

      // 检测到按键按下事件 (从HIGH变为LOW)
      if (buttonState == LOW)
      {
        Serial.println("BOOT key pressed, entering AP mode...");
        displaySystemInfo("BOOT KEY PRESSED",
                          "Entering AP configuration mode",
                          "Please wait...");
        initAPMode();
      }
    }
  }

  lastButtonState = reading;
}

// PWM定时器中断服务函数
void IRAM_ATTR onPwmTimer()
{
  // 增加全局定时器计数
  pwmGlobalTimer++;

  // 通道1 PWM控制
  if (pwmFrequency1 > 0 && pwmHighTime1 < pwmPeriod1 && pwmHighTime1!=0)
  {
    pwmTimer1++;

    // 周期结束，重置状态
    if (pwmTimer1 >= pwmPeriod1)
    {
      pwmTimer1 = 0;
      pwmState1 = true;
        digitalWrite(RELAY1_PIN, HIGH);
    }
    // 高电平时间结束，转为低电平
    else if (pwmTimer1 >= pwmHighTime1 && pwmState1)
    {
      pwmState1 = false;
      digitalWrite(RELAY1_PIN, LOW);
    }
  }

  // 通道2 PWM控制
  if (pwmFrequency2 > 0 && pwmHighTime2 < pwmPeriod2 && pwmHighTime2!=0)
  {
    pwmTimer2++;

    // 周期结束，重置状态
    if (pwmTimer2 >= pwmPeriod2)
    {
      pwmTimer2 = 0;
      pwmState2 = true;
      digitalWrite(RELAY2_PIN, HIGH);
    }
    // 高电平时间结束，转为低电平
    else if (pwmTimer2 >= pwmHighTime2 && pwmState2)
    {
      pwmState2 = false;
      digitalWrite(RELAY2_PIN, LOW);
    }
  }
}

// 初始化PWM定时器
void initPwmTimer()
{
  // 创建一个100微秒的定时器（降低分辨率）
  pwmTimer = timerBegin(0, 8000, true); // 分频系数8000，产生100微秒分辨率
  timerAttachInterrupt(pwmTimer, &onPwmTimer, true);
  timerAlarmWrite(pwmTimer, 1, true); // 每100微秒触发一次中断
  timerAlarmEnable(pwmTimer);

  Serial.println("PWM定时器初始化完成 - 100微秒分辨率");
}

void setup()
{
  Serial.begin(115200);
  delay(100);
  Serial.println("\nESP32 C3 Data Acquisition System Starting");

  // 初始化BOOT键
  pinMode(BOOT_PIN, INPUT_PULLUP);

  // 初始化PWM引脚
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);

  // 应用初始状态
  digitalWrite(RELAY1_PIN, LOW);
  digitalWrite(RELAY2_PIN, LOW);

  // 初始化显示
  display.init();
  display.flipScreenVertically();
  displaySystemInfo("SYSTEM BOOTING",
                    "Initializing components...",
                    "Press BOOT key to configure",
                    "GPIO9: BOOT KEY");
  delay(1000);

  // 初始化NVS
  if (!initNVS())
  {
    Serial.println("NVS initialization failed, entering AP mode");
    initAPMode();
    return;
  }

  // 加载配置
  loadConfig();

  // 应用PWM配置
  portENTER_CRITICAL(&timerMux);
  pwmFrequency1 = config.pwmFrequency1;
  pwmFrequency2 = config.pwmFrequency2;
  pwmDutyCycle1 = config.pwmDutyCycle1;
  pwmDutyCycle2 = config.pwmDutyCycle2;
  pwmPhase1 = config.pwmPhase1;
  pwmPhase2 = config.pwmPhase2;
  portEXIT_CRITICAL(&timerMux);

  // 尝试连接WiFi，失败则进入AP配置模式
  if (!connectWiFi())
  {
    initAPMode();
    return;
  }

  // 配置AD7606控制引脚
  pinMode(OS0, OUTPUT);
  pinMode(OS1, OUTPUT);
  pinMode(OS2, OUTPUT);
  pinMode(RANGE, OUTPUT);
  pinMode(SWITCH1, OUTPUT);

  // 设置为±10V量程，关闭过采样
  digitalWrite(RANGE, HIGH);
  digitalWrite(OS0, HIGH);
  digitalWrite(OS1, HIGH);
  digitalWrite(OS2, LOW);
  digitalWrite(SWITCH1, LOW);

  // 初始化AD7606
  // 复位AD7606
  pinMode(RESET, OUTPUT);
  digitalWrite(RESET, HIGH);
  delay(1);
  digitalWrite(RESET, LOW);
  delay(10);
  Serial.println("AD7606初始化完成");

  // 初始化UDP套接字
  if (!initUDPSocket())
  {
    Serial.println("UDP socket initialization failed, entering AP mode");
    initAPMode();
    return;
  }

  displaySystemInfo("SYSTEM READY",
                    "WiFi: Connected",
                    "UDP: Initialized",
                    "Data acquisition started",
                    "Press BOOT key to reconfigure");

  updatePwmParameters();

  // 初始化PWM定时器（降低分辨率）
  initPwmTimer();
}

void loop()
{
  int16_t Data[8];
  String dataString = "";
  unsigned long currentTime = micros();
  static unsigned long lastUdpTime = 0;

  // 检测BOOT键状态
  checkBootKey();

  // 系统处于配置模式或WiFi未连接
  if (!config.isConfigured || WiFi.status() != WL_CONNECTED)
  {
    server.handleClient();
    return;
  }

  // 根据配置的频率更新数据（频率转间隔时间：1000000ms/频率）
  unsigned long updateInterval = (unsigned long)(1000000.0 / config.udpFrequency);
  if (currentTime - lastUdpTime >= updateInterval)
  {
    //Serial.println(currentTime);
    Serial.println(currentTime);
    Serial.println(lastUdpTime);

    lastUdpTime = currentTime;

    // 生成正弦波数据
    generateSineWaveData(Data);
    // 构建要发送的数据字符串
    for (uint8_t i = 0; i < 8; i++)
    {
      float voltage = adcToVoltage(Data[i]);
      dataString += String(voltage, 3);
      if (i < 7)
      {
        dataString += ",";
      }
    }

    // 发送UDP数据
    if (!sendUDPData(dataString.c_str(), dataString.length()))
    {
      Serial.println("UDP data transmission failed, reinitializing socket...");
      initUDPSocket();
    }
    else
    {
      Serial.print("UDP data sent (");
      Serial.print(config.udpFrequency);
      Serial.print("Hz): ");
      Serial.println(dataString);
    }

     Serial.println("-------------------");
     display.clear();
     display.drawString(0, 0, "LIP:" + WiFi.localIP().toString() + ":" + String(localPort));
     display.drawString(0, 10, "RIP:" + String(config.serverIp)  + ":" + String(config.serverPort));
     display.drawString(0, 20, "CH1:" + String(adcToVoltage(Data[0]), 3));
     display.drawString(68, 20, "CH2:" + String(adcToVoltage(Data[1]), 3));
     display.drawString(0, 30, "CH3:" + String(adcToVoltage(Data[2]), 3));
     display.drawString(68, 30, "CH4:" + String(adcToVoltage(Data[3]), 3));
     display.drawString(0, 40, "CH5:" + String(adcToVoltage(Data[4]), 3));
     display.drawString(68, 40, "CH6:" + String(adcToVoltage(Data[5]), 3));
     display.drawString(0, 50, "CH7:" + String(adcToVoltage(Data[6]), 3));
     display.drawString(68, 50, "CH8:" + String(adcToVoltage(Data[7]), 3));

    // // 显示PWM状态（调整显示精度匹配分辨率）
    // //display.drawString(0, 60, "PWM1:" + String(pwmFrequency1, 2) + "Hz " + String(pwmDutyCycle1, 1) + "%" + " PWM2:" + String(pwmFrequency2, 2) + "Hz " + String(pwmDutyCycle2, 1) + "%");
     display.display();

    
  }
}