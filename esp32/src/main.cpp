/*
 * ========================================================================================
 * ⚡ ESP32 ASIC HYDRO CONTROLLER – ES32D26 Edition (WITH PID, SMOOTH DAMPER & ANTI-STUCK)
 * ========================================================================================
 * Плата: Eletechsup ES32D26 (ESP32-DevKitC 38-PIN)
 * Реле CH1..CH8: Сдвиговый регистр 74HC595 (GPIO12 SER, GPIO13 OE, GPIO23 RCLK, GPIO22 SRCLK)
 * Входы IN1..IN8: Сдвиговый регистр 74HC165 (GPIO15 QH, GPIO2 CLK, GPIO0 SH)
 * Температура Tin: DS18B20 на шине OneWire (GPIO19)
 * Температура Tout: DS18B20 на шине OneWire (GPIO18)
 * Заслонка 4-20mA: Выход Io2 (GPIO26 / DAC2) с плавным ходом (Slew Rate Limiter)
 * Датчик давления: RS485 Modbus RTU (IO1 TX0, IO3 RX0, IO21 DIR)
 * ========================================================================================
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <Update.h>
#include <Preferences.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ModbusMaster.h>
#include <esp_task_wdt.h>
#include <QuickPID.h>

#include "config.h"

// ================= УПРАВЛЕНИЕ ОТЛАДКОЙ =================
#ifdef DEBUG_ENABLED
  #define DBG_PRINT(x)       
  #define DBG_PRINTLN(x)     
  #define DBG_PRINTF(...)    
  #define DBG_DETAIL(x)
  #define DBG_INFO(x)
  #define DBG_WARN(x)
  #define DBG_ERROR(x)
#else
  #define DBG_PRINT(x)
  #define DBG_PRINTLN(x)
  #define DBG_PRINTF(...)
  #define DBG_DETAIL(x)
  #define DBG_INFO(x)
  #define DBG_WARN(x)
  #define DBG_ERROR(x)
#endif

// ================= ПИНЫ 74HC595 (РЕЛЕ) =================
constexpr uint8_t PIN_SER_74HC595   = 12;
constexpr uint8_t PIN_OE_74HC595    = 13;
constexpr uint8_t PIN_RCLK_74HC595  = 23;
constexpr uint8_t PIN_SRCLK_74HC595 = 22;

// ================= ПИНЫ 74HC165 (ВХОДЫ) =================
constexpr uint8_t PIN_QH_74HC165  = 15;
constexpr uint8_t PIN_CLK_74HC165 = 2;
constexpr uint8_t PIN_SH_74HC165  = 0;

// ================= ПИНЫ DS18B20 =================
constexpr uint8_t TEMP_IN_PIN  = 19;
constexpr uint8_t TEMP_OUT_PIN = 18;

OneWire oneWireIn(TEMP_IN_PIN);
DallasTemperature sensorsIn(&oneWireIn);
OneWire oneWireOut(TEMP_OUT_PIN);
DallasTemperature sensorsOut(&oneWireOut);

float lastTempIn = -127.0;
float lastTempOut = -127.0;
bool tempInOnline = false;
bool tempOutOnline = false;

// ================= DAC ВЫХОД ЗАСЛОНКИ (ПЛАВНЫЙ ХОД) =================
constexpr uint8_t DAMPER_OUTPUT_PIN = 26;
float currentDamperPercent = 0.0; // Текущее фактическое положение (%)
float targetDamperPercent = 0.0;  // Целевое положение (%)

// Настройки плавности хода (Slew Rate): 0.5% каждые 100 мс = 100% за 20 секунд
constexpr float DAMPER_STEP_PCT = 0.5f;             // Шаг изменения (%)
constexpr unsigned long DAMPER_RAMP_INTERVAL = 100; // Интервал шага (мс)
unsigned long lastDamperRampTick = 0;

// ================= ПИД-РЕГУЛЯТОР (PID) =================
float pidSetpoint = 42.0; // Целевая температура Tout по умолчанию (°C)
float pidInput = 0.0;    // Входное значение (текущая Tout)
float pidOutput = 0.0;   // Выходное значение (0..100%)
float pidKp = 5.0, pidKi = 0.1, pidKd = 2.0; // Коэффициенты по умолчанию
bool isPidEnabled = false; // По умолчанию отключен (ручной режим)
bool isPidInverted = true; // true = REVERSE (для охлаждения), false = DIRECT

QuickPID hydroPID(&pidInput, &pidOutput, &pidSetpoint, pidKp, pidKi, pidKd,
                 QuickPID::pMode::pOnError,
                 QuickPID::dMode::dOnMeas,
                 QuickPID::iAwMode::iAwCondition,
                 isPidInverted ? QuickPID::Action::reverse : QuickPID::Action::direct);

unsigned long lastPidCompute = 0;
constexpr unsigned long PID_COMPUTE_INTERVAL = 2000; // Расчет ПИД каждые 2 секунды

// ================= ЦИКЛ АНТИЗАЛИПАНИЯ ЗАСЛОНКИ =================
unsigned long lastAntiStuckRun = 0;
constexpr unsigned long ANTI_STUCK_INTERVAL = 86400000UL; // 24 часа

bool isAntiStuckActive = false;
unsigned long antiStuckStageTimer = 0;
uint8_t antiStuckStep = 0;
float preAntiStuckDamperPos = 0.0;
bool preAntiStuckPidState = false;

// ================= RS485 MODBUS =================
constexpr uint8_t RS485_DIR_PIN = 21;
constexpr uint8_t PRESSURE_SENSOR_ADDR = 1;
ModbusMaster node;

float currentPressureKpa = 0.0;
float currentPressureBar = 0.0;
bool isPressureSensorOnline = false;

void preTransmission() {
  digitalWrite(RS485_DIR_PIN, HIGH);
  delayMicroseconds(100);
}

void postTransmission() {
  delayMicroseconds(100);
  digitalWrite(RS485_DIR_PIN, LOW);
  delayMicroseconds(100);
}

// ================= ГЛОБАЛЬНЫЕ ПЕРЕМЕННЫЕ =================
bool isAutoMode = true;
uint8_t relayStateMask = 0x00;
bool ota_active = false;
bool mqttConnected = false;
bool startupComplete = false;
unsigned long lastMqttReconnectAttempt = 0;

// Таймеры
unsigned long lastMqttRetry = 0;
unsigned long lastOtaStatusPublish = 0;
constexpr unsigned long OTA_STATUS_INTERVAL = 60000;
unsigned long lastRelayStatusPublish = 0;
constexpr unsigned long RELAY_STATUS_INTERVAL = 60000;
unsigned long lastTempCheck = 0;
constexpr unsigned long TEMP_CHECK_INTERVAL = 5000;
unsigned long lastPressureCheck = 0;
constexpr unsigned long PRESSURE_CHECK_INTERVAL = 2000;
unsigned long wifiReconnectStart = 0;
bool wifiReconnecting = false;
unsigned long lastMqttPublish = 0;
constexpr unsigned long MQTT_PUBLISH_INTERVAL = 30000;

// ================= СЕТЕВЫЕ ОБЪЕКТЫ =================
WebServer server(80);
WiFiClient espClient;
PubSubClient client(espClient);
Preferences preferences;

// ================= ПРОТОТИПЫ ФУНКЦИЙ =================
void sendByteRelay();
void setRelayChannel(uint8_t channel, bool state, bool fromMQTT = false);
bool getRelayChannel(uint8_t channel);
uint8_t readByteInputs();
bool readDigitalInput(uint8_t channel);
void readAndPublishTemperatures();
void readAndPublishPressure();
void applyDamperDAC(float percent);
void setDamperPercent(float percent);
void processDamperRamp();
void publishAllRelayStates();
void publishMasterState();
void publishSystemMode();
void publishPidStatus();
void processPID();
void handleAntiStuck();
void callback(char* topic, byte* payload, unsigned int length);
void reconnectMQTT();
void setupWebOTA();
void handleFirmwareUpload();
void handleFirmwareUploadPage();
void handleWiFiAsync();
void sendHADiscovery();
void publishRelayState(uint8_t channel, bool force = false);
void emergencyShutdown();
void enableRelays();
void printRelayState();
bool isMasterOn();

// ================= СТАТУС МАСТЕР-ВЫКЛЮЧАТЕЛЯ =================
bool isMasterOn() {
  return (relayStateMask != 0x00);
}

void printRelayState() {
  DBG_PRINTF("Relay mask: 0x%02X (", relayStateMask);
  for (int i = 1; i <= 8; i++) {
    if (getRelayChannel(i)) {
      DBG_PRINTF("CH%d=ON ", i);
    } else {
      DBG_PRINTF("CH%d=OFF ", i);
    }
  }
  DBG_PRINTLN(")");
}

// ================= УПРАВЛЕНИЕ РЕЛЕ 74HC595 =================
void sendByteRelay() {
  digitalWrite(PIN_RCLK_74HC595, LOW);
  delayMicroseconds(1);

  for (int i = 0; i < 8; ++i) {
    digitalWrite(PIN_SER_74HC595, (relayStateMask & (1 << i)) ? HIGH : LOW);
    digitalWrite(PIN_SRCLK_74HC595, LOW);
    delayMicroseconds(1);
    digitalWrite(PIN_SRCLK_74HC595, HIGH);
    delayMicroseconds(1);
  }

  digitalWrite(PIN_RCLK_74HC595, HIGH);
}

bool getRelayChannel(uint8_t channel) {
  if (channel < 1 || channel > 8) return false;
  return (relayStateMask & (1 << (8 - channel))) != 0;
}

// ================= ОСНОВНАЯ ФУНКЦИЯ УПРАВЛЕНИЯ РЕЛЕ =================
void setRelayChannel(uint8_t channel, bool state, bool fromMQTT) {
  if (!startupComplete) return;
  if (channel < 1 || channel > 8) return;

  uint8_t bit = (1 << (8 - channel));
  bool currentState = (relayStateMask & bit) != 0;
  if (currentState == state) return;

  if (state) {
    relayStateMask |= bit;
  } else {
    relayStateMask &= ~bit;
  }

  sendByteRelay();

  preferences.begin("asic_storage", false);
  preferences.putUChar("relays", relayStateMask);
  preferences.end();

  publishRelayState(channel, true);
  publishMasterState();
}

// ================= ПУБЛИКАЦИЯ СОСТОЯНИЯ РЕЛЕ В MQTT =================
void publishRelayState(uint8_t channel, bool force) {
  if (!client.connected()) return;

  const char* payload = getRelayChannel(channel) ? "ON" : "OFF";
  char topic[32];
  
  if (channel <= 4) snprintf(topic, sizeof(topic), "asic/relay%d/state", channel);
  else if (channel == 5) snprintf(topic, sizeof(topic), "asic/pump/state");
  else if (channel == 6) snprintf(topic, sizeof(topic), "asic/valve/state");
  else if (channel == 7) snprintf(topic, sizeof(topic), "asic/relay7/state");
  else if (channel == 8) snprintf(topic, sizeof(topic), "asic/relay8/state");
  else return;

  client.publish(topic, payload, false);

  if (force) {
    delay(50);
    client.publish(topic, payload, false);
  }
}

void publishMasterState() {
  if (!client.connected()) return;
  client.publish("asic/master/state", isMasterOn() ? "ON" : "OFF", false);
}

void publishAllRelayStates() {
  if (!client.connected()) return;
  for (int i = 1; i <= 8; i++) {
    publishRelayState(i, false);
    delay(10);
  }
  publishMasterState();
}

// ================= ЧТЕНИЕ ДИСКРЕТНЫХ ВХОДОВ =================
uint8_t readByteInputs() {
  uint8_t input_byte = 0x00;

  digitalWrite(PIN_SH_74HC165, LOW);
  delayMicroseconds(1);
  digitalWrite(PIN_SH_74HC165, HIGH);
  delayMicroseconds(1);

  for (int i = 0; i < 8; ++i) {
    uint8_t bit = digitalRead(PIN_QH_74HC165);
    if (bit == HIGH) input_byte |= (1 << i);
    digitalWrite(PIN_CLK_74HC165, LOW);
    delayMicroseconds(1);
    digitalWrite(PIN_CLK_74HC165, HIGH);
    delayMicroseconds(1);
  }

  return input_byte;
}

bool readDigitalInput(uint8_t channel) {
  if (channel < 1 || channel > 8) return false;
  uint8_t raw = readByteInputs();
  bool rawBit = (raw & (1 << (7 - (channel - 1)))) != 0;
  return !rawBit;
}

// ================= УПРАВЛЕНИЕ ЗАСЛОНКОЙ (ПЛАВНЫЙ ХОД 4-20 мА) =================

// Непосредственное физическое обновление ЦАП
void applyDamperDAC(float percent) {
  percent = constrain(percent, 0.0f, 100.0f);
  currentDamperPercent = percent;

  float currentmA = 4.0f + ((percent / 100.0f) * 16.0f);
  uint8_t dacValue = (uint8_t)round((currentmA / 20.0f) * 255.0f);
  dacWrite(DAMPER_OUTPUT_PIN, dacValue);
}

// Установка целевого положения
void setDamperPercent(float percent) {
  targetDamperPercent = constrain(percent, 0.0f, 100.0f);

  if (client.connected()) {
    char strBuf[16];
    snprintf(strBuf, sizeof(strBuf), "%.1f", targetDamperPercent);
    client.publish("asic/damper/state", strBuf, false);
    client.publish("asic/damper/percentage/state", strBuf, false);
  }
}

// Обработчик плавного перемещения (вызывается в loop)
void processDamperRamp() {
  if (millis() - lastDamperRampTick >= DAMPER_RAMP_INTERVAL) {
    lastDamperRampTick = millis();

    if (abs(currentDamperPercent - targetDamperPercent) > 0.05f) {
      if (currentDamperPercent < targetDamperPercent) {
        currentDamperPercent += DAMPER_STEP_PCT;
        if (currentDamperPercent > targetDamperPercent) currentDamperPercent = targetDamperPercent;
      } else {
        currentDamperPercent -= DAMPER_STEP_PCT;
        if (currentDamperPercent < targetDamperPercent) currentDamperPercent = targetDamperPercent;
      }

      applyDamperDAC(currentDamperPercent);

      if (client.connected()) {
        char strBuf[16];
        float currentmA = 4.0f + ((currentDamperPercent / 100.0f) * 16.0f);
        snprintf(strBuf, sizeof(strBuf), "%.2f", currentmA);
        client.publish("asic/damper/current_ma/state", strBuf, false);
      }
    }
  }
}

// ================= ЛОГИКА ПИД-РЕГУЛЯТОРА =================
void processPID() {
  if (!isPidEnabled || isAntiStuckActive) return;

  if (millis() - lastPidCompute >= PID_COMPUTE_INTERVAL) {
    lastPidCompute = millis();

    // Защита: если датчик Tout отвалился — открываем на 100%
    if (!tempOutOnline || lastTempOut <= -50.0) {
      setDamperPercent(100.0);
      return;
    }

    pidInput = lastTempOut; // Входное значение — температура обратки

    if (hydroPID.Compute()) {
      setDamperPercent(pidOutput);
    }
  }
}

// ================= ЛОГИКА ЕЖЕСУТОЧНОГО СМАХИВАНИЯ =================
void handleAntiStuck() {
  unsigned long currentMillis = millis();

  static bool firstRunCheck = true;
  if (firstRunCheck) {
    if (currentMillis > 600000UL) { // Старт первого цикла через 10 минут после включения
      lastAntiStuckRun = currentMillis;
      firstRunCheck = false;
    }
  }

  if (!isAntiStuckActive) {
    if (currentMillis - lastAntiStuckRun >= ANTI_STUCK_INTERVAL) {
      lastAntiStuckRun = currentMillis;
      
      if (isMasterOn()) {
        preAntiStuckDamperPos = targetDamperPercent;
        preAntiStuckPidState = isPidEnabled;
        
        if (isPidEnabled) {
          isPidEnabled = false;
          hydroPID.SetMode(QuickPID::Control::manual);
        }

        isAntiStuckActive = true;
        antiStuckStep = 1;
        antiStuckStageTimer = currentMillis;
        setDamperPercent(100.0); // Открываем на 100%
      }
    }
  } else {
    if (antiStuckStep == 1) {
      if (currentMillis - antiStuckStageTimer >= 10000) { // 10 сек держим целевой 100%
        antiStuckStep = 2;
        antiStuckStageTimer = currentMillis;
        setDamperPercent(0.0); // Закрываем на 0%
      }
    } 
    else if (antiStuckStep == 2) {
      if (currentMillis - antiStuckStageTimer >= 10000) { // 10 сек держим целевой 0%
        antiStuckStep = 3;
        antiStuckStageTimer = currentMillis;
        setDamperPercent(preAntiStuckDamperPos); // Возвращаем рабочее положение
      }
    }
    else if (antiStuckStep == 3) {
      if (currentMillis - antiStuckStageTimer >= 5000) {
        isAntiStuckActive = false;
        if (preAntiStuckPidState) {
          isPidEnabled = true;
          hydroPID.SetMode(QuickPID::Control::automatic);
        }
      }
    }
  }
}

void publishPidStatus() {
  if (!client.connected()) return;
  char strBuf[16];

  client.publish("asic/pid/enable/state", isPidEnabled ? "ON" : "OFF", true);
  client.publish("asic/pid/invert/state", isPidInverted ? "ON" : "OFF", true);

  snprintf(strBuf, sizeof(strBuf), "%.1f", pidSetpoint);
  client.publish("asic/pid/setpoint/state", strBuf, true);

  snprintf(strBuf, sizeof(strBuf), "%.2f", pidKp);
  client.publish("asic/pid/kp/state", strBuf, true);

  snprintf(strBuf, sizeof(strBuf), "%.2f", pidKi);
  client.publish("asic/pid/ki/state", strBuf, true);

  snprintf(strBuf, sizeof(strBuf), "%.2f", pidKd);
  client.publish("asic/pid/kd/state", strBuf, true);
}

// ================= ОПРОС ДАТЧИКОВ =================
void readAndPublishPressure() {
  uint8_t result = node.readHoldingRegisters(0x0004, 1);

  if (result == node.ku8MBSuccess) {
    int16_t rawVal = (int16_t)node.getResponseBuffer(0);
    currentPressureKpa = rawVal / 10.0f;
    currentPressureBar = currentPressureKpa / 100.0f;
    isPressureSensorOnline = true;

    if (client.connected()) {
      char strBuf[16];
      snprintf(strBuf, sizeof(strBuf), "%.2f", currentPressureBar);
      client.publish("asic/sensor/pressure/state", strBuf, false);
      client.publish("asic/sensor/pressure/status", "online", true);

      snprintf(strBuf, sizeof(strBuf), "%.1f", currentPressureKpa);
      client.publish("asic/sensor/pressure_kpa/state", strBuf, false);
    }
  } else {
    isPressureSensorOnline = false;
    if (client.connected()) {
      client.publish("asic/sensor/pressure/status", "offline", true);
    }
  }
}

void readAndPublishTemperatures() {
  char msgBuffer[16];

  sensorsIn.requestTemperatures();
  float tempIn = sensorsIn.getTempCByIndex(0);

  if (tempIn > -55.0 && tempIn < 125.0) {
    lastTempIn = tempIn;
    tempInOnline = true;
    if (client.connected()) {
      snprintf(msgBuffer, sizeof(msgBuffer), "%.1f", tempIn);
      client.publish("asic/sensor/temp_in/state", msgBuffer, false);
      client.publish("asic/sensor/temp_in/status", "online", true);
    }
  } else {
    tempInOnline = false;
    if (client.connected()) client.publish("asic/sensor/temp_in/status", "offline", true);
  }

  sensorsOut.requestTemperatures();
  float tempOut = sensorsOut.getTempCByIndex(0);

  if (tempOut > -55.0 && tempOut < 125.0) {
    lastTempOut = tempOut;
    tempOutOnline = true;
    if (client.connected()) {
      snprintf(msgBuffer, sizeof(msgBuffer), "%.1f", tempOut);
      client.publish("asic/sensor/temp_out/state", msgBuffer, false);
      client.publish("asic/sensor/temp_out/status", "online", true);
    }
  } else {
    tempOutOnline = false;
    if (client.connected()) client.publish("asic/sensor/temp_out/status", "offline", true);
  }
}

void publishSystemMode() {
  if (!client.connected()) return;
  client.publish("asic/mode/state", isAutoMode ? "AUTO" : "MANUAL", true);
}

// ================= SETUP =================
void setup() {
  esp_task_wdt_init(10, true);
  esp_task_wdt_add(NULL);

  // ИНИЦИАЛИЗАЦИЯ РЕЛЕ С ПОДТЯЖКОЙ НА 13 OE
  digitalWrite(PIN_OE_74HC595, HIGH);
  pinMode(PIN_OE_74HC595, OUTPUT);
  pinMode(PIN_SER_74HC595, OUTPUT);
  pinMode(PIN_RCLK_74HC595, OUTPUT);
  pinMode(PIN_SRCLK_74HC595, OUTPUT);

  pinMode(PIN_QH_74HC165, INPUT);
  pinMode(PIN_CLK_74HC165, OUTPUT);
  pinMode(PIN_SH_74HC165, OUTPUT);

  relayStateMask = 0x00;
  sendByteRelay();

  // ЗАДЕРЖКА СТАБИЛИЗАЦИИ
  for (int i = 0; i < 10; i++) {
    delay(1000);
    esp_task_wdt_reset();
  }

  // ЗАГРУЗКА ИЗ NVS
  preferences.begin("asic_storage", true);
  relayStateMask = preferences.getUChar("relays", 0x00);
  isAutoMode = preferences.getBool("auto_mode", true);
  isPidEnabled = preferences.getBool("pid_enable", false);
  isPidInverted = preferences.getBool("pid_invert", true);
  pidSetpoint = preferences.getFloat("pid_setpoint", 42.0);
  pidKp = preferences.getFloat("pid_kp", 5.0);
  pidKi = preferences.getFloat("pid_ki", 0.1);
  pidKd = preferences.getFloat("pid_kd", 2.0);
  preferences.end();

  // ИНИЦИАЛИЗАЦИЯ ПИД
  hydroPID.SetOutputLimits(0, 100);
  hydroPID.SetSampleTimeUs(PID_COMPUTE_INTERVAL * 1000);
  hydroPID.SetTunings(pidKp, pidKi, pidKd);
  hydroPID.SetControllerDirection(isPidInverted ? QuickPID::Action::reverse : QuickPID::Action::direct);
  hydroPID.SetMode(isPidEnabled ? QuickPID::Control::automatic : QuickPID::Control::manual);

  sendByteRelay();
  digitalWrite(PIN_OE_74HC595, LOW);

  pinMode(TEMP_IN_PIN, INPUT_PULLUP);
  pinMode(TEMP_OUT_PIN, INPUT_PULLUP);
  sensorsIn.begin();
  sensorsOut.begin();

  pinMode(RS485_DIR_PIN, OUTPUT);
  digitalWrite(RS485_DIR_PIN, LOW);
  
  Serial.begin(9600, SERIAL_8N1);
  node.begin(PRESSURE_SENSOR_ADDR, Serial);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  WiFi.setSleep(WIFI_PS_NONE);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  wifiReconnectStart = millis();
  wifiReconnecting = true;

  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  client.setBufferSize(2048);

  setupWebOTA();

  if (client.connected()) {
    publishAllRelayStates();
    publishSystemMode();
    publishPidStatus();
    setDamperPercent(targetDamperPercent);
  }

  startupComplete = true;
}

// ================= MAIN LOOP =================
void loop() {
  esp_task_wdt_reset();
  
  server.handleClient();

  if (ota_active) {
    delay(10);
    return;
  }

  static unsigned long startupTime = millis();
  if (millis() - startupTime < 10000) {
    delay(10);
    return;
  }

  handleWiFiAsync();

  // Обработка плавного движения заслонки
  processDamperRamp();

  if (millis() - lastTempCheck >= TEMP_CHECK_INTERVAL) {
    readAndPublishTemperatures();
    lastTempCheck = millis();
  }

  if (millis() - lastPressureCheck >= PRESSURE_CHECK_INTERVAL) {
    readAndPublishPressure();
    lastPressureCheck = millis();
  }

  // Расчет ПИД и Проверка Антизалипания
  processPID();
  handleAntiStuck();

  if (WiFi.status() == WL_CONNECTED) {
    if (!client.connected()) {
      reconnectMQTT();
    } else {
      client.loop();
      
      if (millis() - lastMqttPublish >= MQTT_PUBLISH_INTERVAL) {
        client.publish("asic/status", "online", true);
        lastMqttPublish = millis();
      }

      if (millis() - lastOtaStatusPublish >= OTA_STATUS_INTERVAL) {
        client.publish("asic/ota/status", "ready", true);
        lastOtaStatusPublish = millis();
      }

      if (millis() - lastRelayStatusPublish >= RELAY_STATUS_INTERVAL) {
        publishAllRelayStates();
        publishSystemMode();
        publishPidStatus();
        lastRelayStatusPublish = millis();
      }
    }
  }

  yield();
}

// ================= АСИНХРОННЫЙ WI-FI =================
void handleWiFiAsync() {
  if (WiFi.status() == WL_CONNECTED) {
    if (wifiReconnecting) {
      MDNS.end();
      MDNS.begin(ota_hostname);
      MDNS.addService("http", "tcp", 80);
      wifiReconnecting = false;
    }
    return;
  }

  if (!wifiReconnecting) {
    WiFi.disconnect();
    WiFi.begin(ssid, password);
    wifiReconnectStart = millis();
    wifiReconnecting = true;
  }

  if (millis() - wifiReconnectStart > 30000) {
    WiFi.disconnect(true);
    WiFi.begin(ssid, password);
    wifiReconnectStart = millis();
  }
}

// ================= MQTT ПОДКЛЮЧЕНИЕ =================
void reconnectMQTT() {
  if (ota_active || WiFi.status() != WL_CONNECTED) return;
  
  if (millis() - lastMqttRetry < 5000) return;
  lastMqttRetry = millis();

  if (client.connect("ESP32_ASIC_Hydro", mqtt_user, mqtt_pass, "asic/status", 0, true, "offline")) {
    client.publish("asic/status", "online", true);
    
    client.subscribe("asic/master/set");
    client.subscribe("asic/relay1/set");
    client.subscribe("asic/relay2/set");
    client.subscribe("asic/relay3/set");
    client.subscribe("asic/relay4/set");
    client.subscribe("asic/pump/set");
    client.subscribe("asic/valve/set");
    client.subscribe("asic/damper/set");
    client.subscribe("asic/sensor/leak/set");
    client.subscribe("asic/mode/set");
    client.subscribe("asic/ota");
    client.subscribe("asic/reset_nvs");

    // ПИД топики
    client.subscribe("asic/pid/enable/set");
    client.subscribe("asic/pid/invert/set");
    client.subscribe("asic/pid/setpoint/set");
    client.subscribe("asic/pid/kp/set");
    client.subscribe("asic/pid/ki/set");
    client.subscribe("asic/pid/kd/set");
    
    sendHADiscovery();
    delay(100);
    
    publishAllRelayStates();
    publishSystemMode();
    publishPidStatus();
    setDamperPercent(targetDamperPercent);
    
    lastRelayStatusPublish = millis();
    lastMqttPublish = millis();
  }
}

// ================= MQTT DISCOVERY (Home Assistant) =================
void sendHADiscovery() {
  char payload[1024];  
  const char* dev_av = R"raw("device":{"identifiers":["esp32_asic_hydro"],"name":"ASIC Hydro Controller","manufacturer":"Eletechsup","model":"ES32D26","sw_version":"v2.7.0"},"availability":[{"topic":"asic/status"}])raw";

  #define PUB_DISCOVERY(component, object_id, ...) \
    snprintf(payload, sizeof(payload), "{%s,%s}", dev_av, (__VA_ARGS__)); \
    client.publish("homeassistant/" component "/asic_hydro/" object_id "/config", payload, true); \
    delay(10);

  // 1. МАСТЕР-ВЫКЛЮЧАТЕЛЬ
  PUB_DISCOVERY("switch", "master_switch", 
    R"raw("name":"Master Switch","unique_id":"esp32_asic_master","state_topic":"asic/master/state","command_topic":"asic/master/set","payload_on":"ON","payload_off":"OFF","icon":"mdi:power")raw");

  // 2. РЕЖИМ РАБОТЫ
  PUB_DISCOVERY("select", "system_mode", 
    R"raw("name":"Control Mode","unique_id":"esp32_asic_mode","state_topic":"asic/mode/state","command_topic":"asic/mode/set","options":["AUTO","MANUAL"],"icon":"mdi:cog-box")raw");

  // 3. ПИД-РЕГУЛЯТОР В HA
  PUB_DISCOVERY("switch", "pid_enable", 
    R"raw("name":"PID Temperature Auto-Control","unique_id":"esp32_asic_pid_en","state_topic":"asic/pid/enable/state","command_topic":"asic/pid/enable/set","payload_on":"ON","payload_off":"OFF","icon":"mdi:thermostat-auto")raw");

  PUB_DISCOVERY("switch", "pid_invert", 
    R"raw("name":"PID Invert Direction (Reverse)","unique_id":"esp32_asic_pid_inv","state_topic":"asic/pid/invert/state","command_topic":"asic/pid/invert/set","payload_on":"ON","payload_off":"OFF","icon":"mdi:swap-horizontal","entity_category":"config")raw");

  PUB_DISCOVERY("number", "pid_setpoint", 
    R"raw("name":"Target Temperature Tout","unique_id":"esp32_asic_pid_sp","state_topic":"asic/pid/setpoint/state","command_topic":"asic/pid/setpoint/set","min":20,"max":85,"step":0.5,"unit_of_measurement":"°C","icon":"mdi:target-account")raw");

  PUB_DISCOVERY("number", "pid_kp", 
    R"raw("name":"PID Kp","unique_id":"esp32_asic_pid_kp","state_topic":"asic/pid/kp/state","command_topic":"asic/pid/kp/set","min":0,"max":50,"step":0.1,"entity_category":"config")raw");
  PUB_DISCOVERY("number", "pid_ki", 
    R"raw("name":"PID Ki","unique_id":"esp32_asic_pid_ki","state_topic":"asic/pid/ki/state","command_topic":"asic/pid/ki/set","min":0,"max":10,"step":0.01,"entity_category":"config")raw");
  PUB_DISCOVERY("number", "pid_kd", 
    R"raw("name":"PID Kd","unique_id":"esp32_asic_pid_kd","state_topic":"asic/pid/kd/state","command_topic":"asic/pid/kd/set","min":0,"max":50,"step":0.1,"entity_category":"config")raw");

  // 4. РЕЛЕ ASIC
  PUB_DISCOVERY("switch", "asic_1", R"raw("name":"ASIC 1","unique_id":"esp32_asic_r1","state_topic":"asic/relay1/state","command_topic":"asic/relay1/set","payload_on":"ON","payload_off":"OFF","icon":"mdi:server-network")raw");
  PUB_DISCOVERY("switch", "asic_2", R"raw("name":"ASIC 2","unique_id":"esp32_asic_r2","state_topic":"asic/relay2/state","command_topic":"asic/relay2/set","payload_on":"ON","payload_off":"OFF","icon":"mdi:server-network")raw");
  PUB_DISCOVERY("switch", "asic_3", R"raw("name":"ASIC 3","unique_id":"esp32_asic_r3","state_topic":"asic/relay3/state","command_topic":"asic/relay3/set","payload_on":"ON","payload_off":"OFF","icon":"mdi:server-network")raw");
  PUB_DISCOVERY("switch", "asic_4", R"raw("name":"ASIC 4","unique_id":"esp32_asic_r4","state_topic":"asic/relay4/state","command_topic":"asic/relay4/set","payload_on":"ON","payload_off":"OFF","icon":"mdi:server-network")raw");

  // 5. РЕЛЕ ГИДРАВЛИКИ
  PUB_DISCOVERY("switch", "pump", R"raw("name":"Coolant Pump","unique_id":"esp32_asic_pump","state_topic":"asic/pump/state","command_topic":"asic/pump/set","payload_on":"ON","payload_off":"OFF","icon":"mdi:pump")raw");
  PUB_DISCOVERY("switch", "heat_valve", R"raw("name":"Heat Dump Valve","unique_id":"esp32_asic_valve","state_topic":"asic/valve/state","command_topic":"asic/valve/set","payload_on":"ON","payload_off":"OFF","icon":"mdi:pipe-valve")raw");

  // 6. ЗАСЛОНКА
  PUB_DISCOVERY("number", "damper_set", R"raw("name":"Heat Valve Damper Open","unique_id":"esp32_asic_damper_set","state_topic":"asic/damper/state","command_topic":"asic/damper/set","min":0,"max":100,"step":1,"unit_of_measurement":"%","icon":"mdi:angle-acute")raw");
  PUB_DISCOVERY("sensor", "damper_current", R"raw("name":"Heat Valve Damper Current","unique_id":"esp32_asic_damper_ma","state_topic":"asic/damper/current_ma/state","unit_of_measurement":"mA","icon":"mdi:current-ac")raw");

  // 7. СЕНСОРЫ
  PUB_DISCOVERY("sensor", "pressure_bar", R"raw("name":"Coolant Pressure Bar","unique_id":"esp32_asic_press_bar","state_topic":"asic/sensor/pressure/state","unit_of_measurement":"bar","device_class":"pressure")raw");
  PUB_DISCOVERY("sensor", "temp_in", R"raw("name":"Coolant Temp IN","unique_id":"esp32_asic_tin","state_topic":"asic/sensor/temp_in/state","unit_of_measurement":"°C","device_class":"temperature")raw");
  PUB_DISCOVERY("sensor", "temp_out", R"raw("name":"Coolant Temp OUT","unique_id":"esp32_asic_tout","state_topic":"asic/sensor/temp_out/state","unit_of_measurement":"°C","device_class":"temperature")raw");

  #undef PUB_DISCOVERY
}

// ================= MQTT CALLBACK =================
void callback(char* topic, byte* payload, unsigned int length) {
  if (length == 0) return;
  
  char message[64];
  if (length > sizeof(message) - 1) length = sizeof(message) - 1;
  memcpy(message, payload, length);
  message[length] = '\0';

  bool isOn = (strcmp(message, "ON") == 0);

  // ===== ПИД РЕГУЛЯТОР =====
  if (strcmp(topic, "asic/pid/enable/set") == 0) {
    isPidEnabled = isOn;
    hydroPID.SetMode(isPidEnabled ? QuickPID::Control::automatic : QuickPID::Control::manual);
    
    preferences.begin("asic_storage", false);
    preferences.putBool("pid_enable", isPidEnabled);
    preferences.end();
    
    publishPidStatus();
    return;
  }

  if (strcmp(topic, "asic/pid/invert/set") == 0) {
    isPidInverted = isOn;
    hydroPID.SetControllerDirection(isPidInverted ? QuickPID::Action::reverse : QuickPID::Action::direct);
    
    preferences.begin("asic_storage", false);
    preferences.putBool("pid_invert", isPidInverted);
    preferences.end();
    
    publishPidStatus();
    return;
  }

  if (strcmp(topic, "asic/pid/setpoint/set") == 0) {
    float val = atof(message);
    if (val >= 20.0 && val <= 85.0) {
      pidSetpoint = val;
      preferences.begin("asic_storage", false);
      preferences.putFloat("pid_setpoint", pidSetpoint);
      preferences.end();
      publishPidStatus();
    }
    return;
  }

  if (strcmp(topic, "asic/pid/kp/set") == 0) {
    pidKp = atof(message);
    hydroPID.SetTunings(pidKp, pidKi, pidKd);
    preferences.begin("asic_storage", false);
    preferences.putFloat("pid_kp", pidKp);
    preferences.end();
    publishPidStatus();
    return;
  }

  if (strcmp(topic, "asic/pid/ki/set") == 0) {
    pidKi = atof(message);
    hydroPID.SetTunings(pidKp, pidKi, pidKd);
    preferences.begin("asic_storage", false);
    preferences.putFloat("pid_ki", pidKi);
    preferences.end();
    publishPidStatus();
    return;
  }

  if (strcmp(topic, "asic/pid/kd/set") == 0) {
    pidKd = atof(message);
    hydroPID.SetTunings(pidKp, pidKi, pidKd);
    preferences.begin("asic_storage", false);
    preferences.putFloat("pid_kd", pidKd);
    preferences.end();
    publishPidStatus();
    return;
  }

  // ===== МАСТЕР-ВЫКЛЮЧАТЕЛЬ =====
  if (strcmp(topic, "asic/master/set") == 0) {
    if (!startupComplete) return;
    relayStateMask = isOn ? 0xFC : 0x00;
    sendByteRelay();

    preferences.begin("asic_storage", false);
    preferences.putUChar("relays", relayStateMask);
    preferences.end();

    publishAllRelayStates();
    return;
  }

  // ===== РЕЖИМ РАБОТЫ =====
  if (strcmp(topic, "asic/mode/set") == 0) {
    if (strcmp(message, "AUTO") == 0) isAutoMode = true;
    else if (strcmp(message, "MANUAL") == 0) isAutoMode = false;
    else return;
    
    preferences.begin("asic_storage", false);
    preferences.putBool("auto_mode", isAutoMode);
    preferences.end();

    publishSystemMode();
    return;
  }

  // ===== РЕЛЕ =====
  if (strcmp(topic, "asic/relay1/set") == 0) setRelayChannel(1, isOn, true);
  else if (strcmp(topic, "asic/relay2/set") == 0) setRelayChannel(2, isOn, true);
  else if (strcmp(topic, "asic/relay3/set") == 0) setRelayChannel(3, isOn, true);
  else if (strcmp(topic, "asic/relay4/set") == 0) setRelayChannel(4, isOn, true);
  else if (strcmp(topic, "asic/pump/set") == 0) setRelayChannel(5, isOn, true);
  else if (strcmp(topic, "asic/valve/set") == 0) setRelayChannel(6, isOn, true);
  
  // ===== ЗАСЛОНКА =====
  else if (strcmp(topic, "asic/damper/set") == 0) {
    if (isPidEnabled) {
      isPidEnabled = false;
      hydroPID.SetMode(QuickPID::Control::manual);
      publishPidStatus();
    }
    float pct = atof(message);
    setDamperPercent(pct);
  }

  // ===== АВАРИЙНЫЙ ОСТАНОВ =====
  else if (strcmp(topic, "asic/sensor/leak/set") == 0 && isOn) {
    if (isAutoMode) {
      relayStateMask = 0x01;
      sendByteRelay();

      preferences.begin("asic_storage", false);
      preferences.putUChar("relays", relayStateMask);
      preferences.end();

      isPidEnabled = false;
      hydroPID.SetMode(QuickPID::Control::manual);
      setDamperPercent(0.0);
      
      if (client.connected()) client.publish("asic/emergency", "LEAK", true);
      publishAllRelayStates();
      publishPidStatus();
    }
  }

  // ===== СБРОС NVS =====
  else if (strcmp(topic, "asic/reset_nvs") == 0 && strcmp(message, "RESET") == 0) {
    preferences.begin("asic_storage", false);
    preferences.clear();
    preferences.end();
    
    relayStateMask = 0x00;
    isAutoMode = true;
    isPidEnabled = false;
    isPidInverted = true;
    sendByteRelay();
    
    publishAllRelayStates();
    publishSystemMode();
    publishPidStatus();
  }

  // ===== OTA =====
  else if (strcmp(topic, "asic/ota") == 0 && strcmp(message, "update") == 0) {
    ota_active = true;
    relayStateMask = 0x00;
    sendByteRelay();
    if (client.connected()) client.disconnect();
  }
}

// ================= WEB OTA =================
void setupWebOTA() {
  server.on("/", HTTP_GET, []() {
    if (!server.authenticate(http_username, http_password)) return server.requestAuthentication();
    handleFirmwareUploadPage();
  });
  
  server.on("/update", HTTP_POST, []() {
    if (!server.authenticate(http_username, http_password)) return server.requestAuthentication();
    server.send(200, "text/plain", "Update finished. Restarting...");
    delay(1000);
    ESP.restart();
  }, handleFirmwareUpload);
  
  server.on("/status", HTTP_GET, []() {
    if (!server.authenticate(http_username, http_password)) return server.requestAuthentication();
    String json = "{";
    json += "\"master\":\"" + String(isMasterOn() ? "ON" : "OFF") + "\",";
    json += "\"mode\":\"" + String(isAutoMode ? "AUTO" : "MANUAL") + "\",";
    json += "\"pid_enabled\":\"" + String(isPidEnabled ? "ON" : "OFF") + "\",";
    json += "\"pid_inverted\":\"" + String(isPidInverted ? "ON" : "OFF") + "\",";
    json += "\"pid_setpoint\":" + String(pidSetpoint) + ",";
    json += "\"relays\":\"" + String(relayStateMask, HEX) + "\",";
    json += "\"temp_in\":" + String(lastTempIn) + ",";
    json += "\"temp_out\":" + String(lastTempOut) + ",";
    json += "\"pressure\":" + String(currentPressureBar) + ",";
    json += "\"damper\":" + String(currentDamperPercent);
    json += "}";
    server.send(200, "application/json", json);
  });
  
  server.begin();
}

void handleFirmwareUploadPage() {
  const char html[] PROGMEM = R"rawliteral(
  <!DOCTYPE html>
  <html>
  <head>
    <title>⚡ ASIC Hydro OTA</title>
    <meta charset='utf-8'>
    <meta name='viewport' content='width=device-width, initial-scale=1'>
    <style>
      body { font-family: Arial; max-width: 600px; margin: 50px auto; padding: 20px; }
      .container { background: #f5f5f5; padding: 30px; border-radius: 10px; }
      h2 { color: #2c3e50; }
      button { background: #3498db; color: white; padding: 12px 30px; border: none; border-radius: 5px; cursor: pointer; }
    </style>
  </head>
  <body>
    <div class='container'>
      <h2>⚡ ASIC Hydro Controller</h2>
      <form method='POST' action='/update' enctype='multipart/form-data'>
        <input type='file' name='firmware' accept='.bin' required><br><br>
        <button type='submit'>🚀 Upload Firmware</button>
      </form>
    </div>
  </body>
  </html>
  )rawliteral";
  server.send(200, "text/html", html);
}

void handleFirmwareUpload() {
  HTTPUpload& upload = server.upload();
  if (upload.status == UPLOAD_FILE_START) {
    ota_active = true;
    relayStateMask = 0x00;
    sendByteRelay();
    if (client.connected()) client.disconnect();
    if (!Update.begin(UPDATE_SIZE_UNKNOWN)) Update.printError(Serial);
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) Update.printError(Serial);
  } else if (upload.status == UPLOAD_FILE_END) {
    if (!Update.end(true)) Update.printError(Serial);
  }
}