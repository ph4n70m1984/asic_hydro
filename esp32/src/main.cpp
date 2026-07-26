/*
 * ========================================================================================
 * ⚡ ESP32 ASIC HYDRO CONTROLLER – NATIVE ESP-IDF ASYNC (v3.8.2 QoS1 & Retain Edition)
 * ========================================================================================
 * Плата: Eletechsup ES32D26 (ESP32-DevKitC 38-PIN)
 * Поддерживаемые режимы (выбор 1 из 2):
 *  1. VALVE_4_20MA     : Кран / заслонка (DIP 1 = ON, DIP 3 = OFF). Диапазон 4..20 мА.
 *  2. DRY_COOLER_0_10V : Сухая градирня / VFD (DIP 1 = OFF, DIP 3 = ON). Диапазон 0..10 В.
 * Изменения v3.8.2:
 *  - Все публикации MQTT переведены на QoS 1 и Retain = true
 *  - Все подписки MQTT переведены на QoS 1
 *  - LWT конфигурация переведена на QoS 1
 * ========================================================================================
 */

#include <WiFi.h>
#include <ESPmDNS.h>
#include <WebServer.h>
#include <Update.h>
#include <Preferences.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <ModbusMaster.h>
#include <esp_task_wdt.h>
#include <QuickPID.h>

// НАТИВНЫЙ АСИНХРОННЫЙ MQTT ОТ ESPRESSIF
#include "mqtt_client.h"

#include "config.h"

// ================= ПРОФИЛИ ВЫХОДА (ИЗОЛИРОВАННЫЙ ВЫБОР) =================
enum OutputProfile {
  PROFILE_VALVE_4_20MA = 0,    // Заслонка / Кран (4..20 мА)
  PROFILE_DRY_COOLER_0_10V = 1 // Сухая градирня / Частотник (0..10 В)
};

OutputProfile currentProfile = PROFILE_VALVE_4_20MA;

// ================= ПИНЫ 74HC595 (РЕЛЕ) =================
constexpr uint8_t PIN_SER_74HC595   = 12;
constexpr uint8_t PIN_OE_74HC595    = 13;
constexpr uint8_t PIN_RCLK_74HC595  = 23;
constexpr uint8_t PIN_SRCLK_74HC595 = 22;

// ================= ПИНЫ 74HC165 (ВХОДЫ IN1..IN8) =================
constexpr uint8_t PIN_QH_74HC165  = 15;
constexpr uint8_t PIN_CLK_74HC165 = 2;
constexpr uint8_t PIN_SH_74HC165  = 0;

uint8_t lastInputStates = 0x00;
unsigned long lastInputScan = 0;
constexpr unsigned long INPUT_SCAN_INTERVAL = 50;

// ================= ПИНЫ DS18B20 =================
constexpr uint8_t TEMP_IN_PIN  = 19;
constexpr uint8_t TEMP_OUT_PIN = 18;
OneWire oneWireIn(TEMP_IN_PIN);
DallasTemperature sensorsIn(&oneWireIn);
OneWire oneWireOut(TEMP_OUT_PIN);
DallasTemperature sensorsOut(&oneWireOut);

float lastTempIn = -127.0f, lastTempOut = -127.0f;
float filteredTempOut = -127.0f; 
constexpr float EMA_ALPHA = 0.2f; 
bool tempInOnline = false, tempOutOnline = false;

// ================= ВЫХОД ЦАП (GPIO26 / DAC2) =================
constexpr uint8_t DAMPER_OUTPUT_PIN = 26;
float currentDamperPercent = 0.0f, targetDamperPercent = 0.0f; 

constexpr float DAMPER_STEP_PCT = 0.5f;
constexpr unsigned long DAMPER_RAMP_INTERVAL = 100;
unsigned long lastDamperRampTick = 0;

// ================= ПИД-РЕГУЛЯТОР И АВТОВОЗВРАТ =================
float pidSetpoint = 42.0f, pidInput = 0.0f, pidOutput = 0.0f;
float pidKp = 3.5f, pidKi = 0.05f, pidKd = 0.8f;
bool isPidEnabled = false, isPidInverted = true;

bool isForceManual = false; 
unsigned long manualOverrideStartTime = 0;
constexpr unsigned long PID_AUTO_REVERT_TIMEOUT = 15 * 60 * 1000UL; // 15 минут

QuickPID hydroPID(&pidInput, &pidOutput, &pidSetpoint, pidKp, pidKi, pidKd,
                 QuickPID::pMode::pOnError, QuickPID::dMode::dOnMeas,
                 QuickPID::iAwMode::iAwCondition, QuickPID::Action::reverse);
unsigned long lastPidCompute = 0;
constexpr unsigned long PID_COMPUTE_INTERVAL = 2000;
constexpr float PID_DEADBAND_PCT = 1.0f;

// ================= АНТИЗАЛИПАНИЕ (FSM) =================
unsigned long lastAntiStuckRun = 0;
constexpr unsigned long ANTI_STUCK_INTERVAL = 86400000UL; // 24 часа
bool isAntiStuckActive = false;

enum AntiStuckState {
  ANTI_STUCK_IDLE = 0,
  ANTI_STUCK_MOVE_100,
  ANTI_STUCK_WAIT_100,
  ANTI_STUCK_MOVE_0,
  ANTI_STUCK_WAIT_0
};

AntiStuckState antiStuckStep = ANTI_STUCK_IDLE;
unsigned long antiStuckStepTimer = 0;
constexpr unsigned long ANTI_STUCK_MOVE_TIME = 15000UL; // 15 секунд

// ================= MODBUS =================
constexpr uint8_t RS485_DIR_PIN = 21;
constexpr uint8_t PRESSURE_SENSOR_ADDR = 1;
ModbusMaster node;
float currentPressureBar = 0.0f;

void preTransmission() { digitalWrite(RS485_DIR_PIN, HIGH); delayMicroseconds(100); }
void postTransmission() { delayMicroseconds(100); digitalWrite(RS485_DIR_PIN, LOW); delayMicroseconds(100); }

// ================= СИСТЕМНЫЕ ФЛАГИ =================
bool isAutoMode = true;
uint8_t relayStateMask = 0x00;
bool ota_active = false;
bool startupComplete = false;

volatile bool isMqttConnected = false;
volatile bool needSendDiscovery = false;
volatile bool needPublishAllStates = false;

// Таймеры
unsigned long lastTempCheck = 0;
constexpr unsigned long TEMP_CHECK_INTERVAL = 2000;
unsigned long lastPressureCheck = 0;
constexpr unsigned long PRESSURE_CHECK_INTERVAL = 2000;
unsigned long lastRelayStatusPublish = 0;
constexpr unsigned long RELAY_STATUS_INTERVAL = 60000;
unsigned long lastTimerPublish = 0;
constexpr unsigned long TIMER_PUBLISH_INTERVAL = 10000; 

// ================= СЕТЕВЫЕ ОБЪЕКТЫ =================
esp_mqtt_client_handle_t mqtt_client = NULL;
WebServer server(80);
Preferences preferences;

// ================= ПРОТОТИПЫ =================
void sendByteRelay();
void setRelayChannel(uint8_t channel, bool state);
bool getRelayChannel(uint8_t channel);
void publishRelayState(uint8_t channel);
void publishMasterState();
void publishAllRelayStates();
void publishSystemMode();
void publishPidStatus();
void setDamperPercent(float percent, bool isManual = false);
void applyDamperDAC(float percent);
void sendHADiscovery();
void publishRevertTimer();
void publishActuatorMetrics(float percent);
void publishProfileStatus();
void processAntiStuck();
bool isMasterOn() { return (relayStateMask != 0x00); }

// ЕДИНАЯ ФУНКЦИЯ ПУБЛИКАЦИИ: QoS = 1, RETAIN = true (по умолчанию)
void mqttPublish(const char* topic, const char* payload, bool retain = true) {
  if (mqtt_client && isMqttConnected) {
    esp_mqtt_client_publish(mqtt_client, topic, payload, 0, 1, retain ? 1 : 0);
  }
}

// ЕДИНАЯ ФУНКЦИЯ ПОДПИСКИ: QoS = 1
void mqttSubscribe(const char* topic) {
  if (mqtt_client && isMqttConnected) {
    esp_mqtt_client_subscribe(mqtt_client, topic, 1);
  }
}

// ================= ПУБЛИКАЦИЯ СТАТУСА И МЕТРИК ВЫБРАННОГО РЕЖИМА =================
void publishProfileStatus() {
  if (!isMqttConnected) return;
  const char* profStr = (currentProfile == PROFILE_VALVE_4_20MA) ? "VALVE_4_20MA" : "DRY_COOLER_0_10V";
  mqttPublish("asic/profile/state", profStr, true);
}

void publishActuatorMetrics(float percent) {
  if (!isMqttConnected) return;

  char strBuf[16];
  if (currentProfile == PROFILE_VALVE_4_20MA) {
    float currentmA = 4.0f + ((percent / 100.0f) * 16.0f);
    snprintf(strBuf, sizeof(strBuf), "%.2f", currentmA);
    mqttPublish("asic/actuator/current_ma/state", strBuf, true);
  } else {
    float voltageV = (percent / 100.0f) * 10.0f;
    snprintf(strBuf, sizeof(strBuf), "%.2f", voltageV);
    mqttPublish("asic/actuator/voltage_v/state", strBuf, true);
  }
}

void publishRevertTimer() {
  if (!isMqttConnected) return;

  char strBuf[32];
  if (isPidEnabled) {
    snprintf(strBuf, sizeof(strBuf), "PID Active");
  } else if (isForceManual) {
    snprintf(strBuf, sizeof(strBuf), "Disabled (Force)");
  } else if (manualOverrideStartTime > 0) {
    unsigned long elapsed = millis() - manualOverrideStartTime; 
    if (elapsed < PID_AUTO_REVERT_TIMEOUT) {
      unsigned long remainingSec = (PID_AUTO_REVERT_TIMEOUT - elapsed) / 1000UL;
      snprintf(strBuf, sizeof(strBuf), "%lu m %lu s", remainingSec / 60UL, remainingSec % 60UL);
    } else {
      snprintf(strBuf, sizeof(strBuf), "Reverting...");
    }
  } else {
    snprintf(strBuf, sizeof(strBuf), "Manual");
  }

  mqttPublish("asic/pid/revert_timer/state", strBuf, true);
}

// ================= НАТИВНЫЙ ОБРАБОТЧИК MQTT =================
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data) {
  esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;
  
  switch ((esp_mqtt_event_id_t)event_id) {
    case MQTT_EVENT_CONNECTED:
      isMqttConnected = true;
      mqttPublish("asic/status", "online", true);

      mqttSubscribe("asic/master/set");
      mqttSubscribe("asic/relay1/set");
      mqttSubscribe("asic/relay2/set");
      mqttSubscribe("asic/relay3/set");
      mqttSubscribe("asic/relay4/set");
      mqttSubscribe("asic/pump/set");
      mqttSubscribe("asic/valve/set");
      mqttSubscribe("asic/relay7/set");
      mqttSubscribe("asic/relay8/set");
      mqttSubscribe("asic/damper/set");
      mqttSubscribe("asic/sensor/leak/set");
      mqttSubscribe("asic/mode/set");
      mqttSubscribe("asic/profile/set");
      mqttSubscribe("asic/ota");
      mqttSubscribe("asic/reset_nvs");
      mqttSubscribe("asic/pid/enable/set");
      mqttSubscribe("asic/pid/invert/set");
      mqttSubscribe("asic/pid/force_manual/set");
      mqttSubscribe("asic/pid/setpoint/set");
      mqttSubscribe("asic/pid/kp/set");
      mqttSubscribe("asic/pid/ki/set");
      mqttSubscribe("asic/pid/kd/set");

      needSendDiscovery = true;
      needPublishAllStates = true;
      break;

    case MQTT_EVENT_DISCONNECTED:
      isMqttConnected = false;
      break;

    case MQTT_EVENT_DATA: {
      if (ota_active) break;
      
      char topic[64] = {0};
      char message[64] = {0};

      int topic_len = event->topic_len < 63 ? event->topic_len : 63;
      int data_len = event->data_len < 63 ? event->data_len : 63;

      memcpy(topic, event->topic, topic_len);
      memcpy(message, event->data, data_len);

      bool isOn = (strcmp(message, "ON") == 0);

      if (strcmp(topic, "asic/profile/set") == 0) {
        if (strcmp(message, "VALVE_4_20MA") == 0) {
          currentProfile = PROFILE_VALVE_4_20MA;
        } else if (strcmp(message, "DRY_COOLER_0_10V") == 0) {
          currentProfile = PROFILE_DRY_COOLER_0_10V;
        }

        preferences.begin("asic_storage", false);
        preferences.putUChar("out_profile", (uint8_t)currentProfile);
        preferences.end();

        applyDamperDAC(currentDamperPercent);
        publishProfileStatus();
        publishActuatorMetrics(currentDamperPercent);
      }
      else if (strcmp(topic, "asic/pid/enable/set") == 0) {
        if (!isForceManual) {
          isPidEnabled = isOn;
          manualOverrideStartTime = 0;
          
          preferences.begin("asic_storage", false);
          preferences.putBool("pid_enable", isPidEnabled);
          preferences.end();

          hydroPID.SetMode(isPidEnabled ? QuickPID::Control::automatic : QuickPID::Control::manual);
          publishPidStatus();
          publishRevertTimer();
        }
      }
      else if (strcmp(topic, "asic/pid/force_manual/set") == 0) {
        isForceManual = isOn;
        if (isForceManual) {
          isPidEnabled = false;
          manualOverrideStartTime = 0;
          hydroPID.SetMode(QuickPID::Control::manual);
        }

        preferences.begin("asic_storage", false);
        preferences.putBool("pid_force_man", isForceManual);
        preferences.putBool("pid_enable", isPidEnabled);
        preferences.end();

        publishPidStatus();
        publishRevertTimer();
      }
      else if (strcmp(topic, "asic/pid/invert/set") == 0) {
        isPidInverted = isOn;
        hydroPID.SetControllerDirection(isPidInverted ? QuickPID::Action::reverse : QuickPID::Action::direct);
        publishPidStatus();
      }
      else if (strcmp(topic, "asic/pid/setpoint/set") == 0) {
        float val = atof(message);
        if (val >= 20.0f && val <= 85.0f) {
          pidSetpoint = val;
          publishPidStatus();
        }
      }
      else if (strcmp(topic, "asic/pid/kp/set") == 0) {
        pidKp = atof(message); hydroPID.SetTunings(pidKp, pidKi, pidKd); publishPidStatus();
      }
      else if (strcmp(topic, "asic/pid/ki/set") == 0) {
        pidKi = atof(message); hydroPID.SetTunings(pidKp, pidKi, pidKd); publishPidStatus();
      }
      else if (strcmp(topic, "asic/pid/kd/set") == 0) {
        pidKd = atof(message); hydroPID.SetTunings(pidKp, pidKi, pidKd); publishPidStatus();
      }
      else if (strcmp(topic, "asic/master/set") == 0) {
        if (!startupComplete) break;
        relayStateMask = isOn ? 0xFF : 0x00;
        sendByteRelay(); publishAllRelayStates();
      }
      else if (strcmp(topic, "asic/mode/set") == 0) {
        isAutoMode = (strcmp(message, "AUTO") == 0); publishSystemMode();
      }
      else if (strcmp(topic, "asic/relay1/set") == 0) setRelayChannel(1, isOn);
      else if (strcmp(topic, "asic/relay2/set") == 0) setRelayChannel(2, isOn);
      else if (strcmp(topic, "asic/relay3/set") == 0) setRelayChannel(3, isOn);
      else if (strcmp(topic, "asic/relay4/set") == 0) setRelayChannel(4, isOn);
      else if (strcmp(topic, "asic/pump/set") == 0) setRelayChannel(5, isOn);
      else if (strcmp(topic, "asic/valve/set") == 0) setRelayChannel(6, isOn);
      else if (strcmp(topic, "asic/relay7/set") == 0) setRelayChannel(7, isOn);
      else if (strcmp(topic, "asic/relay8/set") == 0) setRelayChannel(8, isOn);
      else if (strcmp(topic, "asic/damper/set") == 0) {
        setDamperPercent(atof(message), true);
      }
      else if (strcmp(topic, "asic/reset_nvs") == 0) {
        preferences.begin("asic_storage", false); preferences.clear(); preferences.end();
        relayStateMask = 0x00; isAutoMode = true; isPidEnabled = false; isForceManual = false; isPidInverted = true;
        currentProfile = PROFILE_VALVE_4_20MA;
        sendByteRelay(); publishAllRelayStates(); publishSystemMode(); publishPidStatus(); publishProfileStatus(); publishRevertTimer();
      }
      else if (strcmp(topic, "asic/ota") == 0 && strcmp(message, "update") == 0) {
        ota_active = true; relayStateMask = 0x00; sendByteRelay();
        if (mqtt_client) esp_mqtt_client_stop(mqtt_client);
      }
      break;
    }
    default:
      break;
  }
}

void initNativeMqtt() {
  char uri[128];
  snprintf(uri, sizeof(uri), "mqtt://%s:%d", mqtt_server, mqtt_port);

  esp_mqtt_client_config_t mqtt_cfg = {};
  mqtt_cfg.uri = uri;
  mqtt_cfg.username = mqtt_user;
  mqtt_cfg.password = mqtt_pass;
  mqtt_cfg.lwt_topic = "asic/status";
  mqtt_cfg.lwt_msg = "offline";
  mqtt_cfg.lwt_qos = 1; // QoS = 1 для LWT
  mqtt_cfg.lwt_retain = 1;

  mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
  esp_mqtt_client_register_event(mqtt_client, (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
  esp_mqtt_client_start(mqtt_client);
}

// ================= ЖЕЛЕЗО И РЕЛЕ =================
void sendByteRelay() {
  digitalWrite(PIN_RCLK_74HC595, LOW);
  for (int i = 0; i < 8; ++i) {
    digitalWrite(PIN_SER_74HC595, (relayStateMask & (1 << i)) ? HIGH : LOW);
    digitalWrite(PIN_SRCLK_74HC595, LOW); delayMicroseconds(1); digitalWrite(PIN_SRCLK_74HC595, HIGH);
  }
  digitalWrite(PIN_RCLK_74HC595, HIGH);
}

bool getRelayChannel(uint8_t channel) { return (relayStateMask & (1 << (8 - channel))) != 0; }

void setRelayChannel(uint8_t channel, bool state) {
  if (!startupComplete || channel < 1 || channel > 8) return;
  uint8_t bit = (1 << (8 - channel));
  if (((relayStateMask & bit) != 0) == state) return;
  
  if (state) relayStateMask |= bit; else relayStateMask &= ~bit;
  sendByteRelay(); publishRelayState(channel); publishMasterState();
}

void publishRelayState(uint8_t channel) {
  const char* payload = getRelayChannel(channel) ? "ON" : "OFF";
  char topic[32];
  if (channel <= 4) snprintf(topic, sizeof(topic), "asic/relay%d/state", channel);
  else if (channel == 5) snprintf(topic, sizeof(topic), "asic/pump/state");
  else if (channel == 6) snprintf(topic, sizeof(topic), "asic/valve/state");
  else if (channel == 7) snprintf(topic, sizeof(topic), "asic/relay7/state");
  else if (channel == 8) snprintf(topic, sizeof(topic), "asic/relay8/state");
  mqttPublish(topic, payload, true);
}

void publishMasterState() { mqttPublish("asic/master/state", isMasterOn() ? "ON" : "OFF", true); }
void publishSystemMode() { mqttPublish("asic/mode/state", isAutoMode ? "AUTO" : "MANUAL", true); }
void publishAllRelayStates() { for (int i = 1; i <= 8; i++) publishRelayState(i); publishMasterState(); }

// ================= ФИЗИЧЕСКИЕ ТУМБЛЕРЫ (IN1..IN8) =================
uint8_t readByteInputs() {
  uint8_t input_byte = 0x00;
  digitalWrite(PIN_SH_74HC165, LOW); delayMicroseconds(1); digitalWrite(PIN_SH_74HC165, HIGH); delayMicroseconds(1);
  for (int i = 0; i < 8; ++i) {
    if (digitalRead(PIN_QH_74HC165) == HIGH) input_byte |= (1 << i);
    digitalWrite(PIN_CLK_74HC165, LOW); delayMicroseconds(1); digitalWrite(PIN_CLK_74HC165, HIGH); delayMicroseconds(1);
  }
  return input_byte;
}

void handlePhysicalInputs() {
  if (millis() - lastInputScan < INPUT_SCAN_INTERVAL) return;
  lastInputScan = millis();

  uint8_t raw = readByteInputs();
  for (int i = 1; i <= 8; i++) {
    bool isPressed = !((raw & (1 << (7 - (i - 1)))) != 0);
    uint8_t bit = (1 << (i - 1));
    bool wasPressed = (lastInputStates & bit) != 0;
    
    if (isPressed != wasPressed) {
      if (isPressed) lastInputStates |= bit; else lastInputStates &= ~bit;
      setRelayChannel(i, isPressed);
    }
  }
}

// ================= ДАТЧИКИ И ИСПОЛНИТЕЛЬНЫЕ МЕХАНИЗМЫ =================
void setDamperPercent(float percent, bool isManual) {
  targetDamperPercent = constrain(percent, 0.0f, 100.0f);

  if (isManual) {
    if (isPidEnabled) {
      isPidEnabled = false;
      hydroPID.SetMode(QuickPID::Control::manual);
      manualOverrideStartTime = millis(); 
      
      preferences.begin("asic_storage", false);
      preferences.putBool("pid_enable", isPidEnabled);
      preferences.end();

      publishPidStatus();
      publishRevertTimer();
    }
  }

  char strBuf[16]; snprintf(strBuf, sizeof(strBuf), "%.1f", targetDamperPercent);
  mqttPublish("asic/damper/state", strBuf, true);
  publishActuatorMetrics(targetDamperPercent);
}

void applyDamperDAC(float percent) {
  percent = constrain(percent, 0.0f, 100.0f);
  currentDamperPercent = percent;

  uint8_t dacValue = 0;
  if (currentProfile == PROFILE_VALVE_4_20MA) {
    float currentmA = 4.0f + ((percent / 100.0f) * 16.0f);
    dacValue = (uint8_t)round((currentmA / 20.0f) * 255.0f);
  } else {
    dacValue = (uint8_t)round((percent / 100.0f) * 255.0f);
  }

  dacWrite(DAMPER_OUTPUT_PIN, dacValue);
}

void processDamperRamp() {
  unsigned long now = millis();
  if (now - lastDamperRampTick >= DAMPER_RAMP_INTERVAL) { 
    lastDamperRampTick = now;
    if (abs(currentDamperPercent - targetDamperPercent) > 0.05f) {
      currentDamperPercent += (currentDamperPercent < targetDamperPercent) ? DAMPER_STEP_PCT : -DAMPER_STEP_PCT;
      applyDamperDAC(currentDamperPercent);
      publishActuatorMetrics(currentDamperPercent);
    }
  }
}

void processAntiStuck() {
  unsigned long now = millis();

  if (!isAntiStuckActive && isAutoMode && isPidEnabled) {
    if (now - lastAntiStuckRun >= ANTI_STUCK_INTERVAL) {
      lastAntiStuckRun = now;
      isAntiStuckActive = true;
      antiStuckStep = ANTI_STUCK_MOVE_100;
      mqttPublish("asic/status/info", "Anti-Stuck cycle started", true);
    }
  }

  if (isAntiStuckActive) {
    switch (antiStuckStep) {
      case ANTI_STUCK_MOVE_100:
        setDamperPercent(100.0f, false);
        antiStuckStepTimer = now;
        antiStuckStep = ANTI_STUCK_WAIT_100;
        break;

      case ANTI_STUCK_WAIT_100:
        if (now - antiStuckStepTimer >= ANTI_STUCK_MOVE_TIME) {
          antiStuckStep = ANTI_STUCK_MOVE_0;
        }
        break;

      case ANTI_STUCK_MOVE_0:
        setDamperPercent(0.0f, false);
        antiStuckStepTimer = now;
        antiStuckStep = ANTI_STUCK_WAIT_0;
        break;

      case ANTI_STUCK_WAIT_0:
        if (now - antiStuckStepTimer >= ANTI_STUCK_MOVE_TIME) {
          isAntiStuckActive = false;
          antiStuckStep = ANTI_STUCK_IDLE;
          mqttPublish("asic/status/info", "Anti-Stuck cycle completed", true);
        }
        break;

      default:
        isAntiStuckActive = false;
        antiStuckStep = ANTI_STUCK_IDLE;
        break;
    }
  }
}

void readAndPublishPressure() {
  if (node.readHoldingRegisters(0x0004, 1) == node.ku8MBSuccess) {
    float kpa = (int16_t)node.getResponseBuffer(0) / 10.0f;
    currentPressureBar = kpa / 100.0f;
    char strBuf[16]; snprintf(strBuf, sizeof(strBuf), "%.2f", currentPressureBar);
    mqttPublish("asic/sensor/pressure/state", strBuf, true);
  }
}

void readAndPublishTemperatures() {
  sensorsIn.requestTemperatures(); 
  float tempIn = sensorsIn.getTempCByIndex(0);
  
  sensorsOut.requestTemperatures(); 
  float tempOut = sensorsOut.getTempCByIndex(0);
  
  char strBuf[16];
  if (tempIn > -55.0f && tempIn < 125.0f) {
    lastTempIn = tempIn;
    snprintf(strBuf, sizeof(strBuf), "%.1f", tempIn);
    mqttPublish("asic/sensor/temp_in/state", strBuf, true);
  }
  
  if (tempOut > -55.0f && tempOut < 125.0f) {
    lastTempOut = tempOut; 
    tempOutOnline = true;

    if (filteredTempOut <= -50.0f) {
      filteredTempOut = tempOut;
    } else {
      filteredTempOut = (EMA_ALPHA * tempOut) + ((1.0f - EMA_ALPHA) * filteredTempOut);
    }

    snprintf(strBuf, sizeof(strBuf), "%.1f", tempOut);
    mqttPublish("asic/sensor/temp_out/state", strBuf, true);
  } else { 
    tempOutOnline = false; 
  }
}

// ================= ЛОГИКА ПИД И АВТОВОЗВРАТА =================
void processPID() {
  unsigned long now = millis();

  if (!isPidEnabled && !isForceManual && manualOverrideStartTime > 0) {
    if (now - manualOverrideStartTime >= PID_AUTO_REVERT_TIMEOUT) {
      manualOverrideStartTime = 0;
      isPidEnabled = true; 
      
      preferences.begin("asic_storage", false);
      preferences.putBool("pid_enable", isPidEnabled);
      preferences.end();

      hydroPID.SetMode(QuickPID::Control::automatic);
      publishPidStatus();
      publishRevertTimer();
    }
  }

  if (!isPidEnabled || isAntiStuckActive) return;
  
  if (now - lastPidCompute >= PID_COMPUTE_INTERVAL) { 
    lastPidCompute = now;
    
    if (!tempOutOnline || filteredTempOut <= -50.0f) { 
      setDamperPercent(100.0, false); 
      return; 
    }
    
    pidInput = filteredTempOut;
    
    if (hydroPID.Compute()) {
      if (abs(pidOutput - targetDamperPercent) >= PID_DEADBAND_PCT) {
        setDamperPercent(pidOutput, false);
      }
    }
  }
}

void publishPidStatus() {
  char strBuf[16];
  mqttPublish("asic/pid/enable/state", isPidEnabled ? "ON" : "OFF", true);
  mqttPublish("asic/pid/force_manual/state", isForceManual ? "ON" : "OFF", true);
  mqttPublish("asic/pid/invert/state", isPidInverted ? "ON" : "OFF", true);
  snprintf(strBuf, sizeof(strBuf), "%.1f", pidSetpoint); mqttPublish("asic/pid/setpoint/state", strBuf, true);
  snprintf(strBuf, sizeof(strBuf), "%.2f", pidKp); mqttPublish("asic/pid/kp/state", strBuf, true);
  snprintf(strBuf, sizeof(strBuf), "%.2f", pidKi); mqttPublish("asic/pid/ki/state", strBuf, true);
  snprintf(strBuf, sizeof(strBuf), "%.2f", pidKd); mqttPublish("asic/pid/kd/state", strBuf, true);
}

// ================= HA DISCOVERY =================
void sendHADiscovery() {
  char payload[1024];  
  const char* dev_av = R"raw("device":{"identifiers":["esp32_asic_hydro_board"],"name":"ASIC Hydro Controller","manufacturer":"Eletechsup","model":"ES32D26","sw_version":"v3.8.2"},"availability":[{"topic":"asic/status"}])raw";
  #define PUB_DISC(comp, id, ...) snprintf(payload, sizeof(payload), "{%s,%s}", dev_av, (__VA_ARGS__)); mqttPublish("homeassistant/" comp "/asic_hydro/" id "/config", payload, true); delay(20);

  // 1. СИСТЕМНЫЕ ПЕРЕКЛЮЧАТЕЛИ И СЕЛЕКТОР ПРОФИЛЯ
  PUB_DISC("switch", "master", R"raw("name":"Master Switch","unique_id":"eh_master","state_topic":"asic/master/state","command_topic":"asic/master/set","icon":"mdi:power")raw");
  PUB_DISC("select", "cooling_profile", R"raw("name":"Cooling Equipment Type","unique_id":"eh_profile","state_topic":"asic/profile/state","command_topic":"asic/profile/set","options":["VALVE_4_20MA","DRY_COOLER_0_10V"],"icon":"mdi:fan-auto","entity_category":"config")raw");
  PUB_DISC("switch", "pid_en", R"raw("name":"PID Auto-Control","unique_id":"eh_pid_en","state_topic":"asic/pid/enable/state","command_topic":"asic/pid/enable/set","icon":"mdi:thermostat-auto")raw");
  PUB_DISC("switch", "force_manual", R"raw("name":"Force Manual Mode (Hold PID)","unique_id":"eh_force_man","state_topic":"asic/pid/force_manual/state","command_topic":"asic/pid/force_manual/set","icon":"mdi:hand-back-right","entity_category":"config")raw");
  PUB_DISC("switch", "pid_inv", R"raw("name":"PID Invert Direction","unique_id":"eh_pid_inv","state_topic":"asic/pid/invert/state","command_topic":"asic/pid/invert/set","icon":"mdi:swap-horizontal","entity_category":"config")raw");

  // 2. УСТАВКА И НАСТРОЙКИ ПИД
  PUB_DISC("number", "pid_sp", R"raw("name":"Target Temperature Tout","unique_id":"eh_pid_sp","state_topic":"asic/pid/setpoint/state","command_topic":"asic/pid/setpoint/set","min":20,"max":85,"step":0.5,"unit_of_measurement":"°C","icon":"mdi:target-account")raw");
  PUB_DISC("number", "pid_kp", R"raw("name":"PID Kp","unique_id":"eh_pid_kp","state_topic":"asic/pid/kp/state","command_topic":"asic/pid/kp/set","min":0,"max":50,"step":0.1,"entity_category":"config")raw");
  PUB_DISC("number", "pid_ki", R"raw("name":"PID Ki","unique_id":"eh_pid_ki","state_topic":"asic/pid/ki/state","command_topic":"asic/pid/ki/set","min":0,"max":10,"step":0.01,"entity_category":"config")raw");
  PUB_DISC("number", "pid_kd", R"raw("name":"PID Kd","unique_id":"eh_pid_kd","state_topic":"asic/pid/kd/state","command_topic":"asic/pid/kd/set","min":0,"max":50,"step":0.1,"entity_category":"config")raw");

  // 3. СЕНСОР ТАЙМЕРА АВТОВОЗВРАТА
  PUB_DISC("sensor", "pid_timer", R"raw("name":"PID Auto-Revert Countdown","unique_id":"eh_pid_timer","state_topic":"asic/pid/revert_timer/state","icon":"mdi:timer-sand")raw");

  // 4. ВСЕ 8 РЕЛЕ
  PUB_DISC("switch", "asic_1", R"raw("name":"ASIC 1","unique_id":"eh_r1","state_topic":"asic/relay1/state","command_topic":"asic/relay1/set","icon":"mdi:server")raw");
  PUB_DISC("switch", "asic_2", R"raw("name":"ASIC 2","unique_id":"eh_r2","state_topic":"asic/relay2/state","command_topic":"asic/relay2/set","icon":"mdi:server")raw");
  PUB_DISC("switch", "asic_3", R"raw("name":"ASIC 3","unique_id":"eh_r3","state_topic":"asic/relay3/state","command_topic":"asic/relay3/set","icon":"mdi:server")raw");
  PUB_DISC("switch", "asic_4", R"raw("name":"ASIC 4","unique_id":"eh_r4","state_topic":"asic/relay4/state","command_topic":"asic/relay4/set","icon":"mdi:server")raw");
  PUB_DISC("switch", "pump", R"raw("name":"Coolant Pump","unique_id":"eh_pump","state_topic":"asic/pump/state","command_topic":"asic/pump/set","icon":"mdi:pump")raw");
  PUB_DISC("switch", "valve", R"raw("name":"Heat Valve","unique_id":"eh_valve","state_topic":"asic/valve/state","command_topic":"asic/valve/set","icon":"mdi:pipe-valve")raw");
  PUB_DISC("switch", "relay_7", R"raw("name":"Aux Relay 7","unique_id":"eh_r7","state_topic":"asic/relay7/state","command_topic":"asic/relay7/set","icon":"mdi:toggle-switch")raw");
  PUB_DISC("switch", "relay_8", R"raw("name":"Aux Relay 8","unique_id":"eh_r8","state_topic":"asic/relay8/state","command_topic":"asic/relay8/set","icon":"mdi:toggle-switch")raw");

  // 5. СБРОС ПАМЯТИ
  PUB_DISC("button", "reset_nvs", R"raw("name":"Reset Memory (NVS)","unique_id":"eh_reset_nvs","command_topic":"asic/reset_nvs","payload_press":"RESET","icon":"mdi:restore","entity_category":"config")raw");

  // 6. ДАТЧИКИ И ВЫХОД ЦАП (СЕНСОРЫ мА ИЛИ В)
  PUB_DISC("number", "damper", R"raw("name":"Actuator Target Open","unique_id":"eh_damper","state_topic":"asic/damper/state","command_topic":"asic/damper/set","min":0,"max":100,"unit_of_measurement":"%","icon":"mdi:fan")raw");
  PUB_DISC("sensor", "actuator_ma", R"raw("name":"Valve Current Output (Io2)","unique_id":"eh_damper_ma","state_topic":"asic/actuator/current_ma/state","unit_of_measurement":"mA","icon":"mdi:current-ac")raw");
  PUB_DISC("sensor", "actuator_v", R"raw("name":"Dry Cooler Voltage Output (Vo2)","unique_id":"eh_damper_v","state_topic":"asic/actuator/voltage_v/state","unit_of_measurement":"V","icon":"mdi:sine-wave")raw");
  PUB_DISC("sensor", "press", R"raw("name":"Pressure","unique_id":"eh_press","state_topic":"asic/sensor/pressure/state","unit_of_measurement":"bar","device_class":"pressure")raw");
  PUB_DISC("sensor", "t_in", R"raw("name":"Temp IN","unique_id":"eh_tin","state_topic":"asic/sensor/temp_in/state","unit_of_measurement":"°C","device_class":"temperature")raw");
  PUB_DISC("sensor", "t_out", R"raw("name":"Temp OUT","unique_id":"eh_tout","state_topic":"asic/sensor/temp_out/state","unit_of_measurement":"°C","device_class":"temperature")raw");
  #undef PUB_DISC
}

// ================= WEB OTA =================
void setupWebOTA() {
  server.on("/", HTTP_GET, [](){
    if (!server.authenticate(http_username, http_password)) return server.requestAuthentication();
    const char html[] PROGMEM = R"rawliteral(<!DOCTYPE html><html><head><title>ASIC OTA</title><meta charset='utf-8'></head><body><h2>⚡ ASIC Hydro Controller (v3.8.2)</h2><form method='POST' action='/update' enctype='multipart/form-data'><input type='file' name='firmware' accept='.bin' required><button type='submit'>Upload</button></form></body></html>)rawliteral";
    server.send(200, "text/html", html);
  });
  
  server.on("/update", HTTP_POST, [](){
    if (!server.authenticate(http_username, http_password)) return server.requestAuthentication();
    server.send(200, "text/plain", "Update finished. Restarting...");
    delay(1000);
    ESP.restart();
  }, [](){
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      ota_active = true;
      relayStateMask = 0x00; sendByteRelay();
      if (mqtt_client) esp_mqtt_client_stop(mqtt_client);
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) Update.printError(Serial);
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) Update.printError(Serial);
    } else if (upload.status == UPLOAD_FILE_END) {
      if (!Update.end(true)) Update.printError(Serial);
    }
  });
  
  server.begin();
}

// ================= SETUP =================
void setup() {
  pinMode(PIN_OE_74HC595, OUTPUT); digitalWrite(PIN_OE_74HC595, HIGH);
  pinMode(PIN_SER_74HC595, OUTPUT); pinMode(PIN_RCLK_74HC595, OUTPUT); pinMode(PIN_SRCLK_74HC595, OUTPUT);
  pinMode(PIN_QH_74HC165, INPUT); pinMode(PIN_CLK_74HC165, OUTPUT); pinMode(PIN_SH_74HC165, OUTPUT);

  relayStateMask = 0x00; 
  sendByteRelay();

  delay(1000);

  esp_task_wdt_init(15, true); 
  esp_task_wdt_add(NULL);

  // СЧИТЫВАНИЕ НАСТРОЕК ИЗ NVS
  preferences.begin("asic_storage", true);
  relayStateMask = preferences.getUChar("relays", 0x00);
  isForceManual = preferences.getBool("pid_force_man", false);
  isPidEnabled = isForceManual ? false : preferences.getBool("pid_enable", false);
  pidSetpoint = preferences.getFloat("pid_setpoint", 42.0f);
  currentProfile = (OutputProfile)preferences.getUChar("out_profile", (uint8_t)PROFILE_VALVE_4_20MA);
  preferences.end();

  // БЕЗОПАСНЫЙ СТАРТ В ВЫБРАННОМ РЕЖИМЕ (4.00 мА или 0.00 В)
  applyDamperDAC(0.0f);
  currentDamperPercent = 0.0f;
  targetDamperPercent = 0.0f;

  hydroPID.SetOutputLimits(0, 100);
  hydroPID.SetSampleTimeUs(PID_COMPUTE_INTERVAL * 1000);
  hydroPID.SetTunings(pidKp, pidKi, pidKd);
  hydroPID.SetMode(isPidEnabled ? QuickPID::Control::automatic : QuickPID::Control::manual);

  sendByteRelay(); 
  digitalWrite(PIN_OE_74HC595, LOW);
  
  sensorsIn.begin(); 
  sensorsOut.begin();

  pinMode(RS485_DIR_PIN, OUTPUT); digitalWrite(RS485_DIR_PIN, LOW);
  Serial.begin(9600, SERIAL_8N1); node.begin(PRESSURE_SENSOR_ADDR, Serial);
  node.preTransmission(preTransmission); node.postTransmission(postTransmission);

  WiFi.mode(WIFI_STA);
  WiFi.setHostname(ota_hostname);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
    esp_task_wdt_reset();
  }

  MDNS.begin(ota_hostname);
  MDNS.addService("http", "tcp", 80);

  setupWebOTA();
  initNativeMqtt();
  
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

  unsigned long now = millis();

  if (needSendDiscovery) {
    sendHADiscovery();
    needSendDiscovery = false;
  }

  if (needPublishAllStates) {
    publishAllRelayStates();
    publishSystemMode();
    publishPidStatus();
    publishProfileStatus();
    publishRevertTimer();
    setDamperPercent(targetDamperPercent, false);
    needPublishAllStates = false;
  }

  handlePhysicalInputs();
  processDamperRamp();

  if (now - lastTempCheck >= TEMP_CHECK_INTERVAL) {
    readAndPublishTemperatures();
    lastTempCheck = now;
  }

  if (now - lastPressureCheck >= PRESSURE_CHECK_INTERVAL) {
    readAndPublishPressure();
    lastPressureCheck = now;
  }

  processAntiStuck();
  processPID();

  if (now - lastTimerPublish >= TIMER_PUBLISH_INTERVAL) {
    publishRevertTimer();
    lastTimerPublish = now;
  }

  if (isMqttConnected && now - lastRelayStatusPublish >= RELAY_STATUS_INTERVAL) {
    publishAllRelayStates();
    publishPidStatus();
    publishProfileStatus();
    lastRelayStatusPublish = now;
  }

  delay(1); 
}