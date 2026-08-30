/*
 * ========================================================================================
 * ⚡ ESP32 ASIC HYDRO CONTROLLER – FREERTOS STABLE (v4.3.2 Production Safe Edition)
 * ========================================================================================
 * Плата: Eletechsup ES32D26 (ESP32-DevKitC 38-PIN)
 * 
 * Особенности v4.3.2:
 *  - АППАРАТНЫЙ МЬЮТЕКС: Защита сдвигового регистра 74HC595 от гонки потоков (relayMutex).
 *  - ЗАЩИТА NVS (Wear-Leveling): Отложенное сохранение статусов реле (debounce 2 сек).
 *  - АВТО-РЕЖИМ (Auto Safe Boot): При старте асики принудительно OFF. После 5 сек 
 *    Self-Test при отсутствии аварий асики восстанавливаются из памяти.
 *  - РУЧНОЙ РЕЖИМ (Manual Mode): Мгновенное восстановление, отключение всех защит FSM.
 *  - ИЗОЛЯЦИЯ MODBUS: TaskModbus (Core 0, 2000 мс) + TaskControl (Core 1, 100 мс).
 *  - ДЕТЕКЦИЯ ВОЗДУХА: Буфер 10 точек + пауза 5 сек после пуска насоса + атомарный доступ.
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

// ================= МАСКИ КАНАЛОВ РЕЛЕ =================
constexpr uint8_t ASIC_MASK = 0xF0; // Каналы 1..4 (Биты 7..4: ASIC 1..4)
constexpr uint8_t AUX_MASK  = 0x0F; // Каналы 5..8 (Биты 3..0: Pump, Valve, Relay 7, Relay 8)

// ================= СОСТОЯНИЯ СИСТЕМЫ И АВАРИИ =================
enum class SystemState : uint8_t {
  BOOT = 0,
  SELF_TEST,
  SAFE_STANDBY,
  RUNNING_AUTO,
  MANUAL,
  FAULT_FAILSAFE,
  EMERGENCY_SHUTDOWN
};

enum class FaultCode : uint8_t {
  NONE = 0,
  TEMP_OUT_LOST,
  TEMP_IN_LOST,
  PRESSURE_TIMEOUT,
  PRESSURE_OUT_OF_BOUNDS,
  AIR_IN_SYSTEM
};

SystemState currentSystemState = SystemState::BOOT;
FaultCode currentFault = FaultCode::NONE;

// ================= ПРОФИЛИ ВЫХОДА =================
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
uint8_t debouncedInputStates = 0x00;
unsigned long lastDebounceTime[8] = {0};
constexpr unsigned long DEBOUNCE_DELAY_MS = 50;

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
bool isTempConversionPending = false;
unsigned long tempConversionStartTime = 0;

// ================= ВЫХОД ЦАП (GPIO26 / DAC2) =================
constexpr uint8_t DAMPER_OUTPUT_PIN = 26;
float currentDamperPercent = 0.0f, targetDamperPercent = 0.0f; 

constexpr float DAMPER_STEP_PCT = 0.5f; 

struct DamperCommand {
  float percent;
  bool isManual;
};

// ================= ПИД-РЕГУЛЯТОР И АВТОВОЗВРАТ =================
float pidSetpoint = 42.0f, pidInput = 0.0f, pidOutput = 0.0f;
float pidKp = 3.5f, pidKi = 0.05f, pidKd = 0.8f; 
bool isPidEnabled = false, isPidInverted = true; 

bool isForceManual = false; 
unsigned long manualOverrideStartTime = 0;
constexpr unsigned long PID_AUTO_REVERT_TIMEOUT = 15 * 60 * 1000UL;

QuickPID hydroPID(&pidInput, &pidOutput, &pidSetpoint, pidKp, pidKi, pidKd,
                 QuickPID::pMode::pOnError, QuickPID::dMode::dOnMeas,
                 QuickPID::iAwMode::iAwCondition, QuickPID::Action::reverse);

unsigned long lastPidCompute = 0;
constexpr unsigned long PID_COMPUTE_INTERVAL = 2000;
constexpr float PID_DEADBAND_PCT = 1.0f;

// ================= АНТИЗАЛИПАНИЕ (FSM) =================
unsigned long lastAntiStuckRun = 0;
constexpr unsigned long ANTI_STUCK_INTERVAL = 86400000UL;
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
constexpr unsigned long ANTI_STUCK_MOVE_TIME = 15000UL;

// ================= MODBUS И ДЕТЕКЦИЯ ВОЗДУХА =================
constexpr uint8_t RS485_DIR_PIN = 21;
constexpr uint8_t PRESSURE_SENSOR_ADDR = 1;
ModbusMaster node;
portMUX_TYPE pressureMux = portMUX_INITIALIZER_UNLOCKED;

float currentPressureBar = 0.0f;
bool isPressureOnline = false;
unsigned long lastPressureSuccessTime = 0;
constexpr unsigned long PRESSURE_TIMEOUT_MS = 5000;

constexpr int PRESSURE_SAMPLES = 10;
float pressureHistory[PRESSURE_SAMPLES] = {0};
int pressureSampleIndex = 0;
int pressureSampleCount = 0;
bool isAirDetected = false;
bool lastPumpState = false;
unsigned long pumpStartTime = 0;
constexpr unsigned long PUMP_STABILIZATION_MS = 5000;

void preTransmission() { digitalWrite(RS485_DIR_PIN, HIGH); delayMicroseconds(100); }
void postTransmission() { delayMicroseconds(100); digitalWrite(RS485_DIR_PIN, LOW); delayMicroseconds(100); }

// ================= ТАЙМЕРЫ И ИНТЕРВАЛЫ =================
constexpr unsigned long TIMER_PUBLISH_INTERVAL = 10000;
constexpr unsigned long RELAY_STATUS_INTERVAL = 60000;

// ================= СИСТЕМНЫЕ ФЛАГИ И МЬЮТЕКСЫ =================
bool isAutoMode = true;
uint8_t relayStateMask = 0x00;
uint8_t savedRelaysMask = 0x00;       
bool selfTestComplete = false;        
unsigned long selfTestStartTime = 0;
constexpr unsigned long SELF_TEST_DURATION = 5000; 

bool ota_active = false;
bool startupComplete = false;

volatile bool isMqttConnected = false;
volatile bool needSendDiscovery = false;
volatile bool needPublishAllStates = false;

// Отложенное сохранение NVS (защита от износа)
bool needSaveRelays = false;
unsigned long lastRelayChangeTime = 0;

// FREERTOS ОБЪЕКТЫ
QueueHandle_t damperCmdQueue = NULL;
SemaphoreHandle_t relayMutex = NULL; // Мьютекс для шины 74HC595

// СЕТЕВЫЕ ОБЪЕКТЫ
esp_mqtt_client_handle_t mqtt_client = NULL;
WebServer server(80);
Preferences preferences;

// Статический буфер для HA Discovery
static char discPayloadBuf[1024];

// ================= ПРОТОТИПЫ =================
void sendByteRelay();
void setRelayChannel(uint8_t channel, bool state);
bool getRelayChannel(uint8_t channel);
void publishRelayState(uint8_t channel);
void publishMasterState();
void publishAllRelayStates();
void publishSystemMode();
void publishPidStatus();
void publishSafetyStatus();
void setDamperPercent(float percent, bool isManual = false);
void applyDamperDAC(float percent);
void sendHADiscovery();
void publishRevertTimer();
void publishActuatorMetrics(float percent);
void publishProfileStatus();
void processAntiStuck();
void evaluateSafety();
void saveRelaysToNVS();
void saveSystemModeToNVS();
uint8_t readByteInputs();
bool isMasterOn() { return (relayStateMask != 0x00); }

// ЕДИНАЯ ФУНКЦИЯ ПУБЛИКАЦИИ
void mqttPublish(const char* topic, const char* payload, bool retain = true) {
  if (mqtt_client && isMqttConnected) {
    esp_mqtt_client_publish(mqtt_client, topic, payload, 0, 1, retain ? 1 : 0);
  }
}

// ЕДИНАЯ ФУНКЦИЯ ПОДПИСКИ
void mqttSubscribe(const char* topic) {
  if (mqtt_client && isMqttConnected) {
    esp_mqtt_client_subscribe(mqtt_client, topic, 1);
  }
}

void saveRelaysToNVS() {
  preferences.begin("asic_storage", false);
  preferences.putUChar("relays", relayStateMask);
  savedRelaysMask = relayStateMask;
  preferences.end();
}

void saveSystemModeToNVS() {
  preferences.begin("asic_storage", false);
  preferences.putBool("auto_mode", isAutoMode);
  preferences.end();
}

// ================= ПУБЛИКАЦИЯ СТАТУСА И МЕТРИК =================
void publishSafetyStatus() {
  if (!isMqttConnected) return;

  const char* stateStr = "UNKNOWN";
  switch (currentSystemState) {
    case SystemState::BOOT:               stateStr = "BOOT"; break;
    case SystemState::SELF_TEST:          stateStr = "SELF_TEST"; break;
    case SystemState::SAFE_STANDBY:       stateStr = "SAFE_STANDBY"; break;
    case SystemState::RUNNING_AUTO:       stateStr = "RUNNING_AUTO"; break;
    case SystemState::MANUAL:             stateStr = "MANUAL"; break;
    case SystemState::FAULT_FAILSAFE:     stateStr = "FAULT_FAILSAFE"; break;
    case SystemState::EMERGENCY_SHUTDOWN: stateStr = "EMERGENCY_SHUTDOWN"; break;
  }
  mqttPublish("asic/state", stateStr, true);

  const char* faultStr = "NONE";
  switch (currentFault) {
    case FaultCode::NONE:                   faultStr = "NONE"; break;
    case FaultCode::TEMP_OUT_LOST:          faultStr = "TEMP_OUT_LOST"; break;
    case FaultCode::TEMP_IN_LOST:           faultStr = "TEMP_IN_LOST"; break;
    case FaultCode::PRESSURE_TIMEOUT:       faultStr = "PRESSURE_TIMEOUT"; break;
    case FaultCode::PRESSURE_OUT_OF_BOUNDS: faultStr = "PRESSURE_OUT_OF_BOUNDS"; break;
    case FaultCode::AIR_IN_SYSTEM:          faultStr = "AIR_IN_SYSTEM"; break;
  }
  mqttPublish("asic/fault", faultStr, true);

  bool airState;
  portENTER_CRITICAL(&pressureMux);
  airState = isAirDetected;
  portEXIT_CRITICAL(&pressureMux);
  mqttPublish("asic/sensor/air/state", airState ? "ON" : "OFF", true);
}

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
        
        preferences.begin("asic_storage", false);
        preferences.putBool("pid_inv", isPidInverted);
        preferences.end();

        publishPidStatus();
      }
      else if (strcmp(topic, "asic/pid/setpoint/set") == 0) {
        float val = atof(message);
        if (val >= 20.0f && val <= 85.0f) {
          pidSetpoint = val;
          preferences.begin("asic_storage", false);
          preferences.putFloat("pid_setpoint", pidSetpoint);
          preferences.end();
          publishPidStatus();
        }
      }
      else if (strcmp(topic, "asic/pid/kp/set") == 0) {
        pidKp = atof(message);
        hydroPID.SetTunings(pidKp, pidKi, pidKd);
        preferences.begin("asic_storage", false);
        preferences.putFloat("pid_kp", pidKp);
        preferences.end();
        publishPidStatus();
      }
      else if (strcmp(topic, "asic/pid/ki/set") == 0) {
        pidKi = atof(message);
        hydroPID.SetTunings(pidKp, pidKi, pidKd);
        preferences.begin("asic_storage", false);
        preferences.putFloat("pid_ki", pidKi);
        preferences.end();
        publishPidStatus();
      }
      else if (strcmp(topic, "asic/pid/kd/set") == 0) {
        pidKd = atof(message);
        hydroPID.SetTunings(pidKp, pidKi, pidKd);
        preferences.begin("asic_storage", false);
        preferences.putFloat("pid_kd", pidKd);
        preferences.end();
        publishPidStatus();
      }
      else if (strcmp(topic, "asic/master/set") == 0) {
        if (!startupComplete) break;
        relayStateMask = isOn ? 0xFF : 0x00;
        sendByteRelay(); 
        
        needSaveRelays = true;
        lastRelayChangeTime = millis();
        publishAllRelayStates();
      }
      else if (strcmp(topic, "asic/mode/set") == 0) {
        isAutoMode = (strcmp(message, "AUTO") == 0);
        saveSystemModeToNVS();
        publishSystemMode();
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
        float val = (float)atof(message);
        DamperCommand cmd = { val, true };
        xQueueSend(damperCmdQueue, &cmd, 0);
      }
      else if (strcmp(topic, "asic/reset_nvs") == 0) {
        preferences.begin("asic_storage", false); preferences.clear(); preferences.end();
        relayStateMask = 0x00; isAutoMode = true; isPidEnabled = false; isForceManual = false; isPidInverted = true;
        currentProfile = PROFILE_VALVE_4_20MA;
        sendByteRelay(); publishAllRelayStates(); publishSystemMode(); publishPidStatus(); publishProfileStatus(); publishRevertTimer();
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
  mqtt_cfg.lwt_qos = 1;
  mqtt_cfg.lwt_retain = 1;

  mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
  esp_mqtt_client_register_event(mqtt_client, (esp_mqtt_event_id_t)ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
  esp_mqtt_client_start(mqtt_client);
}

// ================= ЖЕЛЕЗО И РЕЛЕ =================
void sendByteRelay() {
  if (relayMutex != NULL) {
    if (xSemaphoreTake(relayMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
      digitalWrite(PIN_RCLK_74HC595, LOW);
      for (int i = 0; i < 8; ++i) {
        digitalWrite(PIN_SER_74HC595, (relayStateMask & (1 << i)) ? HIGH : LOW);
        digitalWrite(PIN_SRCLK_74HC595, LOW); delayMicroseconds(1); digitalWrite(PIN_SRCLK_74HC595, HIGH);
      }
      digitalWrite(PIN_RCLK_74HC595, HIGH);
      xSemaphoreGive(relayMutex);
    }
  }
}

bool getRelayChannel(uint8_t channel) { return (relayStateMask & (1 << (8 - channel))) != 0; }

void setRelayChannel(uint8_t channel, bool state) {
  if (!startupComplete || channel < 1 || channel > 8) return;
  uint8_t bit = (1 << (8 - channel));
  if (((relayStateMask & bit) != 0) == state) return;
  
  if (state) relayStateMask |= bit; else relayStateMask &= ~bit;
  sendByteRelay(); 
  
  // Отложенное сохранение (защита NVS)
  needSaveRelays = true;
  lastRelayChangeTime = millis();
  
  publishRelayState(channel); 
  publishMasterState();
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

// ================= ФИЗИЧЕСКИЕ ТУМБЛЕРЫ (IN1..IN8) С ДЕБАУНСОМ =================
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
  uint8_t raw = readByteInputs();
  unsigned long now = millis();

  for (int i = 1; i <= 8; i++) {
    bool rawReading = !((raw & (1 << (7 - (i - 1)))) != 0);
    uint8_t bit = (1 << (i - 1));
    bool lastReading = (lastInputStates & bit) != 0;

    if (rawReading != lastReading) {
      lastDebounceTime[i - 1] = now;
      if (rawReading) lastInputStates |= bit; else lastInputStates &= ~bit;
    }

    if ((now - lastDebounceTime[i - 1]) > DEBOUNCE_DELAY_MS) {
      bool debouncedState = (debouncedInputStates & bit) != 0;
      if (rawReading != debouncedState) {
        if (rawReading) debouncedInputStates |= bit; else debouncedInputStates &= ~bit;
        
        // Инвертируем текущее состояние реле (Toggle)
        bool currentRelayState = getRelayChannel(i);
        setRelayChannel(i, !currentRelayState);
      }
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
  if (abs(currentDamperPercent - targetDamperPercent) > 0.05f) {
    currentDamperPercent += (currentDamperPercent < targetDamperPercent) ? DAMPER_STEP_PCT : -DAMPER_STEP_PCT;
    applyDamperDAC(currentDamperPercent);
    publishActuatorMetrics(currentDamperPercent);
  }
}

void processAntiStuck() {
  unsigned long now = millis();

  // Антизалипание работает только в автоматическом режиме регулирования
  if (!isAntiStuckActive && isAutoMode && isPidEnabled && currentSystemState == SystemState::RUNNING_AUTO) {
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

// ================= ДЕТЕКЦИЯ ВОЗДУХА =================
void resetAirDetectionBuffer() {
  pressureSampleIndex = 0;
  pressureSampleCount = 0;
  
  portENTER_CRITICAL(&pressureMux);
  isAirDetected = false;
  portEXIT_CRITICAL(&pressureMux);
  
  for (int i = 0; i < PRESSURE_SAMPLES; i++) {
    pressureHistory[i] = 0.0f;
  }
}

void checkAirInSystem(float newPressure) {
  bool currentPumpState = getRelayChannel(5);

  if (currentPumpState != lastPumpState) {
    lastPumpState = currentPumpState;
    resetAirDetectionBuffer();
    if (currentPumpState) {
      pumpStartTime = millis();
    }
  }

  if (!currentPumpState) {
    portENTER_CRITICAL(&pressureMux);
    isAirDetected = false;
    portEXIT_CRITICAL(&pressureMux);
    return;
  }

  if (millis() - pumpStartTime < PUMP_STABILIZATION_MS) {
    return;
  }

  pressureHistory[pressureSampleIndex] = newPressure;
  pressureSampleIndex = (pressureSampleIndex + 1) % PRESSURE_SAMPLES;
  
  if (pressureSampleCount < PRESSURE_SAMPLES) {
    pressureSampleCount++;
    return;
  }

  float sum = 0;
  for (int i = 0; i < PRESSURE_SAMPLES; i++) {
    sum += pressureHistory[i];
  }
  float mean = sum / (float)PRESSURE_SAMPLES;

  float variance = 0;
  for (int i = 0; i < PRESSURE_SAMPLES; i++) {
    variance += pow(pressureHistory[i] - mean, 2);
  }
  float stdDev = sqrt(variance / (float)PRESSURE_SAMPLES);

  bool detected = (stdDev > 0.08f);
  
  portENTER_CRITICAL(&pressureMux);
  isAirDetected = detected;
  portEXIT_CRITICAL(&pressureMux);
}

// ================= FREERTOS TASK 2: MODBUS ISOLATED (CORE 0) =================
void TaskModbus(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(2000); // Опрос раз в 2 секунды

  for (;;) {
    if (!ota_active) {
      if (node.readHoldingRegisters(0x0004, 1) == node.ku8MBSuccess) {
        float kpa = (int16_t)node.getResponseBuffer(0) / 10.0f;
        float localPressure = kpa / 100.0f;
        
        checkAirInSystem(localPressure);

        portENTER_CRITICAL(&pressureMux);
        currentPressureBar = localPressure;
        lastPressureSuccessTime = millis();
        isPressureOnline = true;
        bool localAirState = isAirDetected;
        portEXIT_CRITICAL(&pressureMux);

        char strBuf[16]; 
        snprintf(strBuf, sizeof(strBuf), "%.2f", localPressure);
        mqttPublish("asic/sensor/pressure/state", strBuf, true);
        mqttPublish("asic/sensor/air/state", localAirState ? "ON" : "OFF", true);
      } else {
        portENTER_CRITICAL(&pressureMux);
        if (millis() - lastPressureSuccessTime > PRESSURE_TIMEOUT_MS) {
          isPressureOnline = false;
        }
        portEXIT_CRITICAL(&pressureMux);
      }
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

void processTemperaturesAsync() {
  unsigned long now = millis();

  if (!isTempConversionPending) {
    sensorsIn.requestTemperatures();
    sensorsOut.requestTemperatures();
    isTempConversionPending = true;
    tempConversionStartTime = now;
  } else {
    if (now - tempConversionStartTime >= 750) {
      isTempConversionPending = false;

      float tempIn = sensorsIn.getTempCByIndex(0);
      float tempOut = sensorsOut.getTempCByIndex(0);

      char strBuf[16];
      if (tempIn > -55.0f && tempIn < 125.0f) {
        lastTempIn = tempIn;
        tempInOnline = true;
        snprintf(strBuf, sizeof(strBuf), "%.1f", tempIn);
        mqttPublish("asic/sensor/temp_in/state", strBuf, true);
      } else {
        tempInOnline = false;
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
  }
}

// ================= SAFETY MANAGER & POLICY ENGINE =================
void evaluateSafety() {
  if (!startupComplete) return;

  // 1. РУЧНОЙ РЕЖИМ (Полное отключение проверок и невмешательство Safety Manager)
  if (!isAutoMode) {
    currentSystemState = SystemState::MANUAL;
    currentFault = FaultCode::NONE;
    return;
  }

  // 2. АВТО-РЕЖИМ: Фаза Self-Test при старте
  if (!selfTestComplete) {
    currentSystemState = SystemState::SELF_TEST;
    if (millis() - selfTestStartTime < SELF_TEST_DURATION) {
      return; // Ждем 5 сек самодиагностики
    }

    // Проверяем статус датчиков
    bool pressOk;
    portENTER_CRITICAL(&pressureMux);
    pressOk = isPressureOnline;
    portEXIT_CRITICAL(&pressureMux);

    if (tempOutOnline && filteredTempOut > -50.0f && filteredTempOut <= 115.0f && pressOk) {
      selfTestComplete = true;
      // Восстанавливаем сохраненное состояние всех реле (включая асики 1..4)
      relayStateMask = savedRelaysMask;
      sendByteRelay();
      publishAllRelayStates();
    } else {
      // Ошибка датчиков при старте: асики остаются OFF, заслонка 100%
      currentSystemState = SystemState::FAULT_FAILSAFE;
      currentFault = (!tempOutOnline) ? FaultCode::TEMP_OUT_LOST : FaultCode::PRESSURE_TIMEOUT;
      setDamperPercent(100.0f, false);
      return;
    }
  }

  // 3. АВТО-РЕЖИМ: Мониторинг в реальном времени
  bool localPressOnline;
  bool localAirDetected;
  portENTER_CRITICAL(&pressureMux);
  localPressOnline = isPressureOnline;
  localAirDetected = isAirDetected;
  portEXIT_CRITICAL(&pressureMux);

  if (!tempOutOnline || filteredTempOut <= -50.0f || filteredTempOut > 115.0f) {
    currentSystemState = SystemState::FAULT_FAILSAFE;
    currentFault = FaultCode::TEMP_OUT_LOST;
    setDamperPercent(100.0f, false);
    return;
  }

  if (!localPressOnline) {
    currentSystemState = SystemState::FAULT_FAILSAFE;
    currentFault = FaultCode::PRESSURE_TIMEOUT;
    setDamperPercent(100.0f, false);
    return;
  }

  if (localAirDetected) {
    currentFault = FaultCode::AIR_IN_SYSTEM;
  } else if (currentFault == FaultCode::AIR_IN_SYSTEM) {
    currentFault = FaultCode::NONE;
  }

  // Штатный сброс критических аварий
  if (currentFault == FaultCode::TEMP_OUT_LOST || currentFault == FaultCode::PRESSURE_TIMEOUT) {
    currentFault = FaultCode::NONE;
  }

  if (isForceManual) {
    currentSystemState = SystemState::MANUAL;
  } else if (isPidEnabled) {
    currentSystemState = SystemState::RUNNING_AUTO;
  } else {
    currentSystemState = SystemState::SAFE_STANDBY;
  }
}

// ================= ЛОГИКА ПИД =================
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
  if (isAutoMode && currentSystemState != SystemState::RUNNING_AUTO) return;
  
  if (now - lastPidCompute >= PID_COMPUTE_INTERVAL) { 
    lastPidCompute = now;
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
  const char* dev_av = R"raw("device":{"identifiers":["esp32_asic_hydro_board"],"name":"ASIC Hydro Controller","manufacturer":"Eletechsup","model":"ES32D26","sw_version":"v4.3.2(FreeRTOS)"},"availability":[{"topic":"asic/status"}])raw";
  #define PUB_DISC(comp, id, ...) snprintf(discPayloadBuf, sizeof(discPayloadBuf), "{%s,%s}", dev_av, (__VA_ARGS__)); mqttPublish("homeassistant/" comp "/asic_hydro/" id "/config", discPayloadBuf, true); delay(20);

  // 1. Системные FSM сенсоры
  PUB_DISC("sensor", "sys_state", R"raw("name":"System State","unique_id":"eh_state","state_topic":"asic/state","icon":"mdi:state-machine")raw");
  PUB_DISC("sensor", "sys_fault", R"raw("name":"Fault Code","unique_id":"eh_fault","state_topic":"asic/fault","icon":"mdi:alert-circle")raw");

  // 2. Детекция завоздушивания (Binary Sensor)
  PUB_DISC("binary_sensor", "air_detected", R"raw("name":"Air In Loop Detected","unique_id":"eh_air_det","state_topic":"asic/sensor/air/state","payload_on":"ON","payload_off":"OFF","device_class":"problem","icon":"mdi:air-filter")raw");

  // 3. Управление питанием и режимами
  PUB_DISC("switch", "master", R"raw("name":"Master Switch","unique_id":"eh_master","state_topic":"asic/master/state","command_topic":"asic/master/set","icon":"mdi:power")raw");
  PUB_DISC("select", "cooling_profile", R"raw("name":"Cooling Equipment Type","unique_id":"eh_profile","state_topic":"asic/profile/state","command_topic":"asic/profile/set","options":["VALVE_4_20MA","DRY_COOLER_0_10V"],"icon":"mdi:fan-auto","entity_category":"config")raw");
  PUB_DISC("select", "sys_mode", R"raw("name":"System Operation Mode","unique_id":"eh_mode","state_topic":"asic/mode/state","command_topic":"asic/mode/set","options":["AUTO","MANUAL"],"icon":"mdi:cog-sync")raw");
  PUB_DISC("switch", "pid_en", R"raw("name":"PID Auto-Control","unique_id":"eh_pid_en","state_topic":"asic/pid/enable/state","command_topic":"asic/pid/enable/set","icon":"mdi:thermostat-auto")raw");
  PUB_DISC("switch", "force_manual", R"raw("name":"Force Manual Mode (Hold PID)","unique_id":"eh_force_man","state_topic":"asic/pid/force_manual/state","command_topic":"asic/pid/force_manual/set","icon":"mdi:hand-back-right","entity_category":"config")raw");
  PUB_DISC("switch", "pid_inv", R"raw("name":"PID Invert Direction","unique_id":"eh_pid_inv","state_topic":"asic/pid/invert/state","command_topic":"asic/pid/invert/set","icon":"mdi:swap-horizontal","entity_category":"config")raw");

  // 4. Параметры ПИД
  PUB_DISC("number", "pid_sp", R"raw("name":"Target Temperature Tout","unique_id":"eh_pid_sp","state_topic":"asic/pid/setpoint/state","command_topic":"asic/pid/setpoint/set","min":20,"max":85,"step":0.5,"unit_of_measurement":"°C","icon":"mdi:target-account")raw");
  PUB_DISC("number", "pid_kp", R"raw("name":"PID Kp","unique_id":"eh_pid_kp","state_topic":"asic/pid/kp/state","command_topic":"asic/pid/kp/set","min":0,"max":50,"step":0.1,"entity_category":"config")raw");
  PUB_DISC("number", "pid_ki", R"raw("name":"PID Ki","unique_id":"eh_pid_ki","state_topic":"asic/pid/ki/state","command_topic":"asic/pid/ki/set","min":0,"max":10,"step":0.01,"entity_category":"config")raw");
  PUB_DISC("number", "pid_kd", R"raw("name":"PID Kd","unique_id":"eh_pid_kd","state_topic":"asic/pid/kd/state","command_topic":"asic/pid/kd/set","min":0,"max":50,"step":0.1,"entity_category":"config")raw");

  PUB_DISC("sensor", "pid_timer", R"raw("name":"PID Auto-Revert Countdown","unique_id":"eh_pid_timer","state_topic":"asic/pid/revert_timer/state","icon":"mdi:timer-sand")raw");

  // 5. Силовые реле
  PUB_DISC("switch", "asic_1", R"raw("name":"ASIC 1","unique_id":"eh_r1","state_topic":"asic/relay1/state","command_topic":"asic/relay1/set","icon":"mdi:server")raw");
  PUB_DISC("switch", "asic_2", R"raw("name":"ASIC 2","unique_id":"eh_r2","state_topic":"asic/relay2/state","command_topic":"asic/relay2/set","icon":"mdi:server")raw");
  PUB_DISC("switch", "asic_3", R"raw("name":"ASIC 3","unique_id":"eh_r3","state_topic":"asic/relay3/state","command_topic":"asic/relay3/set","icon":"mdi:server")raw");
  PUB_DISC("switch", "asic_4", R"raw("name":"ASIC 4","unique_id":"eh_r4","state_topic":"asic/relay4/state","command_topic":"asic/relay4/set","icon":"mdi:server")raw");
  PUB_DISC("switch", "pump", R"raw("name":"Coolant Pump","unique_id":"eh_pump","state_topic":"asic/pump/state","command_topic":"asic/pump/set","icon":"mdi:pump")raw");
  PUB_DISC("switch", "valve", R"raw("name":"Heat Valve","unique_id":"eh_valve","state_topic":"asic/valve/state","command_topic":"asic/valve/set","icon":"mdi:pipe-valve")raw");
  PUB_DISC("switch", "relay_7", R"raw("name":"Aux Relay 7","unique_id":"eh_r7","state_topic":"asic/relay7/state","command_topic":"asic/relay7/set","icon":"mdi:toggle-switch")raw");
  PUB_DISC("switch", "relay_8", R"raw("name":"Aux Relay 8","unique_id":"eh_relay8","state_topic":"asic/relay8/state","command_topic":"asic/relay8/set","icon":"mdi:toggle-switch")raw");

  PUB_DISC("button", "reset_nvs", R"raw("name":"Reset Memory (NVS)","unique_id":"eh_reset_nvs","command_topic":"asic/reset_nvs","payload_press":"RESET","icon":"mdi:restore","entity_category":"config")raw");

  // 6. Датчики и метрики
  PUB_DISC("number", "damper", R"raw("name":"Actuator Target Open","unique_id":"eh_damper","state_topic":"asic/damper/state","command_topic":"asic/damper/set","min":0,"max":100,"unit_of_measurement":"%","icon":"mdi:fan")raw");
  PUB_DISC("sensor", "actuator_ma", R"raw("name":"Valve Current Output (Io2)","unique_id":"eh_actuator_ma","state_topic":"asic/actuator/current_ma/state","unit_of_measurement":"mA","icon":"mdi:current-ac")raw");
  PUB_DISC("sensor", "actuator_v", R"raw("name":"Dry Cooler Voltage Output (Vo2)","unique_id":"eh_actuator_v","state_topic":"asic/actuator/voltage_v/state","unit_of_measurement":"V","icon":"mdi:sine-wave")raw");
  PUB_DISC("sensor", "press", R"raw("name":"Pressure","unique_id":"eh_press","state_topic":"asic/sensor/pressure/state","unit_of_measurement":"bar","device_class":"pressure")raw");
  PUB_DISC("sensor", "t_in", R"raw("name":"Temp IN","unique_id":"eh_tin","state_topic":"asic/sensor/temp_in/state","unit_of_measurement":"°C","device_class":"temperature")raw");
  PUB_DISC("sensor", "t_out", R"raw("name":"Temp OUT","unique_id":"eh_tout","state_topic":"asic/sensor/temp_out/state","unit_of_measurement":"°C","device_class":"temperature")raw");
  #undef PUB_DISC
}

// ================= WEB OTA =================
void setupWebOTA() {
  server.on("/", HTTP_GET, [](){
    if (!server.authenticate(http_username, http_password)) return server.requestAuthentication();
    
    static const char html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="ru">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>ASIC HYDRO // FIRMWARE OVERRIDE</title>
    <style>
        :root {
            --bg: #080b10; --card: rgba(13,19,28,0.9); --primary: #00ff66;
            --primary-glow: rgba(0,255,102,0.4); --accent: #00e5ff; --danger: #ff0055;
            --font: 'Courier New', monospace;
        }
        * { box-sizing: border-box; margin: 0; padding: 0; }
        body { background: var(--bg); color: #c0f0d0; font-family: var(--font); display: flex; justify-content: center; align-items: center; min-height: 100vh; }
        .container { width: 100%; max-width: 500px; padding: 25px; background: var(--card); border: 1px solid var(--primary); box-shadow: 0 0 20px var(--primary-glow); border-radius: 8px; }
        .header { text-align: center; margin-bottom: 20px; border-bottom: 1px dashed var(--primary); padding-bottom: 10px; }
        .header h1 { color: var(--primary); font-size: 1.3rem; letter-spacing: 2px; }
        .drop-zone { border: 2px dashed var(--accent); border-radius: 6px; padding: 25px 15px; text-align: center; cursor: pointer; background: rgba(0,229,255,0.02); }
        .drop-zone:hover { border-color: var(--primary); background: rgba(0,255,102,0.05); }
        .btn-browse { padding: 6px 12px; background: transparent; border: 1px solid var(--accent); color: var(--accent); font-family: var(--font); cursor: pointer; margin-top: 8px; }
        input[type="file"] { display: none; }
        .file-info { margin-top: 15px; font-size: 0.85rem; color: var(--primary); display: none; }
        .progress-wrapper { margin-top: 15px; display: none; }
        .progress-bar-bg { width: 100%; height: 16px; background: #000; border: 1px solid var(--primary); border-radius: 3px; overflow: hidden; }
        .progress-bar-fill { height: 100%; width: 0%; background: linear-gradient(90deg, var(--accent), var(--primary)); }
        .progress-text { display: flex; justify-content: space-between; font-size: 0.75rem; margin-top: 5px; color: var(--accent); }
        .btn-upload { width: 100%; margin-top: 15px; padding: 10px; background: transparent; border: 1px solid var(--primary); color: var(--primary); font-family: var(--font); font-weight: bold; cursor: pointer; display: none; }
        .btn-upload:hover { background: var(--primary); color: #000; }
        .console-log { margin-top: 15px; background: #000; border: 1px solid rgba(0,255,102,0.3); padding: 8px; font-size: 0.75rem; height: 70px; overflow-y: auto; color: #70a080; }
        .console-log .success { color: var(--primary); }
        .console-log .error { color: var(--danger); }
    </style>
</head>
<body>
<div class="container">
    <div class="header">
        <h1>⚡ SYSTEM OVERRIDE</h1>
        <div style="font-size: 0.75rem; color: var(--accent);">ASIC HYDRO CONTROLLER // OTA</div>
    </div>
    <form id="uploadForm">
        <div class="drop-zone" id="dropZone">
            <p>> DROP .BIN FILE HERE</p>
            <button type="button" class="btn-browse" onclick="document.getElementById('fileInput').click()">[ SELECT FILE ]</button>
            <input type="file" id="fileInput" name="firmware" accept=".bin">
        </div>
        <div class="file-info" id="fileInfo"></div>
        <div class="progress-wrapper" id="progressWrapper">
            <div class="progress-bar-bg"><div class="progress-bar-fill" id="progressBar"></div></div>
            <div class="progress-text"><span id="progressStatus">FLASHING...</span><span id="progressPercent">0%</span></div>
        </div>
        <button type="button" class="btn-upload" id="uploadBtn" onclick="uploadFirmware()">[ EXECUTE FLASH ]</button>
    </form>
    <div class="console-log" id="consoleLog"><p>> OTA PIPELINE READY...</p></div>
</div>
<script>
    const dropZone = document.getElementById('dropZone'), fileInput = document.getElementById('fileInput');
    const fileInfo = document.getElementById('fileInfo'), uploadBtn = document.getElementById('uploadBtn');
    const progressWrapper = document.getElementById('progressWrapper'), progressBar = document.getElementById('progressBar');
    const progressPercent = document.getElementById('progressPercent'), progressStatus = document.getElementById('progressStatus');
    const consoleLog = document.getElementById('consoleLog');

    function log(msg, cls='') {
        const p = document.createElement('p'); p.textContent = '> ' + msg; if(cls) p.className = cls;
        consoleLog.appendChild(p); consoleLog.scrollTop = consoleLog.scrollHeight;
    }
    ['dragenter','dragover','dragleave','drop'].forEach(e => dropZone.addEventListener(e, ev => { ev.preventDefault(); ev.stopPropagation(); }));
    dropZone.addEventListener('drop', e => { if(e.dataTransfer.files.length) handleFile(e.dataTransfer.files[0]); });
    fileInput.addEventListener('change', () => { if(fileInput.files.length) handleFile(fileInput.files[0]); });

    function handleFile(file) {
        if(!file.name.endsWith('.bin')) { log('ERROR: .BIN REQUIRED!', 'error'); return; }
        fileInfo.style.display = 'block'; fileInfo.textContent = `TARGET: ${file.name} (${(file.size/1024).toFixed(1)} KB)`;
        uploadBtn.style.display = 'block'; log(`LOADED: ${file.name}`);
    }

    function uploadFirmware() {
        const file = fileInput.files[0]; if(!file) return;
        const formData = new FormData(); formData.append("firmware", file);
        uploadBtn.style.display = 'none'; dropZone.style.display = 'none'; progressWrapper.style.display = 'block';
        log('FLASHING STARTED...', 'success');
        const xhr = new XMLHttpRequest(); xhr.open("POST", "/update", true);
        xhr.upload.onprogress = e => {
            if(e.lengthComputable) {
                const p = Math.round((e.loaded/e.total)*100);
                progressBar.style.width = p + '%'; progressPercent.textContent = p + '%';
            }
        };
        xhr.onload = () => {
            if(xhr.status === 200) {
                progressBar.style.width = '100%'; progressPercent.textContent = '100%';
                progressStatus.textContent = 'COMPLETE!'; log('SUCCESS! REBOOTING IN 10s...', 'success');
                setTimeout(() => window.location.reload(), 10000);
            } else { progressStatus.textContent = 'FAILED!'; log('ERROR: ' + xhr.responseText, 'error'); }
        };
        xhr.onerror = () => log('CONNECTION ERROR!', 'error');
        xhr.send(formData);
    }
</script>
</body>
</html>
    )rawliteral";

    server.send_P(200, "text/html", html);
  });
  
  server.on("/update", HTTP_POST, [](){
    if (!server.authenticate(http_username, http_password)) return server.requestAuthentication();
    
    if (!Update.hasError()) {
      server.send(200, "text/plain", "SUCCESS");
      delay(1000);
      ESP.restart();
    } else {
      server.send(400, "text/plain", "FLASH_ERROR");
    }
  }, [](){
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      ota_active = true;
      relayStateMask = 0x00; 
      sendByteRelay();
      
      if (mqtt_client) esp_mqtt_client_stop(mqtt_client);

      if (!Update.begin(UPDATE_SIZE_UNKNOWN, U_FLASH)) {
        Update.printError(Serial);
        ota_active = false;
      }
    } else if (upload.status == UPLOAD_FILE_WRITE) {
      if (Update.write(upload.buf, upload.currentSize) != upload.currentSize) {
        Update.printError(Serial);
      }
    } else if (upload.status == UPLOAD_FILE_END) {
      if (!Update.end(true)) {
        Update.printError(Serial);
        ota_active = false;
      }
    }
  });
  
  server.begin();
}

// ================= FREERTOS TASK 1: CONTROL & FAILSAFE (CORE 1) =================
void TaskControl(void *pvParameters) {
  TickType_t xLastWakeTime = xTaskGetTickCount();
  const TickType_t xFrequency = pdMS_TO_TICKS(100);

  for (;;) {
    DamperCommand cmd;
    if (xQueueReceive(damperCmdQueue, &cmd, 0) == pdTRUE) {
      setDamperPercent(cmd.percent, cmd.isManual);
    }

    handlePhysicalInputs();
    processDamperRamp();
    processTemperaturesAsync();
    
    evaluateSafety();
    processAntiStuck();
    processPID();

    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// ================= SETUP =================
void setup() {
  pinMode(PIN_OE_74HC595, OUTPUT); digitalWrite(PIN_OE_74HC595, HIGH);
  pinMode(PIN_SER_74HC595, OUTPUT); pinMode(PIN_RCLK_74HC595, OUTPUT); pinMode(PIN_SRCLK_74HC595, OUTPUT);
  pinMode(PIN_QH_74HC165, INPUT); pinMode(PIN_CLK_74HC165, OUTPUT); pinMode(PIN_SH_74HC165, OUTPUT);

  relayMutex = xSemaphoreCreateMutex();

  relayStateMask = 0x00; 
  sendByteRelay();

  delay(500);

  damperCmdQueue = xQueueCreate(10, sizeof(DamperCommand));

  // СЧИТЫВАНИЕ НАСТРОЕК ИЗ NVS
  preferences.begin("asic_storage", true);
  savedRelaysMask = preferences.getUChar("relays", 0x00);
  isAutoMode = preferences.getBool("auto_mode", true);
  isForceManual = preferences.getBool("pid_force_man", false);
  isPidEnabled = isForceManual ? false : preferences.getBool("pid_enable", false);
  pidSetpoint = preferences.getFloat("pid_setpoint", 42.0f);
  pidKp = preferences.getFloat("pid_kp", 3.5f);
  pidKi = preferences.getFloat("pid_ki", 0.05f);
  pidKd = preferences.getFloat("pid_kd", 0.8f);
  isPidInverted = preferences.getBool("pid_inv", true);
  currentProfile = (OutputProfile)preferences.getUChar("out_profile", (uint8_t)PROFILE_VALVE_4_20MA);
  preferences.end();

  // ПОЛИТИКА ЗАГРУЗКИ РЕЛЕ ПРИ СТАРТЕ
  if (isAutoMode) {
    // В АВТО-РЕЖИМЕ: Включаем ТОЛЬКО гидравлику (AUX_MASK 0x0F: насос, клапаны).
    // Каналы 1..4 (ASIC_MASK 0xF0) принудительно OFF. NVS НЕ перезаписываем!
    relayStateMask = (savedRelaysMask & AUX_MASK);
    selfTestComplete = false;
    selfTestStartTime = millis();
    currentSystemState = SystemState::SELF_TEST;
  } else {
    // В РУЧНОМ РЕЖИМЕ: Мгновенно восстанавливаем все реле из NVS без задержек и проверок
    relayStateMask = savedRelaysMask;
    selfTestComplete = true;
    currentSystemState = SystemState::MANUAL;
  }

  // БЕЗОПАСНЫЙ СТАРТ ЦАП
  targetDamperPercent = 0.0f;
  currentDamperPercent = 0.0f;
  applyDamperDAC(0.0f);

  hydroPID.SetOutputLimits(0, 100);
  hydroPID.SetSampleTimeUs(PID_COMPUTE_INTERVAL * 1000);
  hydroPID.SetTunings(pidKp, pidKi, pidKd);
  hydroPID.SetControllerDirection(isPidInverted ? QuickPID::Action::reverse : QuickPID::Action::direct);
  hydroPID.SetMode(isPidEnabled ? QuickPID::Control::automatic : QuickPID::Control::manual);

  sendByteRelay(); 
  digitalWrite(PIN_OE_74HC595, LOW);
  
  // ФИКСАЦИЯ НАЧАЛЬНОГО ПОЛОЖЕНИЯ ТУМБЛЕРОВ
  lastInputStates = readByteInputs();
  debouncedInputStates = lastInputStates;

  sensorsIn.begin(); 
  sensorsOut.begin();
  sensorsIn.setWaitForConversion(false);
  sensorsOut.setWaitForConversion(false);

  pinMode(RS485_DIR_PIN, OUTPUT); digitalWrite(RS485_DIR_PIN, LOW);
  Serial.begin(9600, SERIAL_8N1); node.begin(PRESSURE_SENSOR_ADDR, Serial);
  node.preTransmission(preTransmission); node.postTransmission(postTransmission);

  WiFi.mode(WIFI_STA);
  WiFi.setHostname(ota_hostname);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(200);
  }

  MDNS.begin(ota_hostname);
  MDNS.addService("http", "tcp", 80);

  setupWebOTA();
  initNativeMqtt();
  
  startupComplete = true;

  // ЯДРО 1: ЖЕСТКИЙ РЕАЛ-ТАЙМ КОНТУР УПРАВЛЕНИЯ (100 мс)
  xTaskCreatePinnedToCore(TaskControl, "TaskControl", 8192, NULL, 5, NULL, 1);

  // ЯДРО 0: ИЗОЛИРОВАННЫЙ ОПРОС MODBUS RS485 (2000 мс)
  xTaskCreatePinnedToCore(TaskModbus, "TaskModbus", 4096, NULL, 2, NULL, 0);
}

// ================= MAIN LOOP (CORE 0) =================
void loop() {
  server.handleClient();

  if (!ota_active) {
    unsigned long now = millis();

    // Защита от износа NVS: отложенное сохранение состояний реле
    if (needSaveRelays && (now - lastRelayChangeTime >= 2000)) {
      saveRelaysToNVS();
      needSaveRelays = false;
    }

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
      publishSafetyStatus();
      setDamperPercent(targetDamperPercent, false);
      needPublishAllStates = false;
    }

    static unsigned long lastTimerPub = 0;
    if (now - lastTimerPub >= TIMER_PUBLISH_INTERVAL) {
      publishRevertTimer();
      publishSafetyStatus();
      lastTimerPub = now;
    }

    static unsigned long lastRelayPublish = 0;
    if (isMqttConnected && now - lastRelayPublish >= RELAY_STATUS_INTERVAL) {
      publishAllRelayStates();
      publishPidStatus();
      publishProfileStatus();
      publishSafetyStatus();
      lastRelayPublish = now;
    }
  }

  delay(10);
}