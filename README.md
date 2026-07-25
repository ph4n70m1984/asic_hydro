# ⚡ ESP32 ASIC Hydro & Immersion Cooling Controller (ES32D26)

[![Hardware](https://img.shields.io/badge/Board-Eletechsup%20ES32D26-green.svg)](http://www.eletechsup.com/)
[![MCU](https://img.shields.io/badge/MCU-ESP32--DevKitC-blue.svg)](https://www.espressif.com/)
[![Home Assistant](https://img.shields.io/badge/Home%20Assistant-MQTT%20Discovery-008080.svg)](https://www.home-assistant.io/)
[![Version](https://img.shields.io/badge/Version-v2.7.0-brightgreen.svg)]()
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

Промышленный контроллер автоматики для систем водяного (Hydro) и иммерсионного (Immersion) охлаждения ASIC-майнеров. Прошивка разработана под плату **Eletechsup ES32D26** на базе ESP32 и обеспечивает надежное управление насосами, заслонками сброса тепла (4–20 мА), отслеживание давления по Modbus RTU, авторегулирование температуры по ПИД и асинхронную интеграцию с **Home Assistant**.

---

## 🌟 Ключевые возможности

* **Надежность промышленных ПЛК:**
  * Встроенный аппаратный Watchdog (`esp_task_wdt`) с таймаутом 10 сек.
  * Безопасный запуск: аппаратная блокировка выходов реле при подаче питания.
  * Энергонезависимое хранение маски реле, режима работы и параметров ПИД в **NVS Preferences**.
* **Локальный ПИД-регулятор (`QuickPID`):**
  * Полностью автономный расчет прямо на ESP32 (не зависит от доступности Wi-Fi/MQTT).
  * Настраиваемый диапазон уставки $T_{out}$: **20.0°C – 85.0°C** (подходит для Bitmain Hydro 42°C и кастомных водоблоков T21/S19 55–65°C).
  * Переключатель инверсии ПИД (`Direct` / `Reverse`) для любых типов арматуры и клапанов.
  * Настройка коэффициентов $K_p, K_i, K_d$ «на лету» через MQTT/HA.
* **Плавный ход заслонки (Slew Rate Limiter):**
  * Программный демпфер плавного изменения тока ЦАП (4–20 мА) для защиты механики и исключения гидравлических ударов (0.5% каждые 100 мс = 20 сек полный цикл).
* **Профилактика закисания (Anti-Stuck Cycle):**
  * Ежесуточный цикл авто-смахивания заслонки (проходка 0% $\rightarrow$ 100% $\rightarrow$ 0% $\rightarrow$ рабочее положение).
* **Автоматизация Home Assistant:**
  * Полный **MQTT Discovery** (переключатели, слайдеры уставки, тумблер инверсии, датчики давления и температур).
  * Групповой **Master Switch** для запуска/останова всей гидросистемы одной кнопкой.
* **Мониторинг периферии:**
  * Датчик давления по **RS485 Modbus RTU** (Holding Registers).
  * Две независимые шины OneWire (DS18B20) для температур на входе ($T_{in}$) и выходе ($T_{out}$).
  * Аварийный авто-останов при протечке (триггер `leak`).
* **Обновление и Web-интерфейс:**
  * Встроенный Web-сервер с авторизацией для обновления прошивки «по воздуху» (Web OTA).

---

## 🛠️ Аппаратная архитектура (ES32D26)

| Периферия / Назначение | Чип / Интерфейс | Пины ESP32 (GPIO) | Описание |
| :--- | :--- | :--- | :--- |
| **Реле CH1..CH8** | `74HC595` (Сдвиговый регистр) | `SER`: 12, `OE`: 13, `RCLK`: 23, `SRCLK`: 22 | Управление 8 силовыми каналами |
| **Входы IN1..IN8** | `74HC165` (Сдвиговый регистр) | `QH`: 15, `CLK`: 2, `SH`: 0 | Дискретные входы |
| **Заслонка (4–20 мА)** | Выход `Io2` (ЦАП) | `DAC2`: GPIO26 | Выход управления приводом |
| **Датчик давления** | RS485 Modbus RTU | `TX0`: IO1, `RX0`: IO3, `DIR`: IO21 | Промышленный датчик давления |
| **Температура Tin** | OneWire (DS18B20) | GPIO19 | Контур входа жидкости |
| **Температура Tout** | OneWire (DS18B20) | GPIO18 | Контур выхода (обратка) |

---

## ⚠️ ВАЖНО: Аппаратная доработка (Fix стартовых щелчков)

Для **100% исключения кратковременного срабатывания реле** при подаче питания или перезагрузке ESP32:

* **Компонент:** Резистор **10 кОм** (0.125 Вт / 0.25 Вт).
* **Точки подключения:** Соединить пин **`G13`** (Output Enable `74HC595`) и пин **`3V3`** на гребенке платы.
* **Зачем:** Физический Pull-up блокирует выходы сдвигового регистра `74HC595` в состоянии HIGH на время инициализации микроконтроллера.

---

## 🔌 Назначение каналов реле

* **CH1 — CH4:** Питание майнеров (ASIC 1 — ASIC 4).
* **CH5:** Циркуляционный насос гидроконтура (`asic/pump`).
* **CH6:** Трехходовой клапан / Сервопривод сброса тепла (`asic/valve`).
* **CH7:** Резервное реле (`asic/relay7`).
* **CH8:** Реле аварии / Сигнализация / Дренаж (`asic/relay8`).

---

## 🚀 Сборка и установка

### 1. Подготовка конфигурации
Переименуйте шаблон конфигурации или отредактируйте файл `src/config.h`:

```cpp
#ifndef CONFIG_H
#define CONFIG_H

#define DEBUG_ENABLED 1
#define DEBUG_LEVEL 3

// Wi-Fi
const char* ssid = "YOUR_WIFI_SSID";
const char* password = "YOUR_WIFI_PASSWORD";

// MQTT Broker
const char* mqtt_server = "192.168.1.100";
const int mqtt_port = 1883;
const char* mqtt_user = "mqtt_user";
const char* mqtt_pass = "mqtt_password";

// Web OTA Auth
const char* http_username = "admin";
const char* http_password = "admin_password";
const char* ota_hostname = "asic-hydro-esp32";

#endif
```

### ⚙️ Сборка через PlatformIO (`platformio.ini`)

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200

lib_deps =
    dlloydev/QuickPID @ ^3.1.9
    knolleary/PubSubClient @ ^2.8
    milesburton/DallasTemperature @ ^3.11.0
    paulstoffregen/OneWire @ ^2.3.7
    4-20ma/ModbusMaster @ ^2.0.1
```

🌐 Структура MQTT Топиков
-------------------------

#### 📥 Управление (Subscribed):

*   **asic/master/set** — Мастер-выключатель всей системы (ON / OFF).
    
*   **asic/pid/enable/set** — Включение/выключение авто-ПИД (ON / OFF).
    
*   **asic/pid/invert/set** — Инверсия направления ПИД (ON — Reverse/Охлаждение, OFF — Direct).
    
*   **asic/pid/setpoint/set** — Уставка целевой температуры $T\_{out}$ (число 20.0–85.0).
    
*   **asic/pid/kp/set**, **ki/set**, **kd/set** — Настройка коэффициентов ПИД.
    
*   **asic/damper/set** — Ручная установка заслонки (0–100). _При ручном вводе ПИД автоматически выключается._
    
*   **asic/relay1/set** .. **asic/relay4/set** — Управление майнерами (ON / OFF).
    
*   **asic/pump/set** — Управление насосом (ON / OFF).
    
*   **asic/sensor/leak/set** — Аварийный триггер протечки (ON).
    

##### 📤 Метрики и состояния (Published):

*   **asic/sensor/temp\_in/state** — Температура на входе (°C).
    
*   **asic/sensor/temp\_out/state** — Температура на выходе (°C).
    
*   **asic/sensor/pressure/state** — Давление в контуре (Bar).
    
*   **asic/damper/state** — Целевой % открытия заслонки.
    
*   **asic/damper/current\_ma/state** — Текущий фактический ток управления ЦАП (4–20 мА).
    
*   **asic/status** — Статус контроллера (online / offline).

![screenshot](screen.png)

📜 Лицензия
Этот проект распространяется под лицензией MIT.