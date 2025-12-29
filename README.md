## rs3proxy (ESP-IDF, ESP32‑S3)

Проект для исследования и управления камерой через **DJI RS3** по **USB PTP**:

- **PTP эмуляция на ESP32** (камера “как Sony”, чтобы RS3 принял устройство).
- **Raw proxy режим**: ESP32 как USB‑устройство для RS3 и TCP‑мост на ПК (для sniff/relay).
- **TCP сервер** для логов и команд (в т.ч. OTA и reboot).
- **UI на LCD + touch**: статус и кнопки “Update FW” / “Restart MCU”.

Плата: **Waveshare ESP32-S3-Touch-LCD-1.83**. Полезный гайд Waveshare: [ESP32-S3-Touch-LCD-1.83 — Run the First ESP-IDF Demo](https://www.waveshare.com/wiki/ESP32-S3-Touch-LCD-1.83#Run_the_First_ESP-IDF_Demo)

### Порты / подключение

- **TCP logs + commands**: `CONFIG_RS3_TCP_SERVER_PORT` (по умолчанию **1234**)  
  Подключение: `nc <esp-ip> 1234`
- **PTP raw proxy** (USB bulk <-> TCP бинарный протокол): `CONFIG_RS3_USB_PTP_PROXY_PORT` (по умолчанию **1235**)  
  Используется скриптом `scripts/rs3_ptp_raw_proxy.py` (и/или `scripts/rs3_ptp_proxy.py`).

### Конфигурация (menuconfig)

Все ключевые настройки в `main/Kconfig.projbuild`:

- **Wi‑Fi (STA)**: SSID/password для подключения и получения IP
- **TCP server**: порт логов/команд (по умолчанию 1234)
- **OTA**: `CONFIG_RS3_OTA_URL` (default URL для кнопки “Update FW” и команды `ota <url>`)
- **USB PTP (camera emulation)**:
  - `CONFIG_RS3_USB_PTP_ENABLE`
  - VID/PID/bcdDevice/строки (чтобы совпадать с реальной камерой)
  - **PTP implementation**:
    - **Standard** (`RS3_USB_PTP_IMPL_STD`): базовая реализация для PC‑host
    - **Legacy (RS3)** (`RS3_USB_PTP_IMPL_LEGACY`): текущая реализация под RS3
    - **Raw proxy** (`RS3_USB_PTP_IMPL_PROXY_RAW`): мост USB bulk <-> TCP без модификаций

### Сборка

В корне проекта:

```bash
idf.py set-target esp32s3
idf.py menuconfig
idf.py build
```

Если ты используешь ESP‑IDF из установленного пути, обычно достаточно:

```bash
source /Users/pavel/esp/esp-idf/export.sh
export IDF_COMPONENT_MANAGER=1
cd /Users/pavel/workspace/rs3proxy
idf.py build
```

### Прошивка и монитор

Прошивку/монитор запускай как тебе удобно (COM/tty зависит от системы):

```bash
idf.py -p /dev/ttyACM0 flash
idf.py -p /dev/ttyACM0 monitor
```

### Логи и команды по TCP

Подключиться к логам:

```bash
nc <esp-ip> 1234
```

Команды (TCP):

- `ota <url>`: pull‑OTA обновление (см. `CONFIG_RS3_OTA_URL`)
- `reboot` / `restart` / `reset`: перезагрузка MCU

### PTP debug logging

В прошивке есть compile‑time флаг:

- `RS3_PTP_LOG_DEBUG=0` (по умолчанию): тихо/минимально
- `RS3_PTP_LOG_DEBUG=1`: подробные логи (для диагностики USB/PTP)

### Touch кнопки (LCD)

Внизу экрана две кнопки:

- **Update FW**: запускает OTA с URL из `CONFIG_RS3_OTA_URL`
- **Restart MCU**: вызывает `esp_restart()`

### Скрипты (macOS / USB PTP)

См. `scripts/README.md`:

- `scripts/usb_dump_ptp_enumeration.py`: снять VID/PID/bcdDevice/строки и endpoint‑ы с реальной камеры
- `scripts/ptp_getdeviceinfo.py`: получить сырые байты DeviceInfo
- `scripts/rs3_ptp_raw_proxy.py`: raw proxy (для режима `RS3_USB_PTP_IMPL_PROXY_RAW`)

На macOS часто нужно:

```bash
sudo killall PTPCamera 2>/dev/null || true
```


