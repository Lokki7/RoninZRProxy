## rs3proxy_hello (ESP-IDF)

Базовый **Hello World** проект для **Waveshare ESP32-S3-Touch-LCD-1.83** (ESP32‑S3).

Полезная ссылка на гайд Waveshare: [ESP32-S3-Touch-LCD-1.83 — Run the First ESP-IDF Demo](https://www.waveshare.com/wiki/ESP32-S3-Touch-LCD-1.83#Run_the_First_ESP-IDF_Demo)

### Требования

- Установленный ESP-IDF (через официальный installer / VSCode plugin / manual).
- Активированная среда ESP‑IDF (чтобы `idf.py` был доступен).

### Сборка

В корне проекта:

```bash
idf.py set-target esp32s3
idf.py build
```

### Прошивка и монитор

Прошивку/монитор запускай как тебе удобно (COM/tty зависит от системы):

```bash
idf.py -p /dev/ttyACM0 flash
idf.py -p /dev/ttyACM0 monitor
```

Ожидаемый вывод в логах: приветствие, информация о чипе/flash/PSRAM и `tick` раз в секунду.

### Дальше

Следующие шаги для этой платы обычно такие:

- Поднять питание/PMU (AXP2101) при необходимости.
- Подключить дисплей (ST7789P по SPI) и тач (CST816D по I2C).
- Подключить LVGL и сделать первый экран.


