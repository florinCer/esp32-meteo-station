## Meteo Station

Meteo station is a small project connecting ESP32-WROOM-32S with 1602 16x2 IIC display
and DHT11 Temperature and Humidity sensor.
Application will display data from dht sensor thought display(using I2C) and further
using a REST API server(by JSON data) for other application usage(Android or HTML).
Framework used to develop is Arduino and using lib examples from:
```git
lib_deps =
    winlinvip/SimpleDHT@^1.0.15
    ; johnrickman/LiquidCrystal_I2C@^1.1.3
    https://github.com/johnrickman/LiquidCrystal_I2C.git
    bblanchon/ArduinoJson@^6.21.0
```
JSON data format used by REST API is saved in **``doc\JsonDataImplement\meteo_rest.json``**
