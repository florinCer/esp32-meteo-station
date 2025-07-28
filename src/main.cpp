// MIT License

//     Copyright(c)[2025][Florin Cercelariu][cercelatu.floce@gmail.com]

//     Permission is hereby granted,
//     free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"),
//     to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge,
//     publish, distribute, sublicense, and/ or sell copies of the Software, and to permit persons to whom the
//     Software is furnished to do so, subject to the following conditions :

//     The above copyright notice and this permission notice shall be included in all copies
//     or
//     substantial portions of the Software.

//     THE SOFTWARE IS PROVIDED "AS IS",
//     WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//     FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
//     AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
//     DAMAGES OR OTHER
//     LIABILITY,
//     WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//     OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
//     SOFTWARE.

#include <Arduino.h>
#include <ArduinoJson.h>
#include <LiquidCrystal_I2C.h>
#include <SimpleDHT.h>
#include <WebServer.h>
#include <WiFi.h>
#include <Wire.h>

#define SERIAL_DEBUG 1                                // enable serial debug output
#define LCD_PRINT 1                                   // enable LCD print output for debug purpose
#define LED_STATUS 1                                  // enable LED status output for debug purpose
#define SENSOR_DATA_TO_SERIAL 0                       // enable serial debug output for sensor data(can be disabled to avoid flooding the serial output)
#define WIFI_CONNECT_TIMEOUT 10 * 1000                // ms to seconds timeout for WiFi connection
#define WIFI_SSID "TP-Link_2F40"                      // replace with your WiFi SSID
#define WIFI_PASSWORD "70776690"                      // replace with your WiFi password
#define ESP_32_HOSTNAME "Esp_32_MeteoStation_server"  // hostname for the ESP32 device
#define MAX_OUTPUT_SIZE 300                           // max output size for JSON serialization
// pin mappings
const int pinDHT11 = 13;  // GPIO13, D7 on AZDelivery Devkit V4
const int LED_PIN = 2;
static float temperature = 0;
static float humidity = 0;
static int err = SimpleDHTErrSuccess;
// TODO:  check if space issues appear to reduce string info size to serial(e.g: WL_CONNECTED -> WL_CONN)
static const char *wifi_status_str[] = {
    "WL_NO_SHIELD",
    "WL_IDLE_STATUS",
    "WL_NO_SSID_AVAIL",
    "WL_SCAN_COMPLETED",
    "WL_CONNECTED",
    "WL_CONNECT_FAILED",
    "WL_CONNECTION_LOST",
    "WL_DISCONNECTED"};  // WiFi status strings

// class instances
SimpleDHT11 dht11(pinDHT11);
// I2C default pins(I2C display connections, etc):
// SDA: A4, SCL: A5 for Arduino Uno, Nano, Mini, etc.
// SDA: GPIO21, SCL: GPIO22 for ESP32
// You can override these defaults by calling Wire.begin(SDA, SCL); with your desired pin numbers
LiquidCrystal_I2C lcd(0x27, 16, 2);  // set the LCD address to 0x20 for a 16 chars and 2 line display
StaticJsonDocument<MAX_OUTPUT_SIZE> doc;
char jsonOutput[MAX_OUTPUT_SIZE];  // buffer for JSON output
// Web server running on port 80
WebServer server(80);

// function prototypes
static int dth11_read(float *temperature, float *humidity);
static wl_status_t wifi_connect(const char *ssid, const char *password);
static void prepareJsonData(JsonDocument &doc, float temperature, float humidity);
__inline void blink_led_status(int status_nb, int repeat_ms) {
    // blink LED for status number of times
    for (int i = 0; i < status_nb; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(repeat_ms);
        digitalWrite(LED_PIN, LOW);
        delay(repeat_ms);
    }
}
__inline void setupRestApi() {
    server.on("/api/sensor", HTTP_GET, []() {
        // handle GET request for sensor data
        prepareJsonData(doc, temperature, humidity);
        server.send(200, "application/json", jsonOutput); });
}
__inline static void debugWifi(const wl_status_t status) {
    // debug WiFi connection status by serial or LCD or led status if are available
#if SERIAL_DEBUG
    Serial.print("WiFi connection failed, status=" + String(status) + " (" + String(wifi_status_str[status]) + ")");
#endif
#if LCD_PRINT
    lcd.clear();
    // use other ways to debug if wifi connection failed(e.g: blink led fast for number of error code)
    lcd.print("WiFiErr: ");
    lcd.print(status, 10);
#endif
#if LED_STATUS
    blink_led_status(status, 500);
    delay(1000);  // wait for a while to show the error
    blink_led_status(status, 500);
    delay(1000);  // wait for a while to show the error
#endif
}

void setup() {
    // setup function: put your setup code here, to run once:
    pinMode(LED_PIN, OUTPUT);  // heartbeat LED
    // pinMode(pinDTH11, INPUT_PULLUP);
    lcd.init();  // initialize the lcd
    lcd.backlight();
    lcd.print("Welcome!");
    delay(1500);
#if SERIAL_DEBUG
    Serial.begin(115200);
#endif
    lcd.clear();
    // connect to WiFi
    wl_status_t status = wifi_connect(WIFI_SSID, WIFI_PASSWORD);
    if (status != WL_CONNECTED) {
        debugWifi(status);
    } else {
#if SERIAL_DEBUG
        Serial.print("Connected to: ");
        Serial.println(WiFi.localIP());
#endif
#if LCD_PRINT
        lcd.print("WiFi ON!");
#endif
#if LED_STATUS
        blink_led_status((int)(status == WL_CONNECTED), 1500);
#endif
    }
    // setup REST API
    setupRestApi();
    // start server
    server.begin();
}

// main loop function: called repeatedly after setup()
void loop() {
    // put your main code here, to run repeatedly:
    uint8_t led_value = digitalRead(LED_PIN);
    digitalWrite(LED_PIN, !led_value);
    // DTH11 sensor integration
    err = dth11_read(&temperature, &humidity);

    if (err != SimpleDHTErrSuccess) {
        lcd.clear();
        lcd.print("DHT11Err: ");
        lcd.print(SimpleDHTErrCode(err), 10);
#if SERIAL_DEBUG
        Serial.print("Read DHT11 failed, err=");
        Serial.print(SimpleDHTErrCode(err));
#endif
    } else {
        // send to display
        lcd.setCursor(0, 0);
        lcd.print("Temperat: ");
        lcd.print((float)temperature, 1);
        lcd.print(" C");

        // send to display
        lcd.setCursor(0, 1);
        lcd.print("Humidity: ");
        lcd.print((float)humidity, 1);
        lcd.print(" %");
#if SERIAL_DEBUG && SENSOR_DATA_TO_SERIAL
        // print to serial
        Serial.print((float)temperature);
        Serial.print(" grC, ");
        Serial.print((float)humidity);
        Serial.println(" RH%");
#endif
    }
    // handle REST API requests
    // TODO: check if this can be moved to a separate task
    server.handleClient();

    delay(1500);  // wait for a time
}

static int dth11_read(float *temperature, float *humidity) {
    // start working...
    return dht11.read2(temperature, humidity, NULL);
}

static wl_status_t wifi_connect(const char *ssid, const char *password) {
    // connect to WiFi network
    wl_status_t status = WL_DISCONNECTED;  // assume disconnected first
    unsigned long startAttemptTime = millis();

#if SERIAL_DEBUG
    Serial.print("Connecting to WiFi: " + String(ssid));
#endif
#if LCD_PRINT
    lcd.print("WiFi .");
#endif
    WiFi.setHostname(ESP_32_HOSTNAME);
    WiFi.mode(WIFI_STA);
    status = WiFi.begin(ssid, password);  // try again

    while (status != WL_CONNECTED && millis() - startAttemptTime < WIFI_CONNECT_TIMEOUT) {
        // wait for connection
        delay(1000);             // wait for a while
        status = WiFi.status();  // get current status
#if SERIAL_DEBUG
        Serial.print(".");
#endif
#if LCD_PRINT
        lcd.print(".");
#endif
    }
    return status;
}

void prepareJsonData(JsonDocument &doc, float temperature, float humidity) {
    // prepare JSON data
    doc.clear();
    JsonObject sensor = doc["sensor"].to<JsonObject>();
    sensor["id"] = "dth11_1";
    sensor["name"] = "DTH11 Sensor";
    sensor["status"] = (err == SimpleDHTErrSuccess) ? "active" : "error " + String(SimpleDHTErrCode(err));

    JsonObject sensor_data = sensor["data"].to<JsonObject>();
    JsonObject sensor_data_temperature = sensor_data["temperature"].to<JsonObject>();
    sensor_data_temperature["unit"] = "Celsius";
    sensor_data_temperature["value"] = temperature;

    JsonObject sensor_data_humidity = sensor_data["humidity"].to<JsonObject>();
    sensor_data_humidity["unit"] = "%";
    sensor_data_humidity["value"] = humidity;
    // serialize JSON to output buffer
    serializeJson(doc, jsonOutput, MAX_OUTPUT_SIZE);
}