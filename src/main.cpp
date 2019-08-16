#include <AsyncMqttClient.h>
#include <ESP8266WiFi.h>
#include <Ticker.h>
#include <DHTesp.h>
#include "config.h"

AsyncMqttClient mqttClient;
Ticker mqttReconnectTimer;

WiFiEventHandler wifiConnectHandler;
WiFiEventHandler wifiDisconnectHandler;
Ticker wifiReconnectTimer;

Ticker buttonsReadTimer;

DHTesp dht;

bool lastBtn1State = HIGH;
bool lastBtn2State = HIGH;
bool lastBtn3State = HIGH;
bool lastBtn4State = HIGH;

uint8_t readIteration = 0;
int16_t tSum = 0;
uint16_t hSum = 0;

void sendData() {
    double t = tSum / (config::SENSOR_READ_COUNT * 1.0);
    double h = hSum / (config::SENSOR_READ_COUNT * 1.0);

    mqttClient.publish("variable/room1-air_temperature", 0, false, String(t, 1).c_str());
    mqttClient.publish("variable/room1-air_humidity", 0, false, String(h).c_str());

//    Serial.print("Sent. T: ");
//    Serial.print(t);
//    Serial.print(", h: ");
//    Serial.println(h);
}

void readSensor() {
    auto t = (int16_t) dht.getTemperature();
    auto h = (uint16_t) dht.getHumidity();

    if (dht.getStatus() == DHTesp::ERROR_NONE) {
        readIteration++;

//        Serial.print("[");
//        Serial.print(readIteration);
//        Serial.print("] T: ");
//        Serial.print(t);
//        Serial.print(", h: ");
//        Serial.println(h);

        tSum += t;
        hSum += h;

        if (readIteration == config::SENSOR_READ_COUNT) {
            sendData();

            readIteration = 0;
            tSum = 0;
            hSum = 0;
        }
    } else {
        Serial.print(F("Error reading sensor: "));
        Serial.println(dht.getStatusString());
    }
}

void readButtons() {
    bool btn1State = digitalRead(config::BTN1_PIN);
    bool btn2State = digitalRead(config::BTN2_PIN);
    bool btn3State = digitalRead(config::BTN3_PIN);
    bool btn4State = digitalRead(config::BTN4_PIN);

    if (btn1State != lastBtn1State) {
        if (btn1State == LOW) {   // button 1 pressed
            mqttClient.publish("switch/room1-light/toggle", 0, false);
//            Serial.println(F("Button 1 pressed"));
        }

        lastBtn1State = btn1State;
    }

    if (btn2State != lastBtn2State) {
        if (btn2State == LOW) {   // button 2 pressed
            mqttClient.publish("switch/room1-secondary_light/toggle", 0, false);
//            Serial.println(F("Button 2 pressed"));
        }

        lastBtn2State = btn2State;
    }

    if (btn3State != lastBtn3State) {
        if (btn3State == LOW) {   // button 3 pressed
            mqttClient.publish("switch/room1-fan/toggle", 0, false);
//            Serial.println(F("Button 3 pressed"));
        }

        lastBtn3State = btn3State;
    }

    if (btn4State != lastBtn4State) {
        if (btn4State == LOW) {   // button 4 pressed
            mqttClient.publish("buzzer/corridor-buzzer/unlock", 0, false);
//            Serial.println(F("Button 4 pressed"));
        }

        lastBtn4State = btn4State;
    }
}

void connectToWifi() {
    Serial.println(F("Connecting to Wi-Fi..."));
    WiFi.begin(config::WIFI_SSID, config::WIFI_PASSWORD);
}

void connectToMqtt() {
    Serial.println(F("Connecting to MQTT..."));
    mqttClient.connect();
}

void onWifiConnect(const WiFiEventStationModeGotIP &) {
    connectToMqtt();
}

void onWifiDisconnect(const WiFiEventStationModeDisconnected &event) {
    Serial.print(F("Disconnected from Wi-Fi: "));
    Serial.println(event.reason);
    digitalWrite(LED_BUILTIN, LOW);

    mqttReconnectTimer.detach();
    wifiReconnectTimer.once(2, connectToWifi);
}

void onMqttConnect(bool) {
    Serial.println(F("Connected to MQTT."));
    digitalWrite(LED_BUILTIN, HIGH);
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
    Serial.print(F("Disconnected from MQTT. Reason: "));
    Serial.println((int) reason);

    digitalWrite(LED_BUILTIN, LOW);

    if (WiFi.isConnected()) {
        mqttReconnectTimer.once(2, connectToMqtt);
    }
}

void setup() {
    pinMode(config::BTN1_PIN, INPUT_PULLUP);
    pinMode(config::BTN2_PIN, INPUT_PULLUP);
    pinMode(config::BTN3_PIN, INPUT_PULLUP);
    pinMode(config::BTN4_PIN, INPUT_PULLUP);
    pinMode(LED_BUILTIN, OUTPUT);

    digitalWrite(LED_BUILTIN, LOW);

    Serial.begin(115200);
    Serial.println();
    Serial.println();

    wifiConnectHandler = WiFi.onStationModeGotIP(onWifiConnect);
    wifiDisconnectHandler = WiFi.onStationModeDisconnected(onWifiDisconnect);

    mqttClient.onConnect(onMqttConnect);
    mqttClient.onDisconnect(onMqttDisconnect);
    mqttClient.setServer(config::MQTT_HOST, config::MQTT_PORT);
    mqttClient.setClientId(config::MQTT_ID);
    mqttClient.setCredentials("device", config::MQTT_PASSWORD);

    connectToWifi();

    dht.setup(config::DHT_PIN, DHTesp::DHT11);

    buttonsReadTimer.attach_ms(config::BTN_DEBOUNCE_PERIOD, readButtons);
}

unsigned long last = millis();

void loop() {
    unsigned long time = millis();

    if (time - last >= config::SENSOR_READ_INTERVAL * 1000) {
        readSensor();

        last = time;
    }
}