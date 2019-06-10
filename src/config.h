#ifndef ROOM1_SECONDARY_LIGHT_CONFIG_H
#define ROOM1_SECONDARY_LIGHT_CONFIG_H

#include <Arduino.h>

namespace config {
    const char WIFI_SSID[] = "Solomaha";
    const char WIFI_PASSWORD[] = "solomakha21";

//    const auto MQTT_HOST = IPAddress(192, 168, 1, 230);
    const auto MQTT_HOST = IPAddress(176, 36, 198, 196);
    const uint16_t MQTT_PORT = 1883;
    const char MQTT_ID[] = "room1-table";
    const char MQTT_PASSWORD[] = "jhdfguhkgmnkvgfnkmvvmnktu34cmy77y3";

    const uint8_t BTN1_PIN = D1;
    const uint8_t BTN2_PIN = D2;
    const uint8_t BTN3_PIN = D3;
    const uint8_t BTN4_PIN = D5;
    const uint8_t DHT_PIN = D4;

    const uint8_t BTN_DEBOUNCE_PERIOD = 70;

    const uint8_t SENSOR_READ_INTERVAL = 30;
}

#endif
