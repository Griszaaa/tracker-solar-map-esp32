#ifndef WIFI_LOGGER_H
#define WIFI_LOGGER_H

#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiAP.h>

#define WIFI_LED 3 // Pin diody LED Wi-Fi


class WiFiLogger {
public:
    WiFiLogger();
    void begin(const char* ssid, const char* password, uint16_t port = 23);
    void print(const String &message);
    void println(const String &message);
    void printf(const char *format, ...);
    bool isClientConnected();
    void handleClient();
    String readCommand();
    float readFloat();

private:
    WiFiServer _server;
    WiFiClient _client;
    bool _clientConnected;
    const char* _ssid;
    const char* _password;
    uint16_t _port;
};

extern WiFiLogger Logger;

#endif