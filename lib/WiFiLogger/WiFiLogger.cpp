#include "WiFiLogger.h"
#include <Arduino.h>
#include <stdarg.h>

WiFiLogger Logger;

WiFiLogger::WiFiLogger() : _server(23), _clientConnected(false), _ssid(nullptr), _password(nullptr), _port(23) {}

void WiFiLogger::begin(const char* ssid, const char* password, uint16_t port) {
    _ssid = ssid;
    _password = password;
    _port = port;
    
    // Uruchomienie trybu AP
    WiFi.softAP(_ssid, _password);
    IPAddress myIP = WiFi.softAPIP();
    
    // Uruchomienie serwera TCP
    _server.begin(_port);
    _server.setNoDelay(true);

    // Debugging
    // Serial.begin(115200);
    // Serial.print("Adres IP: ");
    // Serial.println(myIP);
    // Serial.print("Serwer TCP uruchomiony na porcie: ");
    // Serial.println(_port);
}

void WiFiLogger::print(const String &message) {
    // Serial.print(message);
    if (_clientConnected) {
        _client.print(message);
    }
}

void WiFiLogger::println(const String &message) {
    // Serial.println(message);
    if (_clientConnected) {
        _client.println(message);
    }
}

void WiFiLogger::printf(const char *format, ...) {
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    
    // Serial.print(buffer);
    if (_clientConnected) {
        _client.print(buffer);
    }
}

bool WiFiLogger::isClientConnected() {
    return _clientConnected;
}

void WiFiLogger::handleClient() {
    if (!_clientConnected) {
        _client = _server.available();
        if (_client) {
            _clientConnected = true;
            println("Nowy klient połączony");
        }
    } else {
        if (!_client.connected()) {
            _clientConnected = false;
            println("Nowy klient rozłączony");
            _client.stop();
        }
    }
}

String WiFiLogger::readCommand() {
    if (_clientConnected && _client.available()) {
        String command = _client.readStringUntil('\n');
        command.trim(); // usuwa białe znaki i \r\n
        return command;
    }
    return "";
}
