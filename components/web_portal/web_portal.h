#pragma once

#include "esphome/core/component.h"
#include <memory>

#ifdef USE_ESP32
#include <WiFi.h>
#elif defined(USE_ESP8266)
#include <ESP8266WiFi.h>
#endif

#include <ESPAsyncWebServer.h>

namespace esphome {
namespace web_portal {

class WebPortal : public Component {
 public:
  explicit WebPortal(uint16_t port) : port_(port) {}
  void setup() override;
  void loop() override {};
  void set_auth(const std::string &username, const std::string &password) {
    username_ = username;
    password_ = password;
    use_auth_ = true;
  }

 protected:
  uint16_t port_;
  std::unique_ptr<AsyncWebServer> server_;
  bool use_auth_{false};
  std::string username_;
  std::string password_;
};

}  // namespace web_portal
}  // namespace esphome

