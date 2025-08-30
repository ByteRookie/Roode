#include "web_portal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace web_portal {

static const char *const TAG = "web_portal";

static const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang=\"en\">
<head>
  <meta charset=\"UTF-8\">
  <title>Coming Soon</title>
</head>
<body>
  <h1>Coming Soon</h1>
  <p>Stay tuned for updates.</p>
</body>
</html>
)rawliteral";

void WebPortal::setup() {
  this->server_ = std::make_unique<AsyncWebServer>(this->port_);

  this->server_->on("/web", HTTP_GET, [this](AsyncWebServerRequest *request) {
    if (this->use_auth_ && !request->authenticate(this->username_.c_str(), this->password_.c_str())) {
      return request->requestAuthentication();
    }
    request->send_P(200, "text/html", INDEX_HTML);
  });

  this->server_->begin();
  ESP_LOGI(TAG, "Started web portal on port %u", this->port_);
}

}  // namespace web_portal
}  // namespace esphome

