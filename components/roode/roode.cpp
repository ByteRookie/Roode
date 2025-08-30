#include "roode.h"
#include "Arduino.h"
#include <string>
#include <optional>
#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>
#include <ctime>
#include <sstream>
#include <cstring>
#include <cstdlib>
#include <ArduinoJson.h>
#ifdef USE_WEB_SERVER
#include "esphome/components/web_server_base/web_server_base.h"
#include "esphome/core/pgmspace.h"
static const char portal_html[] PROGMEM = R"PORTAL(
#include "web/portal.html"
)PORTAL";
#endif

namespace esphome {
namespace roode {

// When disabled, fallback diagnostics are omitted from the log to reduce noise.
bool Roode::log_fallback_events_ = false;
Roode *Roode::instance_ = nullptr;
static bool scan_running = false;
static bool scan_cancel_requested = false;

static std::string format_timestamp(uint32_t sec_since_boot) {
  uint32_t now_sec = millis() / 1000;
  time_t epoch = time(nullptr) - (now_sec - sec_since_boot);
  struct tm tm_time;
  localtime_r(&epoch, &tm_time);
  char buf[25];
  strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", &tm_time);
  return std::string(buf);
}

static std::string format_epoch(time_t epoch) {
  struct tm tm_time;
  localtime_r(&epoch, &tm_time);
  char buf[25];
  strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", &tm_time);
  return std::string(buf);
}
void Roode::log_event(const std::string &msg) {
  if (!log_fallback_events_) {
    if (msg == "interrupt_fallback" || msg == "interrupt_fallback_polling")
      return;
    if (msg == "int_pin_missed" || msg.rfind("int_pin_missed_sensor_", 0) == 0)
      return;
    if (msg == "xshut_toggled" || msg == "xshut_toggled_on" || msg == "xshut_toggled_off" || msg == "xshut_pulse_off" ||
        msg == "xshut_reinitialize" || msg == "sensor.recovered_via_xshut" || msg.rfind("xshut_sensor_", 0) == 0 ||
        msg.rfind("xshut_pulse_off_sensor_", 0) == 0 || msg.rfind("xshut_reinitialize_sensor_", 0) == 0 ||
        (msg.rfind("sensor_", 0) == 0 && msg.find(".recovered_via_xshut") != std::string::npos))
      return;
  }

  static uint32_t last_int_log = 0;
  if (msg == "interrupt_fallback" || msg == "interrupt_fallback_polling" || msg == "int_pin_missed" ||
      msg.rfind("int_pin_missed_sensor_", 0) == 0) {
    uint32_t now = millis();
    if (last_int_log != 0 && (now - last_int_log) < 5000)
      return;
    last_int_log = now;
  }

  std::string out = msg;
  if (msg == "use_dual_core")
    out += " - launching task on core 1";
  else if (msg.rfind("retry_multicore_", 0) == 0)
    out += " - retry creating task";
  else if (msg == "dual_core_success")
    out += " - task running on core 1";
  else if (msg == "dual_core_failed")
    out += " - task creation failed";
  else if (msg == "fallback_single_core")
    out += " - switching to single core";
  else if (msg == "force_single_core")
    out += " - single core forced by config";
  else if (msg.rfind("xshut_sensor_", 0) == 0) {
    bool on = msg.find("_on") != std::string::npos;
    size_t start = sizeof("xshut_sensor_") - 1;
    size_t end = msg.find('_', start);
    std::string id = msg.substr(start, end - start);
    out += on ? " - sensor " + id + " ON" : " - sensor " + id + " OFF";
  } else if (msg == "xshut_toggled_on")
    out += " - XSHUT pin HIGH";
  else if (msg == "xshut_toggled_off")
    out += " - XSHUT pin LOW";
  else if (msg == "xshut_toggled")
    out += " - XSHUT pin toggled";
  else if (msg.rfind("xshut_pulse_off_sensor_", 0) == 0) {
    std::string id = msg.substr(sizeof("xshut_pulse_off_sensor_") - 1);
    out += " - pulsing LOW for sensor " + id;
  } else if (msg == "xshut_pulse_off") {
    out += " - pulsing LOW";
  } else if (msg.rfind("xshut_reinitialize_sensor_", 0) == 0) {
    std::string id = msg.substr(sizeof("xshut_reinitialize_sensor_") - 1);
    out += " - reinitializing sensor " + id;
  } else if (msg == "xshut_reinitialize") {
    out += " - reinitializing";
  } else if (msg.rfind("sensor_", 0) == 0 && msg.find("_addr") != std::string::npos) {
    size_t start = sizeof("sensor_") - 1;
    size_t end = msg.find('_', start);
    std::string id = msg.substr(start, end - start);
    std::string addr = msg.substr(msg.find("0x") + 2);
    out += " - address 0x" + addr + " for sensor " + id;
  } else if (msg.rfind("sensor_", 0) == 0 && msg.find(".recovered_via_xshut") != std::string::npos) {
    std::string id = msg.substr(sizeof("sensor_") - 1, msg.find('.') - (sizeof("sensor_") - 1));
    out += " - sensor " + id + " recovered via XSHUT";
  } else if (msg == "sensor.recovered_via_xshut") {
    out += " - sensor recovered via XSHUT";
  } else if (msg == "interrupt_fallback_polling" || msg == "interrupt_fallback")
    out += " - INT pin timeout, polling";
  else if (msg == "int_pin_missed")
    out += " - INT miss";
  else if (msg.rfind("int_pin_missed_sensor_", 0) == 0) {
    std::string id = msg.substr(sizeof("int_pin_missed_sensor_") - 1);
    out += " - INT miss sensor " + id;
  } else if (msg.rfind("manual_adjust", 0) == 0)
    out += " - user corrected";
  const char *color = "\033[32m";  // green by default
  if (msg.find("fail") != std::string::npos || msg.find("fallback") != std::string::npos ||
      msg.find("missed") != std::string::npos)
    color = "\033[31m";  // red for errors
  else if (msg.find("retry") != std::string::npos || msg.find("manual_adjust") != std::string::npos ||
           msg.find("reinitialize") != std::string::npos || msg.find("pulse_off") != std::string::npos)
    color = "\033[33m";  // yellow for informational

  std::string colored = std::string(color) + out + "\033[0m";
  ESP_LOGI(TAG, "%s", colored.c_str());
  if (instance_ != nullptr) {
    if (msg.find("reinitialize") != std::string::npos) {
      instance_->update_status_text("reinitializing");
    } else if (msg.find("recovered_via_xshut") != std::string::npos) {
      instance_->update_status_text("ok");
    }
    if (msg == "dual_core_success" || msg == "fallback_single_core" || msg == "force_single_core" ||
        msg == "interrupt_fallback_polling" || msg == "interrupt_recovered") {
      instance_->publish_feature_list();
    }
  }
}

Roode::~Roode() {
  delete entry;
  delete exit;
}

#ifdef USE_WEB_SERVER
bool Roode::check_token_(AsyncWebServerRequest *request) {
  if (portal_password_.empty())
    return true;
  if (request->hasHeader("X-Portal-Token") && request->header("X-Portal-Token") == portal_password_.c_str())
    return true;
  if (request->hasParam("token")) {
    auto param = request->getParam("token");
    if (param->value() == portal_password_.c_str())
      return true;
  }
  request->send(401, "text/plain", "Unauthorized");
  return false;
}
#endif

void Roode::register_server_endpoints() {
#ifdef USE_WEB_SERVER
  if (portal_registered_ || web_server_base::global_web_server_base == nullptr)
    return;
  auto *base = web_server_base::global_web_server_base;

  auto *portal_handler = new AsyncCallbackWebHandler();
  portal_handler->setUri("/portal");
  portal_handler->setMethod(HTTP_GET);
  portal_handler->onRequest([this](AsyncWebServerRequest *request) {
    if (!check_token_(request))
      return;
    request->send_P(200, "text/html", portal_html);
  });
  base->add_handler(portal_handler);

  auto *portal_slash_handler = new AsyncCallbackWebHandler();
  portal_slash_handler->setUri("/portal/");
  portal_slash_handler->setMethod(HTTP_GET);
  portal_slash_handler->onRequest([this](AsyncWebServerRequest *request) {
    if (!check_token_(request))
      return;
    request->send_P(200, "text/html", portal_html);
  });
  base->add_handler(portal_slash_handler);

  auto *root_handler = new AsyncCallbackWebHandler();
  root_handler->setUri("/");
  root_handler->setMethod(HTTP_GET);
  root_handler->onRequest([this](AsyncWebServerRequest *request) {
    if (!check_token_(request))
      return;
    request->redirect("/portal");
  });
  base->add_handler(root_handler);

  portal_registered_ = true;

  auto *settings_handler = new AsyncCallbackWebHandler();
  settings_handler->setUri("/api/settings/current");
  settings_handler->setMethod(HTTP_GET);
  settings_handler->onRequest([this](AsyncWebServerRequest *request) {
    if (!check_token_(request))
      return;
    DynamicJsonDocument doc(512);
    doc["samples"] = samples;
    doc["filter_window"] = filter_window_;
    doc["filter_mode"] = static_cast<int>(filter_mode_);
    doc["orientation"] = static_cast<int>(orientation_);
    doc["invert_direction"] = invert_direction_;
    doc["firmware"] = VERSION;
    doc["last_calibration"] = format_timestamp(last_calibration_sec_);
    JsonObject entry_cfg = doc.createNestedObject("entry");
    JsonObject entry_roi = entry_cfg.createNestedObject("roi");
    entry_roi["center"] = entry->roi->center;
    entry_roi["width"] = entry->roi->width;
    entry_roi["height"] = entry->roi->height;
    JsonObject entry_thr = entry_cfg.createNestedObject("threshold");
    entry_thr["min"] = entry->threshold->min;
    entry_thr["max"] = entry->threshold->max;
    entry_thr["idle"] = entry->threshold->idle;
    JsonObject exit_cfg = doc.createNestedObject("exit");
    JsonObject exit_roi = exit_cfg.createNestedObject("roi");
    exit_roi["center"] = exit->roi->center;
    exit_roi["width"] = exit->roi->width;
    exit_roi["height"] = exit->roi->height;
    JsonObject exit_thr = exit_cfg.createNestedObject("threshold");
    exit_thr["min"] = exit->threshold->min;
    exit_thr["max"] = exit->threshold->max;
    exit_thr["idle"] = exit->threshold->idle;
    std::string out;
    serializeJson(doc, out);
    request->send(200, "application/json", out.c_str());
  });

  auto *scan_status_handler = new AsyncCallbackWebHandler();
  scan_status_handler->setUri("/api/scan/status");
  scan_status_handler->setMethod(HTTP_GET);
  scan_status_handler->onRequest([this](AsyncWebServerRequest *request) {
    if (!check_token_(request))
      return;
    DynamicJsonDocument doc(256);
    doc["running"] = scan_running;
    doc["session_id"] = scan_session_id_.c_str();
    doc["step"] = scan_step_;
    doc["progress"] = scan_progress_;
    doc["last_calibration"] = format_timestamp(last_calibration_sec_);
    std::string out;
    serializeJson(doc, out);
    request->send(200, "application/json", out.c_str());
  });
  base->add_handler(scan_status_handler);

  auto *scan_start_handler = new AsyncCallbackWebHandler();
  scan_start_handler->setUri("/api/scan/start");
  scan_start_handler->setMethod(HTTP_POST);
  scan_start_handler->onRequest([this](AsyncWebServerRequest *request) {
    if (!check_token_(request))
      return;
    DynamicJsonDocument doc(128);
    if (!scan_running) {
      scan_cancel_requested = false;
      scan_running = true;
      scan_session_id_ = std::to_string(millis());
      this->start_passive_scan();
      scan_running = false;
      doc["started"] = true;
      doc["session_id"] = scan_session_id_.c_str();
    } else {
      doc["started"] = false;
      doc["session_id"] = scan_session_id_.c_str();
    }
    std::string out;
    serializeJson(doc, out);
    request->send(200, "application/json", out.c_str());
  });
  base->add_handler(scan_start_handler);

  auto *scan_cancel_handler = new AsyncCallbackWebHandler();
  scan_cancel_handler->setUri("/api/scan/cancel");
  scan_cancel_handler->setMethod(HTTP_POST);
  scan_cancel_handler->onRequest([this](AsyncWebServerRequest *request) {
    if (!check_token_(request))
      return;
    scan_cancel_requested = true;
    DynamicJsonDocument doc(128);
    doc["cancelled"] = true;
    doc["session_id"] = scan_session_id_.c_str();
    std::string out;
    serializeJson(doc, out);
    request->send(200, "application/json", out.c_str());
  });
  base->add_handler(scan_cancel_handler);

  auto *roi_preview_handler = new AsyncCallbackWebHandler();
  roi_preview_handler->setUri("/api/roi/preview");
  roi_preview_handler->setMethod(HTTP_GET);
  roi_preview_handler->onRequest([this](AsyncWebServerRequest *request) {
    if (!check_token_(request))
      return;
    DynamicJsonDocument doc(512);
    if (recommended_settings_.has_value()) {
      JsonObject entry_cfg = doc.createNestedObject("entry");
      JsonObject entry_roi = entry_cfg.createNestedObject("roi");
      entry_roi["center"] = recommended_settings_->entry_roi.center;
      entry_roi["width"] = recommended_settings_->entry_roi.width;
      entry_roi["height"] = recommended_settings_->entry_roi.height;
      JsonObject entry_thr = entry_cfg.createNestedObject("threshold");
      entry_thr["min"] = recommended_settings_->entry_threshold_min;
      entry_thr["max"] = recommended_settings_->entry_threshold_max;
      JsonObject exit_cfg = doc.createNestedObject("exit");
      JsonObject exit_roi = exit_cfg.createNestedObject("roi");
      exit_roi["center"] = recommended_settings_->exit_roi.center;
      exit_roi["width"] = recommended_settings_->exit_roi.width;
      exit_roi["height"] = recommended_settings_->exit_roi.height;
      JsonObject exit_thr = exit_cfg.createNestedObject("threshold");
      exit_thr["min"] = recommended_settings_->exit_threshold_min;
      exit_thr["max"] = recommended_settings_->exit_threshold_max;
      doc["samples"] = recommended_settings_->samples;
      doc["firmware"] = recommended_settings_->firmware.c_str();
      doc["ranging_mode"] = recommended_settings_->ranging_mode.c_str();
    } else {
      JsonObject entry_cfg = doc.createNestedObject("entry");
      JsonObject entry_roi = entry_cfg.createNestedObject("roi");
      entry_roi["center"] = entry->roi->center;
      entry_roi["width"] = entry->roi->width;
      entry_roi["height"] = entry->roi->height;
      JsonObject entry_thr = entry_cfg.createNestedObject("threshold");
      entry_thr["min"] = entry->threshold->min;
      entry_thr["max"] = entry->threshold->max;
      JsonObject exit_cfg = doc.createNestedObject("exit");
      JsonObject exit_roi = exit_cfg.createNestedObject("roi");
      exit_roi["center"] = exit->roi->center;
      exit_roi["width"] = exit->roi->width;
      exit_roi["height"] = exit->roi->height;
      JsonObject exit_thr = exit_cfg.createNestedObject("threshold");
      exit_thr["min"] = exit->threshold->min;
      exit_thr["max"] = exit->threshold->max;
      doc["samples"] = samples;
      doc["firmware"] = VERSION;
      auto mode = this->distanceSensor->get_ranging_mode_override();
      if (mode.has_value())
        doc["ranging_mode"] = mode.value()->name;
    }
    std::string out;
    serializeJson(doc, out);
    request->send(200, "application/json", out.c_str());
  });
  base->add_handler(roi_preview_handler);

  auto *roi_apply_handler = new AsyncCallbackWebHandler();
  roi_apply_handler->setUri("/api/roi/apply");
  roi_apply_handler->setMethod(HTTP_POST);
  roi_apply_handler->onRequest([this](AsyncWebServerRequest *request) {
    if (!check_token_(request))
      return;
    bool ok = this->apply_recommended_settings();
    DynamicJsonDocument doc(128);
    doc["ok"] = ok;
    if (!ok)
      doc["error"] = "no_recommendation";
    std::string out;
    serializeJson(doc, out);
    request->send(200, "application/json", out.c_str());
  });
  base->add_handler(roi_apply_handler);

  auto *scan_sessions_handler = new AsyncCallbackWebHandler();
  scan_sessions_handler->setUri("/api/scan/sessions");
  scan_sessions_handler->setMethod(HTTP_GET);
  scan_sessions_handler->onRequest([this](AsyncWebServerRequest *request) {
    if (!check_token_(request))
      return;
    size_t doc_size = sessions_.size() * 128 + 128;
    DynamicJsonDocument doc(doc_size);
    JsonArray arr = doc.createNestedArray("sessions");
    for (const auto &s : sessions_) {
      JsonObject obj = arr.createNestedObject();
      obj["id"] = s.id;
      obj["timestamp"] = format_epoch(s.timestamp).c_str();
      obj["trials"] = s.trials;
      obj["duration"] = s.duration;
      obj["size"] = s.size;
    }
    std::string out;
    serializeJson(doc, out);
    request->send(200, "application/json", out.c_str());
  });
  base->add_handler(scan_sessions_handler);

  auto *scan_session_handler = new AsyncCallbackWebHandler();
  scan_session_handler->setUri("^/api/scan/session/(.+)$");
  scan_session_handler->setMethod(HTTP_GET);
  scan_session_handler->onRequest([this](AsyncWebServerRequest *request) {
    if (!check_token_(request))
      return;
    uint32_t id = strtoul(request->pathArg(0).c_str(), nullptr, 10);
    const ScanSession *found = nullptr;
    for (const auto &s : sessions_) {
      if (s.id == id) {
        found = &s;
        break;
      }
    }
    size_t doc_size = found ? MAX_SESSION_DATA + 128 : 64;
    DynamicJsonDocument doc(doc_size);
    if (found) {
      doc["id"] = found->id;
      doc["timestamp"] = format_epoch(found->timestamp).c_str();
      doc["trials"] = found->trials;
      doc["duration"] = found->duration;
      doc["size"] = found->size;
      doc["data"] = found->data;
    } else {
      doc["error"] = "not_found";
    }
    std::string out;
    serializeJson(doc, out);
    request->send(200, "application/json", out.c_str());
  });
  base->add_handler(scan_session_handler);

  auto *scan_delete_handler = new AsyncCallbackWebHandler();
  scan_delete_handler->setUri("/api/scan/delete");
  scan_delete_handler->setMethod(HTTP_POST);
  scan_delete_handler->onRequest([this](AsyncWebServerRequest *request) {
    if (!check_token_(request))
      return;
    for (int i = 0; i < MAX_SCAN_SESSIONS; i++)
      session_prefs_[i].erase();
    sessions_.clear();
    session_next_ = 0;
    session_index_pref_.save(&session_next_);
    DynamicJsonDocument doc(64);
    doc["deleted"] = true;
    std::string out;
    serializeJson(doc, out);
    request->send(200, "application/json", out.c_str());
  });
  base->add_handler(scan_delete_handler);

  auto *export_all_handler = new AsyncCallbackWebHandler();
  export_all_handler->setUri("/api/export/all");
  export_all_handler->setMethod(HTTP_GET);
  export_all_handler->onRequest([this](AsyncWebServerRequest *request) {
    if (!check_token_(request))
      return;
    size_t doc_size = sessions_.size() * (MAX_SESSION_DATA + 128) + 128;
    DynamicJsonDocument doc(doc_size);
    JsonArray arr = doc.createNestedArray("sessions");
    for (const auto &s : sessions_) {
      JsonObject obj = arr.createNestedObject();
      obj["id"] = s.id;
      obj["timestamp"] = format_epoch(s.timestamp).c_str();
      obj["trials"] = s.trials;
      obj["duration"] = s.duration;
      obj["size"] = s.size;
      obj["data"] = s.data;
    }
    std::string out;
    serializeJson(doc, out);
    request->send(200, "application/json", out.c_str());
  });
  base->add_handler(export_all_handler);
#endif
}

void Roode::set_auto_calibration_interval_sec(uint32_t sec) { auto_calibration_interval_sec_ = sec; }
void Roode::dump_config() {
  ESP_LOGCONFIG(TAG, "Roode:");
  ESP_LOGCONFIG(TAG, "  Sample size: %d", samples);
  LOG_UPDATE_INTERVAL(this);
  entry->dump_config();
  exit->dump_config();
}

void Roode::setup() {
  ESP_LOGI(SETUP, "Booting Roode %s", VERSION);
  session_index_pref_ = global_preferences->make_preference<uint8_t>(0xBF);
  session_index_pref_.load(&session_next_);
  for (uint8_t i = 0; i < MAX_SCAN_SESSIONS; i++) {
    session_prefs_[i] = global_preferences->make_preference<ScanSession>(0xB0 + i);
    ScanSession s;
    if (session_prefs_[i].load(&s))
      sessions_.push_back(s);
  }
  if (session_next_ >= MAX_SCAN_SESSIONS)
    session_next_ = sessions_.size() % MAX_SCAN_SESSIONS;
  // `register_service` is only available when API services are enabled.
  // Guard the call so that the component can compile even when the
  // USE_API_SERVICES compile-time flag is disabled.
#ifdef USE_API_SERVICES
  this->register_service(&Roode::start_passive_scan, "start_passive_scan");
  this->register_service(&Roode::complete_calibration, "complete_calibration");
#endif
#ifdef USE_WEB_SERVER
  if (web_server_base::global_web_server_base != nullptr) {
    auto *base = web_server_base::global_web_server_base;
    base->init();
    register_server_endpoints();
  } else {
    ESP_LOGW(TAG, "Web server base not initialized, portal not started");
  }
#endif
  if (version_sensor != nullptr) {
    version_sensor->publish_state(VERSION);
  }
  ESP_LOGI(SETUP, "Using sampling with sampling size: %d", samples);

  if (this->distanceSensor->is_failed()) {
    this->mark_failed();
    update_status_text("offline");
    ESP_LOGE(TAG, "Roode cannot be setup without a valid VL53L1X sensor");
    return;
  }

  // Initialize filtering options before calibrating so threshold sampling uses
  // the configured window and mode
  entry->set_filter_window(filter_window_);
  entry->set_filter_mode(filter_mode_);
  exit->set_filter_window(filter_window_);
  exit->set_filter_mode(filter_mode_);

  if (calibration_persistence_) {
    calibration_prefs_[0] = global_preferences->make_preference<CalibrationPrefs>(0xA0);
    calibration_prefs_[1] = global_preferences->make_preference<CalibrationPrefs>(0xA1);
    bool loaded = true;
    for (int i = 0; i < 2; i++) {
      if (calibration_prefs_[i].load(&calibration_data_[i])) {
        Zone *z = i == 0 ? entry : exit;
        z->threshold->idle = calibration_data_[i].baseline_mm;
        z->threshold->min = calibration_data_[i].threshold_min_mm;
        z->threshold->max = calibration_data_[i].threshold_max_mm;
        int valid_count = 0;
        for (int s = 0; s < 5; s++) {
          z->readDistance(distanceSensor);
          if (abs((int) z->getDistance() - (int) z->threshold->idle) < (z->threshold->idle * 0.1))
            valid_count++;
        }
        if (valid_count < 5) {
          loaded = false;
          break;
        }
      } else {
        loaded = false;
        break;
      }
    }
    if (loaded) {
      entry->reset_roi(orientation_ == Parallel ? 167 : 195);
      exit->reset_roi(orientation_ == Parallel ? 231 : 60);
      entry->roi_calibration(entry->threshold->idle, exit->threshold->idle, orientation_);
      exit->roi_calibration(entry->threshold->idle, exit->threshold->idle, orientation_);
      auto *mode = determine_ranging_mode(entry->threshold->idle, exit->threshold->idle);
      distanceSensor->set_ranging_mode(mode);
      publish_sensor_configuration(entry, exit, true);
      publish_sensor_configuration(entry, exit, false);
    } else {
      calibrate_zones();
    }
  } else {
    calibrate_zones();
  }
  last_calibration_sec_ = std::max(calibration_data_[0].last_calibrated_sec, calibration_data_[1].last_calibrated_sec);
#ifdef CONFIG_IDF_TARGET_ESP32
  if (!force_single_core_) {
    log_event("use_dual_core");
    vTaskDelay(pdMS_TO_TICKS(200));
    BaseType_t res = xTaskCreatePinnedToCore(sensor_task, "SensorTask", 4096, this, 1, &sensor_task_handle_, 1);
    multicore_retry_count_ = 0;
    while (res != pdPASS && multicore_retry_count_ < 2) {
      multicore_retry_count_++;
      log_event(std::string("retry_multicore_") + std::to_string(multicore_retry_count_));
      vTaskDelay(pdMS_TO_TICKS(200));
      res = xTaskCreatePinnedToCore(sensor_task, "SensorTask", 4096, this, 1, &sensor_task_handle_, 1);
    }
    if (res == pdPASS) {
      use_sensor_task_ = true;
      log_event("dual_core_success");
    } else {
      log_event("dual_core_failed");
      log_event("fallback_single_core");
      use_sensor_task_ = false;
    }
  } else {
    use_sensor_task_ = false;
    log_event("force_single_core");
  }
#else
  use_sensor_task_ = false;
#endif
  loop_window_start_ = millis();
  loop_time_sum_ = 0;
  loop_count_ = 0;
  if (status_sensor != nullptr)
    status_sensor->publish_state(sensor_status);

  if (loop_time_sensor != nullptr)
    loop_time_sensor->publish_state(0);
  if (cpu_usage_sensor != nullptr)
    cpu_usage_sensor->publish_state(0);
  if (ram_free_sensor != nullptr)
    ram_free_sensor->publish_state(0);
  if (flash_free_sensor != nullptr)
    flash_free_sensor->publish_state(0);
  manual_adjustment_count_ = 0;
  if (manual_adjustment_sensor != nullptr)
    manual_adjustment_sensor->publish_state(0);
  if (xshut_state_binary_sensor != nullptr) {
    auto val = distanceSensor->get_xshut_state();
    if (val.has_value())
      xshut_state_binary_sensor->publish_state(*val);
  }
  if (interrupt_status_sensor != nullptr) {
    auto val = distanceSensor->get_interrupt_state();
    if (val.has_value())
      interrupt_status_sensor->publish_state(*val ? 1 : 0);
  }
  if (people_counter != nullptr)
    expected_counter_ = people_counter->state;

  publish_feature_list();
  update_status_text("ok");
}

void Roode::update() {
  if (distance_entry != nullptr) {
    distance_entry->publish_state(entry->getDistance());
  }
  if (distance_exit != nullptr) {
    distance_exit->publish_state(exit->getDistance());
  }
  if (xshut_state_binary_sensor != nullptr) {
    auto val = distanceSensor->get_xshut_state();
    if (val.has_value())
      xshut_state_binary_sensor->publish_state(*val);
  }
  if (interrupt_status_sensor != nullptr) {
    auto val = distanceSensor->get_interrupt_state();
    if (val.has_value())
      interrupt_status_sensor->publish_state(*val ? 1 : 0);
  }
  if (people_counter != nullptr && fabs(people_counter->state - expected_counter_) > 0.001f) {
    int diff = (int) roundf(people_counter->state - expected_counter_);
    manual_adjustment_count_ += abs(diff);
    expected_counter_ = people_counter->state;
    if (manual_adjustment_sensor != nullptr)
      manual_adjustment_sensor->publish_state(manual_adjustment_count_);
    if (diff != 0) {
      std::string sign = diff > 0 ? "+" : "";
      log_event("manual_adjust " + sign + std::to_string(diff) + " total=" + std::to_string(manual_adjustment_count_));
    }
  }
}

void Roode::loop() {
#ifdef USE_WEB_SERVER
  if (!portal_registered_ && web_server_base::global_web_server_base != nullptr) {
    auto *base = web_server_base::global_web_server_base;
    base->init();
    register_server_endpoints();
  }
#endif
  if (use_sensor_task_) {
    // When running on dual core the sensor loop runs in a separate task
    // Skip execution from main loop
    return;
  }
  uint32_t now = millis();
  if (last_loop_update_ts_ != 0 && (now - last_loop_update_ts_ > restart_timeout_ms_) &&
      (now - last_sensor_restart_ts_ > restart_timeout_ms_)) {
    ESP_LOGW(TAG, "Sensor unresponsive >%ds, restarting...", restart_timeout_ms_ / 1000);
    restart_sensor();
  }
  unsigned long start = micros();
  VL53L1_Error status = this->current_zone->readDistance(distanceSensor);
  if (status == VL53L1_ERROR_NONE)
    last_loop_update_ts_ = millis();
  uint16_t dist = this->current_zone->getDistance();
  if (status == VL53L1_ERROR_NONE && (dist == 0 || dist > 4000)) {
    invalid_read_count_++;
  } else {
    invalid_read_count_ = 0;
  }
  if (invalid_read_count_ > invalid_distance_limit_ && (now - last_sensor_restart_ts_ > restart_timeout_ms_)) {
    ESP_LOGW(TAG, "Consecutive invalid distances, restarting...");
    restart_sensor();
  }
  bool zone_trig = current_zone->getMinDistance() < current_zone->threshold->max &&
                   current_zone->getMinDistance() > current_zone->threshold->min;
  if (!cpu_optimizations_active_ || zone_trig)
    path_tracking(this->current_zone);
  handle_sensor_status();
  this->current_zone = this->current_zone == this->entry ? this->exit : this->entry;
  unsigned long end = micros();
  unsigned long delta = end - start;
  loop_time_sum_ += delta;
  loop_count_++;
  update_metrics();
  uint32_t now_sec = millis() / 1000;
  if (auto_calibration_interval_sec_ > 0 && now_sec - last_calibration_sec_ >= auto_calibration_interval_sec_) {
    bool area_clear = false;
    if (presence_sensor != nullptr) {
      area_clear = !presence_sensor->state;
    } else {
      bool entry_present =
          entry->getMinDistance() < entry->threshold->max && entry->getMinDistance() > entry->threshold->min;
      bool exit_present =
          exit->getMinDistance() < exit->threshold->max && exit->getMinDistance() > exit->threshold->min;
      area_clear = !entry_present && !exit_present;
    }
    if (area_clear) {
      run_zone_calibration(0);
      run_zone_calibration(1);
    } else {
      ESP_LOGD(CALIBRATION, "Auto calibration skipped, presence detected");
    }
  }
  delay(polling_interval_ms_);
}

bool Roode::handle_sensor_status() {
  bool check_status = false;
  std::string text_state;
  if (distanceSensor->is_failed()) {
    text_state = "offline";
  } else if (sensor_status == VL53L1_ERROR_NONE) {
    text_state = "ok";
    if (last_sensor_status != sensor_status) {
      if (status_sensor != nullptr)
        status_sensor->publish_state(sensor_status);
      check_status = true;
    }
  } else if (sensor_status == VL53L1_ERROR_TIME_OUT) {
    text_state = "timeout";
    if (status_sensor != nullptr)
      status_sensor->publish_state(sensor_status);
  } else {
    text_state = "error";
    if (status_sensor != nullptr)
      status_sensor->publish_state(sensor_status);
  }

  update_status_text(text_state);
  last_sensor_status = sensor_status;
  sensor_status = VL53L1_ERROR_NONE;
  return check_status;
}

void Roode::path_tracking(Zone *zone) {
  static int PathTrack[] = {0, 0, 0, 0};
  static int PathTrackFillingSize = 1;  // init this to 1 as we start from state
                                        // where nobody is any of the zones
  static int LeftPreviousStatus = NOBODY;
  static int RightPreviousStatus = NOBODY;
  int CurrentZoneStatus = NOBODY;
  int AllZonesCurrentStatus = 0;
  int AnEventHasOccured = 0;

  uint32_t timeout = state_ == STATE_ENTRY_ACTIVE ? 2500 : 3500;
  if (state_ != STATE_IDLE && millis() - state_started_ts > timeout) {
    state_ = STATE_IDLE;
    ESP_LOGW(TAG, "fsm_timeout_reset");
  }

  // PathTrack algorithm
  if (zone->getMinDistance() < zone->threshold->max && zone->getMinDistance() > zone->threshold->min) {
    // Someone is in the sensing area
    CurrentZoneStatus = SOMEONE;
    if (presence_sensor != nullptr) {
      presence_sensor->publish_state(true);
    }
    if (zone_triggered_start_[zone->id] == 0) {
      zone_triggered_start_[zone->id] = millis();
    }
  }
  if (CurrentZoneStatus == NOBODY) {
    zone_triggered_start_[zone->id] = 0;
  } else if (zone_triggered_start_[zone->id] != 0 && millis() - zone_triggered_start_[zone->id] >= 10000 &&
             millis() - last_valid_crossing_ts_ >= 120000) {
    ESP_LOGI(CALIBRATION, "Fail safe calibration triggered for zone %d", zone->id);
    run_zone_calibration(zone->id);
    fail_safe_triggered_ = true;
    zone_triggered_start_[zone->id] = 0;
  }

  // left zone
  if (zone == (this->invert_direction_ ? this->exit : this->entry)) {
    if (CurrentZoneStatus != LeftPreviousStatus) {
      // event in left zone has occured
      AnEventHasOccured = 1;

      if (CurrentZoneStatus == SOMEONE) {
        state_ = STATE_ENTRY_ACTIVE;
        state_started_ts = millis();
      }

      if (CurrentZoneStatus == SOMEONE) {
        AllZonesCurrentStatus += 1;
      }
      // need to check right zone as well ...
      if (RightPreviousStatus == SOMEONE) {
        // event in right zone has occured
        AllZonesCurrentStatus += 2;
      }
      // remember for next time
      LeftPreviousStatus = CurrentZoneStatus;
    }
  }
  // right zone
  else {
    if (CurrentZoneStatus != RightPreviousStatus) {
      // event in right zone has occured
      AnEventHasOccured = 1;
      if (CurrentZoneStatus == SOMEONE) {
        AllZonesCurrentStatus += 2;
        if (state_ == STATE_ENTRY_ACTIVE) {
          state_ = STATE_BOTH_ACTIVE;
          state_started_ts = millis();
        }
      }
      // need to check left zone as well ...
      if (LeftPreviousStatus == SOMEONE) {
        // event in left zone has occured
        AllZonesCurrentStatus += 1;
      }
      // remember for next time
      RightPreviousStatus = CurrentZoneStatus;
    }
  }

  // if an event has occured
  if (AnEventHasOccured) {
    if (PathTrackFillingSize < 4) {
      PathTrackFillingSize++;
    }

    // if nobody anywhere lets check if an exit or entry has happened
    if ((LeftPreviousStatus == NOBODY) && (RightPreviousStatus == NOBODY)) {
      // check exit or entry only if PathTrackFillingSize is 4 (for example 0 1
      // 3 2) and last event is 0 (nobobdy anywhere)
      if (PathTrackFillingSize == 4) {
        // check exit or entry. no need to check PathTrack[0] == 0 , it is
        // always the case

        if ((PathTrack[1] == 1) && (PathTrack[2] == 3) && (PathTrack[3] == 2)) {
          // This an exit
          ESP_LOGI("Roode pathTracking", "Exit detected.");

          this->updateCounter(-1);
          last_valid_crossing_ts_ = millis();
          if (entry_exit_event_sensor != nullptr) {
            entry_exit_event_sensor->publish_state("Exit");
          }
        } else if ((PathTrack[1] == 2) && (PathTrack[2] == 3) && (PathTrack[3] == 1)) {
          // This an entry
          ESP_LOGI("Roode pathTracking", "Entry detected.");
          this->updateCounter(1);
          last_valid_crossing_ts_ = millis();
          if (entry_exit_event_sensor != nullptr) {
            entry_exit_event_sensor->publish_state("Entry");
          }
        }
      }

      PathTrackFillingSize = 1;
      state_ = STATE_IDLE;
    } else {
      // update PathTrack
      // example of PathTrack update
      // 0
      // 0 1
      // 0 1 3
      // 0 1 3 1
      // 0 1 3 3
      // 0 1 3 2 ==> if next is 0 : check if exit
      PathTrack[PathTrackFillingSize - 1] = AllZonesCurrentStatus;
    }
  }
  if (presence_sensor != nullptr) {
    if (CurrentZoneStatus == NOBODY && LeftPreviousStatus == NOBODY && RightPreviousStatus == NOBODY) {
      // nobody is in the sensing area
      presence_sensor->publish_state(false);
    }
  }
}
void Roode::updateCounter(int delta) {
  if (this->people_counter == nullptr) {
    return;
  }
  auto next = this->people_counter->state + (float) delta;
  ESP_LOGI(TAG, "Updating people count: %d", (int) next);
  expected_counter_ = next;
  auto call = this->people_counter->make_call();
  call.set_value(next);
  call.perform();
}
void Roode::recalibration() { calibrate_zones(); }
void Roode::run_zone_calibration(uint8_t zone_id) {
  ESP_LOGI(CALIBRATION, "Calibration triggered for zone %d", zone_id);
  Zone *z = zone_id == 0 ? entry : exit;
  z->reset_roi(zone_id == 0 ? (orientation_ == Parallel ? 167 : 195) : (orientation_ == Parallel ? 231 : 60));
  z->calibrateThreshold(distanceSensor, 50);
  // Recalculate ROI sizes so thresholds remain consistent
  entry->roi_calibration(entry->threshold->idle, exit->threshold->idle, orientation_);
  exit->roi_calibration(entry->threshold->idle, exit->threshold->idle, orientation_);
  auto *mode = determine_ranging_mode(entry->threshold->idle, exit->threshold->idle);
  distanceSensor->set_ranging_mode(mode);

  calibration_data_[zone_id].baseline_mm = z->threshold->idle;
  calibration_data_[zone_id].threshold_min_mm = z->threshold->min;
  calibration_data_[zone_id].threshold_max_mm = z->threshold->max;
  calibration_data_[zone_id].last_calibrated_sec = millis() / 1000;
  if (calibration_persistence_) {
    calibration_prefs_[zone_id].save(&calibration_data_[zone_id]);
  }

  App.feed_wdt();

  // Publish the updated calibration data so Home Assistant sees the new
  // thresholds and ROI values immediately after a fail-safe recalibration
  publish_sensor_configuration(entry, exit, true);
  publish_sensor_configuration(entry, exit, false);
  last_calibration_sec_ = std::max(calibration_data_[0].last_calibrated_sec, calibration_data_[1].last_calibrated_sec);
  publish_feature_list();
}

bool Roode::apply_recommended_settings() {
  if (!recommended_settings_.has_value()) {
    ESP_LOGW(TAG, "No recommended settings to apply");
    return false;
  }

  ESP_LOGI(TAG, "Applying recommended calibration settings");
  ESP_LOGI(CALIBRATION, "Entry ROI {center: %d, width: %d, height: %d}", recommended_settings_->entry_roi.center,
           recommended_settings_->entry_roi.width, recommended_settings_->entry_roi.height);
  ESP_LOGI(CALIBRATION, "Entry thresholds {min: %d, max: %d}", recommended_settings_->entry_threshold_min,
           recommended_settings_->entry_threshold_max);
  ESP_LOGI(CALIBRATION, "Exit ROI {center: %d, width: %d, height: %d}", recommended_settings_->exit_roi.center,
           recommended_settings_->exit_roi.width, recommended_settings_->exit_roi.height);
  ESP_LOGI(CALIBRATION, "Exit thresholds {min: %d, max: %d}", recommended_settings_->exit_threshold_min,
           recommended_settings_->exit_threshold_max);
  ESP_LOGI(CALIBRATION, "Samples: %d, Ranging mode: %s", recommended_settings_->samples,
           recommended_settings_->ranging_mode.c_str());

  entry->roi->center = recommended_settings_->entry_roi.center;
  entry->roi->width = recommended_settings_->entry_roi.width;
  entry->roi->height = recommended_settings_->entry_roi.height;
  entry->threshold->min = recommended_settings_->entry_threshold_min;
  entry->threshold->max = recommended_settings_->entry_threshold_max;

  exit->roi->center = recommended_settings_->exit_roi.center;
  exit->roi->width = recommended_settings_->exit_roi.width;
  exit->roi->height = recommended_settings_->exit_roi.height;
  exit->threshold->min = recommended_settings_->exit_threshold_min;
  exit->threshold->max = recommended_settings_->exit_threshold_max;

  samples = recommended_settings_->samples;
  entry->set_max_samples(samples);
  exit->set_max_samples(samples);

  const RangingMode *mode = Ranging::Short;
  if (recommended_settings_->ranging_mode == "medium")
    mode = Ranging::Medium;
  else if (recommended_settings_->ranging_mode == "long")
    mode = Ranging::Long;
  distanceSensor->set_ranging_mode(mode);

  publish_sensor_configuration(entry, exit, true);
  publish_sensor_configuration(entry, exit, false);

  calibration_data_[0] = {entry->threshold->idle, entry->threshold->min, entry->threshold->max, millis() / 1000};
  calibration_data_[1] = {exit->threshold->idle, exit->threshold->min, exit->threshold->max, millis() / 1000};
  if (calibration_persistence_) {
    calibration_prefs_[0].save(&calibration_data_[0]);
    calibration_prefs_[1].save(&calibration_data_[1]);
  }
  last_calibration_sec_ = std::max(calibration_data_[0].last_calibrated_sec, calibration_data_[1].last_calibrated_sec);

  recommended_settings_.reset();
  ESP_LOGI(TAG, "Recommended calibration settings applied");
  return true;
}

void Roode::apply_cpu_optimizations(float cpu) {
  if (cpu_optimizations_active_ || cpu <= cpu_opt_activate_threshold_)
    return;
  ESP_LOGW(TAG, "CPU usage %.1f%% exceeded threshold, applying optimizations", cpu);
  polling_interval_ms_ = 30;

  // Avoid extremely small windows and accuracy-reducing filters.
  if (filter_window_ < 5) {
    filter_window_ = 5;
    entry->set_filter_window(5);
    exit->set_filter_window(5);
  }
  if (filter_mode_ != FILTER_MEDIAN) {
    filter_mode_ = FILTER_MEDIAN;
    entry->set_filter_mode(FILTER_MEDIAN);
    exit->set_filter_mode(FILTER_MEDIAN);
  }
  cpu_optimizations_active_ = true;
}

void Roode::reset_cpu_optimizations(float cpu) {
  if (!cpu_optimizations_active_ || cpu > cpu_opt_deactivate_threshold_)
    return;
  ESP_LOGI(TAG, "CPU usage %.1f%% stable, reverting optimizations", cpu);
  polling_interval_ms_ = 10;
  filter_window_ = default_filter_window_;
  entry->set_filter_window(default_filter_window_);
  exit->set_filter_window(default_filter_window_);
  filter_mode_ = default_filter_mode_;
  entry->set_filter_mode(default_filter_mode_);
  exit->set_filter_mode(default_filter_mode_);
  cpu_optimizations_active_ = false;
}

void Roode::update_metrics() {
  uint32_t now = millis();
  if (now - loop_window_start_ < 10000)
    return;
  float cpu = 0.0f;
  if (loop_count_ > 0) {
    float avg_ms = (float) loop_time_sum_ / loop_count_ / 1000.0f;
    if (loop_time_sensor != nullptr)
      loop_time_sensor->publish_state(avg_ms);
    cpu = ((float) loop_time_sum_ / ((now - loop_window_start_) * 1000.0f)) * 100.0f;
    if (cpu_usage_sensor != nullptr)
      cpu_usage_sensor->publish_state(cpu);
  }
  if (ram_free_sensor != nullptr) {
    uint32_t total_heap = ESP.getHeapSize();
    float used_percent = 0;
    if (total_heap > 0) {
      uint32_t used = total_heap - ESP.getFreeHeap();
      used_percent = ((float) used / (float) total_heap) * 100.0f;
    }
    ram_free_sensor->publish_state(used_percent);
  }
  if (flash_free_sensor != nullptr) {
    uint32_t total_flash = ESP.getFlashChipSize();
    float used_percent = 0;
    if (total_flash > 0) {
      uint32_t used = total_flash - ESP.getFreeSketchSpace();
      used_percent = ((float) used / (float) total_flash) * 100.0f;
    }
    flash_free_sensor->publish_state(used_percent);
  }
  apply_cpu_optimizations(cpu);
  reset_cpu_optimizations(cpu);
  loop_time_sum_ = 0;
  loop_count_ = 0;
  loop_window_start_ = now;
}

const RangingMode *Roode::determine_ranging_mode(uint16_t average_entry_zone_distance,
                                                 uint16_t average_exit_zone_distance) {
  uint16_t min = average_entry_zone_distance < average_exit_zone_distance ? average_entry_zone_distance
                                                                          : average_exit_zone_distance;
  uint16_t max = average_entry_zone_distance > average_exit_zone_distance ? average_entry_zone_distance
                                                                          : average_exit_zone_distance;
  if (min <= short_distance_threshold) {
    return Ranging::Short;
  }
  if (max > short_distance_threshold && min <= medium_distance_threshold) {
    return Ranging::Medium;
  }
  if (max > medium_distance_threshold && min <= medium_long_distance_threshold) {
    return Ranging::Long;
  }
  if (max > medium_long_distance_threshold && min <= long_distance_threshold) {
    return Ranging::Longer;
  }
  return Ranging::Longest;
}

void Roode::calibrate_zones() {
  ESP_LOGI(SETUP, "Calibrating sensor zones");

  entry->reset_roi(orientation_ == Parallel ? 167 : 195);
  exit->reset_roi(orientation_ == Parallel ? 231 : 60);

  calibrateDistance();

  entry->roi_calibration(entry->threshold->idle, exit->threshold->idle, orientation_);
  entry->calibrateThreshold(distanceSensor, 50);
  exit->roi_calibration(entry->threshold->idle, exit->threshold->idle, orientation_);
  exit->calibrateThreshold(distanceSensor, 50);

  publish_sensor_configuration(entry, exit, true);
  App.feed_wdt();
  publish_sensor_configuration(entry, exit, false);

  calibration_data_[0] = {entry->threshold->idle, entry->threshold->min, entry->threshold->max, millis() / 1000};
  calibration_data_[1] = {exit->threshold->idle, exit->threshold->min, exit->threshold->max, millis() / 1000};

  if (calibration_persistence_) {
    calibration_prefs_[0].save(&calibration_data_[0]);
    calibration_prefs_[1].save(&calibration_data_[1]);
  }
  ESP_LOGI(SETUP, "Finished calibrating sensor zones");
  last_calibration_sec_ = std::max(calibration_data_[0].last_calibrated_sec, calibration_data_[1].last_calibrated_sec);
  publish_feature_list();
}

void Roode::calibrateDistance() {
  auto *const initial = distanceSensor->get_ranging_mode_override().value_or(Ranging::Longest);
  distanceSensor->set_ranging_mode(initial);

  entry->calibrateThreshold(distanceSensor, 50);
  exit->calibrateThreshold(distanceSensor, 50);

  if (distanceSensor->get_ranging_mode_override().has_value()) {
    return;
  }
  auto *mode = determine_ranging_mode(entry->threshold->idle, exit->threshold->idle);
  if (mode != initial) {
    distanceSensor->set_ranging_mode(mode);
  }
}

void Roode::publish_sensor_configuration(Zone *entry, Zone *exit, bool isMax) {
  if (isMax) {
    if (max_threshold_entry_sensor != nullptr) {
      max_threshold_entry_sensor->publish_state(entry->threshold->max);
    }

    if (max_threshold_exit_sensor != nullptr) {
      max_threshold_exit_sensor->publish_state(exit->threshold->max);
    }
  } else {
    if (min_threshold_entry_sensor != nullptr) {
      min_threshold_entry_sensor->publish_state(entry->threshold->min);
    }
    if (min_threshold_exit_sensor != nullptr) {
      min_threshold_exit_sensor->publish_state(exit->threshold->min);
    }
  }

  if (entry_roi_height_sensor != nullptr) {
    entry_roi_height_sensor->publish_state(entry->roi->height);
  }
  if (entry_roi_width_sensor != nullptr) {
    entry_roi_width_sensor->publish_state(entry->roi->width);
  }

  if (exit_roi_height_sensor != nullptr) {
    exit_roi_height_sensor->publish_state(exit->roi->height);
  }
  if (exit_roi_width_sensor != nullptr) {
    exit_roi_width_sensor->publish_state(exit->roi->width);
  }
}

void Roode::publish_feature_list() {
  auto fmt_bytes = [](uint32_t bytes) {
    char buf[16];
    if (bytes >= 1024UL * 1024UL * 1024UL)
      snprintf(buf, sizeof(buf), "%uGB", bytes / 1024 / 1024 / 1024);
    else if (bytes >= 1024 * 1024)
      snprintf(buf, sizeof(buf), "%uMB", bytes / 1024 / 1024);
    else
      snprintf(buf, sizeof(buf), "%uKB", bytes / 1024);
    return std::string(buf);
  };

  std::vector<std::pair<std::string, std::string>> features;
#ifdef CONFIG_IDF_TARGET_ESP32
  features.push_back({"cpu_mode", use_sensor_task_ ? "dual" : "single"});
  features.push_back({"cpu", ESP.getChipModel()});
  features.push_back({"cpu_cores", std::to_string(ESP.getChipCores())});
#else
  features.push_back({"cpu_mode", "single"});
  features.push_back({"cpu", "ESP8266"});
  features.push_back({"cpu_cores", "1"});
#endif
  features.push_back({"xshut", distanceSensor->get_xshut_state().has_value() ? "enabled" : "disabled"});
  features.push_back({"refresh", distanceSensor->is_interrupt_enabled() ? "interrupt" : "polling"});
  features.push_back({"ram", fmt_bytes(ESP.getHeapSize())});
  features.push_back({"flash", fmt_bytes(ESP.getFlashChipSize())});
  features.push_back({"calibration_value", std::to_string(entry->threshold->idle)});
  uint32_t last_cal_sec = std::max(calibration_data_[0].last_calibrated_sec, calibration_data_[1].last_calibrated_sec);
  features.push_back({"calibration_sec", std::to_string(last_cal_sec)});

  std::string feature_list;
  for (size_t i = 0; i < features.size(); ++i) {
    feature_list += features[i].first + ":" + features[i].second;
    if (i + 1 < features.size())
      feature_list += "\n";
  }
  if (enabled_features_sensor != nullptr)
    enabled_features_sensor->publish_state(feature_list);
  log_event(std::string("features_enabled: ") + feature_list);
}

void Roode::update_status_text(const std::string &status) {
  if (status_text_sensor != nullptr && status != last_status_text_) {
    status_text_sensor->publish_state(status);
    last_status_text_ = status;
  }
}

void Roode::publish_scan_record(const std::string &payload) {
  if (entry_exit_event_sensor != nullptr)
    entry_exit_event_sensor->publish_state(payload);
  log_event(payload);
  if (current_session_blob_.size() + payload.size() + 1 <= MAX_SESSION_DATA) {
    current_session_blob_ += payload;
    current_session_blob_ += '\n';
  }
  scan_record_count_++;
}

void Roode::start_passive_scan() {
  scan_start_ts_ = millis();
  scan_record_count_ = 0;
  current_session_blob_.clear();
  scan_cancel_requested = false;
  scan_running = true;
  scan_session_id_ = std::to_string(scan_start_ts_);
  const std::vector<int> grids{4, 8, 16};
  const char *modes[] = {"short", "medium", "long"};
  constexpr int base_trials = 3;
  constexpr int max_trials = 5;
  scan_step_ = 0;
  scan_total_steps_ = grids.size() * (sizeof(modes) / sizeof(modes[0])) * max_trials;
  scan_progress_ = 0.0f;
  recommended_settings_.reset();
  struct ScanGroup {
    int grid;
    std::string mode;
    std::vector<std::vector<int>> mcps_trials;
    std::vector<std::vector<int>> distance_trials;
  };
  std::vector<ScanGroup> groups;
  for (int grid : grids) {
    if (scan_cancel_requested) {
      publish_scan_record("scan_cancel");
      scan_running = false;
      return;
    }
    if (grid == 16 && scan_time_cap_seconds_ > 0) {
      float elapsed = (millis() - scan_start_ts_) / 1000.0f;
      if (elapsed > scan_time_cap_seconds_) {
        publish_scan_record("scan_time_cap_reached");
        break;
      }
    }
    for (const char *mode : modes) {
      if (scan_cancel_requested) {
        publish_scan_record("scan_cancel");
        scan_running = false;
        return;
      }
      const RangingMode *ranging_mode = Ranging::Short;
      if (strcmp(mode, "medium") == 0)
        ranging_mode = Ranging::Medium;
      else if (strcmp(mode, "long") == 0)
        ranging_mode = Ranging::Long;
      distanceSensor->set_ranging_mode(ranging_mode);
      ScanGroup group{grid, mode, {}, {}};
      std::vector<float> cv_trials;
      for (int trial = 1; trial <= max_trials; ++trial) {
        if (scan_cancel_requested) {
          publish_scan_record("scan_cancel");
          scan_running = false;
          return;
        }
        std::vector<int> mcps(grid * grid, 0);
        std::vector<int> distance(grid * grid, 0);
        std::vector<float> snr(grid * grid, 0.0f);

        ROI roi{4, 4, 0};
        int spacing = 16 / grid;
        for (int y = 0; y < grid; ++y) {
          for (int x = 0; x < grid; ++x) {
            uint8_t cx = static_cast<uint8_t>(x * spacing + spacing / 2);
            uint8_t cy = static_cast<uint8_t>(y * spacing + spacing / 2);
            roi.center = static_cast<uint8_t>((cy * 16) + cx + 1);

            VL53L1_Error status;
            auto dist = distanceSensor->read_distance(&roi, status);
            if (dist.has_value()) {
              auto rate = distanceSensor->read_signal_rate(status);
              if (rate.has_value())
                mcps[y * grid + x] = rate.value();
              auto sn = distanceSensor->read_snr(status);
              if (sn.has_value())
                snr[y * grid + x] = sn.value();
              distance[y * grid + x] = dist.value();
            }
          }
        }

        float mean_mcps = 0.0f;
        for (int v : mcps)
          mean_mcps += v;
        mean_mcps /= mcps.size();
        float var_mcps = 0.0f;
        for (int v : mcps) {
          float diff = v - mean_mcps;
          var_mcps += diff * diff;
        }
        var_mcps /= mcps.size();
        float cv_mask = mean_mcps != 0.0f ? (sqrtf(var_mcps) / mean_mcps) * 100.0f : 0.0f;
        if (variance_cv_mask_sensor != nullptr)
          variance_cv_mask_sensor->publish_state(cv_mask);

        float mean_dist = 0.0f;
        for (int v : distance)
          mean_dist += v;
        mean_dist /= distance.size();
        float var_dist = 0.0f;
        for (int v : distance) {
          float diff = v - mean_dist;
          var_dist += diff * diff;
        }
        var_dist /= distance.size();
        float cv_trial = mean_dist != 0.0f ? (sqrtf(var_dist) / mean_dist) * 100.0f : 0.0f;
        if (trial_bump_cv_sensor != nullptr)
          trial_bump_cv_sensor->publish_state(cv_trial);
        cv_trials.push_back(cv_trial);

        std::stringstream mcps_ss;
        mcps_ss << '[';
        for (size_t i = 0; i < mcps.size(); ++i) {
          if (i)
            mcps_ss << ',';
          mcps_ss << mcps[i];
        }
        mcps_ss << ']';
        std::stringstream dist_ss;
        dist_ss << '[';
        for (size_t i = 0; i < distance.size(); ++i) {
          if (i)
            dist_ss << ',';
          dist_ss << distance[i];
        }
        dist_ss << ']';
        group.mcps_trials.push_back(mcps);
        group.distance_trials.push_back(distance);
        std::stringstream snr_ss;
        snr_ss << '[';
        for (size_t i = 0; i < snr.size(); ++i) {
          if (i)
            snr_ss << ',';
          snr_ss << snr[i];
        }
        snr_ss << ']';
        char ts[25];
        time_t now = time(nullptr);
        struct tm tm_time;
        localtime_r(&now, &tm_time);
        strftime(ts, sizeof(ts), "%Y-%m-%dT%H:%M:%S", &tm_time);
        std::stringstream json;
        json << "{\"type\":\"passive_scan\",\"grid\":\"" << grid << "\"";
        json << ",\"trial\":" << trial;
        json << ",\"ranging\":\"" << mode << "\"";
        json << ",\"timestamp\":\"" << ts << "\"";
        json << ",\"data\":{\"mcps\":" << mcps_ss.str() << ",\"distance\":" << dist_ss.str()
             << ",\"snr\":" << snr_ss.str() << "}}";
        publish_scan_record(json.str());

        scan_step_++;
        scan_progress_ = scan_total_steps_ > 0 ? (float) scan_step_ / scan_total_steps_ : 0.0f;

        if (trial >= base_trials) {
          float avg_cv = 0.0f;
          for (float cv : cv_trials)
            avg_cv += cv;
          avg_cv /= cv_trials.size();
          if (avg_cv <= trial_bump_cv_ || trial == max_trials)
            break;
        }
      }
      groups.push_back(std::move(group));
    }
  }
  // Analyze collected data to determine recommended settings
  bool rec_valid = false;
  const ScanGroup *best_group = nullptr;
  std::vector<float> best_mean;
  std::vector<float> best_var;
  std::vector<bool> best_mask;
  int best_grid = 0;
  std::string best_mode;
  float best_var_score = std::numeric_limits<float>::infinity();
  uint16_t best_budget = std::numeric_limits<uint16_t>::max();

  for (const auto &g : groups) {
    size_t trials = g.mcps_trials.size();
    if (trials == 0)
      continue;
    int cells = g.grid * g.grid;
    std::vector<float> mean(cells, 0.0f);
    std::vector<float> var(cells, 0.0f);
    for (const auto &t : g.mcps_trials) {
      for (int i = 0; i < cells; ++i)
        mean[i] += t[i];
    }
    for (int i = 0; i < cells; ++i)
      mean[i] /= static_cast<float>(trials);
    for (const auto &t : g.mcps_trials) {
      for (int i = 0; i < cells; ++i) {
        float diff = t[i] - mean[i];
        var[i] += diff * diff;
      }
    }
    for (int i = 0; i < cells; ++i)
      var[i] /= static_cast<float>(trials);
    std::vector<float> mean_copy = mean;
    std::nth_element(mean_copy.begin(), mean_copy.begin() + mean_copy.size() / 2, mean_copy.end());
    float med = mean_copy[mean_copy.size() / 2];
    std::vector<float> dev(cells);
    for (int i = 0; i < cells; ++i)
      dev[i] = fabsf(mean[i] - med);
    std::nth_element(dev.begin(), dev.begin() + dev.size() / 2, dev.end());
    float mad = dev[dev.size() / 2];
    std::vector<bool> mask(cells, false);
    float var_sum = 0.0f;
    int mask_count = 0;
    for (int i = 0; i < cells; ++i) {
      bool bright = mean[i] >= med + 0.5f * mad;
      float cv = mean[i] > 0.0f ? sqrtf(var[i]) / mean[i] : std::numeric_limits<float>::infinity();
      if (bright && cv <= 0.25f) {
        mask[i] = true;
        var_sum += var[i];
        mask_count++;
      }
    }
    if (mask_count == 0)
      continue;
    float var_score = var_sum / static_cast<float>(mask_count);
    uint16_t budget = 15;
    if (g.mode == "medium")
      budget = 20;
    else if (g.mode == "long")
      budget = 33;
    if (var_score < best_var_score || (fabsf(var_score - best_var_score) < 1e-3f && budget < best_budget)) {
      best_var_score = var_score;
      best_budget = budget;
      best_group = &g;
      best_mean = mean;
      best_var = var;
      best_mask = mask;
      best_grid = g.grid;
      best_mode = g.mode;
    }
  }

  if (best_group != nullptr) {
    float scale = 16.0f / static_cast<float>(best_grid);
    int cells = best_grid * best_grid;
    int min_x = best_grid, max_x = -1, min_y = best_grid, max_y = -1;
    std::vector<std::pair<float, int>> weighted;
    for (int y = 0; y < best_grid; ++y) {
      for (int x = 0; x < best_grid; ++x) {
        int idx = y * best_grid + x;
        if (best_mask[idx]) {
          if (x < min_x)
            min_x = x;
          if (x > max_x)
            max_x = x;
          if (y < min_y)
            min_y = y;
          if (y > max_y)
            max_y = y;
          weighted.emplace_back(best_mean[idx], idx);
        }
      }
    }
    if (max_x < min_x) {
      min_x = 0;
      max_x = best_grid - 1;
      min_y = 0;
      max_y = best_grid - 1;
    }
    int roi_w = std::max(4, std::min(16, static_cast<int>(roundf((max_x - min_x + 1) * scale))));
    int roi_h = std::max(4, std::min(16, static_cast<int>(roundf((max_y - min_y + 1) * scale))));

    float cx = (best_grid - 1) / 2.0f;
    float cy = (best_grid - 1) / 2.0f;
    size_t N = weighted.size();
    if (N > 0) {
      std::sort(weighted.begin(), weighted.end(), [](const auto &a, const auto &b) { return a.first < b.first; });
      float k = std::max(0.15f, std::min(0.30f, (64.0f / static_cast<float>(N)) * 0.05f));
      int top_n = std::max(1, static_cast<int>(floorf(N * k)));
      float sumw = 0.0f, sumx = 0.0f, sumy = 0.0f;
      for (size_t i = N - top_n; i < N; ++i) {
        int idx = weighted[i].second;
        float w = weighted[i].first;
        int x = idx % best_grid;
        int y = idx / best_grid;
        sumw += w;
        sumx += w * (x + 0.5f);
        sumy += w * (y + 0.5f);
      }
      if (sumw > 0.0f) {
        cx = sumx / sumw;
        cy = sumy / sumw;
      }
    }
    float phys_x = cx * scale;
    float phys_y = cy * scale;

    struct Pt {
      float x;
      float y;
    };
    std::vector<Pt> pts;
    std::vector<float> weights;
    for (int y = 0; y < best_grid; ++y) {
      for (int x = 0; x < best_grid; ++x) {
        int idx = y * best_grid + x;
        if (best_mask[idx]) {
          pts.push_back(Pt{(x + 0.5f) * scale, (y + 0.5f) * scale});
          weights.push_back(best_mean[idx]);
        }
      }
    }
    Pt centers[2];
    if (pts.size() < 2) {
      centers[0] = Pt{phys_x, phys_y};
      centers[1] = Pt{phys_x, phys_y};
    } else {
      centers[0] = pts[0];
      centers[1] = pts[1];
      for (int iter = 0; iter < 100; ++iter) {
        float sx[2] = {0.0f, 0.0f};
        float sy[2] = {0.0f, 0.0f};
        float sw[2] = {0.0f, 0.0f};
        for (size_t i = 0; i < pts.size(); ++i) {
          float d0 = (pts[i].x - centers[0].x) * (pts[i].x - centers[0].x) +
                     (pts[i].y - centers[0].y) * (pts[i].y - centers[0].y);
          float d1 = (pts[i].x - centers[1].x) * (pts[i].x - centers[1].x) +
                     (pts[i].y - centers[1].y) * (pts[i].y - centers[1].y);
          int lbl = d0 <= d1 ? 0 : 1;
          sx[lbl] += pts[i].x * weights[i];
          sy[lbl] += pts[i].y * weights[i];
          sw[lbl] += weights[i];
        }
        Pt newc[2] = {centers[0], centers[1]};
        for (int i = 0; i < 2; ++i) {
          if (sw[i] > 0.0f) {
            newc[i].x = sx[i] / sw[i];
            newc[i].y = sy[i] / sw[i];
          }
        }
        if (fabsf(newc[0].x - centers[0].x) < 0.01f && fabsf(newc[0].y - centers[0].y) < 0.01f &&
            fabsf(newc[1].x - centers[1].x) < 0.01f && fabsf(newc[1].y - centers[1].y) < 0.01f) {
          centers[0] = newc[0];
          centers[1] = newc[1];
          break;
        }
        centers[0] = newc[0];
        centers[1] = newc[1];
      }
    }
    Pt entry_c = centers[0];
    Pt exit_c = centers[1];
    if (centers[0].y > centers[1].y) {
      entry_c = centers[1];
      exit_c = centers[0];
    }
    uint8_t entry_center = static_cast<uint8_t>(roundf(entry_c.y) * 16 + roundf(entry_c.x) + 1);
    uint8_t exit_center = static_cast<uint8_t>(roundf(exit_c.y) * 16 + roundf(exit_c.x) + 1);

    std::vector<uint16_t> all_dist;
    for (const auto &d : best_group->distance_trials)
      all_dist.insert(all_dist.end(), d.begin(), d.end());
    uint16_t thr_min = 0;
    uint16_t thr_max = 0;
    if (!all_dist.empty()) {
      auto dist_copy = all_dist;
      auto mid = dist_copy.begin() + dist_copy.size() / 2;
      std::nth_element(dist_copy.begin(), mid, dist_copy.end());
      float median = *mid;
      size_t idx95 = static_cast<size_t>(roundf(0.95f * (dist_copy.size() - 1)));
      std::nth_element(dist_copy.begin(), dist_copy.begin() + idx95, dist_copy.end());
      float p95 = dist_copy[idx95];
      float min_per = p95 / median;
      if (min_per < 0.92f)
        min_per = 0.92f;
      if (min_per > 0.96f)
        min_per = 0.96f;
      float max_per = std::max(min_per + 0.10f, 0.80f);
      thr_min = static_cast<uint16_t>(median * min_per);
      thr_max = static_cast<uint16_t>(median * max_per);
    }

    float avg_cv = 0.0f;
    int cv_count = 0;
    for (int i = 0; i < cells; ++i) {
      if (best_mean[i] > 0.0f) {
        float cv = sqrtf(best_var[i]) / best_mean[i];
        if (!std::isnan(cv)) {
          avg_cv += cv;
          cv_count++;
        }
      }
    }
    uint8_t rec_samples = 3;
    if (cv_count > 0) {
      avg_cv /= cv_count;
      rec_samples = avg_cv > 0.20f ? 5 : 3;
    }

    ROI entry_roi{static_cast<uint8_t>(roi_w), static_cast<uint8_t>(roi_h), entry_center};
    ROI exit_roi{static_cast<uint8_t>(roi_w), static_cast<uint8_t>(roi_h), exit_center};
    RecommendedSettings rs{entry_roi, exit_roi, thr_min, thr_max, thr_min, thr_max, rec_samples, VERSION, best_mode};
    recommended_settings_ = rs;
    rec_valid = true;
  }

  if (!rec_valid) {
    recommended_settings_ = RecommendedSettings{*entry->roi,
                                                *exit->roi,
                                                entry->threshold->min,
                                                entry->threshold->max,
                                                exit->threshold->min,
                                                exit->threshold->max,
                                                samples,
                                                VERSION,
                                                ""};
  }

  publish_scan_record("scan_complete");
  float elapsed = (millis() - scan_start_ts_) / 1000.0f;
  if (scan_time_cap_seconds_sensor != nullptr)
    scan_time_cap_seconds_sensor->publish_state(elapsed);
  scan_progress_ = 1.0f;
  save_current_scan_session();
  scan_running = false;
}

void Roode::complete_calibration() {
  ESP_LOGI(TAG, "Starting passive calibration scan");
  start_passive_scan();
  ESP_LOGI(TAG, "Passive calibration scan complete");
  if (!recommended_settings_.has_value()) {
    ESP_LOGW(TAG, "No recommended settings generated during scan");
    return;
  }
  apply_recommended_settings();
}

void Roode::save_current_scan_session() {
  ScanSession sess{};
  sess.id = scan_start_ts_;
  sess.timestamp = time(nullptr);
  sess.trials = scan_record_count_;
  sess.duration = (millis() - scan_start_ts_) / 1000.0f;
  size_t len = std::min(current_session_blob_.size(), (size_t) MAX_SESSION_DATA - 1);
  memcpy(sess.data, current_session_blob_.c_str(), len);
  sess.data[len] = '\0';
  sess.size = len;
  if (sessions_.size() < MAX_SCAN_SESSIONS) {
    if (session_next_ >= sessions_.size())
      sessions_.push_back(sess);
    else
      sessions_[session_next_] = sess;
  } else {
    sessions_[session_next_] = sess;
  }
  session_prefs_[session_next_].save(&sess);
  session_next_ = (session_next_ + 1) % MAX_SCAN_SESSIONS;
  session_index_pref_.save(&session_next_);
}

void Roode::restart_sensor() {
  uint32_t now = millis();
  if (now - last_sensor_restart_ts_ > restart_timeout_ms_)
    restart_attempt_count_ = 0;
  restart_attempt_count_++;
  ESP_LOGW(TAG, "sensor_restart_attempt_%u", restart_attempt_count_);
  log_event(std::string("sensor_restart_attempt_") + std::to_string(restart_attempt_count_));
  distanceSensor->restart();
  last_sensor_restart_ts_ = now;
  invalid_read_count_ = 0;
  if (restart_attempt_count_ >= max_restart_attempts_) {
    ESP_LOGE(TAG, "sensor_restart_escalating_reset");
    log_event("sensor_restart_escalating_reset");
    ESP.restart();
  }
}

void Roode::sensor_task(void *param) {
  auto *self = static_cast<Roode *>(param);
  for (;;) {
    self->use_sensor_task_ = true;
    uint32_t now = millis();
    if (self->last_loop_update_ts_ != 0 && (now - self->last_loop_update_ts_ > self->restart_timeout_ms_) &&
        (now - self->last_sensor_restart_ts_ > self->restart_timeout_ms_)) {
      ESP_LOGW(TAG, "Sensor unresponsive >%ds, restarting...", self->restart_timeout_ms_ / 1000);
      self->restart_sensor();
    }
    unsigned long start = micros();
    VL53L1_Error status = self->current_zone->readDistance(self->distanceSensor);
    if (status == VL53L1_ERROR_NONE)
      self->last_loop_update_ts_ = millis();
    uint16_t dist = self->current_zone->getDistance();
    if (status == VL53L1_ERROR_NONE && (dist == 0 || dist > 4000)) {
      self->invalid_read_count_++;
    } else {
      self->invalid_read_count_ = 0;
    }
    if (self->invalid_read_count_ > self->invalid_distance_limit_ &&
        (now - self->last_sensor_restart_ts_ > self->restart_timeout_ms_)) {
      ESP_LOGW(TAG, "Consecutive invalid distances, restarting...");
      self->restart_sensor();
    }
    bool zone_trig = self->current_zone->getMinDistance() < self->current_zone->threshold->max &&
                     self->current_zone->getMinDistance() > self->current_zone->threshold->min;
    if (!self->cpu_optimizations_active_ || zone_trig)
      self->path_tracking(self->current_zone);
    self->handle_sensor_status();
    self->current_zone = self->current_zone == self->entry ? self->exit : self->entry;
    unsigned long end = micros();
    unsigned long delta = end - start;
    self->loop_time_sum_ += delta;
    self->loop_count_++;
    self->update_metrics();
    vTaskDelay(pdMS_TO_TICKS(self->polling_interval_ms_));
  }
}
}  // namespace roode
}  // namespace esphome
