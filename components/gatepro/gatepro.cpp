#include "esphome/core/log.h"
#include "gatepro.h"

namespace esphome {
namespace gatepro {

static const char* const TAG = "gatepro";

// Helper / misc functions
void GatePro::queue_gatepro_cmd(GateProCmd cmd) {
  this->tx_queue.push(this->get_command_string(cmd));
}

void GatePro::publish() {
  // Update the internal position tracking
  this->position_ = this->position;
  this->publish_state();
}

// GatePro logic functions
void GatePro::process() {
  if (!this->rx_queue.size()) {
    return;
  }
  std::string msg = this->rx_queue.front();
  this->rx_queue.pop();

  ESP_LOGD(TAG, "UART RX: %s", msg.c_str());

  // Process ACK RS status message (position info)
  // example: ACK RS:00,80,C4,C6,3E,16,FF,FF,FF\r\n
  if (msg.substr(0, 6) == "ACK RS") {
    if (msg.length() < 18) {
      ESP_LOGE(TAG, "ACK RS message too short: %s", msg.c_str());
      return;
    }
    
    // Check if we're in a special state that should override position reporting
    if (this->gate_closed) {
      // If the gate is closed, we should maintain the closed position (1.0)
      // regardless of what the status message says
      if (msg.find("A2,00,40,00") != std::string::npos) {
        ESP_LOGD(TAG, "Gate is closed, ignoring position update from: %s", msg.c_str());
        return;
      } else {
        // If we get a different status message, the gate might be moving
        ESP_LOGI(TAG, "Gate was closed but received different status, clearing closed state");
        this->gate_closed = false;
      }
    }
    
    // Extract the position value (hex)
    std::string position_hex = msg.substr(16, 2);
    
    // Convert hex to integer safely
    char* end;
    int percentage = strtol(position_hex.c_str(), &end, 16);
    
    // Check if conversion was successful
    if (*end != '\0') {
      ESP_LOGE(TAG, "Failed to parse position from ACK RS message: %s", msg.c_str());
      return;
    }
    
    // percentage correction with known offset, if necessary
    if (percentage > 100) {
      percentage -= this->known_percentage_offset;
    }
    
    float new_position = (float)percentage / 100;
    
    // Define a small tolerance to avoid state jumping due to minor fluctuations
    const float position_tolerance = 0.02f;
    
    // Only update and publish if position has changed significantly
    if (fabs(this->position - new_position) > position_tolerance) {
      ESP_LOGD(TAG, "Position changed from %.2f to %.2f", this->position, new_position);
      this->position = new_position;
      this->position_ = new_position; // Update internal position tracking as well
      this->publish_state();
    } else {
      ESP_LOGV(TAG, "Position change too small (%.2f to %.2f), ignoring", this->position, new_position);
    }
    return;
  }

  // Event message from the motor
  // example: $V1PKF0,17,Closed;src=0001\r\n
  if (msg.substr(0, 7) == "$V1PKF0") {
    ESP_LOGI(TAG, "Received motor event: %s", msg.c_str());
    bool state_changed = false;
    
    // Check if this was triggered by remote (src=0001 or similar)
    if (msg.find("src=") != std::string::npos && msg.find("src=P") == std::string::npos) {
      this->remote_triggered_ = true;
      ESP_LOGI(TAG, "Operation triggered by remote control");
    }
    
    if (msg.substr(11, 7) == "Opening") {
      ESP_LOGI(TAG, "Gate is opening (remote control or other trigger)");
      this->operation_finished = false;
      this->current_operation = cover::COVER_OPERATION_OPENING;
      this->last_operation_ = cover::COVER_OPERATION_OPENING;
      // Reset the gate_closed flag when opening starts
      this->gate_closed = false;
      
      // Record operation start time
      this->last_operation_time_ = millis();
      state_changed = true;
    }
    else if (msg.substr(11, 6) == "Opened") {
      ESP_LOGI(TAG, "Gate is fully open");
      this->operation_finished = true;
      this->position = cover::COVER_OPEN; // 0.0f
      this->current_operation = cover::COVER_OPERATION_IDLE;
      // Set a flag to indicate the gate is in open state
      this->gate_open = true;
      this->gate_closed = false; // Ensure closed flag is reset
      
      // Calculate operation duration
      this->operation_duration_ = millis() - this->last_operation_time_;
      ESP_LOGI(TAG, "Gate opening operation took %u ms", this->operation_duration_);
      
      // Record time when gate was opened (for open duration tracking)
      this->open_time_ = millis();
      state_changed = true;
    }
    else if (msg.substr(11, 7) == "Closing" || msg.substr(11, 11) == "AutoClosing") {
      ESP_LOGI(TAG, "Gate is closing (remote control or auto-close)");
      this->operation_finished = false;
      this->current_operation = cover::COVER_OPERATION_CLOSING;
      this->last_operation_ = cover::COVER_OPERATION_CLOSING;
      // Reset the gate_open flag when closing starts
      this->gate_open = false;
      
      // Record operation start time
      this->last_operation_time_ = millis();
      state_changed = true;
    }
    else if (msg.substr(11, 6) == "Closed") {
      ESP_LOGI(TAG, "Gate is fully closed");
      this->operation_finished = true;
      this->position = cover::COVER_CLOSED; // 1.0f
      this->current_operation = cover::COVER_OPERATION_IDLE;
      // Set a flag to indicate the gate is in closed state
      this->gate_closed = true;
      this->gate_open = false; // Ensure open flag is reset
      
      // Calculate operation duration
      this->operation_duration_ = millis() - this->last_operation_time_;
      ESP_LOGI(TAG, "Gate closing operation took %u ms", this->operation_duration_);
      
      // Reset open time
      this->open_time_ = 0;
      state_changed = true;
    }
    else if (msg.substr(11, 7) == "Stopped") {
      ESP_LOGI(TAG, "Gate has stopped");
      this->operation_finished = true;
      this->current_operation = cover::COVER_OPERATION_IDLE;
      // Request status to get current position
      this->queue_gatepro_cmd(GATEPRO_CMD_READ_STATUS);
      state_changed = true;
    }
    
    // Publish state if it changed
    if (state_changed) {
      this->publish_state();
    }
    
    return;
  }
}

// Cover component logic functions
void GatePro::control(const cover::CoverCall &call) {
  if (call.get_stop()) {
    this->start_direction_(cover::COVER_OPERATION_IDLE);
    return;
  }

  if (call.get_position().has_value()) {
    auto pos = *call.get_position();
    if (pos == this->position) {
      return;
    }
    auto op = pos < this->position ? cover::COVER_OPERATION_CLOSING : cover::COVER_OPERATION_OPENING;
    this->target_position_ = pos;
    this->start_direction_(op);
  }
}

void GatePro::start_direction_(cover::CoverOperation dir) {
  if (this->current_operation == dir) {
    return;
  }

  switch (dir) {
    case cover::COVER_OPERATION_IDLE:
      if (this->operation_finished) {
        break;
      }
      this->queue_gatepro_cmd(GATEPRO_CMD_STOP);
      break;
    case cover::COVER_OPERATION_OPENING:
      this->queue_gatepro_cmd(GATEPRO_CMD_OPEN);
      break;
    case cover::COVER_OPERATION_CLOSING:
      this->queue_gatepro_cmd(GATEPRO_CMD_CLOSE);
      break;
    default:
      return;
  }
}

void GatePro::correction_after_operation() {
  if (this->operation_finished) {
    if (this->current_operation == cover::COVER_OPERATION_IDLE &&
        this->last_operation_ == cover::COVER_OPERATION_CLOSING &&
        this->position != cover::COVER_CLOSED) {
      this->position = cover::COVER_CLOSED;
      return;
    }

    if (this->current_operation == cover::COVER_OPERATION_IDLE &&
        this->last_operation_ == cover::COVER_OPERATION_OPENING &&
        this->position != cover::COVER_OPEN) {
      this->position = cover::COVER_OPEN;
    }
  }
}

void GatePro::stop_at_target_position() {
  if (this->target_position_ &&
      this->target_position_ != cover::COVER_OPEN &&
      this->target_position_ != cover::COVER_CLOSED) {
    const float diff = abs(this->position - this->target_position_);
    if (diff < this->acceptable_diff) {
      this->make_call().set_command_stop().perform();
    }
  }
}

// UART operations
void GatePro::read_uart() {
  int available = this->available();
  if (!available) {
    return;
  }
  
  uint8_t* bytes = new uint8_t[available];
  this->read_array(bytes, available);
  this->preprocess(this->convert(bytes, available));
  delete[] bytes;
}

void GatePro::write_uart() {
  if (this->tx_queue.size()) {
    const char* msg = this->tx_queue.front();
    this->write_str(msg);
    ESP_LOGD(TAG, "UART TX: %s", msg);
    this->tx_queue.pop();
  }
}

std::string GatePro::convert(uint8_t* bytes, size_t len) {
  std::string res;
  char buf[5];
  for (size_t i = 0; i < len; i++) {
    if (bytes[i] == 7) {
      res += "\\a";
    } else if (bytes[i] == 8) {
      res += "\\b";
    } else if (bytes[i] == 9) {
      res += "\\t";
    } else if (bytes[i] == 10) {
      res += "\\n";
    } else if (bytes[i] == 11) {
      res += "\\v";
    } else if (bytes[i] == 12) {
      res += "\\f";
    } else if (bytes[i] == 13) {
      res += "\\r";
    } else if (bytes[i] == 27) {
      res += "\\e";
    } else if (bytes[i] == 34) {
      res += "\\\"";
    } else if (bytes[i] == 39) {
      res += "\\'";
    } else if (bytes[i] == 92) {
      res += "\\\\";
    } else if (bytes[i] < 32 || bytes[i] > 127) {
      sprintf(buf, "\\x%02X", bytes[i]);
      res += buf;
    } else {
      res += bytes[i];
    }
  }
  return res;
}

void GatePro::preprocess(std::string msg) {
  uint8_t delimiter_location = msg.find(this->delimiter);
  uint8_t msg_length = msg.length();
  if (msg_length > delimiter_location + this->delimiter_length) {
    std::string msg1 = msg.substr(0, delimiter_location + this->delimiter_length);
    this->rx_queue.push(msg1);
    std::string msg2 = msg.substr(delimiter_location + this->delimiter_length);
    this->preprocess(msg2);
    return;
  }
  this->rx_queue.push(msg);
}

// Component functions
cover::CoverTraits GatePro::get_traits() {
  auto traits = cover::CoverTraits();
  traits.set_is_assumed_state(false);
  traits.set_supports_position(true);
  traits.set_supports_tilt(false);
  traits.set_supports_toggle(true);
  traits.set_supports_stop(true);
  return traits;
}

void GatePro::setup() {
  ESP_LOGD(TAG, "Setting up GatePro component..");
  this->last_operation_ = cover::COVER_OPERATION_CLOSING;
  this->current_operation = cover::COVER_OPERATION_IDLE;
  this->operation_finished = true;
  this->queue_gatepro_cmd(GATEPRO_CMD_READ_STATUS);
  this->blocker = false;
  this->target_position_ = 0.0f;
  this->last_operation_time_ = millis();
  this->operation_duration_ = 0;
  this->remote_triggered_ = false;
  this->open_time_ = 0;
}

// Publish additional attributes to Home Assistant
void GatePro::publish_attributes_() {
  // Simply update the state with current position
  this->publish_state();
  
  // Log additional information that would normally be attributes
  // Publish trigger source
  if (this->remote_triggered_) {
    ESP_LOGD(TAG, "Last trigger: remote");
  } else {
    ESP_LOGD(TAG, "Last trigger: manual");
  }
  
  // Publish operation duration in seconds
  float duration_sec = this->operation_duration_ / 1000.0f;
  ESP_LOGD(TAG, "Last operation duration: %.2f seconds", duration_sec);
  
  // Publish current state as text
  if (this->gate_closed) {
    ESP_LOGD(TAG, "Gate state: closed");
  } else if (this->gate_open) {
    ESP_LOGD(TAG, "Gate state: open");
  } else if (this->current_operation == cover::COVER_OPERATION_OPENING) {
    ESP_LOGD(TAG, "Gate state: opening");
  } else if (this->current_operation == cover::COVER_OPERATION_CLOSING) {
    ESP_LOGD(TAG, "Gate state: closing");
  } else {
    ESP_LOGD(TAG, "Gate state: stopped");
  }
}

// Check if gate has been open for too long
void GatePro::check_open_duration_() {
  if (this->gate_open && this->open_time_ > 0) {
    uint32_t now = millis();
    uint32_t open_duration = now - this->open_time_;
    
    // If gate has been open for longer than the threshold, log a warning
    if (open_duration > this->open_duration_warning_threshold_) {
      static uint32_t last_warning = 0;
      
      // Only warn once every minute to avoid log spam
      if (now - last_warning > 60000) {
        ESP_LOGW(TAG, "Gate has been open for %u seconds (threshold: %u seconds)", 
                 open_duration / 1000, this->open_duration_warning_threshold_ / 1000);
        ESP_LOGW(TAG, "WARNING: Gate open for extended period");
        ESP_LOGW(TAG, "Open duration: %u seconds", open_duration / 1000);
        last_warning = now;
      }
    }
  }
}

void GatePro::update() {
  this->publish();
  this->stop_at_target_position();
  
  this->write_uart();

  // Periodically request status to detect external changes (remote control operations)
  static uint32_t last_status_request = 0;
  uint32_t now = millis();
  if (now - last_status_request > 2000) { // Every 2 seconds
    this->queue_gatepro_cmd(GATEPRO_CMD_READ_STATUS);
    last_status_request = now;
  }

  this->correction_after_operation();
  
  // Publish additional attributes and check open duration
  this->publish_attributes_();
  this->check_open_duration_();
  
  // Reset remote_triggered flag after update
  this->remote_triggered_ = false;
}

void GatePro::loop() {
  // keep reading uart for changes
  this->read_uart();
  this->process();
}

void GatePro::dump_config() {
  ESP_LOGCONFIG(TAG, "GatePro sensor dump config");
}

}  // namespace gatepro
}  // namespace esphome
