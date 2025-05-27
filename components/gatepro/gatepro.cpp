#include "esphome/core/log.h"
#include "gatepro.h"

namespace esphome {
namespace gatepro {

////////////////////////////////////
static const char* TAG = "gatepro";


////////////////////////////////////////////
// Helper / misc functions
////////////////////////////////////////////
void GatePro::queue_gatepro_cmd(GateProCmd cmd) {
  this->tx_queue.push(this->get_command_string(cmd));
}

void GatePro::publish() {
    // publish on each tick
    /*if (this->position_ == this->position) {
      return;
    }*/

    this->position_ = this->position;
    this->publish_state();
}

////////////////////////////////////////////
// GatePro logic functions
////////////////////////////////////////////
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
    
    // Extract the pattern from the message
    std::string current_pattern = "";
    if (msg.length() >= 21) {
      current_pattern = msg.substr(10, 11);
    }
    
    // Main logic: Only update states when the gate is in motion or when the state is unknown
    // This prevents state jumping when the gate is stationary
    bool should_update_state = !this->operation_finished || this->gate_state_ == STATE_UNKNOWN;
    
    // Check for the specific pattern that indicates a closed gate
    // A2,00,40,00 is the pattern seen in logs when gate is closed
    if (current_pattern == "A2,00,40,00") {
      // Track consecutive pattern readings for stability
      if (this->last_pattern_seen_ == current_pattern) {
        this->consecutive_pattern_readings_++;
        ESP_LOGD(TAG, "Consecutive closed pattern readings: %d", this->consecutive_pattern_readings_);
      } else {
        // Reset counter if pattern changed
        this->last_pattern_seen_ = current_pattern;
        this->consecutive_pattern_readings_ = 1;
        ESP_LOGD(TAG, "New pattern detected (closed): %s", current_pattern.c_str());
      }
      
      // Only update state if the gate is in motion or the state is unknown
      // AND we've seen the pattern consistently
      if (should_update_state && this->consecutive_pattern_readings_ >= 3) {
        uint32_t now = millis();
        
        if (this->gate_state_ != STATE_CLOSED) {
          ESP_LOGI(TAG, "Detected closed gate pattern (%d readings), updating state to closed", 
                   this->consecutive_pattern_readings_);
          GateProState old_state = this->gate_state_;
          this->gate_state_ = STATE_CLOSED;
          this->position = cover::COVER_CLOSED; // 1.0f
          this->position_ = cover::COVER_CLOSED;
          this->current_operation = cover::COVER_OPERATION_IDLE;
          this->operation_finished = true; // Mark operation as finished if we detect a stable state
          this->last_state_change_ = now;
          this->log_state_change(old_state, this->gate_state_);
          this->publish_state();
        }
      }
      return;
    }
    
    // Check for the specific pattern that indicates an open gate
    // A2,E3,40,00 is the pattern seen in logs when gate is open
    if (current_pattern == "A2,E3,40,00") {
      // Track consecutive pattern readings for stability
      if (this->last_pattern_seen_ == current_pattern) {
        this->consecutive_pattern_readings_++;
        ESP_LOGD(TAG, "Consecutive open pattern readings: %d", this->consecutive_pattern_readings_);
      } else {
        // Reset counter if pattern changed
        this->last_pattern_seen_ = current_pattern;
        this->consecutive_pattern_readings_ = 1;
        ESP_LOGD(TAG, "New pattern detected (open): %s", current_pattern.c_str());
      }
      
      // Only update state if the gate is in motion or the state is unknown
      // AND we've seen the pattern consistently
      if (should_update_state && this->consecutive_pattern_readings_ >= 3) {
        uint32_t now = millis();
        
        if (this->gate_state_ != STATE_OPEN) {
          ESP_LOGI(TAG, "Detected open gate pattern (%d readings), updating state to open", 
                   this->consecutive_pattern_readings_);
          GateProState old_state = this->gate_state_;
          this->gate_state_ = STATE_OPEN;
          this->position = cover::COVER_OPEN; // 0.0f
          this->position_ = cover::COVER_OPEN;
          this->current_operation = cover::COVER_OPERATION_IDLE;
          this->operation_finished = true; // Mark operation as finished if we detect a stable state
          this->last_state_change_ = now;
          this->log_state_change(old_state, this->gate_state_);
          this->publish_state();
        }
      }
      return;
    }
    
    // If we get here, we've seen a different pattern
    if (!current_pattern.empty() && this->last_pattern_seen_ != current_pattern) {
      this->last_pattern_seen_ = current_pattern;
      this->consecutive_pattern_readings_ = 1;
      ESP_LOGD(TAG, "New pattern detected (other): %s", current_pattern.c_str());
    } else if (!current_pattern.empty()) {
      this->consecutive_pattern_readings_++;
      ESP_LOGD(TAG, "Consecutive other pattern readings: %d for %s", 
               this->consecutive_pattern_readings_, current_pattern.c_str());
    }
    
    // For position updates, only process them if the gate is in motion
    // This prevents position updates when the gate is stationary
    if (!this->operation_finished) {
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
      
      // Snap to extremes for stability
      if (new_position <= 0.05f) {
        new_position = 0.0f; // Fully open
      } else if (new_position >= 0.95f) {
        new_position = 1.0f; // Fully closed
      }
      
      // Update position only while in motion
      this->position = new_position;
      this->position_ = new_position;
      this->publish_state();
      
      ESP_LOGD(TAG, "Updated position during motion: %.2f", new_position);
    }
    return;
  }

  // Event message from the motor
  // example: $V1PKF0,17,Closed;src=0001\r\n
  if (msg.substr(0, 7) == "$V1PKF0") {
    ESP_LOGI(TAG, "Received motor event: %s", msg.c_str());
    GateProState old_state = this->gate_state_;
    uint32_t now = millis();
    
    // Reset pattern detection when we receive direct motor events
    this->last_pattern_seen_ = "";
    this->consecutive_pattern_readings_ = 0;
    
    if (msg.substr(11, 7) == "Opening") {
      ESP_LOGI(TAG, "Gate is opening");
      this->operation_finished = false;
      this->current_operation = cover::COVER_OPERATION_OPENING;
      this->last_operation_ = cover::COVER_OPERATION_OPENING;
      this->gate_state_ = STATE_OPENING;
      this->last_state_change_ = now;
      this->log_state_change(old_state, this->gate_state_);
      this->publish_state();
      return;
    }
    else if (msg.substr(11, 6) == "Opened") {
      ESP_LOGI(TAG, "Gate is fully open");
      this->operation_finished = true;
      this->position = cover::COVER_OPEN; // 0.0f
      this->position_ = cover::COVER_OPEN;
      this->current_operation = cover::COVER_OPERATION_IDLE;
      this->gate_state_ = STATE_OPEN;
      this->last_state_change_ = now;
      this->log_state_change(old_state, this->gate_state_);
      this->publish_state();
      return;
    }
    else if (msg.substr(11, 7) == "Closing" || msg.substr(11, 11) == "AutoClosing") {
      ESP_LOGI(TAG, "Gate is closing");
      this->operation_finished = false;
      this->current_operation = cover::COVER_OPERATION_CLOSING;
      this->last_operation_ = cover::COVER_OPERATION_CLOSING;
      this->gate_state_ = STATE_CLOSING;
      this->last_state_change_ = now;
      this->log_state_change(old_state, this->gate_state_);
      this->publish_state();
      return;
    }
    else if (msg.substr(11, 6) == "Closed") {
      ESP_LOGI(TAG, "Gate is fully closed");
      this->operation_finished = true;
      this->position = cover::COVER_CLOSED; // 1.0f
      this->position_ = cover::COVER_CLOSED;
      this->current_operation = cover::COVER_OPERATION_IDLE;
      this->gate_state_ = STATE_CLOSED;
      this->last_state_change_ = now;
      this->log_state_change(old_state, this->gate_state_);
      this->publish_state();
      return;
    }
    else if (msg.substr(11, 7) == "Stopped") {
      ESP_LOGI(TAG, "Gate has stopped");
      this->operation_finished = true;
      this->current_operation = cover::COVER_OPERATION_IDLE;
      this->gate_state_ = STATE_STOPPED;
      this->last_state_change_ = now;
      this->log_state_change(old_state, this->gate_state_);
      // Request status to get current position
      this->queue_gatepro_cmd(GATEPRO_CMD_READ_STATUS);
      this->publish_state();
      return;
    }
  }
}
// Cover component logic functions
////////////////////////////////////////////
void GatePro::control(const cover::CoverCall &call) {
  // Handle stop command
  if (call.get_stop()) {
    ESP_LOGI(TAG, "Cover STOP command received");
    this->queue_gatepro_cmd(GATEPRO_CMD_STOP);
    this->current_operation = cover::COVER_OPERATION_IDLE;
    this->operation_finished = true;
    this->publish_state();
    return;
  }

  // Handle open command
  if (call.get_position().has_value()) {
    auto pos = *call.get_position();
    
    // Fully open command
    if (pos == cover::COVER_OPEN) {
      ESP_LOGI(TAG, "Cover OPEN command received");
      this->queue_gatepro_cmd(GATEPRO_CMD_OPEN);
      this->current_operation = cover::COVER_OPERATION_OPENING;
      this->last_operation_ = cover::COVER_OPERATION_OPENING;
      this->operation_finished = false;
      this->target_position_ = cover::COVER_OPEN;
      this->publish_state();
      return;
    }
    
    // Fully close command
    if (pos == cover::COVER_CLOSED) {
      ESP_LOGI(TAG, "Cover CLOSE command received");
      this->queue_gatepro_cmd(GATEPRO_CMD_CLOSE);
      this->current_operation = cover::COVER_OPERATION_CLOSING;
      this->last_operation_ = cover::COVER_OPERATION_CLOSING;
      this->operation_finished = false;
      this->target_position_ = cover::COVER_CLOSED;
      this->publish_state();
      return;
    }
    
    // Partial position - determine direction
    ESP_LOGI(TAG, "Cover position command: %.2f", pos);
    this->target_position_ = pos;
    
    // Determine direction based on current position
    bool closing = pos < this->position;
    
    // Log the operation
    ESP_LOGI(TAG, "Partial %s to position: %.2f", 
             closing ? "closing" : "opening", pos);
    
    // Send the appropriate command
    this->queue_gatepro_cmd(closing ? GATEPRO_CMD_CLOSE : GATEPRO_CMD_OPEN);
    
    // Update state variables
    this->current_operation = closing ? cover::COVER_OPERATION_CLOSING : cover::COVER_OPERATION_OPENING;
    this->last_operation_ = this->current_operation;
    this->operation_finished = false;
    
    this->publish_state();
  }
}

void GatePro::start_direction_(cover::CoverOperation dir) {
  // If we're already moving in this direction, don't send duplicate commands
  // But allow sending the same command for IDLE (stop) as it's important for safety
  if (this->current_operation == dir && dir != cover::COVER_OPERATION_IDLE) {
    ESP_LOGD(TAG, "Already moving in the requested direction");
    return;
  }

  // Update the current operation before sending the command
  // This ensures the state is updated immediately for better UI responsiveness
  cover::CoverOperation old_operation = this->current_operation;
  this->current_operation = dir;

  // Log the operation change
  ESP_LOGI(TAG, "Changing operation: %s -> %s", 
           old_operation == cover::COVER_OPERATION_IDLE ? "idle" : 
           old_operation == cover::COVER_OPERATION_OPENING ? "opening" : "closing",
           dir == cover::COVER_OPERATION_IDLE ? "idle" : 
           dir == cover::COVER_OPERATION_OPENING ? "opening" : "closing");

  // Send the appropriate command based on the requested direction
  switch (dir) {
    case cover::COVER_OPERATION_IDLE:
      // Even if operation is finished, still send STOP for safety
      this->queue_gatepro_cmd(GATEPRO_CMD_STOP);
      
      // Mark the operation as finished
      this->operation_finished = true;
      break;
      
    case cover::COVER_OPERATION_OPENING:
      this->queue_gatepro_cmd(GATEPRO_CMD_OPEN);
      
      // Mark the operation as in progress
      this->operation_finished = false;
      
      // Update the last operation
      this->last_operation_ = cover::COVER_OPERATION_OPENING;
      break;
      
    case cover::COVER_OPERATION_CLOSING:
      this->queue_gatepro_cmd(GATEPRO_CMD_CLOSE);
      
      // Mark the operation as in progress
      this->operation_finished = false;
      
      // Update the last operation
      this->last_operation_ = cover::COVER_OPERATION_CLOSING;
      break;
      
    default:
      ESP_LOGE(TAG, "Unknown operation requested: %d", dir);
      return;
  }
  
  // Publish the state immediately to update the UI
  this->publish_state();
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

void GatePro::update_state_from_position(float position) {
  // Only update state from position if we're not already in a definitive state
  if (this->gate_state_ == STATE_OPENING || this->gate_state_ == STATE_CLOSING) {
    return;
  }
  
  GateProState old_state = this->gate_state_;
  
  // Determine state based on position
  if (position <= 0.05f) {
    // Position is very close to 0, consider it fully open
    this->gate_state_ = STATE_OPEN;
    this->position = cover::COVER_OPEN;
    this->position_ = cover::COVER_OPEN;
    this->operation_finished = true;
  } else if (position >= 0.95f) {
    // Position is very close to 1, consider it fully closed
    this->gate_state_ = STATE_CLOSED;
    this->position = cover::COVER_CLOSED;
    this->position_ = cover::COVER_CLOSED;
    this->operation_finished = true;
  } else {
    // In between, consider it stopped at an intermediate position
    this->gate_state_ = STATE_STOPPED;
  }
  
  // Log state change if it occurred
  if (old_state != this->gate_state_) {
    this->log_state_change(old_state, this->gate_state_);
  }
}

void GatePro::log_state_change(GateProState old_state, GateProState new_state) {
  const char* old_state_str = "unknown";
  const char* new_state_str = "unknown";
  
  // Convert state enum to string for logging
  switch (old_state) {
    case STATE_UNKNOWN: old_state_str = "unknown"; break;
    case STATE_OPENING: old_state_str = "opening"; break;
    case STATE_OPEN: old_state_str = "open"; break;
    case STATE_CLOSING: old_state_str = "closing"; break;
    case STATE_CLOSED: old_state_str = "closed"; break;
    case STATE_STOPPED: old_state_str = "stopped"; break;
  }
  
  switch (new_state) {
    case STATE_UNKNOWN: new_state_str = "unknown"; break;
    case STATE_OPENING: new_state_str = "opening"; break;
    case STATE_OPEN: new_state_str = "open"; break;
    case STATE_CLOSING: new_state_str = "closing"; break;
    case STATE_CLOSED: new_state_str = "closed"; break;
    case STATE_STOPPED: new_state_str = "stopped"; break;
  }
  
  ESP_LOGI(TAG, "Gate state changed: %s -> %s", old_state_str, new_state_str);
  this->last_state_change_ = millis();
}

////////////////////////////////////////////
// UART operations
////////////////////////////////////////////

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
  //ESP_LOGD(TAG, "%s", res.c_str());
  return res;
}

void GatePro::preprocess(std::string msg) {
    // Check for direct commands from ESPHome UART buttons
    if (msg.substr(0, 9) == "FULL OPEN") {
        ESP_LOGI(TAG, "Received direct FULL OPEN command");
        this->queue_gatepro_cmd(GATEPRO_CMD_OPEN);
        return;
    } else if (msg.substr(0, 10) == "FULL CLOSE") {
        ESP_LOGI(TAG, "Received direct FULL CLOSE command");
        this->queue_gatepro_cmd(GATEPRO_CMD_CLOSE);
        return;
    } else if (msg.substr(0, 2) == "RS") {
        ESP_LOGI(TAG, "Received direct RS command");
        this->queue_gatepro_cmd(GATEPRO_CMD_READ_STATUS);
        return;
    } else if (msg.substr(0, 4) == "STOP") {
        ESP_LOGI(TAG, "Received direct STOP command");
        this->queue_gatepro_cmd(GATEPRO_CMD_STOP);
        return;
    }
    
    // Normal message processing
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

////////////////////////////////////////////
// Component functions
////////////////////////////////////////////
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
  this->gate_state_ = STATE_UNKNOWN;
  this->last_status_request_ = 0;
  this->last_state_change_ = 0;
  this->force_state_update_ = true;
  this->consecutive_position_readings_ = 0;
  this->last_position_reading_ = -1.0f;
  this->last_pattern_seen_ = "";
  this->consecutive_pattern_readings_ = 0;
  this->queue_gatepro_cmd(GATEPRO_CMD_READ_STATUS);
  this->blocker = false;
  this->target_position_ = 0.0f;
}

void GatePro::update() {
  uint32_t now = millis();
  
  // Always publish the current state to ensure ESPHome stays in sync
  this->publish();
  
  // Check if we need to stop at target position
  this->stop_at_target_position();
  
  // Process any pending UART messages
  this->write_uart();

  // If we're in an unknown state or if we need to force an update
  if (this->gate_state_ == STATE_UNKNOWN || this->force_state_update_) {
    this->queue_gatepro_cmd(GATEPRO_CMD_READ_STATUS);
    this->force_state_update_ = false;
  }

  this->correction_after_operation();
}

void GatePro::loop() {
  // keep reading uart for changes
  this->read_uart();
  this->process();
}

void GatePro::dump_config(){
    ESP_LOGCONFIG(TAG, "GatePro sensor dump config");
}

}  // namespace gatepro
}  // namespace esphome