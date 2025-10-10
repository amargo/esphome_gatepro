#include "esphome/core/log.h"
#include "gatepro.h"
#include <vector>
#include <functional>

namespace esphome {
namespace gatepro {

////////////////////////////////////
static const char* TAG = "gatepro";

////////////////////////////////////////////
// Helper / misc functions
////////////////////////////////////////////
std::string GatePro::get_command_string(GateProCmd cmd) {
   static char cmd_buffer[100];
   
   auto it = GateProCmdTemplates.find(cmd);
   if (it == GateProCmdTemplates.end()) {
      ESP_LOGE(TAG, "Unknown command type: %d", cmd);
      return "";
   }
   
   const char* template_str = it->second;
   
   // Special case for WRITE_PARAMS - no source parameter needed
   if (cmd == GATEPRO_CMD_WRITE_PARAMS) {
      return std::string(template_str);
   }
   
   // Format with source parameter
   snprintf(cmd_buffer, sizeof(cmd_buffer), template_str, this->source_.c_str());
   return std::string(cmd_buffer);
}

void GatePro::queue_gatepro_cmd(GateProCmd cmd) {
   std::string cmd_str = this->get_command_string(cmd);
   if (!cmd_str.empty()) {
      // Prevent queue overflow
      if (this->tx_queue.size() >= MAX_QUEUE_SIZE) {
         ESP_LOGW(TAG, "TX queue full, dropping oldest command");
         this->tx_queue.pop();
      }
      this->tx_queue.push(cmd_str);
      ESP_LOGD(TAG, "Queued command: %s (queue size: %zu)", cmd_str.c_str(), this->tx_queue.size());
   }
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

  // Read param example: ACK RP,1:1,0,0,1,2,2,0,0,0,3,0,0,3,0,0,0,0\r\n
  if (msg.substr(0, 6) == "ACK RP") {
      this->parse_params(msg);
      return;
   }

   // ACK WP example: ACK WP,1\r\n
   if (msg.substr(0, 6) == "ACK WP") {
      ESP_LOGD(TAG, "Write params acknowledged");
      return;
   }

   // Devinfo example: ACK READ DEVINFO:P500BU,PS21053C,V01\r\n
   if (msg.substr(0, 16) == "ACK READ DEVINFO") {
      if (this->txt_devinfo) {
        this->txt_devinfo->publish_state(msg.substr(17, msg.size() - (17 + 4)));
      }
      return;
   }

   // Learn status example: ACK LEARN STATUS:SYSTEM LEARN COMPLETE,0\r\n
   if (msg.substr(0, 16) == "ACK LEARN STATUS") {
      if (this->txt_learn_status) {
        this->txt_learn_status->publish_state(msg.substr(17, msg.size() - (17 + 4)));
      }
      return;
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
    // Check if anything on UART buffer
    int available = this->available();
    if (!available) {
        return;
    }
    
    // Buffer overflow protection - clear if too large
    if (this->msg_buff.length() > MAX_UART_BUFFER_SIZE) {
        ESP_LOGW(TAG, "UART buffer overflow (%zu bytes), clearing buffer", this->msg_buff.length());
        this->msg_buff.clear();
    }
    
    // Use stack-based buffer to avoid dynamic allocation
    uint8_t bytes[UART_READ_BUFFER_SIZE];
    int to_read = std::min(available, (int)UART_READ_BUFFER_SIZE);
    
    // Read available data in chunks if necessary
    while (available > 0 && this->msg_buff.length() < MAX_UART_BUFFER_SIZE) {
        int chunk_size = std::min(available, (int)UART_READ_BUFFER_SIZE);
        this->read_array(bytes, chunk_size);
        this->msg_buff += this->convert(bytes, chunk_size);
        available -= chunk_size;
        
        // Update available count
        available = this->available();
    }

    // Process all complete messages in the buffer
    size_t pos;
    int processed_messages = 0;
    const int MAX_MESSAGES_PER_CYCLE = 5; // Prevent infinite loops
    
    while ((pos = this->msg_buff.find(this->delimiter)) != std::string::npos && 
           processed_messages < MAX_MESSAGES_PER_CYCLE) {
        
        // Extract complete message
        std::string complete_msg = this->msg_buff.substr(0, pos + this->delimiter_length);
        
        // Add to processing queue
        this->rx_queue.push(complete_msg);
        
        // Remove processed message from buffer
        this->msg_buff = this->msg_buff.substr(pos + this->delimiter_length);
        
        processed_messages++;
        
        ESP_LOGD(TAG, "Processed message %d: %s", processed_messages, complete_msg.c_str());
    }
    
    // Log if we hit the message limit
    if (processed_messages >= MAX_MESSAGES_PER_CYCLE) {
        ESP_LOGD(TAG, "Processed maximum messages per cycle (%d), remaining buffer: %zu bytes", 
                 MAX_MESSAGES_PER_CYCLE, this->msg_buff.length());
    }
}

void GatePro::write_uart() {
   if (this->tx_queue.size()) {
      std::string cmd_str = this->tx_queue.front();
      cmd_str += this->tx_delimiter;
      this->write_str(cmd_str.c_str());
      ESP_LOGD(TAG, "UART TX[%d]: %s", this->tx_queue.size(), cmd_str.c_str());
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

////////////////////////////////////////////
// Parameter functions
////////////////////////////////////////////
void GatePro::set_param(int idx, int val) {
   ESP_LOGD(TAG, "Initiating setting param %d to %d", idx, val);
   
   // Validate parameter index
   if (idx < 0 || idx >= 17) {  // GatePro has 17 parameters (0-16)
      ESP_LOGE(TAG, "Invalid parameter index: %d (valid range: 0-16)", idx);
      return;
   }
   
   this->param_no_pub = true;
   this->queue_gatepro_cmd(GATEPRO_CMD_READ_PARAMS);

   this->paramTaskQueue.push(
      [this, idx, val](){
         ESP_LOGD(TAG, "Setting param %d to %d", idx, val);
         // Ensure params vector is large enough
         if (this->params.size() <= idx) {
            this->params.resize(idx + 1, 0);
         }
         this->params[idx] = val;
         this->write_params();
      });
}

void GatePro::publish_params() {
   if (!this->param_no_pub && this->params.size() >= 16) {  // Ensure we have enough parameters
      // Safely publish parameters with bounds checking
      if (this->speed_slider && this->params.size() > 3) 
         this->speed_slider->publish_state(this->params[3]);
      if (this->decel_dist_slider && this->params.size() > 4) 
         this->decel_dist_slider->publish_state(this->params[4]);
      if (this->decel_speed_slider && this->params.size() > 5) 
         this->decel_speed_slider->publish_state(this->params[5]);
      if (this->max_amp_slider && this->params.size() > 6) 
         this->max_amp_slider->publish_state(this->params[6]);
      if (this->auto_close_slider && this->params.size() > 1) 
         this->auto_close_slider->publish_state(this->params[1]);
      if (this->sw_permalock && this->params.size() > 15) 
         this->sw_permalock->publish_state(this->params[15]);
      if (this->sw_infra1 && this->params.size() > 13) 
         this->sw_infra1->publish_state(this->params[13]);
      if (this->sw_infra2 && this->params.size() > 14) 
         this->sw_infra2->publish_state(this->params[14]);
   }
}

void GatePro::parse_params(std::string msg) {
   this->params.clear();
   // example: ACK RP,1:1,0,0,1,2,2,0,0,0,3,0,0,3,0,0,0,0\r\n
   //                   ^-9  
   msg = msg.substr(9, 33);
   size_t start = 0;
   size_t end;

   // efficiently split on ','
   while((end = msg.find(',', start)) != std::string::npos) {
      this->params.push_back(stoi(msg.substr(start, end - start)));
      start = end + 1;
   }
   this->params.push_back(stoi(msg.substr(start)));

   ESP_LOGD(TAG, "Parsed current params: %zu", this->params.size());
   for (size_t i = 0; i < this->params.size(); ++i) {
      ESP_LOGD(TAG, "  [%zu] = %d", i, this->params[i]);
   }

   this->publish_params();

   // write new params if any task is up
   while (!this->paramTaskQueue.empty()) {
      auto task = this->paramTaskQueue.front();
      this->paramTaskQueue.pop();
      task();
      this->param_no_pub = false;
   }
}

void GatePro::write_params() {
   std::string msg = "WP,1:";
   for (size_t i = 0; i < this->params.size(); i++) {
      msg += std::to_string(this->params[i]);
      if (i != this->params.size() -1) {
         msg += ",";
      }
   }
   ESP_LOGD(TAG, "BUILT PARAMS: %s", msg.c_str());
   this->tx_queue.push(msg);

   // read params again just to update frontend and make sure :)
   this->queue_gatepro_cmd(GATEPRO_CMD_READ_PARAMS);
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
   this->msg_buff = "";
   this->queue_gatepro_cmd(GATEPRO_CMD_READ_STATUS);
   this->blocker = false;
   this->target_position_ = 0.0f;
   
   // Initialize parameter system
   this->queue_gatepro_cmd(GATEPRO_CMD_READ_PARAMS);
   this->queue_gatepro_cmd(GATEPRO_CMD_DEVINFO);
   this->queue_gatepro_cmd(GATEPRO_CMD_READ_LEARN_STATUS);

   // Setup basic operation button callbacks
   if (this->btn_open) {
      this->btn_open->add_on_press_callback([this]() {
         ESP_LOGD(TAG, "Open button pressed");
         this->queue_gatepro_cmd(GATEPRO_CMD_OPEN);
      });
   }
   if (this->btn_close) {
      this->btn_close->add_on_press_callback([this]() {
         ESP_LOGD(TAG, "Close button pressed");
         this->queue_gatepro_cmd(GATEPRO_CMD_CLOSE);
      });
   }
   if (this->btn_stop) {
      this->btn_stop->add_on_press_callback([this]() {
         ESP_LOGD(TAG, "Stop button pressed");
         this->queue_gatepro_cmd(GATEPRO_CMD_STOP);
      });
   }
   
   // Setup advanced button callbacks
   if (this->btn_learn) {
      this->btn_learn->add_on_press_callback([this]() {
         ESP_LOGD(TAG, "Learn button pressed");
         this->queue_gatepro_cmd(GATEPRO_CMD_LEARN);
      });
   }
   if (this->btn_params_od) {
      this->btn_params_od->add_on_press_callback([this]() {
         ESP_LOGD(TAG, "Params OD button pressed");
         this->queue_gatepro_cmd(GATEPRO_CMD_READ_PARAMS);
      });
   }
   if (this->btn_remote_learn) {
      this->btn_remote_learn->add_on_press_callback([this]() {
         ESP_LOGD(TAG, "Remote learn button pressed");
         this->queue_gatepro_cmd(GATEPRO_CMD_REMOTE_LEARN);
      });
   }
   
   // Set up number slider callbacks
      this->speed_slider->add_on_state_callback([this](float value){
         int int_value = (int)value;
         if (this->params.size() > 3 && this->params[3] == int_value) {
            return;
         }
         this->set_param(3, int_value);
      });
   }

   if (decel_dist_slider) {
      this->decel_dist_slider->add_on_state_callback([this](float value){
         int int_value = (int)value;
         if (this->params.size() > 4 && this->params[4] == int_value) {
            return;
         }
         this->set_param(4, int_value);
      });
   }

   if (decel_speed_slider) {
      this->decel_speed_slider->add_on_state_callback([this](float value){
         int int_value = (int)value;
         if (this->params.size() > 5 && this->params[5] == int_value) {
            return;
         }
         this->set_param(5, int_value);
      });
   }

   if (max_amp_slider) {
      this->max_amp_slider->add_on_state_callback([this](float value){
         int int_value = (int)value;
         if (this->params.size() > 6 && this->params[6] == int_value) {
            return;
         }
         this->set_param(6, int_value);
      });
   }

   if (auto_close_slider) {
      this->auto_close_slider->add_on_state_callback([this](float value){
         int int_value = (int)value;
         if (this->params.size() > 1 && this->params[1] == int_value) {
            return;
         }
         this->set_param(1, int_value);
      });
   }

   // Set up switch callbacks
   if (sw_permalock) {
      this->sw_permalock->add_on_state_callback([this](bool state){
         if (this->params.size() > 15 && this->params[15] == (state ? 1 : 0)) {
            return;
         }
         this->set_param(15, state ? 1 : 0);
      });
   }

   if (sw_infra1) {
      this->sw_infra1->add_on_state_callback([this](bool state){
         if (this->params.size() > 13 && this->params[13] == (state ? 1 : 0)) {
            return;
         }
         this->set_param(13, state ? 1 : 0);
      });
   }

   if (sw_infra2) {
      this->sw_infra2->add_on_state_callback([this](bool state){
         if (this->params.size() > 14 && this->params[14] == (state ? 1 : 0)) {
            return;
         }
         this->set_param(14, state ? 1 : 0);
      });
   }
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