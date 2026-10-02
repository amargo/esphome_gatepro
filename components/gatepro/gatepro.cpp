#include "esphome/core/log.h"
#include "gatepro.h"
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <vector>
#include <functional>

namespace esphome {
namespace gatepro {

////////////////////////////////////
static const char* TAG = "gatepro";

using namespace protocol;

////////////////////////////////////////////
// Helper / misc functions
////////////////////////////////////////////
std::string GatePro::get_command_string(GateProCmd cmd) {
   char cmd_buffer[100];
   
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
   int len = snprintf(cmd_buffer, sizeof(cmd_buffer), template_str, this->source_.c_str());
   if (len < 0 || (size_t) len >= sizeof(cmd_buffer)) {
      ESP_LOGE(TAG, "Command too long, not sending (cmd %d)", cmd);
      return "";
   }
   return std::string(cmd_buffer);
}

void GatePro::queue_gatepro_cmd(GateProCmd cmd) {
   std::string cmd_str = this->get_command_string(cmd);
   if (cmd_str.empty()) {
      return;
   }
   // STOP is safety relevant: it must never wait behind other commands
   this->enqueue_tx_(cmd_str, cmd == GATEPRO_CMD_STOP);
}

static bool is_motion_cmd(const std::string &cmd) {
   return starts_with(cmd, "FULL OPEN") || starts_with(cmd, "FULL CLOSE") || starts_with(cmd, "PED OPEN");
}

void GatePro::enqueue_tx_(const std::string &cmd, bool priority) {
   // A new STOP/motion command supersedes any motion command still waiting,
   // so a stale OPEN/CLOSE can never be sent after the user pressed STOP.
   if (priority || is_motion_cmd(cmd)) {
      for (auto it = this->tx_queue.begin(); it != this->tx_queue.end();) {
         if (is_motion_cmd(*it)) {
            ESP_LOGD(TAG, "Dropping superseded command: %s", it->c_str());
            it = this->tx_queue.erase(it);
         } else {
            ++it;
         }
      }
   }

   // Avoid flooding the bus with identical requests (e.g. repeated RS polls)
   for (const auto &queued : this->tx_queue) {
      if (queued == cmd) {
         ESP_LOGV(TAG, "Command already queued: %s", cmd.c_str());
         return;
      }
   }

   // Prevent queue overflow: drop the oldest non-STOP command
   if (this->tx_queue.size() >= MAX_QUEUE_SIZE) {
      auto victim = this->tx_queue.begin();
      while (victim != this->tx_queue.end() && starts_with(*victim, "STOP")) {
         ++victim;
      }
      if (victim == this->tx_queue.end()) {
         ESP_LOGW(TAG, "TX queue full, dropping new command: %s", cmd.c_str());
         return;
      }
      ESP_LOGW(TAG, "TX queue full, dropping command: %s", victim->c_str());
      this->tx_queue.erase(victim);
   }

   if (priority) {
      this->tx_queue.push_front(cmd);
   } else {
      this->tx_queue.push_back(cmd);
   }
   ESP_LOGD(TAG, "Queued command: %s (queue size: %zu)", cmd.c_str(), this->tx_queue.size());
}

void GatePro::publish() {
    if (this->position_ != this->position) {
      this->position_ = this->position;
      this->publish_ticks_left_ = PUBLISH_AFTER_TICKS;
    } else if (this->publish_ticks_left_ == 0) {
      return;
    } else {
      this->publish_ticks_left_--;
    }
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
  if (starts_with(msg, "ACK RS")) {
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
    
    // Motion started outside ESPHome (remote control, missed $V1PKF0 event,
    // reboot mid-travel): derive the direction from the status itself.
    if (this->current_operation == cover::COVER_OPERATION_IDLE) {
      const bool opening = status_is_opening(msg);
      if (opening || status_is_moving(msg)) {
        ESP_LOGI(TAG, "Motion detected from status: %s", opening ? "opening" : "closing");
        GateProState old_state = this->gate_state_;
        this->operation_finished = false;
        this->current_operation = opening ? cover::COVER_OPERATION_OPENING : cover::COVER_OPERATION_CLOSING;
        this->last_operation_ = this->current_operation;
        this->gate_state_ = opening ? STATE_OPENING : STATE_CLOSING;
        this->log_state_change(old_state, this->gate_state_);
      }
    }

    // For position updates, only process them if the gate is in motion
    // This prevents position updates when the gate is stationary
    if (!this->operation_finished || this->current_operation != cover::COVER_OPERATION_IDLE) {
      // Extract the position value (hex, 0-100 after offset correction)
      int percentage;
      if (!parse_position(msg, percentage)) {
        ESP_LOGW(TAG, "Ignoring invalid position in ACK RS message: %s", msg.c_str());
        return;
      }
      
      // While moving, the controller may report 0/100 % long before the end
      // stop (e.g. after a direction change). End positions are only set by
      // the Opened/Closed events.
      if (this->current_operation != cover::COVER_OPERATION_IDLE) {
        percentage = std::max(1, std::min(99, percentage));
      }
      float new_position = (float)percentage / 100;
      
      if (this->operation_finished && this->current_operation == cover::COVER_OPERATION_IDLE) {
        if (new_position <= 0.01f) {
            new_position = 0.0f;
        } else if (new_position >= 0.99f) {
            new_position = 1.0f;
        }
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
  if (starts_with(msg, "$V1PKF0")) {
    ESP_LOGI(TAG, "Received motor event: %s", msg.c_str());
    GateProState old_state = this->gate_state_;
    uint32_t now = millis();
    
    // Reset pattern detection when we receive direct motor events
    this->last_pattern_seen_ = "";
    this->consecutive_pattern_readings_ = 0;
    
    if (field_equals(msg, 11, "Opening")) {
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
    else if (field_equals(msg, 11, "Opened")) {
      ESP_LOGI(TAG, "Gate is fully open");
      this->stop_at_target_ = false;
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
    else if (field_equals(msg, 11, "Closing") || field_equals(msg, 11, "AutoClosing")) {
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
    else if (field_equals(msg, 11, "Closed")) {
      ESP_LOGI(TAG, "Gate is fully closed");
      this->stop_at_target_ = false;
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
    else if (field_equals(msg, 11, "PedOpening")) {
      ESP_LOGI(TAG, "Gate is opening (pedestrian)");
      this->stop_at_target_ = false;
      this->operation_finished = false;
      this->current_operation = cover::COVER_OPERATION_OPENING;
      this->last_operation_ = cover::COVER_OPERATION_OPENING;
      this->gate_state_ = STATE_OPENING;
      this->last_state_change_ = now;
      this->log_state_change(old_state, this->gate_state_);
      this->publish_state();
      return;
    }
    else if (field_equals(msg, 11, "PedOpened")) {
      // Pedestrian opening ends at a partial position, not at COVER_OPEN.
      ESP_LOGI(TAG, "Pedestrian opening finished");
      this->operation_finished = true;
      this->current_operation = cover::COVER_OPERATION_IDLE;
      this->gate_state_ = STATE_STOPPED;
      this->last_state_change_ = now;
      this->log_state_change(old_state, this->gate_state_);
      this->publish_state();
      return;
    }
    else if (field_equals(msg, 11, "Stopped")) {
      ESP_LOGI(TAG, "Gate has stopped");
      this->stop_at_target_ = false;
      this->operation_finished = true;
      this->current_operation = cover::COVER_OPERATION_IDLE;
      // Only request status on first Stopped event to avoid RS command flooding.
      // The motor sends Stopped repeatedly (~200ms) which would overflow the TX queue.
      bool was_already_stopped = (this->gate_state_ == STATE_STOPPED);
      this->gate_state_ = STATE_STOPPED;
      this->last_state_change_ = now;
      this->log_state_change(old_state, this->gate_state_);
      if (!was_already_stopped) {
        this->queue_gatepro_cmd(GATEPRO_CMD_READ_STATUS);
      }
      this->publish_state();
      return;
    }
  }
  
  // Read param example: ACK RP,1:1,0,0,1,2,2,0,0,0,3,0,0,3,0,0,0,0\r\n
  if (starts_with(msg, "ACK RP")) {
      this->parse_params(msg);
      return;
   }

   // ACK WP example: ACK WP,1\r\n
   if (starts_with(msg, "ACK WP")) {
      ESP_LOGD(TAG, "Write params acknowledged");
      return;
   }

   // Devinfo example: ACK READ DEVINFO:P500BU,PS21053C,V01\r\n
   if (starts_with(msg, "ACK READ DEVINFO")) {
      if (this->txt_devinfo) {
        this->txt_devinfo->publish_state(payload_after(msg, 17));
      }
      return;
   }

   // Learn status example: ACK LEARN STATUS:SYSTEM LEARN COMPLETE,0\r\n
   if (starts_with(msg, "ACK LEARN STATUS")) {
      if (this->txt_learn_status) {
        this->txt_learn_status->publish_state(payload_after(msg, 17));
      }
      return;
   }
}
// Cover component logic functions
////////////////////////////////////////////
void GatePro::control(const cover::CoverCall &call) {
  // Handle stop command
  if (call.get_stop()) {
    ESP_LOGI(TAG, "Cover STOP command received");
    this->stop_at_target_ = false;
    this->queue_gatepro_cmd(GATEPRO_CMD_STOP);
    this->current_operation = cover::COVER_OPERATION_IDLE;
    this->operation_finished = true;
    this->publish_state();
    return;
  }

  // Handle open command
  if (call.get_position().has_value()) {
    auto pos = *call.get_position();
    if (std::isnan(pos) || pos < cover::COVER_CLOSED || pos > cover::COVER_OPEN) {
      ESP_LOGW(TAG, "Ignoring invalid position: %.2f", pos);
      return;
    }

    // Fully open command
    if (pos == cover::COVER_OPEN) {
      ESP_LOGI(TAG, "Cover OPEN command received");
      this->stop_at_target_ = false;
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
      this->stop_at_target_ = false;
      this->queue_gatepro_cmd(GATEPRO_CMD_CLOSE);
      this->current_operation = cover::COVER_OPERATION_CLOSING;
      this->last_operation_ = cover::COVER_OPERATION_CLOSING;
      this->operation_finished = false;
      this->target_position_ = cover::COVER_CLOSED;
      this->publish_state();
      return;
    }
    
    // Partial position - ignore if we're already there, otherwise the
    // "opening" fallback below would drive the gate fully open.
    if (this->current_operation == cover::COVER_OPERATION_IDLE &&
        std::fabs(pos - this->position) < this->acceptable_diff) {
      ESP_LOGI(TAG, "Already at requested position %.2f", pos);
      return;
    }

    // Partial position - determine direction
    ESP_LOGI(TAG, "Cover position command: %.2f", pos);
    this->target_position_ = pos;
    this->stop_at_target_ = true;
    
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
    return;
  }

  // Handle toggle command (advertised in traits)
  if (call.get_toggle().has_value()) {
    auto next = this->make_call();
    if (this->current_operation != cover::COVER_OPERATION_IDLE) {
      next.set_command_stop();
    } else if (this->position == cover::COVER_OPEN ||
               (this->position != cover::COVER_CLOSED &&
                this->last_operation_ == cover::COVER_OPERATION_OPENING)) {
      next.set_command_close();
    } else {
      next.set_command_open();
    }
    next.perform();
  }
}

void GatePro::correction_after_operation() {
    // Only correct position when the motor confirmed a definitive end state
    // via a "Closed" or "Opened" event (gate_state_ == STATE_CLOSED/STATE_OPEN).
    // Do NOT correct for STATE_STOPPED: the gate may have stopped mid-travel,
    // and stale last_operation_/target_position_ values would force wrong position.
    if (this->operation_finished &&
        this->current_operation == cover::COVER_OPERATION_IDLE) {
      if (this->gate_state_ == STATE_CLOSED && this->position != cover::COVER_CLOSED) {
        this->position = cover::COVER_CLOSED;
        return;
      }
      if (this->gate_state_ == STATE_OPEN && this->position != cover::COVER_OPEN) {
        this->position = cover::COVER_OPEN;
      }
  }

  // This function is called from process() which has access to msg
  // These message handlers were moved to process() method
}

void GatePro::stop_at_target_position() {
  if (!this->stop_at_target_ || this->current_operation == cover::COVER_OPERATION_IDLE) {
    return;
  }
  // Also stop if we overshot the target between two status polls,
  // otherwise the gate would travel to its end position.
  const bool reached =
      this->current_operation == cover::COVER_OPERATION_CLOSING
          ? this->position <= this->target_position_ + this->acceptable_diff
          : this->position >= this->target_position_ - this->acceptable_diff;
  if (reached) {
    ESP_LOGI(TAG, "Target position %.2f reached (%.2f), stopping", this->target_position_, this->position);
    this->stop_at_target_ = false;
    this->make_call().set_command_stop().perform();
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
    
    // Read available data in chunks if necessary
    while (available > 0 && this->msg_buff.length() < MAX_UART_BUFFER_SIZE) {
        int chunk_size = std::min(available, (int)UART_READ_BUFFER_SIZE);
        if (!this->read_array(bytes, chunk_size)) {
            ESP_LOGW(TAG, "UART read failed");
            break;
        }
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
        
        // Add to processing queue (bounded: drop oldest if the consumer lags)
        if (this->rx_queue.size() >= MAX_RX_QUEUE_SIZE) {
            ESP_LOGW(TAG, "RX queue full, dropping oldest message");
            this->rx_queue.pop();
        }
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
   if (!this->tx_queue.empty()) {
      std::string cmd_str = this->tx_queue.front();
      this->tx_queue.pop_front();
      ESP_LOGD(TAG, "UART TX[%zu]: %s", this->tx_queue.size(), cmd_str.c_str());
      cmd_str += this->tx_delimiter;
      this->write_str(cmd_str.c_str());
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
      snprintf(buf, sizeof(buf), "\\x%02X", bytes[i]);
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
   
   // Validate parameter index and value
   if (idx < 0 || idx >= (int) NUM_PARAMS) {
      ESP_LOGE(TAG, "Invalid parameter index: %d (valid range: 0-%d)", idx, (int) NUM_PARAMS - 1);
      return;
   }
   if (val < 0 || val > MAX_PARAM_VALUE) {
      ESP_LOGE(TAG, "Invalid value %d for parameter %d (valid range: 0-%d)", val, idx, MAX_PARAM_VALUE);
      return;
   }
   if (this->paramTaskQueue.size() >= MAX_PARAM_TASKS) {
      ESP_LOGW(TAG, "Too many pending parameter writes, ignoring param %d", idx);
      return;
   }

   this->param_no_pub = true;
   this->queue_gatepro_cmd(GATEPRO_CMD_READ_PARAMS);

   // Applied only after a fresh, fully valid parameter read (see parse_params),
   // so we never write a partial or stale parameter set back to the motor.
   this->paramTaskQueue.push(
      [this, idx, val](){
         ESP_LOGD(TAG, "Setting param %d to %d", idx, val);
         this->params[idx] = val;
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
      if (this->small_gate_timer && this->params.size() > 7) 
        this->small_gate_timer->publish_state(this->params[7]);
      if (this->force_detection_number && this->params.size() > 9) 
        this->force_detection_number->publish_state(this->params[9]);
   }
}

void GatePro::parse_params(const std::string &msg) {
   // example: ACK RP,1:1,0,0,1,2,2,0,0,0,3,0,0,3,0,0,0,0\r\n
   //                   ^-9
   std::vector<int> parsed;
   if (!protocol::parse_params(msg, parsed)) {
      ESP_LOGE(TAG, "Invalid parameter response, ignoring: %s", msg.c_str());
      if (!this->paramTaskQueue.empty()) {
         ESP_LOGW(TAG, "Discarding %zu pending parameter write(s)", this->paramTaskQueue.size());
         while (!this->paramTaskQueue.empty()) {
            this->paramTaskQueue.pop();
         }
      }
      this->param_no_pub = false;
      return;
   }
   this->params = parsed;

   ESP_LOGD(TAG, "Parsed current params: %zu", this->params.size());
   for (size_t i = 0; i < this->params.size(); ++i) {
      ESP_LOGD(TAG, "  [%zu] = %d", i, this->params[i]);
   }

   this->publish_params();

   // apply pending changes and write them in a single WP command
   if (!this->paramTaskQueue.empty()) {
      while (!this->paramTaskQueue.empty()) {
         auto task = this->paramTaskQueue.front();
         this->paramTaskQueue.pop();
         task();
      }
      this->param_no_pub = false;
      this->write_params();
   }
}

void GatePro::write_params() {
   if (this->params.size() != NUM_PARAMS) {
      ESP_LOGE(TAG, "Refusing to write incomplete parameter set (%d/%d)", (int) this->params.size(), (int) NUM_PARAMS);
      return;
   }
   std::string msg = "WP,1:";
   for (size_t i = 0; i < this->params.size(); i++) {
      msg += std::to_string(this->params[i]);
      if (i != this->params.size() -1) {
         msg += ",";
      }
   }
   ESP_LOGD(TAG, "BUILT PARAMS: %s", msg.c_str());
   this->enqueue_tx_(msg);

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
   this->last_state_change_ = 0;
   this->force_state_update_ = true;
   this->last_pattern_seen_ = "";
   this->consecutive_pattern_readings_ = 0;
   this->msg_buff = "";
   this->queue_gatepro_cmd(GATEPRO_CMD_READ_STATUS);
   this->target_position_ = 0.0f;

   // Initialize parameter system
   this->queue_gatepro_cmd(GATEPRO_CMD_READ_PARAMS);
   this->queue_gatepro_cmd(GATEPRO_CMD_DEVINFO);
   this->queue_gatepro_cmd(GATEPRO_CMD_READ_LEARN_STATUS);

   // Setup basic operation button callbacks
   if (this->btn_open) {
      this->btn_open->add_on_press_callback([this]() {
         ESP_LOGD(TAG, "Open button pressed");
         this->stop_at_target_ = false;
         this->queue_gatepro_cmd(GATEPRO_CMD_OPEN);
      });
   }
   if (this->btn_close) {
      this->btn_close->add_on_press_callback([this]() {
         ESP_LOGD(TAG, "Close button pressed");
         this->stop_at_target_ = false;
         this->queue_gatepro_cmd(GATEPRO_CMD_CLOSE);
      });
   }
   if (this->btn_stop) {
      this->btn_stop->add_on_press_callback([this]() {
         ESP_LOGD(TAG, "Stop button pressed");
         this->stop_at_target_ = false;
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
   if (this->speed_slider) {
      this->speed_slider->add_on_state_callback([this](float value){
         int int_value = (int)value;
         if (this->params.size() > 3 && this->params[3] == int_value) {
            return;
         }
         // Group 4: 0-3 (1=default, 1=50%, 2=70%, 3=85%, 4=100%)
         this->set_param(3, int_value);
      });
   }

   if (this->decel_dist_slider) {
      this->decel_dist_slider->add_on_state_callback([this](float value){
         int int_value = (int)value;
         if (this->params.size() > 4 && this->params[4] == int_value) {
            return;
         }
         // Group 5: 0-4 (1=default, 1=75%, 2=80%, 3=85%, 4=90%, 5=95%)
         this->set_param(4, int_value);
      });
   }

   if (this->decel_speed_slider) {
      this->decel_speed_slider->add_on_state_callback([this](float value){
         int int_value = (int)value;
         if (this->params.size() > 5 && this->params[5] == int_value) {
            return;
         }
         // Group 6: 0-3 (1=default, 1=80%, 2=60%, 3=40%, 4=25%)
         this->set_param(5, int_value);
      });
   }

   if (this->max_amp_slider) {
      this->max_amp_slider->add_on_state_callback([this](float value){
         int int_value = (int)value;
         if (this->params.size() > 6 && this->params[6] == int_value) {
            return;
         }
         // Group 7: 0-9 (1=default, 1=2A, 2=3A, 3=4A, 4=5A, 5=6A, 6=7A, 7=8A, 8=9A, 9=10A, A=11A, C=12A, E=13A)
         this->set_param(6, int_value);
      });
   }

   if (this->auto_close_slider) {
      this->auto_close_slider->add_on_state_callback([this](float value){
         int int_value = (int)value;
         if (this->params.size() > 1 && this->params[1] == int_value) {
            return;
         }
         // Group 2: 0-8 (0=disabled, 1=5s, 2=15s, 3=30s, 4=45s, 5=60s, 6=80s, 7=120s, 8=180s)
         this->set_param(1, int_value);
      });
   }

   // Set up switch callbacks
   if (this->sw_permalock) {
      this->sw_permalock->add_on_state_callback([this](bool state){
         if (this->params.size() > 15 && this->params[15] == (state ? 1 : 0)) {
            return;
         }
         // Group L: L-0: disabled, L-1: enabled
         this->set_param(15, state ? 1 : 0);
      });
   }

   // infra1
  if (this->sw_infra1) {
    this->sw_infra1->add_on_state_callback([this](bool state){
        if (this->params.size() > 13 && this->params[13] == (state ? 1 : 0)) {
          return;
        }
        // Group H: H-0: disabled, H-1: enabled
        this->set_param(13, state ? 1 : 0);
    });
   }

   // infra2
   if (this->sw_infra2) {
    this->sw_infra2->add_on_state_callback([this](bool state){
      if (this->params.size() > 14 && this->params[14] == (state ? 1 : 0)) {
        return;
      }
      // Group J: J-0: disabled, J-1: enabled
      this->set_param(14, state ? 1 : 0);
    });
   }

  // Small gate timer callback
  if (this->small_gate_timer) {
      this->small_gate_timer->add_on_state_callback([this](float value){
        int int_value = (int)value;
        if (this->params.size() > 7 && this->params[7] == int_value) {
            return;
        }
        // Group 8: 1-6 (1=3s, 2=6s, 3=9s, 4=12s, 5=15s, 6=18s)
        this->set_param(7, int_value);
      });
  }

   // Force detection timer callback
   if (this->force_detection_number) {
    this->force_detection_number->add_on_state_callback([this](float value){
        int int_value = (int)value;
        if (this->params.size() > 9 && this->params[9] == int_value) {
            return;
        }
       // Group A: 0-3 (0=disabled, 1=Stop + reverse 1mp, 2=Stop + reverse 3mp, 3=Stop + reverse to end)
       this->set_param(9, int_value);
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
  if (this->gate_state_ == STATE_UNKNOWN || 
      this->force_state_update_ || 
      this->current_operation != cover::COVER_OPERATION_IDLE) {
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
    ESP_LOGCONFIG(TAG, "GatePro:");
    ESP_LOGCONFIG(TAG, "  Source: %s", this->source_.c_str());
    LOG_UPDATE_INTERVAL(this);
}

}  // namespace gatepro
}  // namespace esphome
