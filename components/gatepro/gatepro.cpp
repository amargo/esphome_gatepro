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

  //ESP_LOGD(TAG, "UART RX: %s", (const char*)msg.c_str());

  // example: ACK RS:00,80,C4,C6,3E,16,FF,FF,FF\r\n
  //                          ^- percentage in hex
  if (msg.substr(0, 6) == "ACK RS") {
    // status only matters when in motion (operation not finished) 
    if (this->operation_finished) {
      return;
    }
    msg = msg.substr(16, 2);
    int percentage = stoi(msg, 0, 16);
    // percentage correction with known offset, if necessary
    if (percentage > 100) {
      percentage -= this->known_percentage_offset;
    }
    this->position = (float)percentage / 100;

    return;
  }

  // Event message from the motor
  // example: $V1PKF0,17,Closed;src=0001\r\n
  if (msg.substr(0, 7) == "$V1PKF0") {
    if (msg.substr(11, 7) == "Opening") {
      this->operation_finished = false;
      this->current_operation = cover::COVER_OPERATION_OPENING;
      this->last_operation_ = cover::COVER_OPERATION_OPENING;
      return;
    }
    if (msg.substr(11, 6) == "Opened") {
      this->operation_finished = true;
      this->target_position_ = 0.0f;
      this->current_operation = cover::COVER_OPERATION_IDLE;
      return;
    }
    if (msg.substr(11, 7) == "Closing") {
      this->operation_finished = false;
      this->current_operation = cover::COVER_OPERATION_CLOSING;
      this->last_operation_ = cover::COVER_OPERATION_CLOSING;
      return;
    }
    if (msg.substr(11, 6) == "Closed") {
      this->operation_finished = true;
      this->target_position_ = 0.0f;
      this->current_operation = cover::COVER_OPERATION_IDLE;
      return;
    }
    if (msg.substr(11, 7) == "Stopped") {
      this->target_position_ = 0.0f;
      this->current_operation = cover::COVER_OPERATION_IDLE;
      return;
    }
  }
}


////////////////////////////////////////////
// Cover component logic functions
////////////////////////////////////////////
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
  if (this->current_operation == dir ) {
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

////////////////////////////////////////////
// UART operations
////////////////////////////////////////////

void GatePro::read_uart() {
    int available = this->available();
    if (!available) {
        return;
    }
    
    uint8_t* bytes = new byte[available];
    this->read_array(bytes, available);
    this->preprocess(this->convert(bytes, available));
}

void GatePro::write_uart() {
  if (this->tx_queue.size()) {
    this->write_str(this->tx_queue.front());
    //ESP_LOGD(TAG, "UART TX: %s", this->tx_queue.front());
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
    this->queue_gatepro_cmd(GATEPRO_CMD_READ_STATUS);
    this->blocker = false;
    this->target_position_ = 0.0f;
}

void GatePro::update() {
    this->publish();
    this->stop_at_target_position();
    
    this->write_uart();

    if (this->current_operation != cover::COVER_OPERATION_IDLE) {
        this->queue_gatepro_cmd(GATEPRO_CMD_READ_STATUS);
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