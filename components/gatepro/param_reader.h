#pragma once

#include "esphome.h"

namespace esphome {
namespace gatepro {

class GateProParamReader : public Component, public uart::UARTDevice {
 public:
  GateProParamReader(uart::UARTComponent *parent) : uart::UARTDevice(parent) {}

  // TextSensors for each parameter group
  text_sensor::TextSensor *auto_close{nullptr};            // group 2
  text_sensor::TextSensor *op_speed{nullptr};              // group 4
  text_sensor::TextSensor *decel_start{nullptr};           // group 5
  text_sensor::TextSensor *decel_speed{nullptr};           // group 6
  text_sensor::TextSensor *torque_sense{nullptr};          // group 7
  text_sensor::TextSensor *pedestrian{nullptr};            // group 8
  text_sensor::TextSensor *lamp_flash{nullptr};            // group 9
  text_sensor::TextSensor *torque_reaction{nullptr};       // group A
  text_sensor::TextSensor *full_open_button{nullptr};      // group C
  text_sensor::TextSensor *ped_button{nullptr};            // group E
  text_sensor::TextSensor *infra_beam{nullptr};            // group H
  text_sensor::TextSensor *operation_mode{nullptr};        // group P

  void setup() override {
    // Initialize text sensors with default values
    if (auto_close != nullptr)
      auto_close->publish_state("Unknown");
    if (op_speed != nullptr)
      op_speed->publish_state("Unknown");
    if (decel_start != nullptr)
      decel_start->publish_state("Unknown");
    if (decel_speed != nullptr)
      decel_speed->publish_state("Unknown");
    if (torque_sense != nullptr)
      torque_sense->publish_state("Unknown");
    if (pedestrian != nullptr)
      pedestrian->publish_state("Unknown");
    if (lamp_flash != nullptr)
      lamp_flash->publish_state("Unknown");
    if (torque_reaction != nullptr)
      torque_reaction->publish_state("Unknown");
    if (full_open_button != nullptr)
      full_open_button->publish_state("Unknown");
    if (ped_button != nullptr)
      ped_button->publish_state("Unknown");
    if (infra_beam != nullptr)
      infra_beam->publish_state("Unknown");
    if (operation_mode != nullptr)
      operation_mode->publish_state("Unknown");
  }

  void loop() override {
    // Read full line when available
    while (available()) {
      int c = read();
      if (c == '\n') {
        std::string line(this->buffer_);
        this->buffer_.clear();
        parse_line(line);
      } else if (c >= 0) {
        this->buffer_ += static_cast<char>(c);
      }
    }
  }

  // Method to request parameter values
  void request_param(const std::string &group, const std::string &gate_src) {
    std::string command = "RP," + group + ":;" + gate_src;
    write_str(command);
    ESP_LOGD("gatepro", "Requesting parameter group %s", group.c_str());
  }

  // Method to request all parameters
  void request_all_params(const std::string &gate_src) {
    // Request each parameter group
    request_param("2", gate_src);  // Auto-close
    request_param("4", gate_src);  // Operating speed
    request_param("5", gate_src);  // Deceleration start
    request_param("6", gate_src);  // Deceleration speed
    request_param("7", gate_src);  // Torque sensing
    request_param("8", gate_src);  // Pedestrian
    request_param("9", gate_src);  // Lamp flash
    request_param("A", gate_src);  // Torque reaction
    request_param("C", gate_src);  // Full open button
    request_param("E", gate_src);  // Pedestrian button
    request_param("H", gate_src);  // Infra beam
    request_param("P", gate_src);  // Operation mode
  }

 protected:
  std::string buffer_;

  void parse_line(const std::string &line) {
    ESP_LOGD("gatepro", "Parsing line: %s", line.c_str());
    
    // Match response format: ACK RP,X:Y;
    size_t pos = line.find("ACK RP,");
    if (pos != std::string::npos) {
      // Extract group and value
      if (line.length() >= pos + 9) {
        char group = line[pos + 7];
        
        // Find the value between : and ;
        size_t value_start = line.find(':', pos + 8);
        size_t value_end = line.find(';', value_start + 1);
        
        if (value_start != std::string::npos && value_end != std::string::npos) {
          std::string value = line.substr(value_start + 1, value_end - value_start - 1);
          
          // Update the appropriate text sensor
          update_sensor_value(group, value);
        }
      }
    }
  }

  void update_sensor_value(char group, const std::string &value) {
    ESP_LOGD("gatepro", "Updating group %c with value %s", group, value.c_str());
    
    // Map parameter values to human-readable descriptions
    std::string display_value = value;
    
    // Add parameter name to the value for better readability
    switch (group) {
      case '2':
        if (auto_close != nullptr) {
          if (value == "0") display_value = "OFF (2:0)";
          else if (value == "1") display_value = "5s (2:1)";
          else if (value == "2") display_value = "15s (2:2)";
          else if (value == "3") display_value = "30s (2:3)";
          else if (value == "4") display_value = "45s (2:4)";
          else if (value == "5") display_value = "60s (2:5)";
          else if (value == "6") display_value = "80s (2:6)";
          else if (value == "7") display_value = "120s (2:7)";
          else if (value == "8") display_value = "180s (2:8)";
          auto_close->publish_state(display_value);
        }
        break;
      case '4':
        if (op_speed != nullptr) {
          if (value == "1") display_value = "50% (4:1)";
          else if (value == "2") display_value = "70% (4:2)";
          else if (value == "3") display_value = "85% (4:3)";
          else if (value == "4") display_value = "100% (4:4)";
          op_speed->publish_state(display_value);
        }
        break;
      case '5':
        if (decel_start != nullptr) {
          if (value == "1") display_value = "75% (5:1)";
          else if (value == "2") display_value = "80% (5:2)";
          else if (value == "3") display_value = "85% (5:3)";
          else if (value == "4") display_value = "90% (5:4)";
          else if (value == "5") display_value = "95% (5:5)";
          decel_start->publish_state(display_value);
        }
        break;
      case '6':
        if (decel_speed != nullptr) {
          if (value == "1") display_value = "80% (6:1)";
          else if (value == "2") display_value = "60% (6:2)";
          else if (value == "3") display_value = "40% (6:3)";
          else if (value == "4") display_value = "25% (6:4)";
          decel_speed->publish_state(display_value);
        }
        break;
      case '7':
        if (torque_sense != nullptr) {
          if (value == "1") display_value = "2A (7:1)";
          else if (value == "2") display_value = "3A (7:2)";
          else if (value == "3") display_value = "4A (7:3)";
          else if (value == "4") display_value = "5A (7:4)";
          else if (value == "5") display_value = "6A (7:5)";
          else if (value == "6") display_value = "7A (7:6)";
          else if (value == "7") display_value = "8A (7:7)";
          else if (value == "8") display_value = "9A (7:8)";
          else if (value == "9") display_value = "10A (7:9)";
          else if (value == "A") display_value = "11A (7:A)";
          else if (value == "C") display_value = "12A (7:C)";
          else if (value == "E") display_value = "13A (7:E)";
          torque_sense->publish_state(display_value);
        }
        break;
      case '8':
        if (pedestrian != nullptr) {
          if (value == "1") display_value = "3s (8:1)";
          else if (value == "2") display_value = "6s (8:2)";
          else if (value == "3") display_value = "9s (8:3)";
          else if (value == "4") display_value = "12s (8:4)";
          else if (value == "5") display_value = "15s (8:5)";
          else if (value == "6") display_value = "18s (8:6)";
          pedestrian->publish_state(display_value);
        }
        break;
      case '9':
        if (lamp_flash != nullptr) {
          lamp_flash->publish_state(value + " (9:" + value + ")");
        }
        break;
      case 'A':
        if (torque_reaction != nullptr) {
          if (value == "0") display_value = "Stop (A:0)";
          else if (value == "1") display_value = "Stop+Rev 1s (A:1)";
          else if (value == "2") display_value = "Stop+Rev 3s (A:2)";
          else if (value == "3") display_value = "Stop+Rev to End (A:3)";
          torque_reaction->publish_state(display_value);
        }
        break;
      case 'C':
        if (full_open_button != nullptr) {
          full_open_button->publish_state(value + " (C:" + value + ")");
        }
        break;
      case 'E':
        if (ped_button != nullptr) {
          ped_button->publish_state(value + " (E:" + value + ")");
        }
        break;
      case 'H':
        if (infra_beam != nullptr) {
          infra_beam->publish_state(value + " (H:" + value + ")");
        }
        break;
      case 'P':
        if (operation_mode != nullptr) {
          operation_mode->publish_state(value + " (P:" + value + ")");
        }
        break;
      default:
        ESP_LOGW("gatepro", "Unknown parameter group: %c", group);
        break;
    }
  }
};

}  // namespace gatepro
}  // namespace esphome
