#pragma once

#include "esphome/core/component.h"
#include "esphome/core/entity_base.h"
#include "esphome/core/helpers.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/text_sensor/text_sensor.h"

namespace esphome {
namespace gatepro {

class GateProParamReader : public Component, public uart::UARTDevice {
 public:
  GateProParamReader() = default;
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
  
  // Set UART device
  void set_uart(uart::UARTComponent *uart) {
    this->set_uart_parent(uart);
    ESP_LOGI("gatepro", "UART parent set explicitly");
  }
  
  // Setter methods for text sensors
  void set_auto_close(text_sensor::TextSensor *sens) { this->auto_close = sens; }
  void set_op_speed(text_sensor::TextSensor *sens) { this->op_speed = sens; }
  void set_decel_start(text_sensor::TextSensor *sens) { this->decel_start = sens; }
  void set_decel_speed(text_sensor::TextSensor *sens) { this->decel_speed = sens; }
  void set_torque_sense(text_sensor::TextSensor *sens) { this->torque_sense = sens; }
  void set_pedestrian(text_sensor::TextSensor *sens) { this->pedestrian = sens; }
  void set_lamp_flash(text_sensor::TextSensor *sens) { this->lamp_flash = sens; }
  void set_torque_reaction(text_sensor::TextSensor *sens) { this->torque_reaction = sens; }
  void set_full_open_button(text_sensor::TextSensor *sens) { this->full_open_button = sens; }
  void set_ped_button(text_sensor::TextSensor *sens) { this->ped_button = sens; }
  void set_infra_beam(text_sensor::TextSensor *sens) { this->infra_beam = sens; }
  void set_operation_mode(text_sensor::TextSensor *sens) { this->operation_mode = sens; }

  void setup() override {
    ESP_LOGCONFIG("gatepro", "Setting up GatePro Parameter Reader...");
    
    // Initialize text sensors with default values
    if (auto_close != nullptr) {
      auto_close->publish_state("Unknown");
      ESP_LOGI("gatepro", "Auto Close sensor initialized");
    }
    if (op_speed != nullptr) {
      op_speed->publish_state("Unknown");
      ESP_LOGI("gatepro", "Op Speed sensor initialized");
    }
    if (decel_start != nullptr) {
      decel_start->publish_state("Unknown");
      ESP_LOGI("gatepro", "Decel Start sensor initialized");
    }
    if (decel_speed != nullptr) {
      decel_speed->publish_state("Unknown");
      ESP_LOGI("gatepro", "Decel Speed sensor initialized");
    }
    if (torque_sense != nullptr) {
      torque_sense->publish_state("Unknown");
      ESP_LOGI("gatepro", "Torque Sense sensor initialized");
    }
    if (pedestrian != nullptr) {
      pedestrian->publish_state("Unknown");
      ESP_LOGI("gatepro", "Pedestrian sensor initialized");
    }
    if (lamp_flash != nullptr) {
      lamp_flash->publish_state("Unknown");
      ESP_LOGI("gatepro", "Lamp Flash sensor initialized");
    }
    if (torque_reaction != nullptr) {
      torque_reaction->publish_state("Unknown");
      ESP_LOGI("gatepro", "Torque Reaction sensor initialized");
    }
    if (full_open_button != nullptr) {
      full_open_button->publish_state("Unknown");
      ESP_LOGI("gatepro", "Full Open Button sensor initialized");
    }
    if (ped_button != nullptr) {
      ped_button->publish_state("Unknown");
      ESP_LOGI("gatepro", "Ped Button sensor initialized");
    }
    if (infra_beam != nullptr) {
      infra_beam->publish_state("Unknown");
      ESP_LOGI("gatepro", "Infra Beam sensor initialized");
    }
    if (operation_mode != nullptr) {
      operation_mode->publish_state("Unknown");
      ESP_LOGI("gatepro", "Operation Mode sensor initialized");
    }
  }
  
  void dump_config() override {
    ESP_LOGCONFIG("gatepro", "GatePro Parameter Reader:");
    ESP_LOGCONFIG("gatepro", "  UART Communication Set Up");
    
    if (auto_close != nullptr) {
      ESP_LOGCONFIG("gatepro", "  Auto Close: '%s'", auto_close->get_name().c_str());
    } else {
      ESP_LOGCONFIG("gatepro", "  Auto Close: NOT CONFIGURED");
    }
    
    if (op_speed != nullptr) {
      ESP_LOGCONFIG("gatepro", "  Operating Speed: '%s'", op_speed->get_name().c_str());
    } else {
      ESP_LOGCONFIG("gatepro", "  Operating Speed: NOT CONFIGURED");
    }
    
    if (decel_start != nullptr) {
      ESP_LOGCONFIG("gatepro", "  Deceleration Start: '%s'", decel_start->get_name().c_str());
    } else {
      ESP_LOGCONFIG("gatepro", "  Deceleration Start: NOT CONFIGURED");
    }
    
    if (decel_speed != nullptr) {
      ESP_LOGCONFIG("gatepro", "  Deceleration Speed: '%s'", decel_speed->get_name().c_str());
    } else {
      ESP_LOGCONFIG("gatepro", "  Deceleration Speed: NOT CONFIGURED");
    }
    
    if (torque_sense != nullptr) {
      ESP_LOGCONFIG("gatepro", "  Torque Sensing: '%s'", torque_sense->get_name().c_str());
    } else {
      ESP_LOGCONFIG("gatepro", "  Torque Sensing: NOT CONFIGURED");
    }
    
    if (pedestrian != nullptr) {
      ESP_LOGCONFIG("gatepro", "  Pedestrian: '%s'", pedestrian->get_name().c_str());
    } else {
      ESP_LOGCONFIG("gatepro", "  Pedestrian: NOT CONFIGURED");
    }
    
    if (infra_beam != nullptr) {
      ESP_LOGCONFIG("gatepro", "  Infrared Beam: '%s'", infra_beam->get_name().c_str());
    } else {
      ESP_LOGCONFIG("gatepro", "  Infrared Beam: NOT CONFIGURED");
    }
    
    ESP_LOGCONFIG("gatepro", "  Use the 'Read All Parameters' button to update all values at once");
  }

  void loop() override {
    // Process any available UART data
    while (available()) {
      uint8_t c = read();
      
      // Handle line endings
      if (c == '\n' || c == ';') {
        // Process the line if we have data
        if (!buffer_.empty()) {
          ESP_LOGI("gatepro", "Received line: %s", buffer_.c_str());
          parse_line(buffer_);
          buffer_.clear();
        }
      } else if (c != '\r') {
        // Add character to buffer
        buffer_ += (char)c;
        
        // Safety check for buffer size
        if (buffer_.length() > 100) {
          ESP_LOGW("gatepro", "Buffer overflow, clearing: %s", buffer_.c_str());
          buffer_.clear();
        }
      }
    }
  }

  // Method to request parameter values
  void request_param(const std::string &group, const std::string &gate_src) {
    std::string command = "RP," + group + ":;" + gate_src;
    write_str(command.c_str());
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
    ESP_LOGI("gatepro", "Parsing line: %s", line.c_str());
    
    // Check for different response formats
    if (line.find("ACK") == std::string::npos) {
      ESP_LOGW("gatepro", "Not a valid ACK response");
      return;
    }
    
    // Match full parameter array response: ACK RP,1:0,3,0,2,2,2,4,2,0,3,0,2,3,1,0,0,0
    // Also check for alternative formats like: ACK RP,1:0,3,0,2,2,2,4,2,0,3,0,2,3,1,0,0,0;src=ESPHOME
    size_t pos = line.find("ACK RP,1");
    if (pos != std::string::npos) {
      ESP_LOGI("gatepro", "Found parameter array response");
      // Extract the 17-element array
      size_t value_start = line.find(':', pos + 7);
      
      // Handle different response formats
      size_t value_end;
      if (line.find(';', value_start + 1) != std::string::npos) {
        value_end = line.find(';', value_start + 1);
      } else {
        // If no semicolon, use the end of the string
        value_end = line.length();
      }
      
      if (value_start != std::string::npos && value_start < value_end) {
        std::string values = line.substr(value_start + 1, value_end - value_start - 1);
        ESP_LOGI("gatepro", "Parsing parameter array: %s", values.c_str());
        
        // Split the values by comma
        std::vector<std::string> params;
        size_t start = 0;
        size_t end = values.find(',');
        
        while (end != std::string::npos) {
          params.push_back(values.substr(start, end - start));
          start = end + 1;
          end = values.find(',', start);
        }
        
        // Add the last parameter
        params.push_back(values.substr(start));
        
        // Log the parsed parameters
        ESP_LOGI("gatepro", "Parsed %d parameters", params.size());
        for (size_t i = 0; i < params.size(); i++) {
          ESP_LOGI("gatepro", "Param[%d] = %s", i, params[i].c_str());
        }
        
        // Process each parameter based on its position
        if (params.size() >= 17) {
          ESP_LOGI("gatepro", "Processing parameters array with %d elements", params.size());
          
          // Helper function to convert string to int safely
          auto safe_string_to_int = [](const std::string &str) -> int {
            int result = 0;
            // Check if the string contains only digits
            for (char c : str) {
              if (c < '0' || c > '9') {
                ESP_LOGI("gatepro", "Invalid digit in string: '%c' in '%s'", c, str.c_str());
                return 0; // Return 0 for invalid input
              }
              result = result * 10 + (c - '0');
            }
            ESP_LOGI("gatepro", "Converted '%s' to %d", str.c_str(), result);
            return result;
          };
          
          // Group 2 - Auto close (base=0)
          if (auto_close != nullptr) {
            int raw_index = safe_string_to_int(params[1]);
            update_auto_close(raw_index);
          }
          
          // Group 4 - Operating speed (base=1)
          if (op_speed != nullptr) {
            int raw_index = safe_string_to_int(params[3]);
            update_op_speed(raw_index);
          }
          
          // Group 5 - Deceleration start (base=1)
          if (decel_start != nullptr) {
            int raw_index = safe_string_to_int(params[4]);
            update_decel_start(raw_index);
          }
          
          // Group 6 - Deceleration speed (base=1)
          if (decel_speed != nullptr) {
            int raw_index = safe_string_to_int(params[5]);
            update_decel_speed(raw_index);
          }
          
          // Group 7 - Torque sensing (base=1)
          if (torque_sense != nullptr) {
            int raw_index = safe_string_to_int(params[6]);
            update_torque_sense(raw_index);
          }
          
          // Group 8 - Pedestrian (base=1)
          if (pedestrian != nullptr) {
            int raw_index = safe_string_to_int(params[7]);
            update_pedestrian(raw_index);
          }
          
          // Group A - Torque reaction (base=0)
          if (torque_reaction != nullptr) {
            int raw_index = safe_string_to_int(params[9]);
            update_torque_reaction(raw_index);
          }
          
          // Group H - Infrared beam (base=0)
          if (infra_beam != nullptr) {
            int raw_index = safe_string_to_int(params[13]);
            update_infra_beam(raw_index);
          }
        }
      }
    }
    
    // Also handle individual parameter responses (for backward compatibility)
    pos = line.find("ACK RP,");
    if (pos != std::string::npos) {
      // Extract group and value
      if (line.length() >= pos + 9) {
        char group = line[pos + 7];
        
        // Skip if it's the full array response we already processed
        if (group != '1') {
          // Find the value between : and ;
          size_t value_start = line.find(':', pos + 8);
          size_t value_end = line.find(';', value_start + 1);
          
          if (value_start != std::string::npos && value_end != std::string::npos) {
            std::string value = line.substr(value_start + 1, value_end - value_start - 1);
            
            // Update the appropriate text sensor directly
            update_sensor_value(group, value);
          }
        }
      }
    }
  }

  // Update methods for each parameter type based on raw index and base value
  void update_auto_close(int raw_index) {
    // Group 2 - Auto close (base=0)
    if (auto_close == nullptr) return;
    
    int value = 0 + raw_index; // base=0
    std::string display_value;
    
    switch (value) {
      case 0: display_value = "OFF (2:0)"; break;
      case 1: display_value = "5s (2:1)"; break;
      case 2: display_value = "15s (2:2)"; break;
      case 3: display_value = "30s (2:3)"; break;
      case 4: display_value = "45s (2:4)"; break;
      case 5: display_value = "60s (2:5)"; break;
      case 6: display_value = "80s (2:6)"; break;
      case 7: display_value = "120s (2:7)"; break;
      case 8: display_value = "180s (2:8)"; break;
      default: display_value = "Unknown (2:" + std::to_string(value) + ")"; break;
    }
    
    ESP_LOGD("gatepro", "Auto-close: raw=%d, value=%d, display=%s", raw_index, value, display_value.c_str());
    auto_close->publish_state(display_value);
  }
  
  void update_op_speed(int raw_index) {
    // Group 4 - Operating speed (base=1)
    if (op_speed == nullptr) return;
    
    int value = 1 + raw_index; // base=1
    std::string display_value;
    
    switch (value) {
      case 1: display_value = "50% (4:1)"; break;
      case 2: display_value = "70% (4:2)"; break;
      case 3: display_value = "85% (4:3)"; break;
      case 4: display_value = "100% (4:4)"; break;
      default: display_value = "Unknown (4:" + std::to_string(value) + ")"; break;
    }
    
    ESP_LOGD("gatepro", "Operating speed: raw=%d, value=%d, display=%s", raw_index, value, display_value.c_str());
    op_speed->publish_state(display_value);
  }
  
  void update_decel_start(int raw_index) {
    // Group 5 - Deceleration start (base=1)
    if (decel_start == nullptr) return;
    
    int value = 1 + raw_index; // base=1
    std::string display_value;
    
    switch (value) {
      case 1: display_value = "75% (5:1)"; break;
      case 2: display_value = "80% (5:2)"; break;
      case 3: display_value = "85% (5:3)"; break;
      case 4: display_value = "90% (5:4)"; break;
      case 5: display_value = "95% (5:5)"; break;
      default: display_value = "Unknown (5:" + std::to_string(value) + ")"; break;
    }
    
    ESP_LOGD("gatepro", "Deceleration start: raw=%d, value=%d, display=%s", raw_index, value, display_value.c_str());
    decel_start->publish_state(display_value);
  }
  
  void update_decel_speed(int raw_index) {
    // Group 6 - Deceleration speed (base=1)
    if (decel_speed == nullptr) return;
    
    int value = 1 + raw_index; // base=1
    std::string display_value;
    
    switch (value) {
      case 1: display_value = "80% (6:1)"; break;
      case 2: display_value = "60% (6:2)"; break;
      case 3: display_value = "40% (6:3)"; break;
      case 4: display_value = "25% (6:4)"; break;
      default: display_value = "Unknown (6:" + std::to_string(value) + ")"; break;
    }
    
    ESP_LOGD("gatepro", "Deceleration speed: raw=%d, value=%d, display=%s", raw_index, value, display_value.c_str());
    decel_speed->publish_state(display_value);
  }
  
  void update_torque_sense(int raw_index) {
    // Group 7 - Torque sensing (base=1)
    if (torque_sense == nullptr) return;
    
    int value = 1 + raw_index; // base=1
    std::string display_value;
    
    switch (value) {
      case 1: display_value = "2A (7:1)"; break;
      case 2: display_value = "3A (7:2)"; break;
      case 3: display_value = "4A (7:3)"; break;
      case 4: display_value = "5A (7:4)"; break;
      case 5: display_value = "6A (7:5)"; break;
      case 6: display_value = "7A (7:6)"; break;
      case 7: display_value = "8A (7:7)"; break;
      case 8: display_value = "9A (7:8)"; break;
      case 9: display_value = "10A (7:9)"; break;
      case 10: display_value = "11A (7:A)"; break;
      case 12: display_value = "12A (7:C)"; break;
      case 14: display_value = "13A (7:E)"; break;
      default: display_value = "Unknown (7:" + std::to_string(value) + ")"; break;
    }
    
    ESP_LOGD("gatepro", "Torque sensing: raw=%d, value=%d, display=%s", raw_index, value, display_value.c_str());
    torque_sense->publish_state(display_value);
  }
  
  void update_pedestrian(int raw_index) {
    // Group 8 - Pedestrian (base=1)
    if (pedestrian == nullptr) return;
    
    int value = 1 + raw_index; // base=1
    std::string display_value;
    
    switch (value) {
      case 1: display_value = "3s (8:1)"; break;
      case 2: display_value = "6s (8:2)"; break;
      case 3: display_value = "9s (8:3)"; break;
      case 4: display_value = "12s (8:4)"; break;
      case 5: display_value = "15s (8:5)"; break;
      case 6: display_value = "18s (8:6)"; break;
      default: display_value = "Unknown (8:" + std::to_string(value) + ")"; break;
    }
    
    ESP_LOGD("gatepro", "Pedestrian: raw=%d, value=%d, display=%s", raw_index, value, display_value.c_str());
    pedestrian->publish_state(display_value);
  }
  
  void update_torque_reaction(int raw_index) {
    // Group A - Torque reaction (base=0)
    if (torque_reaction == nullptr) return;
    
    int value = 0 + raw_index; // base=0
    std::string display_value;
    
    switch (value) {
      case 0: display_value = "Stop (A:0)"; break;
      case 1: display_value = "Stop+Rev 1s (A:1)"; break;
      case 2: display_value = "Stop+Rev 3s (A:2)"; break;
      case 3: display_value = "Stop+Rev to End (A:3)"; break;
      default: display_value = "Unknown (A:" + std::to_string(value) + ")"; break;
    }
    
    ESP_LOGD("gatepro", "Torque reaction: raw=%d, value=%d, display=%s", raw_index, value, display_value.c_str());
    torque_reaction->publish_state(display_value);
  }
  
  void update_infra_beam(int raw_index) {
    // Group H - Infrared beam (base=0)
    if (infra_beam == nullptr) return;
    
    int value = 0 + raw_index; // base=0
    std::string display_value;
    
    switch (value) {
      case 0: display_value = "Disabled (H:0)"; break;
      case 1: display_value = "Enabled (H:1)"; break;
      default: display_value = "Unknown (H:" + std::to_string(value) + ")"; break;
    }
    
    ESP_LOGD("gatepro", "Infrared beam: raw=%d, value=%d, display=%s", raw_index, value, display_value.c_str());
    infra_beam->publish_state(display_value);
  }
  
  // For backward compatibility with individual parameter responses
  void update_sensor_value(char group, const std::string &value) {
    ESP_LOGD("gatepro", "Updating group %c with direct value %s", group, value.c_str());
    
    // Convert the value string to an integer for the raw index
    int raw_index = 0;
    
    // Handle hexadecimal values (A, C, E)
    if (value.length() == 1) {
      if (value[0] == 'A') raw_index = 10;
      else if (value[0] == 'C') raw_index = 12;
      else if (value[0] == 'E') raw_index = 14;
      else if (value[0] >= '0' && value[0] <= '9') raw_index = value[0] - '0';
      else {
        ESP_LOGW("gatepro", "Failed to convert value '%s' to integer", value.c_str());
        return;
      }
    } else {
      // Check if the string contains only digits
      bool is_valid = true;
      for (char c : value) {
        if (c < '0' || c > '9') {
          is_valid = false;
          break;
        }
      }
      
      if (is_valid) {
        // Convert digit by digit
        for (char c : value) {
          raw_index = raw_index * 10 + (c - '0');
        }
      } else {
        ESP_LOGW("gatepro", "Failed to convert value '%s' to integer", value.c_str());
        return;
      }
    }
    
    // Call the appropriate update method based on the group
    switch (group) {
      case '2': update_auto_close(raw_index); break;
      case '4': update_op_speed(raw_index - 1); break;  // Adjust for base=1
      case '5': update_decel_start(raw_index - 1); break;  // Adjust for base=1
      case '6': update_decel_speed(raw_index - 1); break;  // Adjust for base=1
      case '7': update_torque_sense(raw_index - 1); break;  // Adjust for base=1
      case '8': update_pedestrian(raw_index - 1); break;  // Adjust for base=1
      case 'A': update_torque_reaction(raw_index); break;
      case 'H': update_infra_beam(raw_index); break;
      
      // For other parameters, just display the raw value
      case '9':
        if (lamp_flash != nullptr) {
          lamp_flash->publish_state("Value " + value + " (9:" + value + ")");
        }
        break;
      case 'C':
        if (full_open_button != nullptr) {
          full_open_button->publish_state("Value " + value + " (C:" + value + ")");
        }
        break;
      case 'E':
        if (ped_button != nullptr) {
          ped_button->publish_state("Value " + value + " (E:" + value + ")");
        }
        break;
      case 'P':
        if (operation_mode != nullptr) {
          operation_mode->publish_state("Value " + value + " (P:" + value + ")");
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
