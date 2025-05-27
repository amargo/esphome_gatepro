#pragma once

#include <map>
#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/cover/cover.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace gatepro {

enum GateProCmd : uint8_t {
  GATEPRO_CMD_OPEN,
  GATEPRO_CMD_CLOSE,
  GATEPRO_CMD_STOP,
  GATEPRO_CMD_READ_STATUS,
};

enum GateProState : uint8_t {
  STATE_UNKNOWN,
  STATE_OPENING,
  STATE_OPEN,
  STATE_CLOSING,
  STATE_CLOSED,
  STATE_STOPPED
};

// Forward declaration of the GatePro class
class GatePro;


class GatePro : public cover::Cover, public PollingComponent, public uart::UARTDevice {
 public:
  void setup() override;
  void update() override;
  void loop() override;
  void dump_config() override;
  cover::CoverTraits get_traits() override;
  
  // Set the source parameter for commands
  void set_source(const std::string &source) { this->source_ = source; }
  
  // Generate command strings based on the source parameter
  const char* get_command_string(GateProCmd cmd) {
    // Use a static array of strings to ensure memory remains valid
    static char open_cmd[50];
    static char close_cmd[50];
    static char stop_cmd[50];
    static char rs_cmd[50];
    
    // Format the commands with the source parameter
    switch (cmd) {
      case GATEPRO_CMD_OPEN:
        snprintf(open_cmd, sizeof(open_cmd), "FULL OPEN;src=%s\r\n", this->source_.c_str());
        ESP_LOGD("gatepro", "Generated OPEN command: %s", open_cmd);
        return open_cmd;
      
      case GATEPRO_CMD_CLOSE:
        snprintf(close_cmd, sizeof(close_cmd), "FULL CLOSE;src=%s\r\n", this->source_.c_str());
        ESP_LOGD("gatepro", "Generated CLOSE command: %s", close_cmd);
        return close_cmd;
      
      case GATEPRO_CMD_STOP:
        snprintf(stop_cmd, sizeof(stop_cmd), "STOP;src=%s\r\n", this->source_.c_str());
        ESP_LOGD("gatepro", "Generated STOP command: %s", stop_cmd);
        return stop_cmd;
      
      case GATEPRO_CMD_READ_STATUS:
        snprintf(rs_cmd, sizeof(rs_cmd), "RS;src=%s\r\n", this->source_.c_str());
        ESP_LOGD("gatepro", "Generated READ_STATUS command: %s", rs_cmd);
        return rs_cmd;
      
      default:
        ESP_LOGE("gatepro", "Unknown command type: %d", cmd);
        return "";
    }
  }

 protected:

  void update_state_from_position(float position);
  void log_state_change(GateProState old_state, GateProState new_state);

  // abstract (cover) logic
  void control(const cover::CoverCall &call) override;
  void start_direction_(cover::CoverOperation dir);

  // device logic
  std::string convert(uint8_t*, size_t);
  void preprocess(std::string);
  void process();
  void queue_gatepro_cmd(GateProCmd cmd);
  void read_uart();
  void write_uart();
  void debug();
  std::queue<const char*> tx_queue;
  std::queue<std::string> rx_queue;
  bool blocker;
  
  // sensor logic
  void correction_after_operation();
  cover::CoverOperation last_operation_{cover::COVER_OPERATION_OPENING};
  void publish();
  void stop_at_target_position();

  // UART parser constants
  const std::string delimiter = "\\r\\n";
  const uint8_t delimiter_length = delimiter.length();

  const int known_percentage_offset = 128;
  const float acceptable_diff = 0.05f;
  float target_position_;
  float position_;
  bool operation_finished;
  cover::CoverCall* last_call_;

  GateProState gate_state_{STATE_UNKNOWN};
  uint32_t last_status_request_{0};
  uint32_t last_state_change_{0};
  bool force_state_update_{false};
  uint8_t consecutive_position_readings_{0};
  float last_position_reading_{-1.0f};
  
  // Pattern detection variables
  std::string last_pattern_seen_{""};
  uint8_t consecutive_pattern_readings_{0};
  
  // Source parameter for commands (default value as fallback)
  std::string source_{"P00287D7"};
};

}  // namespace gatepro
}  // namespace esphome