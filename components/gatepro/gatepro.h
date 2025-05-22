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

// Forward declaration of the GatePro class
class GatePro;

// Command mapping will be created dynamically based on the source parameter

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
    static std::string cmd_str;
    switch (cmd) {
      case GATEPRO_CMD_OPEN:
        cmd_str = "FULL OPEN;src=" + this->source_ + "\r\n";
        break;
      case GATEPRO_CMD_CLOSE:
        cmd_str = "FULL CLOSE;src=" + this->source_ + "\r\n";
        break;
      case GATEPRO_CMD_STOP:
        cmd_str = "STOP;src=" + this->source_ + "\r\n";
        break;
      case GATEPRO_CMD_READ_STATUS:
        cmd_str = "RS;src=" + this->source_ + "\r\n";
        break;
    }
    return cmd_str.c_str();
  }

 protected:
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
  
  // Source parameter for commands (default value as fallback)
  std::string source_{"P00287D7"};
  
  // Flags to track gate states
  bool gate_closed{false};
  bool gate_open{false};
};

}  // namespace gatepro
}  // namespace esphome