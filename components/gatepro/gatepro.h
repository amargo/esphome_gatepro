#pragma once

#include <map>
#include <vector>
#include "esphome.h"
#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/cover/cover.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/button/button.h"
#include "esphome/components/number/number.h"
#include "esphome/components/switch/switch.h"

namespace esphome {
namespace gatepro {

enum GateProCmd : uint8_t {
   GATEPRO_CMD_OPEN,
   GATEPRO_CMD_CLOSE,
   GATEPRO_CMD_STOP,
   GATEPRO_CMD_READ_STATUS,
   GATEPRO_CMD_READ_PARAMS,
   GATEPRO_CMD_WRITE_PARAMS,
   GATEPRO_CMD_LEARN,
   GATEPRO_CMD_DEVINFO,
   GATEPRO_CMD_READ_LEARN_STATUS,
   GATEPRO_CMD_REMOTE_LEARN,
   GATEPRO_CMD_CLEAR_REMOTE_LEARN, // untested
   GATEPRO_CMD_RESTORE, // untested
   GATEPRO_CMD_PED_OPEN, // untested
   GATEPRO_CMD_READ_FUNCTION, // untested
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


// Command templates - will be formatted with source parameter
const std::map<GateProCmd, const char*> GateProCmdTemplates = {
   {GATEPRO_CMD_OPEN, "FULL OPEN;src=%s"},
   {GATEPRO_CMD_CLOSE, "FULL CLOSE;src=%s"},
   {GATEPRO_CMD_STOP, "STOP;src=%s"},
   {GATEPRO_CMD_READ_STATUS, "RS;src=%s"},
   {GATEPRO_CMD_READ_PARAMS, "RP,1:;src=%s"},
   {GATEPRO_CMD_WRITE_PARAMS, "WP,1:"},  // No source needed for write params
   {GATEPRO_CMD_LEARN, "AUTO LEARN;src=%s"},
   {GATEPRO_CMD_DEVINFO, "READ DEVINFO;src=%s"},
   {GATEPRO_CMD_READ_LEARN_STATUS, "READ LEARN STATUS;src=%s"},
   {GATEPRO_CMD_REMOTE_LEARN, "REMOTE LEARN;src=%s"},
   {GATEPRO_CMD_CLEAR_REMOTE_LEARN, "CLEAR REMOTE LEARN;src=%s"},
   {GATEPRO_CMD_RESTORE, "RESTORE;src=%s"},
   {GATEPRO_CMD_PED_OPEN, "PED OPEN;src=%s"},
   {GATEPRO_CMD_READ_FUNCTION, "READ FUNCTION;src=%s"},
};

class GatePro : public cover::Cover, public PollingComponent, public uart::UARTDevice {
 public:
      // Basic operation button components
      button::Button *btn_open{nullptr};
      void set_btn_open(button::Button *btn) { btn_open = btn; }
      button::Button *btn_close{nullptr};
      void set_btn_close(button::Button *btn) { btn_close = btn; }
      button::Button *btn_stop{nullptr};
      void set_btn_stop(button::Button *btn) { btn_stop = btn; }
      
      // Switch components
      switch_::Switch *sw_permalock{nullptr};
      void set_sw_permalock(switch_::Switch *sw) { sw_permalock = sw; }
      switch_::Switch *sw_infra1{nullptr};
      void set_sw_infra1(switch_::Switch *sw) { sw_infra1 = sw; }
      switch_::Switch *sw_infra2{nullptr};
      void set_sw_infra2(switch_::Switch *sw) { sw_infra2 = sw; }
      
      // Button components
      esphome::button::Button *btn_learn;
      void set_btn_learn(esphome::button::Button *btn) { btn_learn = btn; }
      esphome::button::Button *btn_params_od;
      void set_btn_params_od(esphome::button::Button *btn) { btn_params_od = btn; }
      esphome::button::Button *btn_remote_learn;
      void set_btn_remote_learn(esphome::button::Button *btn) { btn_remote_learn = btn; }
      
      // Text sensor components
      text_sensor::TextSensor *txt_devinfo{nullptr};
      void set_txt_devinfo(esphome::text_sensor::TextSensor *txt) { txt_devinfo = txt; }
      text_sensor::TextSensor *txt_learn_status{nullptr};
      void set_txt_learn_status(esphome::text_sensor::TextSensor *txt) { txt_learn_status = txt; }

      // Number slider components
      number::Number *speed_slider{nullptr};
      void set_speed_slider(number::Number *slider) { speed_slider = slider; }
      number::Number *decel_dist_slider{nullptr};
      void set_decel_dist_slider(number::Number *slider) { decel_dist_slider = slider; }
      number::Number *decel_speed_slider{nullptr};
      void set_decel_speed_slider(number::Number *slider) { decel_speed_slider = slider; }
      number::Number *max_amp_slider{nullptr};
      void set_max_amp_slider(number::Number *slider) { max_amp_slider = slider; }
      number::Number *auto_close_slider{nullptr};
      void set_auto_close_slider(number::Number *slider) { auto_close_slider = slider; }

      // Parameter logic
      void set_param(int idx, int val);

  void setup() override;
  void update() override;
  void loop() override;
  void dump_config() override;
  cover::CoverTraits get_traits() override;
  
  // Set the source parameter for commands
  void set_source(const std::string &source) { this->source_ = source; }
  
  // Get formatted command string with source parameter
  std::string get_command_string(GateProCmd cmd);

 protected:
      // Parameter logic
      std::vector<int> params;
      void parse_params(std::string msg);
      bool param_no_pub = false;
      void publish_params();
      void write_params();
      std::queue<std::function<void()>> paramTaskQueue;
      std::string devinfo = "N/A";

  void update_state_from_position(float position);
  void log_state_change(GateProState old_state, GateProState new_state);

  // abstract (cover) logic
  void control(const cover::CoverCall &call) override;
  void start_direction_(cover::CoverOperation dir);

  // device logic
  std::string convert(uint8_t*, size_t);
  void process();
  void queue_gatepro_cmd(GateProCmd cmd);
  void read_uart();
  void write_uart();
  void debug();
  std::queue<std::string> tx_queue;
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
  const std::string tx_delimiter = "\r\n";
  static const size_t MAX_UART_BUFFER_SIZE = 512;  // Maximum buffer size to prevent memory issues
  static const size_t UART_READ_BUFFER_SIZE = 256; // Stack buffer size for reading
  static const size_t MAX_QUEUE_SIZE = 10;         // Maximum queue size to prevent memory issues

  const int known_percentage_offset = 128;
  const float acceptable_diff = 0.05f;
  float target_position_;
  float position_;
  bool operation_finished;
  cover::CoverCall* last_call_;

  GateProState gate_state_{STATE_UNKNOWN};
  uint32_t last_state_change_{0};
  bool force_state_update_{false};
  uint8_t consecutive_position_readings_{0};
  float last_position_reading_{-1.0f};
  
  // Pattern detection variables
  std::string last_pattern_seen_{""};
  uint8_t consecutive_pattern_readings_{0};
  
  // UART message buffer for stable message processing
  std::string msg_buff{""};
  
  // Source parameter for commands (default value as fallback)
  std::string source_{"P00287D7"};
};

}  // namespace gatepro
}  // namespace esphome