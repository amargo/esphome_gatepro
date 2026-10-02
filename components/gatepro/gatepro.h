#pragma once

#include <deque>
#include <functional>
#include <map>
#include <queue>
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
#include "gatepro_protocol.h"

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
      esphome::button::Button *btn_learn{nullptr};
      void set_btn_learn(esphome::button::Button *btn) { btn_learn = btn; }
      esphome::button::Button *btn_params_od{nullptr};
      void set_btn_params_od(esphome::button::Button *btn) { btn_params_od = btn; }
      esphome::button::Button *btn_remote_learn{nullptr};
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
      number::Number *small_gate_timer{nullptr};
      void set_small_gate_timer(number::Number *slider) { small_gate_timer = slider; }
      number::Number *force_detection_number{nullptr};
      void set_force_detection_number(number::Number *num) { force_detection_number = num; }

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
      void parse_params(const std::string &msg);
      bool param_no_pub = false;
      void publish_params();
      void write_params();
      std::queue<std::function<void()>> paramTaskQueue;
  void log_state_change(GateProState old_state, GateProState new_state);

  // abstract (cover) logic
  void control(const cover::CoverCall &call) override;

  // device logic
  std::string convert(uint8_t*, size_t);
  void process();
  void queue_gatepro_cmd(GateProCmd cmd);
  void enqueue_tx_(const std::string &cmd, bool priority = false);
  void read_uart();
  void write_uart();
  std::deque<std::string> tx_queue;
  std::queue<std::string> rx_queue;
  
  // sensor logic
  void correction_after_operation();
  cover::CoverOperation last_operation_{cover::COVER_OPERATION_OPENING};
  void publish();
  static const uint8_t PUBLISH_AFTER_TICKS = 10;  // extra publishes after a change
  uint8_t publish_ticks_left_{PUBLISH_AFTER_TICKS};
  void stop_at_target_position();

  // UART parser constants
  const std::string delimiter = "\\r\\n";
  const uint8_t delimiter_length = delimiter.length();
  const std::string tx_delimiter = "\r\n";
  static const size_t MAX_UART_BUFFER_SIZE = 512;  // Maximum buffer size to prevent memory issues
  static const size_t UART_READ_BUFFER_SIZE = 256; // Stack buffer size for reading
  static const size_t MAX_QUEUE_SIZE = 10;         // Maximum queue size to prevent memory issues
  static const size_t MAX_RX_QUEUE_SIZE = 10;      // Maximum pending RX messages
  static const size_t MAX_PARAM_TASKS = 8;         // Maximum pending parameter writes

  const float acceptable_diff = 0.05f;
  bool stop_at_target_{false};  // true only while a partial-position move is in progress
  float target_position_{0.0f};
  float position_{0.0f};
  bool operation_finished{true};

  GateProState gate_state_{STATE_UNKNOWN};
  uint32_t last_state_change_{0};
  uint32_t last_motion_cmd_ms_{0};                       // when OPEN/CLOSE/PED OPEN was last queued
  static const uint32_t STALE_STOPPED_GRACE_MS = 2000;   // ignore repeated "Stopped" this long after a motion command
  static const uint32_t MOTION_DETECT_HOLDOFF_MS = 1000; // no RS-based motion detection this soon after a state change
  bool force_state_update_{false};
  
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