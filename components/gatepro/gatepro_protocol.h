#pragma once

// Pure GatePro UART protocol helpers. No ESPHome dependency, so they can be
// unit-tested on the host (tests/test_protocol.cpp). None of these functions
// may throw: ESP builds run without exception handling.

#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

namespace esphome {
namespace gatepro {
namespace protocol {

// Messages are stored with control characters escaped (see GatePro::convert()),
// so the line terminator is the literal 4-char sequence "\\r\\n".
static const char *const RX_TERMINATOR = "\\r\\n";
static const size_t NUM_PARAMS = 17;             // GatePro has 17 parameter groups (0-16)
static const int MAX_PARAM_VALUE = 15;           // Upper bound for any parameter value
static const int KNOWN_PERCENTAGE_OFFSET = 128;  // Added by the controller to some positions
static const size_t PARAMS_OFFSET = 9;           // strlen("ACK RP,1:")
static const size_t POSITION_OFFSET = 16;        // "ACK RS:00,80,C4,<C6>,..."

// Parameter values are the 0-based index of the controller's menu option
// (BOXER 500 manual v290124, "Programozható funkciók táblázata"): menu "4-1"
// (first speed option) is sent as 0, while groups whose menu starts at 0
// ("2-0", "9-0", "A-0", "E-0", "F-0", "H-0", "J-0", "L-0") map 1:1.
// Confirmed on a device: raw [6]=4 is the factory "7-5" (6 A), raw [7]=1 the
// factory "8-2" (6 s). Highest valid value per index:
static const int PARAM_MAX_VALUES[NUM_PARAMS] = {
    1,  // 0  "1" opening direction: left, right
    8,  // 1  "2" auto close: off, 5, 15, 30, 45, 60, 80, 120, 180 s
    2,  // 2  "3" safety device mode (installer only)
    3,  // 3  "4" speed: 50, 70, 85, 100 %
    4,  // 4  "5" deceleration starts at 75, 80, 85, 90, 95 % of travel
    3,  // 5  "6" deceleration speed: 80, 60, 40, 25 %
    8,  // 6  "7" force: 2..10 A (11-13 A are BOXER 800 only and rejected)
    5,  // 7  "8" pedestrian opening: 3, 6, 9, 12, 15, 18 s
    1,  // 8  "9" warning light: on movement, 3 s before
    3,  // 9  "A" obstacle reaction: stop, reverse 1 s, reverse 3 s, reverse fully
    3,  // 10 "C" remote button for full open: A-D
    4,  // 11 "E" remote button for pedestrian open: off, A-D
    4,  // 12 "F" remote button for external output: off, A-D
    1,  // 13 "H" photocell 1
    1,  // 14 "J" photocell 2
    1,  // 15 "L" stop terminal (NC contact; blocks the gate when enabled unwired)
    1,  // 16 "P" button logic: open/stop/close/stop, open/stop/close
};

inline bool param_value_valid(size_t idx, int value) {
  return idx < NUM_PARAMS && value >= 0 && value <= PARAM_MAX_VALUES[idx];
}

inline bool starts_with(const std::string &s, const char *prefix) {
  return s.compare(0, strlen(prefix), prefix) == 0;
}

// Bounds-checked field comparison: never throws, unlike substr().
inline bool field_equals(const std::string &s, size_t pos, const char *value) {
  const size_t len = strlen(value);
  return s.size() >= pos + len && s.compare(pos, len, value) == 0;
}

// Returns the part of the message after `pos`, without the trailing terminator.
inline std::string payload_after(const std::string &s, size_t pos) {
  if (pos >= s.size()) {
    return "";
  }
  size_t end = s.size();
  const size_t term_len = strlen(RX_TERMINATOR);
  if (end - pos >= term_len && s.compare(end - term_len, term_len, RX_TERMINATOR) == 0) {
    end -= term_len;
  }
  return s.substr(pos, end - pos);
}

// Strict integer parser (rejects empty/garbage input).
inline bool parse_int(const std::string &s, int base, int &out) {
  if (s.empty()) {
    return false;
  }
  char *end = nullptr;
  long val = strtol(s.c_str(), &end, base);
  if (end == s.c_str() || *end != '\0') {
    return false;
  }
  out = (int) val;
  return true;
}

// Parses "ACK RP,1:<v0>,<v1>,...,<v16>\\r\\n". `out` is only written on success.
inline bool parse_params(const std::string &msg, std::vector<int> &out) {
  const std::string payload = payload_after(msg, PARAMS_OFFSET);
  if (payload.empty()) {
    return false;
  }
  std::vector<int> parsed;
  size_t start = 0;
  while (true) {
    size_t end = payload.find(',', start);
    std::string field = end == std::string::npos ? payload.substr(start) : payload.substr(start, end - start);
    int value;
    if (!parse_int(field, 10, value) || value < 0 || value > MAX_PARAM_VALUE || parsed.size() >= NUM_PARAMS) {
      return false;
    }
    parsed.push_back(value);
    if (end == std::string::npos) {
      break;
    }
    start = end + 1;
  }
  if (parsed.size() != NUM_PARAMS) {
    return false;
  }
  out = parsed;
  return true;
}

// Extracts the 0-100 position from an "ACK RS:..." message.
inline bool parse_position(const std::string &msg, int &percentage) {
  if (msg.size() < POSITION_OFFSET + 2) {
    return false;
  }
  int value;
  if (!parse_int(msg.substr(POSITION_OFFSET, 2), 16, value)) {
    return false;
  }
  if (value > 100) {
    value -= KNOWN_PERCENTAGE_OFFSET;
  }
  if (value < 0 || value > 100) {
    return false;
  }
  percentage = value;
  return true;
}

static const size_t MOVING_TOKEN_OFFSET = 13;  // "ACK RS:00,80,<C4>,..."

// 3rd status token "C4" means the motor is running.
inline bool status_is_moving(const std::string &msg) {
  return field_equals(msg, MOVING_TOKEN_OFFSET, "C4");
}

// 3rd status token "A2": motor idle at an end stop ("ACK RS:00,80,A2,00,40,00,...").
inline bool status_is_at_end(const std::string &msg) {
  return field_equals(msg, MOVING_TOKEN_OFFSET, "A2");
}

// 3rd status token "E6": motor stopped midway ("ACK RS:00,80,E6,34,00,01,..." = 52 %).
inline bool status_is_stopped_midway(const std::string &msg) {
  return field_equals(msg, MOVING_TOKEN_OFFSET, "E6");
}

// The controller adds KNOWN_PERCENTAGE_OFFSET to the position while opening.
inline bool status_is_opening(const std::string &msg) {
  if (msg.size() < POSITION_OFFSET + 2) {
    return false;
  }
  int raw;
  return parse_int(msg.substr(POSITION_OFFSET, 2), 16, raw) && raw > 100;
}

}  // namespace protocol
}  // namespace gatepro
}  // namespace esphome
