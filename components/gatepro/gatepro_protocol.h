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
