// Host-side unit tests for the GatePro UART protocol helpers.
// Build: g++ -std=c++17 -Wall -Wextra -Werror -fno-exceptions -I components/gatepro tests/test_protocol.cpp -o build/test_protocol
#include <cstdio>
#include <string>
#include <vector>

#include "gatepro_protocol.h"

using namespace esphome::gatepro::protocol;

static int failures = 0;

#define CHECK(cond)                                                    \
  do {                                                                 \
    if (!(cond)) {                                                     \
      std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);      \
      ++failures;                                                      \
    }                                                                  \
  } while (0)

// Messages arrive with control characters escaped by GatePro::convert(),
// so "\\r\\n" below is the literal 4-character terminator.
static const std::string RP_OK = "ACK RP,1:1,0,0,1,2,2,0,0,0,3,0,0,3,0,0,0,0\\r\\n";

static void test_starts_with() {
  CHECK(starts_with("ACK RS:00", "ACK RS"));
  CHECK(!starts_with("ACK", "ACK RS"));
  CHECK(!starts_with("", "A"));
}

static void test_field_equals() {
  const std::string ev = "$V1PKF0,17,Closed;src=0001\\r\\n";
  CHECK(field_equals(ev, 11, "Closed"));
  CHECK(!field_equals(ev, 11, "Closing"));
  CHECK(!field_equals("$V1PKF0", 11, "Closed"));  // shorter than offset: no throw
  CHECK(!field_equals("", 0, "A"));
}

static void test_payload_after() {
  CHECK(payload_after("ACK READ DEVINFO:P500BU,PS21053C,V01\\r\\n", 17) == "P500BU,PS21053C,V01");
  CHECK(payload_after("ACK READ DEVINFO:\\r\\n", 17) == "");
  CHECK(payload_after("ACK READ DEVINFO", 17) == "");  // pos past end
  CHECK(payload_after("ACK X:abc", 6) == "abc");       // no terminator
}

static void test_parse_int() {
  int v = -1;
  CHECK(parse_int("C6", 16, v) && v == 198);
  CHECK(parse_int("7", 10, v) && v == 7);
  CHECK(!parse_int("", 10, v));
  CHECK(!parse_int("zz", 16, v));
  CHECK(!parse_int("12x", 10, v));
}

static void test_parse_params_ok() {
  std::vector<int> p;
  CHECK(parse_params(RP_OK, p));
  CHECK(p.size() == NUM_PARAMS);
  if (p.size() == NUM_PARAMS) {
    CHECK(p[0] == 1);
    CHECK(p[3] == 1);
    CHECK(p[4] == 2);
    CHECK(p[9] == 3);
    CHECK(p[12] == 3);
    CHECK(p[16] == 0);
  }
}

static void test_parse_params_rejects() {
  const char *bad[] = {
      "ACK RP,1:1,0,0,1,2,2,0,0,0,3,0,0,3,0,0,0\\r\\n",      // 16 values
      "ACK RP,1:1,0,0,1,2,2,0,0,0,3,0,0,3,0,0,0,0,0\\r\\n",  // 18 values
      "ACK RP,1:1,,0,1,2,2,0,0,0,3,0,0,3,0,0,0,0\\r\\n",     // empty field
      "ACK RP,1:1,0,0,1,2,2,0,0,0,3,0,0,3,0,0,0,16\\r\\n",   // value > MAX_PARAM_VALUE
      "ACK RP,1:1,0,0,1,2,2,0,0,0,X,0,0,3,0,0,0,0\\r\\n",    // non-digit
      "ACK RP\\r\\n",                                       // no payload
      "ACK RP",                                             // shorter than offset
  };
  for (const char *msg : bad) {
    std::vector<int> p = {42};
    CHECK(!parse_params(msg, p));
    CHECK(p.size() == 1 && p[0] == 42);  // output untouched on failure
  }
}

static void test_parse_position() {
  int pct = -1;
  // index 16..17 = "C6" = 198 -> 198 - 128 = 70
  CHECK(parse_position("ACK RS:00,80,C4,C6,3E,16,FF,FF,FF\\r\\n", pct) && pct == 70);
  CHECK(parse_position("ACK RS:00,80,C4,32,3E,16,FF,FF,FF\\r\\n", pct) && pct == 50);
  CHECK(!parse_position("ACK RS:00,80,C4,FF,3E,16,FF,FF,FF\\r\\n", pct));  // 127 %
  CHECK(!parse_position("ACK RS:00,80,C4,ZZ,3E,16,FF,FF,FF\\r\\n", pct));
  CHECK(!parse_position("ACK RS:00", pct));  // too short
}

static void test_status_motion() {
  // "ACK RS:00,80,C4,C6,..." : 3rd token C4 = moving, C6 = 198 > 100 = opening
  const std::string opening = "ACK RS:00,80,C4,C6,3E,16,FF,FF,FF\\r\\n";
  const std::string closing = "ACK RS:00,80,C4,32,3E,16,FF,FF,FF\\r\\n";
  const std::string idle    = "ACK RS:00,80,A2,00,40,00,FF,FF,FF\\r\\n";  // real device, closed
  CHECK(status_is_opening(opening));
  CHECK(status_is_moving(opening));
  CHECK(!status_is_opening(closing));
  CHECK(status_is_moving(closing));
  CHECK(!status_is_opening(idle));
  CHECK(!status_is_moving(idle));
  CHECK(!status_is_moving("ACK RS:00"));   // too short: no throw
  CHECK(!status_is_opening("ACK RS:00"));
}

// Status lines captured from a real P500BU controller on 2026-10-03.
static void test_status_idle_kinds() {
  const std::string closed  = "ACK RS:00,80,A2,00,40,00,FF,FF,FF\\r\\n";
  const std::string stopped = "ACK RS:00,80,E6,34,00,01,FF,FF,FF\\r\\n";
  const std::string opening = "ACK RS:00,80,C4,B2,1D,0A,FF,FF,FF\\r\\n";
  int pct = -1;
  CHECK(status_is_at_end(closed));
  CHECK(!status_is_stopped_midway(closed));
  CHECK(parse_position(closed, pct) && pct == 0);
  CHECK(status_is_stopped_midway(stopped));
  CHECK(!status_is_at_end(stopped));
  CHECK(parse_position(stopped, pct) && pct == 52);
  CHECK(!status_is_at_end(opening) && !status_is_stopped_midway(opening));
  CHECK(status_is_opening(opening) && parse_position(opening, pct) && pct == 50);
  CHECK(!status_is_at_end("ACK RS:00"));  // too short: no throw
}

int main() {
  test_starts_with();
  test_field_equals();
  test_payload_after();
  test_parse_int();
  test_parse_params_ok();
  test_parse_params_rejects();
  test_parse_position();
  test_status_motion();
  test_status_idle_kinds();
  if (failures) {
    std::printf("%d check(s) failed\n", failures);
    return 1;
  }
  std::printf("All protocol tests passed\n");
  return 0;
}
