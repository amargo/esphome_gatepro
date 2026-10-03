# GatePro Hardening Follow-ups Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers-extended-cc:subagent-driven-development (recommended) or superpowers-extended-cc:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Land the hardening work, cover the UART protocol parsing with host tests that also run in CI, make CI compile the component from the PR itself, remove dead code, and verify on the real gate.

**Architecture:** Move the pure string/protocol helpers out of `gatepro.cpp` into a header-only `gatepro_protocol.h` that has no ESPHome dependency. A plain `g++` test binary can then exercise them on the host, with `-fno-exceptions` like the ESP target. CI gets a `unit-tests` job and also compiles `examples/gatepro_boxer_local.yaml`, which uses `../components` instead of GitHub `main`.

**Tech Stack:** C++17, ESPHome 2026.9 (ESP-IDF 5.5.5), Python config validation, GitHub Actions (`esphome/build-action@v6`), g++ on ubuntu-latest.

Background: `docs/hardening-review.md` (findings #16–#18 are open and addressed here) and `docs/upstream-comparison.md` (U1–U6 → Tasks 6–9).

---

## File structure

| File | Responsibility |
|------|----------------|
| `components/gatepro/gatepro_protocol.h` (new) | Pure, exception-free parsing helpers and protocol constants. No ESPHome includes. |
| `components/gatepro/gatepro.cpp` | Uses `protocol::` helpers; loses its private copies. |
| `components/gatepro/gatepro.h` | Includes `gatepro_protocol.h`; drops duplicated constants and dead members. |
| `tests/test_protocol.cpp` (new) | Host unit tests for `gatepro_protocol.h`. |
| `.github/workflows/esphome.yml` | Adds a `unit-tests` job and the local-component compile. |
| `.gitignore` | Ignores `/build/` (host test output). |

## Environment notes (Windows)

- Compile firmware from **PowerShell or cmd**, not Git Bash: the ESP-IDF installer aborts under MSYS with `ERROR: MSys/Mingw is not supported`.
- There is no host `g++` on the dev machine. Run the unit tests in WSL (`wsl g++ …`) or rely on the CI `unit-tests` job.
- ESPHome venv used during the review: `python -m venv .venv && .venv\Scripts\pip install esphome`.

---

### Task 0: Put the hardening work on its own branch and commit it

**Goal:** Move the uncommitted hardening changes (plus the earlier STOP-flood and correction fix) off `chore/unify-github-workflows` onto `fix/hardening` and commit them.

**Files:**
- Commit: `components/gatepro/gatepro.cpp`, `components/gatepro/gatepro.h`, `components/gatepro/cover.py`, `docs/hardening-review.md`, `docs/superpowers/plans/2026-09-25-gatepro-hardening-followups.md*`
- Leave alone: `.gitignore` (staged `*.txt`; belongs to the owner, ask before committing it)

**Acceptance Criteria:**
- [ ] `fix/hardening` branches from `main` and contains exactly one commit with the files above
- [ ] `chore/unify-github-workflows` has no uncommitted changes left, except `.gitignore` if the owner keeps it there

**Verify:** `git log --oneline main..fix/hardening` → 1 commit; `git diff --stat main fix/hardening` → only the files listed

**Steps:**

- [ ] **Step 1: Create the branch from `main` and carry the working tree over**

The chore branch only touches `.github/` and `renovate.json`, so the switch does not conflict with the modified component files.

```bash
git switch -c fix/hardening main
git status --short
```

Expected: ` M components/gatepro/{cover.py,gatepro.cpp,gatepro.h}`, `?? docs/`, and `M  .gitignore`.

- [ ] **Step 2: Commit (no AI attribution, short message)**

```bash
git add components/gatepro/gatepro.cpp components/gatepro/gatepro.h components/gatepro/cover.py docs/
git commit -m "fix(gatepro): harden UART parsing, command queue and param writes"
```

---

### Task 1: Extract protocol helpers into `gatepro_protocol.h` with host tests

**Goal:** Protocol parsing lives in a dependency-free header and is covered by host unit tests compiled with `-fno-exceptions`.

**Files:**
- Create: `components/gatepro/gatepro_protocol.h`
- Create: `tests/test_protocol.cpp`
- Modify: `components/gatepro/gatepro.cpp` (helper block after `TAG`, the `ACK RS` position block, the body of `GatePro::parse_params`)
- Modify: `components/gatepro/gatepro.h` (the `NUM_PARAMS`, `MAX_PARAM_VALUE` and `known_percentage_offset` members)
- Modify: `.gitignore`

**Acceptance Criteria:**
- [ ] `tests/test_protocol.cpp` passes with `g++ -std=c++17 -Wall -Wextra -Werror -fno-exceptions`
- [ ] `gatepro.cpp` no longer defines `starts_with`, `field_equals`, `payload_after` or `parse_int`
- [ ] Firmware compile of `examples/gatepro_boxer_local.yaml` succeeds with no `gatepro` warnings

**Verify:** `mkdir -p build && g++ -std=c++17 -Wall -Wextra -Werror -fno-exceptions -I components/gatepro tests/test_protocol.cpp -o build/test_protocol && ./build/test_protocol` → `All protocol tests passed`

**Steps:**

- [ ] **Step 1: Write the failing test** – `tests/test_protocol.cpp`

```cpp
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

int main() {
  test_starts_with();
  test_field_equals();
  test_payload_after();
  test_parse_int();
  test_parse_params_ok();
  test_parse_params_rejects();
  test_parse_position();
  if (failures) {
    std::printf("%d check(s) failed\n", failures);
    return 1;
  }
  std::printf("All protocol tests passed\n");
  return 0;
}
```

- [ ] **Step 2: Run it and check that it fails**

Run: `mkdir -p build && g++ -std=c++17 -Wall -Wextra -Werror -fno-exceptions -I components/gatepro tests/test_protocol.cpp -o build/test_protocol`
Expected: FAIL with `fatal error: gatepro_protocol.h: No such file or directory`

- [ ] **Step 3: Create `components/gatepro/gatepro_protocol.h`**

```cpp
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

}  // namespace protocol
}  // namespace gatepro
}  // namespace esphome
```

- [ ] **Step 4: Run the test and check that it passes**

Run: `mkdir -p build && g++ -std=c++17 -Wall -Wextra -Werror -fno-exceptions -I components/gatepro tests/test_protocol.cpp -o build/test_protocol && ./build/test_protocol`
Expected: `All protocol tests passed`

- [ ] **Step 5: Make `gatepro.h` use the protocol header**

Add after `#include "esphome/components/switch/switch.h"`:

```cpp
#include "gatepro_protocol.h"
```

Delete these three members (the protocol header now provides them):

```cpp
  static const size_t NUM_PARAMS = 17;             // GatePro has 17 parameter groups (0-16)
  static const int MAX_PARAM_VALUE = 15;           // Upper bound for any parameter value
```

```cpp
  const int known_percentage_offset = 128;
```

- [ ] **Step 6: Make `gatepro.cpp` use the protocol header**

1. Delete everything from the comment `// Messages are stored with control characters escaped (see convert()),` down to and including the closing `}` of `static bool parse_int(...)`. Put this in its place:

```cpp
using namespace protocol;
```

2. In `process()`, replace the block from `// Extract the position value (hex)` through the closing `}` of `if (percentage < 0 || percentage > 100) { … }` with:

```cpp
      // Extract the position value (hex, 0-100 after offset correction)
      int percentage;
      if (!parse_position(msg, percentage)) {
        ESP_LOGW(TAG, "Ignoring invalid position in ACK RS message: %s", msg.c_str());
        return;
      }
```

3. In `GatePro::parse_params`, replace everything from `const std::string payload = payload_after(msg, 9);` through `this->params = parsed;` with the code below. Qualify the call: an unqualified `parse_params` inside the member function would find the member itself.

```cpp
   std::vector<int> parsed;
   if (!protocol::parse_params(msg, parsed)) {
      ESP_LOGE(TAG, "Invalid parameter response, ignoring: %s", msg.c_str());
      if (!this->paramTaskQueue.empty()) {
         ESP_LOGW(TAG, "Discarding %zu pending parameter write(s)", this->paramTaskQueue.size());
         while (!this->paramTaskQueue.empty()) {
            this->paramTaskQueue.pop();
         }
      }
      this->param_no_pub = false;
      return;
   }
   this->params = parsed;
```

4. `set_param` and `write_params` already use `NUM_PARAMS` and `MAX_PARAM_VALUE` unqualified. The `using namespace protocol;` above resolves them, so no change is needed there.

- [ ] **Step 7: Ignore host build output**

Append to `.gitignore`:

```
/build/
```

- [ ] **Step 8: Firmware compile (PowerShell)**

```powershell
Set-Location examples
..\.venv\Scripts\esphome.exe compile gatepro_boxer_local.yaml 2>&1 | Select-String "gatepro.*(warning|error)|Successfully compiled|FAILED"
```

Expected: `INFO Successfully compiled program.` and no `gatepro` warning or error lines.

- [ ] **Step 9: Commit**

```bash
git add components/gatepro/gatepro_protocol.h components/gatepro/gatepro.cpp components/gatepro/gatepro.h tests/test_protocol.cpp .gitignore
git commit -m "test(gatepro): extract protocol helpers and add host tests"
```

---

### Task 2: CI compiles the PR's own component and runs the unit tests

**Goal:** Every PR compiles `components/gatepro` from the checkout (not GitHub `main`) and runs `tests/test_protocol.cpp`.

**Files:**
- Modify: `.github/workflows/esphome.yml`

**Dependency:** this file is also changed on `chore/unify-github-workflows`. Merge that PR first and rebase `fix/hardening` onto `main`, or do this task on top of it.

**Acceptance Criteria:**
- [ ] The `compile` matrix includes `gatepro_boxer_local.yaml`
- [ ] A new `unit-tests` job builds and runs the protocol tests
- [ ] Both jobs are green on the PR

**Verify:** `gh pr checks <PR#>` → all checks `pass`

**Steps:**

- [ ] **Step 1: Add the local example to the matrix**

In `jobs.compile.strategy.matrix.file`:

```yaml
        file:
          - gatepro_boxer_example.yaml
          - gatepro_boxer_local.yaml
```

- [ ] **Step 2: Add the unit-test job** (a sibling of `compile` under `jobs:`)

```yaml
  unit-tests:
    runs-on: ubuntu-latest
    steps:
      - name: Checkout source code
        uses: actions/checkout@v7

      - name: Protocol unit tests
        run: |
          mkdir -p build
          g++ -std=c++17 -Wall -Wextra -Werror -fno-exceptions \
            -I components/gatepro tests/test_protocol.cpp -o build/test_protocol
          ./build/test_protocol
```

- [ ] **Step 3: Push and check CI**

```bash
git add .github/workflows/esphome.yml
git commit -m "ci: compile local gatepro component and run protocol tests"
git push -u origin fix/hardening
gh pr create --base main --title "fix(gatepro): hardening" --body "Hardening of UART parsing, TX queue and parameter writes. Details: docs/hardening-review.md"
gh pr checks --watch
```

Expected: `compile (gatepro_boxer_example.yaml, …)`, `compile (gatepro_boxer_local.yaml, …)` and `unit-tests` all pass.

---

### Task 3: Remove dead code

**Goal:** Delete members that are declared or assigned but never used (review finding #16).

**Files:**
- Modify: `components/gatepro/gatepro.h`
- Modify: `components/gatepro/gatepro.cpp`

**Acceptance Criteria:**
- [ ] None of `update_state_from_position`, `start_direction_`, `debug`, `blocker`, `consecutive_position_readings_`, `last_position_reading_` or `std::string devinfo` appears in `components/gatepro/`
- [ ] Unit tests and the firmware compile still pass

**Verify:** `grep -nE "update_state_from_position|start_direction_|void debug|blocker|consecutive_position_readings_|last_position_reading_|std::string devinfo" components/gatepro/*` → no output

**Steps:**

- [ ] **Step 1: Header – delete these declarations from `gatepro.h`**

```cpp
      std::string devinfo = "N/A";
```
```cpp
  void update_state_from_position(float position);
```
```cpp
  void start_direction_(cover::CoverOperation dir);
```
```cpp
  void debug();
```
```cpp
  bool blocker{false};
```
```cpp
  uint8_t consecutive_position_readings_{0};
  float last_position_reading_{-1.0f};
```

- [ ] **Step 2: Source – delete from `gatepro.cpp`**

- the whole `void GatePro::start_direction_(cover::CoverOperation dir) { … }` function
- the whole `void GatePro::update_state_from_position(float position) { … }` function
- these lines in `setup()`:

```cpp
   this->consecutive_position_readings_ = 0;
   this->last_position_reading_ = -1.0f;
```
```cpp
   this->blocker = false;
```

- [ ] **Step 3: Verify**

Run the grep from **Verify**, then the Task 1 test command, then the Task 1 Step 8 firmware compile.
Expected: no grep output, `All protocol tests passed`, `Successfully compiled program.`

- [ ] **Step 4: Commit**

```bash
git add components/gatepro/gatepro.h components/gatepro/gatepro.cpp
git commit -m "refactor(gatepro): remove unused members"
```

---

### Task 4: Real-device verification

**Goal:** Confirm the safety-relevant behaviour on the real gate before merging.

**Order:** run this after Tasks 6–9 so a single flash covers everything; steps 8–11 belong to those tasks.

**Files:** none (manual test). Record the results in `docs/hardening-review.md` §2.

**Acceptance Criteria:**
- [ ] Every scenario below behaves as expected, and the log excerpts are recorded

**Verify:** `esphome logs examples/gatepro_boxer_local.yaml` (PowerShell) while running each scenario

**Steps:**

- [ ] **Step 1: Flash** – `esphome run examples/gatepro_boxer_local.yaml` (PowerShell). Check that `dump_config` prints `GatePro:` and `Source: …`.
- [ ] **Step 2: STOP priority** – press Close, then press Stop within 1 s. Expect `UART TX[..]: STOP;src=…` as the next TX line, and no `FULL CLOSE` after it.
- [ ] **Step 3: Partial position** – from closed, set the position to 50 %. Expect `Target position 0.50 reached (…), stopping` once. There must be no repeated STOP on later updates.
- [ ] **Step 4: Stale target** – right after step 3, press the Open button. Expect the gate to open fully, without stopping at 50 %.
- [ ] **Step 5: Same position** – with the gate idle at 50 %, set 50 % again. Expect `Already at requested position 0.50` and no movement.
- [ ] **Step 6: Toggle** – run the HA `cover.toggle` service while idle-closed, then while moving, then while open. Expect open → stop → close.
- [ ] **Step 7: Parameter write** – change "Auto Close Timer" by one step. Expect one `WP,1:` line with 17 values, then `ACK WP`, then a fresh `ACK RP` that shows the new value. Revert it afterwards.
- [ ] **Step 8: Parameter range guard (Task 6)** – from the HA developer tools, set the speed number to a value above the Task 6 maximum. Expect `Invalid value … for parameter 3` and no `WP,1:` line.
- [ ] **Step 9: Motion by remote (Task 7)** – with HA showing the gate idle, open it with the **physical remote**. Expect `Motion detected from status: opening` (or `$V1PKF0 … Opening`), and HA shows *opening* within about 1 s.
- [ ] **Step 10: Direction change mid-travel (Task 7)** – while opening, press Close. The reported position must stay within 1–99 % until a `Closed` event arrives.
- [ ] **Step 11: Pedestrian open (Task 8)** – trigger a pedestrian opening with the remote. Expect `Gate is opening (pedestrian)`, then `Pedestrian opening finished`, and HA shows a partial position with state idle. Then check that the idle log contains **no** `publish` storm (Task 9): no more than 10 state publishes after motion stops.
- [ ] **Step 12: Record** – add the rows to `docs/hardening-review.md` §2 and commit:

```bash
git add docs/hardening-review.md
git commit -m "docs: record real-device verification of gatepro hardening"
```

---

### Task 5: Branch cleanup (needs owner confirmation)

**Goal:** Delete the fully merged `feature/precheck-for-write-params`.

**Files:** none

**Acceptance Criteria:**
- [ ] The owner confirmed the deletion
- [ ] The branch is gone locally and on `origin`

**Verify:** `git branch -a | grep precheck` → no output

**Steps:**

- [ ] **Step 1: Re-check that it is merged** – `git rev-list --count main..feature/precheck-for-write-params` → `0`
- [ ] **Step 2: Ask the owner.** Only after a yes:

```bash
git branch -d feature/precheck-for-write-params
git push origin --delete feature/precheck-for-write-params
```

---

### Task 6: Per-parameter value ranges (U1, U2)

**Goal:** `set_param` rejects any value outside the documented range of its parameter group, and the example sliders use the encoding the controller actually expects.

**Files:**
- Modify: `components/gatepro/gatepro_protocol.h`
- Modify: `tests/test_protocol.cpp`
- Modify: `components/gatepro/gatepro.cpp` (`GatePro::set_param`)
- Modify: `examples/gatepro_boxer_example.yaml` (`min_value` / `max_value` of the number entities)

**Acceptance Criteria:**
- [ ] The 0- vs 1-based encoding is confirmed on the device and written down in `docs/upstream-comparison.md` U2
- [ ] `param_value_valid()` is unit-tested, and `set_param` uses it
- [ ] Example slider ranges match `PARAM_MAX_VALUES`

**Verify:** Task 1 test command → `All protocol tests passed`; firmware compile OK

**Steps:**

- [ ] **Step 1: Device check (decides the table)**

Flash the current firmware. Press **Get Parameters** and note the logged `[3]`…`[7]` values from `Parsed current params`. In the controller's own menu (or its manual), read the speed, decel distance, decel speed, max current and pedestrian time settings.

- If speed = 50 % is logged as `[3] = 0`, the encoding is **0-based**: the fork is right and our sliders are off by one.
- If it is logged as `[3] = 1`, the encoding is **1-based**: add 1 to the maxima of idx 3–7 in Step 3.

Record the result under U2 in `docs/upstream-comparison.md`.

- [ ] **Step 2: Write the failing test** – add to `tests/test_protocol.cpp` above `int main()`:

```cpp
static void test_param_value_valid() {
  CHECK(param_value_valid(0, 0));
  CHECK(param_value_valid(0, 1));
  CHECK(!param_value_valid(0, 2));       // opening direction: left/right only
  CHECK(param_value_valid(1, 8));        // auto close: up to 180 s
  CHECK(!param_value_valid(1, 9));
  CHECK(param_value_valid(6, PARAM_MAX_VALUES[6]));
  CHECK(!param_value_valid(6, PARAM_MAX_VALUES[6] + 1));
  CHECK(!param_value_valid(3, -1));
  CHECK(!param_value_valid(NUM_PARAMS, 0));  // index out of range
}
```

and call `test_param_value_valid();` in `main()` after `test_parse_position();`.

Run the Task 1 test command. Expected: compile error `'param_value_valid' was not declared in this scope`.

- [ ] **Step 3: Implement** – add to `gatepro_protocol.h` after `POSITION_OFFSET`:

```cpp
// Highest valid value per parameter group (index 0-16). Source: option lists in
// markv9401/rowra44 esphome_external_components cover.py (SELECTS/SWITCHES),
// 0-based. If the Task 6 device check showed 1-based encoding, idx 3-7 are +1.
static const int PARAM_MAX_VALUES[NUM_PARAMS] = {
    1,  // 0  opening direction (left/right)
    8,  // 1  auto close (off,5,15,30,45,60,80,120,180 s)
    2,  // 2  security device method
    3,  // 3  operational speed (50/70/85/100 %)
    4,  // 4  deceleration distance (75..95 %)
    3,  // 5  deceleration speed (80/60/40/25 %)
    7,  // 6  max current (2..9 A)
    5,  // 7  pedestrian duration (3..18 s)
    1,  // 8  flashing light mode
    3,  // 9  overload action
    3,  // 10 remote button: open (A-D)
    4,  // 11 remote button: pedestrian (off,A-D)
    4,  // 12 remote button: external device (off,A-D)
    1,  // 13 infrared 1
    1,  // 14 infrared 2
    1,  // 15 stop terminal / permanent lock
    1,  // 16 operating method
};

inline bool param_value_valid(size_t idx, int value) {
  return idx < NUM_PARAMS && value >= 0 && value <= PARAM_MAX_VALUES[idx];
}
```

Run the Task 1 test command. Expected: `All protocol tests passed`.

- [ ] **Step 4: Use it in `set_param`** (`gatepro.cpp`) – replace

```cpp
   if (val < 0 || val > MAX_PARAM_VALUE) {
      ESP_LOGE(TAG, "Invalid value %d for parameter %d (valid range: 0-%d)", val, idx, MAX_PARAM_VALUE);
      return;
   }
```

with

```cpp
   if (!param_value_valid(idx, val)) {
      ESP_LOGE(TAG, "Invalid value %d for parameter %d (valid range: 0-%d)", val, idx, PARAM_MAX_VALUES[idx]);
      return;
   }
```

The index is already validated just above, so `PARAM_MAX_VALUES[idx]` is in bounds. `parse_params` keeps the generic `MAX_PARAM_VALUE` check on purpose: a value the device reports must not block all later writes.

- [ ] **Step 5: Align the example sliders** – in `examples/gatepro_boxer_example.yaml`, use the ranges below (0-based case). In the 1-based case, use `min_value: 1` and `max_value: PARAM_MAX_VALUES[idx]` for idx 3–7:

| entity id | idx | 0-based min / max |
|-----------|-----|-------------------|
| `speed_slider` | 3 | 0 / 3 |
| `decel_dist_slider` | 4 | 0 / 4 |
| `decel_speed_slider` | 5 | 0 / 3 |
| `max_amp_slider` | 6 | 0 / 7 |
| `auto_close_slider` | 1 | 0 / 8 |
| `small_gate_timer` | 7 | 0 / 5 |
| `force_detection_number` | 9 | 0 / 3 |

Also fix the range comments in `GatePro::setup()` (`// Group 4: 0-3 (1=default, 1=50%, …)` and the like) to match.

- [ ] **Step 6: Compile and commit**

Firmware compile (Task 1 Step 8), then:

```bash
git add components/gatepro/gatepro_protocol.h components/gatepro/gatepro.cpp tests/test_protocol.cpp examples/gatepro_boxer_example.yaml docs/upstream-comparison.md
git commit -m "fix(gatepro): validate parameter values per group"
```

---

### Task 7: Motion detection from `RS` and position clamp while moving (U3, U4)

**Goal:** Detect movement that started outside ESPHome (remote, missed event, reboot mid-travel) from the status reply, and keep the position inside 1–99 % while moving.

**Files:**
- Modify: `components/gatepro/gatepro_protocol.h`
- Modify: `tests/test_protocol.cpp`
- Modify: `components/gatepro/gatepro.cpp` (`process()`, `ACK RS` branch)

**Acceptance Criteria:**
- [ ] `status_is_opening()` and `status_is_moving()` are unit-tested
- [ ] An idle cover switches to opening/closing when `RS` reports motion
- [ ] The position published while moving is never 0 or 100 %

**Verify:** Task 1 test command → `All protocol tests passed`; firmware compile OK

**Steps:**

- [ ] **Step 1: Write the failing test** – add above `int main()` and call it from `main()`:

```cpp
static void test_status_motion() {
  // "ACK RS:00,80,C4,C6,..." : 3rd token C4 = moving, C6 = 198 > 100 = opening
  const std::string opening = "ACK RS:00,80,C4,C6,3E,16,FF,FF,FF\\r\\n";
  const std::string closing = "ACK RS:00,80,C4,32,3E,16,FF,FF,FF\\r\\n";
  const std::string idle    = "ACK RS:00,A2,00,40,00,16,FF,FF,FF\\r\\n";
  CHECK(status_is_opening(opening));
  CHECK(status_is_moving(opening));
  CHECK(!status_is_opening(closing));
  CHECK(status_is_moving(closing));
  CHECK(!status_is_opening(idle));
  CHECK(!status_is_moving(idle));
  CHECK(!status_is_moving("ACK RS:00"));   // too short: no throw
  CHECK(!status_is_opening("ACK RS:00"));
}
```

Run the Task 1 test command. Expected: compile error `'status_is_opening' was not declared in this scope`.

- [ ] **Step 2: Implement** – add to `gatepro_protocol.h` after `parse_position`:

```cpp
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
```

Run the tests. Expected: `All protocol tests passed`.

- [ ] **Step 3: Use it in `process()`** – in the `ACK RS` branch, directly **before** the line
`// For position updates, only process them if the gate is in motion`, insert:

```cpp
    // Motion started outside ESPHome (remote control, missed $V1PKF0 event,
    // reboot mid-travel): derive the direction from the status itself.
    if (this->current_operation == cover::COVER_OPERATION_IDLE) {
      const bool opening = status_is_opening(msg);
      if (opening || status_is_moving(msg)) {
        ESP_LOGI(TAG, "Motion detected from status: %s", opening ? "opening" : "closing");
        GateProState old_state = this->gate_state_;
        this->operation_finished = false;
        this->current_operation = opening ? cover::COVER_OPERATION_OPENING : cover::COVER_OPERATION_CLOSING;
        this->last_operation_ = this->current_operation;
        this->gate_state_ = opening ? STATE_OPENING : STATE_CLOSING;
        this->log_state_change(old_state, this->gate_state_);
      }
    }

```

Note: the closed/open pattern checks earlier in this branch `return` before this point only for the idle patterns `A2,00,40,00` / `A2,E3,40,00`, whose 3rd token is not `C4`, so moving statuses always reach this code.

- [ ] **Step 4: Clamp while moving** – in the same branch, replace

```cpp
      float new_position = (float)percentage / 100;
```

with

```cpp
      // While moving, the controller may report 0/100 % long before the end
      // stop (e.g. after a direction change). End positions are only set by
      // the Opened/Closed events.
      if (this->current_operation != cover::COVER_OPERATION_IDLE) {
        percentage = std::max(1, std::min(99, percentage));
      }
      float new_position = (float)percentage / 100;
```

- [ ] **Step 5: Compile and commit**

Firmware compile (Task 1 Step 8), then:

```bash
git add components/gatepro/gatepro_protocol.h components/gatepro/gatepro.cpp tests/test_protocol.cpp
git commit -m "fix(gatepro): detect motion from status and clamp moving position"
```

---

### Task 8: Pedestrian open events (U5)

**Goal:** `PedOpening` / `PedOpened` motor events drive the cover state.

**Files:**
- Modify: `components/gatepro/gatepro.cpp` (`process()`, `$V1PKF0` branch)

**Acceptance Criteria:**
- [ ] `PedOpening` → state *opening*; `PedOpened` → idle at a partial position (`STATE_STOPPED`)
- [ ] The firmware compiles

**Verify:** firmware compile OK; Task 4 Step 11 on the device

**Steps:**

- [ ] **Step 1: Add the events** – in the `$V1PKF0` branch, insert before `else if (field_equals(msg, 11, "Stopped")) {`:

```cpp
    else if (field_equals(msg, 11, "PedOpening")) {
      ESP_LOGI(TAG, "Gate is opening (pedestrian)");
      this->stop_at_target_ = false;
      this->operation_finished = false;
      this->current_operation = cover::COVER_OPERATION_OPENING;
      this->last_operation_ = cover::COVER_OPERATION_OPENING;
      this->gate_state_ = STATE_OPENING;
      this->last_state_change_ = now;
      this->log_state_change(old_state, this->gate_state_);
      this->publish_state();
      return;
    }
    else if (field_equals(msg, 11, "PedOpened")) {
      // Pedestrian opening ends at a partial position, not at COVER_OPEN.
      ESP_LOGI(TAG, "Pedestrian opening finished");
      this->operation_finished = true;
      this->current_operation = cover::COVER_OPERATION_IDLE;
      this->gate_state_ = STATE_STOPPED;
      this->last_state_change_ = now;
      this->log_state_change(old_state, this->gate_state_);
      this->publish_state();
      return;
    }
```

`"PedOpening"` and `"Opening"` are compared at the same offset 11, so they cannot match each other.

- [ ] **Step 2: Compile and commit**

Firmware compile (Task 1 Step 8), then:

```bash
git add components/gatepro/gatepro.cpp
git commit -m "feat(gatepro): handle pedestrian open events"
```

---

### Task 9: Publish throttling (U6)

**Goal:** Stop calling `publish_state()` every 0.2 s when nothing changes. Publish on change, then for at most 10 more ticks.

**Files:**
- Modify: `components/gatepro/gatepro.h`
- Modify: `components/gatepro/gatepro.cpp` (`GatePro::publish`)

**Acceptance Criteria:**
- [ ] With the gate idle, at most 10 state publishes follow the last change
- [ ] The firmware compiles

**Verify:** firmware compile OK; Task 4 Step 11

**Steps:**

- [ ] **Step 1: Header** – in `gatepro.h`, next to `void publish();`, add:

```cpp
  static const uint8_t PUBLISH_AFTER_TICKS = 10;  // extra publishes after a change
  uint8_t publish_ticks_left_{PUBLISH_AFTER_TICKS};
```

- [ ] **Step 2: Replace `GatePro::publish()`** in `gatepro.cpp`:

```cpp
void GatePro::publish() {
    if (this->position_ != this->position) {
      this->position_ = this->position;
      this->publish_ticks_left_ = PUBLISH_AFTER_TICKS;
    } else if (this->publish_ticks_left_ == 0) {
      return;
    } else {
      this->publish_ticks_left_--;
    }
    this->publish_state();
}
```

State changes from motor events still publish immediately via the `publish_state()` calls in `process()`.

- [ ] **Step 3: Compile and commit**

Firmware compile (Task 1 Step 8), then:

```bash
git add components/gatepro/gatepro.h components/gatepro/gatepro.cpp
git commit -m "perf(gatepro): throttle idle state publishing"
```
