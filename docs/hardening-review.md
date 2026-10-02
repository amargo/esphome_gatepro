# GatePro component – hardening review (2026-09-25)

Scope: `components/gatepro/gatepro.cpp`, `gatepro.h`, `cover.py`, plus the
state of the branches. Baseline: `main` @ `6d2b920`.

## 1. Findings

Severity: **critical** = crash/reboot or unsafe gate movement,
**high** = wrong device state or wrong config written to the motor,
**medium** = robustness / resource issue, **low** = hygiene.

| # | Sev. | Where (baseline) | Finding | Status |
|---|------|------------------|---------|--------|
| 1 | critical | `parse_params` | `stoi()` on untrusted UART data. ESP builds run without exception handling, so a garbled `ACK RP` (empty field, non-digit) aborts and reboots the node. | fixed – `parse_int()` (strtol + full validation) |
| 2 | critical | `process()` | Unchecked `substr()` offsets (`msg.substr(11, …)`, `substr(17, size-21)`): an out-of-range position throws, and a length underflow publishes garbage. | fixed – `starts_with`, `field_equals`, `payload_after` |
| 3 | critical | `queue_gatepro_cmd` | When the TX queue is full the **oldest** entry is dropped, which could be a pending `STOP`. `STOP` also waited behind queued commands, and a stale `FULL OPEN/CLOSE` could still go out after `STOP`. | fixed – `STOP` jumps to the front, supersedes pending motion commands and is never evicted |
| 4 | high | `set_param` | `params.resize(idx + 1, 0)` + `write_params()` could write a **partial/zeroed parameter set** to the controller if the preceding read was short or invalid. | fixed – `WP` is only sent after a fresh read with exactly 17 valid values, in one write |
| 5 | high | `stop_at_target_position` | Unqualified `abs()` on float (risk of the int overload); the target is never cleared, so a STOP is re-sent on every `update()`; a stale target stops the gate later when it is moved from a button or remote; overshoot between two polls → the gate runs to its end stop. | fixed – `std::fabs`, `stop_at_target_` flag, overshoot-aware check |
| 6 | high | `control()` | A partial position equal to the current position fell into the "opening" branch → **full open**. NaN or out-of-range positions were not rejected. | fixed |
| 7 | medium | `control()` | Traits advertise toggle, but `control()` ignored `get_toggle()`. | fixed |
| 8 | medium | `read_uart` | `rx_queue` unbounded; `read_array()` result ignored; unused `to_read`. | fixed – bounded queue (10), read error handled |
| 9 | medium | `ACK RS` | Position after the offset correction not range-checked (`0xFF` → 127 %). | fixed – values outside 0–100 are dropped |
| 10 | medium | `gatepro.h` | `btn_learn`, `btn_params_od`, `btn_remote_learn`, `target_position_`, `operation_finished`, `blocker` uninitialised. | fixed |
| 11 | medium | `cover.py` | `source` is pasted into serial commands without validation (`;`, CR/LF injection). | fixed – `[A-Za-z0-9_-]{1,32}` |
| 12 | medium | `cover.py` | No UART final validation; the headers pull in `sensor`, `number`, `switch`, `text_sensor` without `AUTO_LOAD`. | fixed – `FINAL_VALIDATE_SCHEMA` (9600 baud, TX+RX), `AUTO_LOAD` |
| 13 | low | `cover.py` | Mutates the global `cover.COVER_OPERATIONS` with a non-existent enum value; unused imports. | fixed |
| 14 | low | `get_command_string` | Static buffer, truncation of `snprintf` output not detected; `sprintf` in `convert()`. | fixed |
| 15 | low | `dump_config` | Printed nothing useful. | fixed – source, update interval |
| 16 | low | `gatepro.*` | Dead code: `update_state_from_position` (inverted logic: `<=0.05` → OPEN), `start_direction_`, `debug()`, `blocker`, `consecutive_position_readings_`, `last_position_reading_`, `devinfo`. | **open** – plan task 3 |
| 17 | low | CI | The CI compiles `gatepro_boxer_example.yaml`, which pulls the component from GitHub `main`, so a PR's own C++ changes are never compiled. | **open** – plan task 2 |
| 18 | low | tests | No host-side tests for the UART protocol parsing. | **open** – plan task 1 |

## 2. Verification results

| Check | Result |
|-------|--------|
| `python -m py_compile components/gatepro/cover.py` | OK |
| ESPHome 2026.9.0 config validation + C++ codegen (`examples/gatepro_boxer_local.yaml`, local component) | OK |
| Full firmware compile (ESP32 `wemos_d1_mini32`, ESP-IDF 5.5.5, run from PowerShell) | OK – no warnings from `gatepro`; RAM 27.1 % (49 024 / 180 736 B), Flash 52.1 % (956 511 / 1 835 008 B) |
| First compile | Emitted `check_uart_settings() is deprecated … Removed in 2027.3.0`. The call was removed; `FINAL_VALIDATE_SCHEMA` covers the check. |

Note: the ESP-IDF installer refuses to run under Git Bash/MSYS
(`ERROR: MSys/Mingw is not supported`). On Windows, compile from PowerShell or cmd.
| Host unit tests of the protocol parser | none yet (plan task 1) |
| Real-device test | not done (plan task 4) |

Diff size against `main` (whitespace-insensitive): 3 files, +290 / −90.
The working copy had CRLF line endings while the repo stores LF; the touched
files were normalised back to LF.

## 3. Branches

| Branch | vs `main` | Assessment |
|--------|-----------|------------|
| `main` | = `origin/main` | – |
| `chore/unify-github-workflows` | +2 / −0 | CI/Renovate only; ready for a PR. Currently also carries the uncommitted hardening work. |
| `feature/precheck-for-write-params` | +0 / −8 | Fully merged; can be deleted locally and on `origin`. |
| `upstream/main`, `upstream/dev` (markv9401) = `rowra44/main`, `rowra44/dev` (commit-identical) | diverged since 2024-11 | Different architecture (`constants.h`, select entities). Same `stoi` / unchecked `substr` weaknesses, but it holds valuable **protocol knowledge** (per-parameter ranges, motion detection from `RS`, pedestrian events). See `docs/upstream-comparison.md`. |
