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

## 4. Code review of `fix/hardening` (2026-10-02)

Static review of the whole branch: no critical issues; STOP handling, queue
eviction, dedupe vs. RS polling and parser bounds were confirmed correct.

| # | Sev. | Finding | Status |
|---|------|---------|--------|
| R1 | important | A `Stopped` event processed right after a motion command reset the operation and cancelled a partial-position target. The device log (§5) shows this happening: the `Stopped` events are replies to earlier STOP commands, not spontaneous repeats. | fixed – `Stopped` within 2 s of a queued OPEN/CLOSE/PED OPEN is ignored when already stopped |
| R2 | important | RS-based motion detection could fire on an in-flight status right after a stop and make the state flap; no publish at the detection site. | fixed – 1 s hold-off after any state change / user STOP; publishes on detection |
| R3 | minor | After `Stopped`/`PedOpened` the position stayed at the last in-motion reading; the gate coasts further (device test: shown 52 %, actual 54 %). | fixed – the first `E6` status after a stop sets the position |
| R4 | minor | `publish()` throttling rarely re-arms, because `process()` updates `position_` itself. Harmless: all state changes publish explicitly. | accepted |
| R5 | minor | A pending parameter write waits forever if the `ACK RP` never arrives (no timeout). | open |

## 5. Device log analysis (`logs/logs_driveway-gate_run.txt`)

Real-device run on 2026-03-22 (ESPHome 2026.3.0, firmware built from the GitHub
`main` of that time, i.e. **before** this hardening). Scenario: partial open to
52 %, then close. `gatepro` logged at INFO, no raw UART debug.

| # | Observation (timestamps from the log) | Meaning | Status |
|---|----------------------------------------|---------|--------|
| L1 | 11:03:59.96 – 11:04:10.0: `Cover STOP command received` every ~200 ms for 10 s, 24× `TX queue full, dropping oldest command`. | The old `stop_at_target_position()` never cleared its target, so it re-sent STOP on every `update()`. This confirms finding #5 and the TX-queue issue #3 in production. | fixed (`stop_at_target_` flag, STOP dedupe/priority) |
| L2 | 33 `Stopped` events, all between the first STOP and 11:04:12.05; none after the gate moved again. | `Stopped` is the motor's **reply to each STOP**. It is not a spontaneous 200 ms repeat as the old code comment claimed. | comments corrected |
| L3 | CLOSE at 11:04:10.56 → 5 more `Stopped` replies → op shown `IDLE` at 11:04:11.03 → `Closing` only at 11:04:12.45. | A backlogged STOP reply cancelled the new command's state (R1), so the UI showed idle for about 1.4 s while the gate started closing. | fixed (R1) |
| L4 | Command → motor event latency: `Opening` 1.4 s (empty queue), `Closing` 1.9 s (backlogged queue). `Closed` arrived 3.5 s after the position had reached 0 %. | The 2 s stale-`Stopped` window covers a normal start. Near the end stop the motor creeps for several seconds. The 1–99 % clamp keeps the cover "closing" until `Closed`. | info |
| L5 | Same physical spot: 48 % at the end of opening, 60 % at the start of closing. Speed is about 8 %/s opening and about 6.5 %/s closing. | The RS position is **not consistent between directions**: the opening scale or the `+128` offset assumption is off. Partial positions differ depending on direction. | resolved – the 2026-10-03 raw capture shows a consistent scale (stop at 52 %, close starts at 52 %); the mismatch came from the old code |
| L6 | 640 cover state publishes in about 90 s (about 7/s, also while idle). | Confirms the publish storm. | mitigated (Task 9); `process()` still publishes on each RS while moving |
| L7 | Parameter read at boot: speed 2, decel distance 4, decel speed 2, max current 3, auto close 5, pedestrian time 1, force detection 1. | Not enough on its own to settle the 0- vs 1-based encoding (plan Task 6). Needs the controller's own menu values for comparison. | **open** |
| L8 | `Closed` event has `src=0001`; the movement events have `src=P00287D7`. | Motor-originated vs. command-originated events. The parser ignores `src`, so this is harmless. | info |

Next device run: set `logger: logs: gatepro: DEBUG` and enable the `uart` debug
sequence, so that raw `ACK RS` lines are captured for L5 and `ACK RP` for L7.

## 6. Device test 2026-10-03 (hardened firmware)

Open → stop after ~6 s → close via Home Assistant, raw UART captured
(`logs/gate-events_2026-10-03_open-stop-close.txt`).

- STOP: exactly one `STOP`, one `Stopped`, one `RS`; no queue overflow (cf. L1).
- Coalesced reads (`ACK STOP` + `Stopped`, `ACK RS` + `Closed`) are split correctly.
- Status layout `ACK RS:00,80,<state>,<pos>,<t5>,<t6>,FF,FF,FF`: state `C4` moving,
  `A2` idle at an end stop, `E6` stopped midway; `<pos>` hex, +0x80 while opening.
- **Bug found:** the closed/open pattern was read at offset 10 instead of 13, so it
  never matched; after boot the cover stayed `UNKNOWN`, showed *open* while closed, and
  polled RS every tick. Fixed by classifying the status from the state token and the
  position (`status_is_at_end`, `status_is_stopped_midway`), which also restores a
  midway position after boot.
