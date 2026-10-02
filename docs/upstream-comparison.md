# Comparison with rowra44 / markv9401 `esphome_external_components`

Compared on 2026-10-02.

- Source: `git@github.com:rowra44/esphome_external_components.git`. Its `main` is `643abfc` and its `dev` is `1775de8`. Both are **commit-identical** to `markv9401/esphome_external_components` (our `upstream` remote), so this compares one code base, not two.
- Our side: `fix/hardening` working tree (see `docs/hardening-review.md`).
- Common ancestor: `f9755cd` (2024-11-21). Since then the fork has 345 commits touching `components/gatepro`, the last on 2026-06-21.

## 1. What the fork does better (worth porting)

| # | Fork feature | Where in the fork | Our state | Value | Plan task |
|---|--------------|-------------------|-----------|-------|-----------|
| U1 | **Per-parameter value ranges.** Every parameter group has an explicit option list (e.g. idx 3 speed: 4 values `0..3`; idx 6 max current: 8 values `0..7`; idx 8 flashing light: `0..1`). | `cover.py` `SELECTS` | We only check a generic `0..15`, so an out-of-range value can still be written to the controller. | high (safety) | 6 |
| U2 | **0-based vs 1-based value encoding.** The fork writes `0..3` for speed, `0..4` for decel distance, `0..3` for decel speed, `0..7` for max current and `0..5` for pedestrian time. Our example sliders use `1..4`, `1..5`, `1..4`, `1..9` and `1..6`. One of the two is off by one; at the top of our slider ranges we may be writing values the controller does not know. | `cover.py` `SELECTS` vs our `examples/*.yaml` | **unverified** – must be checked on the device | high | 6 (step 1: device check) |
| U3 | **Motion detection from the `RS` status alone.** A raw position above 100 (offset +128) means *opening*. Otherwise the 3rd token `C4` (`msg[13..14]`) means *moving*, hence *closing*. This finds the state after a reboot, or when a `$V1PKF0` event is missed (gate moved by remote). | `gatepro.cpp` `ACK_RS` case, `constants.h` `STATUS_OP_MOVING` | We subtract the offset but throw away the direction. With an unknown state we rely on the `A2,00,40,00` / `A2,E3,40,00` patterns. | medium | 7 |
| U4 | **Clamp the position to 1–99 % while moving.** If a direction change is commanded mid-travel, the controller reports 0 / 100 % long before the end stop, and the cover gets stuck "closed/open but still moving". | `gatepro.cpp` `ACK_RS` case | We publish 0 / 100 % while moving. | medium | 7 |
| U5 | **Pedestrian events** `PedOpening` / `PedOpened` (+ `ACK PED OPEN`). | `constants.h` `MotorEvents` | Not recognised: during a pedestrian opening the cover stays idle/closed. | medium | 8 |
| U6 | **Publish throttling.** Publish only on change, plus `AFTER_TICK_MAX = 10` extra ticks after a change. | `gatepro.cpp` `publish()` | We call `publish_state()` on every update (every 0.2 s), so Home Assistant receives about 5 updates per second even when idle. | low | 9 |
| U7 | Selects for all 17 parameter groups, generic button mapping (`PED OPEN`, `RS`, …), and dedicated `GateProSwitch` / `GateProSelect` / `GateProButton` classes. | `cover.py`, `gatepro_*.h` | We have a subset (sliders, 3 switches, buttons). | feature, not hardening | – (out of scope) |

## 2. What the fork still gets wrong (do **not** port)

Every point below was already fixed in our hardening and still applies to the fork:

| Fork issue | Our finding # |
|------------|---------------|
| `stoi()` in `get_position_percentage()` and `parse_params()` → abort on bad data | 1 |
| Unchecked `substr()` in `identify_current_msg_type`, DEVINFO and LEARN STATUS (`size - 21` underflow) | 2 |
| `tx_queue` unbounded, no STOP priority | 3 |
| `set_param` and `publish_params` index `params[idx]` with no bounds check. `select->publish_state(options[params[idx]])` reads out of bounds when the device reports a value outside the option list | 4 |
| `stop_at_target_position` with unqualified `abs` and no reset of the target | 5 |
| Uninitialised `btn_*`, `target_position_`, `position_`, `operation_finished`, `last_call_` | 10 |
| `cover.COVER_OPERATIONS` mutation with a non-existent enum value | 13 |

Plus defects that exist **only in the fork**:

- `read_uart()`: `new uint8_t[available]` is never freed, so it **leaks memory on every UART read**. It also takes only one message per call, so the buffer can grow without limit.
- `tx_queue` is `std::queue<const char*>`, and `write_params()` pushes `params_cmd.c_str()`. A second `write_params()` before TX reallocates the string, which leaves a **dangling pointer**.
- `GateProMsgTypeMapping` contains the key `GATEPRO_MSG_ACK_RP` twice. The `ACK WP` entry is silently dropped.
- `auto_close` has 9 options but 10 values.
- `STOP` is suppressed when `operation_finished` is true. Ours always sends it, which is safer.
- The `source` (`P00287D7`) is hard-coded in every command. Ours is configurable and validated.

## 3. Conclusion

The fork is worth mining for **protocol knowledge** (U1–U5), not for code. Its implementation is less robust than ours. The plan
(`docs/superpowers/plans/2026-09-25-gatepro-hardening-followups.md`) adds tasks 6–9 for U1–U6. Task 6 starts with a device check, because the 0- vs 1-based question (U2) decides the ranges.
