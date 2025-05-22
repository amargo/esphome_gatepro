# ESPHome External Components
Central repo for ESPHome external components to be used by the ESPHome server

# Components

## GatePro Boxer Gate Motor Integration

GatePro and similar brands use TMT CHOW controllers with an ESP32-based WiFi box to control gate operations. By flashing the ESP32 (or replacing it) with this ESPHome firmware, you can achieve seamless integration with Home Assistant.

### Hardware Setup

#### UART Connection Guide

The GatePro controller board has two serial connectors:

1. **Left upper four solder points** (3V3 / TXD / RXD / GND): 
   - This connects directly to the ESP32 module's UART0
   - Used for flashing and debugging

2. **Right side 6-pin connector**: 
   - This is the bus to the motor controller
   - **This is the one you need to use for motor control!**

#### Wiring the 6-pin Connector

The right-side 6-pin connector pinout:

```
[1] Empty
[2] Yellow (LED/status - not UART)
[3] Red    (+5V power)
[4] Blue   ← MOTOR TX → connect to ESP32 RX (GPIO3)
[5] White  ← MOTOR RX → connect to ESP32 TX (GPIO1)
[6] Black  GND        → connect to ESP32 GND
```

Connect as follows:
- ESP32 GPIO1 (TX0) → White wire
- ESP32 GPIO3 (RX0) ← Blue wire

**Note**: The ESP32 is already powered internally by 3.3V from the module. Do not connect 5V to the ESP32's 3V3 pin.

#### Verification with Multimeter

You can verify connections using a multimeter in continuity mode:
- Place one probe on the black wire of the right connector and the other on the left side GND point (or ESP32 GND pad). It should beep.
- Similarly, you can check the blue-white pair: the blue wire connects to the left side RXD point, and the white wire connects to the TXD point.

### Software Configuration

#### Features

- **Basic Gate Control**: Open, close, and stop operations
- **Position Tracking**: Accurate position tracking during operation
- **Remote Control Detection**: Detects when the gate is operated by remote control
- **Operation Timing**: Tracks and reports the duration of opening/closing operations
- **Open Duration Warning**: Configurable warning when gate remains open for too long
- **Detailed State Reporting**: Provides detailed state information as attributes

#### Configuration Options

| Option | Default | Description |
|--------|---------|-------------|
| `source` | `P00287D7` | Source identifier for commands sent to the gate |
| `open_duration_warning` | `5min` | Time threshold after which a warning is triggered if gate remains open |
| `update_interval` | `60s` | How often to poll the gate status |

#### Example YAML Configuration

A complete example configuration file is available in the [examples directory](examples/gatepro_boxer_example.yaml).

Below is a basic configuration example:

```yaml
substitutions:
  devicename: "driveway-gate"
  name: Driveway Gate
  # Gate SRC code that must be appended to all commands
  # IMPORTANT: This unique identifier authenticates commands to your specific gate
  # Each gate has its own unique SRC code - you must use your gate's code here
  gate_src: "src=P00287D7\r\n"

esphome:
  name: ${devicename}
  friendly_name: ${name}
  project:
    name: esphome.web
    version: dev  

esp32:
  board: wemos_d1_mini32

# Include base configuration
<<: !include .base.yaml

# Disable UART logging
logger:
  baud_rate: 0
  # level: DEBUG

web_server:
  version: 3

captive_portal:

external_components:
  - source:
      type: git
      url: https://github.com/amargo/esphome_gatepro_external_components
      ref: main
    components: [ gatepro ]
    refresh: 0s

uart:
  baud_rate: 9600
  tx_pin: GPIO1
  rx_pin: GPIO3
  debug:
    direction: BOTH
    dummy_receiver: false
    sequence:
      - lambda: |-
          UARTDebug::log_string(direction, bytes);

cover:
  - platform: gatepro
    name: "${name}"
    device_class: gate
    update_interval: 0.5s

# Control buttons
button:
  - platform: restart
    name: "Reboot"
    id: reboot_btn
    entity_category: "diagnostic"
  - platform: uart
    name: "Gate Open"
    data: "FULL OPEN;${gate_src}"
    entity_category: "diagnostic"
  - platform: uart
    name: "Gate Close"
    data: "FULL CLOSE;${gate_src}"
    entity_category: "diagnostic"
  - platform: uart
    name: "Gate Stop"
    data: "RS;${gate_src}"
    entity_category: diagnostic    

  # Learn operation buttons
  - platform: uart
    name: "AUTO LEARN"
    data: "AUTO LEARN;${gate_src}"
    entity_category: "diagnostic"
  - platform: uart
    name: "READ LEARN STATUS"
    data: "READ LEARN STATUS;${gate_src}"
    entity_category: "diagnostic"

  # Parameter buttons
  - platform: uart
    name: "RP btn (read params)"
    data: "RP,1:;${gate_src}"
    entity_category: "diagnostic"

  # Advanced control examples
  # Speed control
  - platform: uart
    name: "Operating Speed 50 %"
    data: "WP,4:1;${gate_src}"    # 4-1 = 50 %
    entity_category: "diagnostic"
  - platform: uart
    name: "Operating Speed 100 %"
    data: "WP,4:4;${gate_src}"    # 4-4 = 100 %
    entity_category: "diagnostic"

  # Auto-closing function
  - platform: uart
    name: "Auto-close OFF"
    data: "WP,2:0;${gate_src}"    # 2-0 = disabled
    entity_category: "diagnostic"
  - platform: uart
    name: "Auto-close 60 s"
    data: "WP,2:5;${gate_src}"    # 2-5 = 60 seconds
    entity_category: "diagnostic"

  # Pedestrian gate function
  - platform: uart
    name: "Pedestrian 6 s"
    data: "WP,8:2;${gate_src}"    # 8-2 = 6 seconds (default)
    entity_category: "diagnostic"

# Sensors
sensor:
  - platform: wifi_signal
    name: "WiFi Signal dB sensor"
    id: wifi_signal_db
    update_interval: 60s
    entity_category: "diagnostic"
  - platform: uptime
    type: seconds
    name: "Uptime sensor"
    id: uptime_sensor
    update_interval: 60s
    entity_category: "diagnostic"

text_sensor:
  - platform: version
    name: "ESPHome Version"
    id: esphome_version
    entity_category: "diagnostic"
```

### AUTO LEARN Process

The AUTO LEARN feature is crucial for proper gate operation. It allows the motor to learn the gate's travel limits and operation parameters. Here's how to use it:

1. **Important**: Before initiating AUTO LEARN, disable UART logging by setting:
   ```yaml
   logger:
     baud_rate: 0
   ```

2. **Initiating AUTO LEARN**:
   - After flashing your ESP32 and connecting it to Home Assistant, locate the "AUTO LEARN" button in the Home Assistant interface
   - Press the "AUTO LEARN" button
   - The gate will begin a calibration sequence:
     - It will open fully
     - Then close fully
     - Then open partially
     - Finally close again

3. **Verifying AUTO LEARN Status**:
   - Use the "READ LEARN STATUS" button to check if the learning process completed successfully
   - A successful response indicates the gate is properly calibrated

4. **Troubleshooting**:
   - If AUTO LEARN fails, ensure:
     - All wiring connections are correct
     - The gate can move freely without obstructions
     - UART logging is disabled
     - Try power cycling the gate controller

### Parameter Settings

The GatePro controller accepts various parameters that can be modified. A complete set of advanced commands is available in the [examples directory](examples/gatepro_boxer_advanced_commands.yaml).

#### Basic Parameters

1. **Reading Parameters**:
   - Use the "RP btn (read params)" button to read current parameters
   - The response format is: `ACK RP,1:1,0,0,1,2,2,0,0,0,3,0,0,3,0,0,0,0\r\n`

2. **Speed Settings**:
   - "WriteSpeed 1" sets the gate to slow speed
   - "WriteSpeed 4" sets the gate to fast speed
   - The 4th parameter in the WP command controls speed (values 1-4)

3. **Permanent Lock**:
   - "PermaLock ON" enables the permanent lock feature
   - "PermaLock OFF" disables the permanent lock
   - The last parameter (17th) controls this feature (0=off, 1=on)

#### Advanced Parameters

The GatePro controller supports a more efficient way to control parameters using targeted commands. Instead of overwriting the entire parameter array with `WP,1:...`, you can target specific parameters with their function code.

This approach has several advantages:
- **Cleaner**: Only modifies the specific parameter you want to change
- **Simpler**: Easier to understand what each command does
- **Safer**: Other parameters remain unchanged

##### Parameter Command Format

```
WP,<function_code>:<value>;src=XXXXXXX\r\n
```

Where:
- `<function_code>` is the parameter group (2, 4, 5, 6, 7, 8, A)
- `<value>` is the setting value for that parameter

The following advanced parameters can be controlled:

1. **Auto-closing Function** (Parameter 2):
   - `WP,2:0;` - Disabled
   - `WP,2:1;` - 5 seconds
   - `WP,2:2;` - 15 seconds (default)
   - `WP,2:3;` - 30 seconds
   - `WP,2:4;` - 45 seconds
   - `WP,2:5;` - 60 seconds
   - `WP,2:6;` - 80 seconds
   - `WP,2:7;` - 120 seconds
   - `WP,2:8;` - 180 seconds

2. **Operating Speed** (Parameter 4):
   - `WP,4:1;` - 50%
   - `WP,4:2;` - 70%
   - `WP,4:3;` - 85%
   - `WP,4:4;` - 100%

3. **Deceleration Distance** (Parameter 5):
   - `WP,5:1;` - Start deceleration at 75% of travel
   - `WP,5:2;` - Start deceleration at 80% of travel
   - `WP,5:3;` - Start deceleration at 85% of travel
   - `WP,5:4;` - Start deceleration at 90% of travel
   - `WP,5:5;` - Start deceleration at 95% of travel

4. **Deceleration Speed** (Parameter 6):
   - `WP,6:1;` - 80% of normal speed
   - `WP,6:2;` - 60% of normal speed
   - `WP,6:3;` - 40% of normal speed
   - `WP,6:4;` - 25% of normal speed

5. **Force and Torque Sensing** (Parameter 7):
   - `WP,7:1;` - 2A
   - `WP,7:2;` - 3A
   - `WP,7:3;` - 4A
   - `WP,7:4;` - 5A
   - `WP,7:5;` - 6A (default)
   - `WP,7:6;` - 7A
   - `WP,7:7;` - 8A
   - `WP,7:8;` - 9A
   - `WP,7:9;` - 10A
   - `WP,7:A;` - 11A (BOXER800 only)
   - `WP,7:C;` - 12A (BOXER800 only)
   - `WP,7:E;` - 13A (BOXER800 only)

6. **Pedestrian Gate Function** (Parameter 8):
   - `WP,8:1;` - 3 seconds
   - `WP,8:2;` - 6 seconds (default)
   - `WP,8:3;` - 9 seconds
   - `WP,8:4;` - 12 seconds
   - `WP,8:5;` - 15 seconds
   - `WP,8:6;` - 18 seconds

7. **Torque Sensing Reaction** (Parameter A):
   - `WP,A:0;` - Stop
   - `WP,A:1;` - Stop + Reverse for 1 second
   - `WP,A:2;` - Stop + Reverse for 3 seconds
   - `WP,A:3;` - Stop + Reverse to end position

### Home Assistant Integration

#### Dashboard Configuration

After integrating your GatePro gate with ESPHome and Home Assistant, you can add a nice tile to your dashboard for easy control. A complete example tile configuration is available in the [examples directory](examples/home_assistant_tile_config.yaml).

Here's a basic tile configuration example:

```yaml
type: tile
entity: cover.driveway_gate_driveway_gate
name: Gate open / close
features:
  - type: cover-open-close
  - type: cover-position
tap_action:
  action: toggle
hold_action:
  action: more-info
show_entity_picture: true
vertical: true
features_position: bottom
```

This configuration creates a clean, user-friendly tile with open/close buttons and position indicator. Tapping the tile toggles the gate, while holding it opens the detailed information panel.

### Troubleshooting

1. **Gate Not Responding to Commands**:
   - Ensure UART logging is disabled (`baud_rate: 0`)
   - Verify the correct GPIO pins are used (GPIO1 for TX, GPIO3 for RX)
   - Check physical connections between ESP32 and the gate controller
   - Ensure the gate_src parameter matches your gate's source code

2. **Erratic Behavior**:
   - Run the AUTO LEARN process again
   - Check for any physical obstructions
   - Verify power supply is stable

3. **Communication Issues**:
   - Enable UART debug temporarily to see the communication
   - Verify baud rate is set to 9600
   - Check that TX/RX wires are not reversed

4. **Advanced Configuration**:
   - For advanced control options, check the `examples/gatepro_boxer_advanced_commands.yaml` file
   - This file contains numerous solutions for controlling various parameters:
     - Operating speed settings
     - Auto-closing timers
     - Deceleration distance and speed
     - Torque sensing and reaction settings
     - Pedestrian gate timing options
