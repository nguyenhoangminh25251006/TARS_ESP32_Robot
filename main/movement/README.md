# Movement UART (xiaozhi -> ESP8266)

This module provides a simple UART-based movement command sender for Xiaozhi (ESP32). The ESP8266 (robot) listens on UART for text commands and handles servos/motors.

## Files
- `movement_commands.h` — enum for supported commands and conversion helpers
- `uart_movement.h` / `uart_movement.cc` — UART driver wrapper and send API
- `voice_mapping.h` — helper that maps Vietnamese/English voice text to `MovementCommand`

## Protocol
- Commands are sent as ASCII lines terminated with `\n`.
- Canonical command strings are e.g. `MOVE_FORWARD`, `TURN_LEFT`, `MOVE_CM_10`, `FOLLOW_ME`, etc.
- If unknown command name is provided, the raw string will be sent as-is (useful for device-specific commands).

Example payloads sent on the wire:
- `MOVE_FORWARD\n`
- `TURN_DEGREE_90_LEFT\n`
- `FOLLOW_ME\n`

**Note:** UART-based movement support (UartMovement / external ESP8266) is deprecated. The preferred and supported approach is the onboard `ServoController` (default pins: `SERVO_LEFT_PIN = GPIO18`, `SERVO_RIGHT_PIN = GPIO19`).

Use the ServoController API or the `self.movement.send` tool to perform canonical movement commands (e.g., `MOVE_FORWARD`, `TURN_LEFT`). Example usage is:

```cpp
// Initialize (done in Application::Initialize())
ServoController::GetInstance().Initialize();

// Perform command
ServoController::GetInstance().PerformCommand(MovementCommand::MOVE_FORWARD);
```

**Safety:** When running servos, use a dedicated servo power supply and common ground.

**Compatibility:** For backward compatibility the UART implementation is still present in the tree (deprecated) but is not included in the default build and will not be used by normal flows.

## Servo Controller (ESP32)

Starting with the ServoController implementation, the ESP32 can perform movement commands locally (no ESP8266 required). The controller uses the LEDC PWM peripheral to drive two hobby servos (left and right). By default the pins are:
- `SERVO_LEFT_PIN` = GPIO18
- `SERVO_RIGHT_PIN` = GPIO19

These can be changed by defining the macros at build time or editing the header. Initialize with:

```cpp
ServoController::GetInstance().Initialize();
```

Use `PerformCommand(MovementCommand cmd)` to perform short moves (the MCP tool will schedule these automatically when the command is recognized).

## Voice integration
Use `VoiceTextToCommand()` in `voice_mapping.h` to convert recognized Vietnamese/English phrases into `MovementCommand` values and then call `SendCommand()`.

## Notes
- This module intentionally keeps the protocol simple (ASCII commands) to make the ESP8266 side easier to implement and debug.
- If you need reliability guarantees, consider adding sequence numbers, acknowledgements, or checksums on top of this protocol.
