#pragma once

#include <string>
#include <unordered_map>

// Movement command enumeration
enum class MovementCommand {
    // Core movements
    MOVE_FORWARD,
    MOVE_BACKWARD,
    TURN_LEFT,
    TURN_RIGHT,
    STOP,
    SLOW_DOWN,
    SPEED_UP,
    HOLD_POSITION,

    // TARS-style
    ADVANCE,
    RETREAT,
    ROTATE_LEFT,
    ROTATE_RIGHT,
    HALT,
    STANDBY,
    NAVIGATING,
    ADJUST_COURSE,
    LAUGH,
    POSE,

    // Precise movements
    MOVE_CM_10,
    MOVE_CM_50,
    TURN_DEGREE_90_LEFT,
    TURN_DEGREE_90_RIGHT,
    ALIGN,
    CALIBRATE_MOVE,

    // Situational
    FOLLOW_ME,
    COME_HERE,
    GO_AWAY,
    CIRCLE,
    PATROL,
    RETURN_HOME,

    // Unknown
    UNKNOWN
};

// Convert command enum to canonical ASCII string to send over UART.
// Keep messages compact and human-readable for debugging.
static inline const char* MovementCommandToString(MovementCommand cmd) {
    switch (cmd) {
        // Core
        case MovementCommand::MOVE_FORWARD: return "MOVE_FORWARD";
        case MovementCommand::MOVE_BACKWARD: return "MOVE_BACKWARD";
        case MovementCommand::TURN_LEFT: return "TURN_LEFT";
        case MovementCommand::TURN_RIGHT: return "TURN_RIGHT";
        case MovementCommand::STOP: return "STOP";
        case MovementCommand::SLOW_DOWN: return "SLOW_DOWN";
        case MovementCommand::SPEED_UP: return "SPEED_UP";
        case MovementCommand::HOLD_POSITION: return "HOLD_POSITION";
        // TARS
        case MovementCommand::ADVANCE: return "ADVANCE";
        case MovementCommand::RETREAT: return "RETREAT";
        case MovementCommand::ROTATE_LEFT: return "ROTATE_LEFT";
        case MovementCommand::ROTATE_RIGHT: return "ROTATE_RIGHT";
        case MovementCommand::HALT: return "HALT";
        case MovementCommand::STANDBY: return "STANDBY";
        case MovementCommand::NAVIGATING: return "NAVIGATING";
        case MovementCommand::ADJUST_COURSE: return "ADJUST_COURSE";
        case MovementCommand::LAUGH: return "LAUGH";
        case MovementCommand::POSE: return "POSE";
        // Precise
        case MovementCommand::MOVE_CM_10: return "MOVE_CM_10";
        case MovementCommand::MOVE_CM_50: return "MOVE_CM_50";
        case MovementCommand::TURN_DEGREE_90_LEFT: return "TURN_DEGREE_90_LEFT";
        case MovementCommand::TURN_DEGREE_90_RIGHT: return "TURN_DEGREE_90_RIGHT";
        case MovementCommand::ALIGN: return "ALIGN";
        case MovementCommand::CALIBRATE_MOVE: return "CALIBRATE_MOVE";
        // Situational
        case MovementCommand::FOLLOW_ME: return "FOLLOW_ME";
        case MovementCommand::COME_HERE: return "COME_HERE";
        case MovementCommand::GO_AWAY: return "GO_AWAY";
        case MovementCommand::CIRCLE: return "CIRCLE";
        case MovementCommand::PATROL: return "PATROL";
        case MovementCommand::RETURN_HOME: return "RETURN_HOME";
        default: return "UNKNOWN";
    }
}

// Parse a string into a MovementCommand. The function is case-insensitive and accepts the canonical
// names from MovementCommandToString. If not found it returns MovementCommand::UNKNOWN.
static inline MovementCommand StringToMovementCommand(const std::string& s) {
    static const std::unordered_map<std::string, MovementCommand> map = {
        {"MOVE_FORWARD", MovementCommand::MOVE_FORWARD},
        {"MOVE_BACKWARD", MovementCommand::MOVE_BACKWARD},
        {"TURN_LEFT", MovementCommand::TURN_LEFT},
        {"TURN_RIGHT", MovementCommand::TURN_RIGHT},
        {"STOP", MovementCommand::STOP},
        {"SLOW_DOWN", MovementCommand::SLOW_DOWN},
        {"SPEED_UP", MovementCommand::SPEED_UP},
        {"HOLD_POSITION", MovementCommand::HOLD_POSITION},
        {"ADVANCE", MovementCommand::ADVANCE},
        {"RETREAT", MovementCommand::RETREAT},
        {"ROTATE_LEFT", MovementCommand::ROTATE_LEFT},
        {"ROTATE_RIGHT", MovementCommand::ROTATE_RIGHT},
        {"HALT", MovementCommand::HALT},
        {"STANDBY", MovementCommand::STANDBY},
        {"NAVIGATING", MovementCommand::NAVIGATING},
        {"ADJUST_COURSE", MovementCommand::ADJUST_COURSE},
        {"LAUGH", MovementCommand::LAUGH},
        {"POSE", MovementCommand::POSE},
        {"MOVE_CM_10", MovementCommand::MOVE_CM_10},
        {"MOVE_CM_50", MovementCommand::MOVE_CM_50},
        {"TURN_DEGREE_90_LEFT", MovementCommand::TURN_DEGREE_90_LEFT},
        {"TURN_DEGREE_90_RIGHT", MovementCommand::TURN_DEGREE_90_RIGHT},
        {"ALIGN", MovementCommand::ALIGN},
        {"CALIBRATE_MOVE", MovementCommand::CALIBRATE_MOVE},
        {"FOLLOW_ME", MovementCommand::FOLLOW_ME},
        {"COME_HERE", MovementCommand::COME_HERE},
        {"GO_AWAY", MovementCommand::GO_AWAY},
        {"CIRCLE", MovementCommand::CIRCLE},
        {"PATROL", MovementCommand::PATROL},
        {"RETURN_HOME", MovementCommand::RETURN_HOME},
    };

    std::string key = s;
    for (auto &c : key) c = toupper((unsigned char)c);
    auto it = map.find(key);
    if (it == map.end()) return MovementCommand::UNKNOWN;
    return it->second;
}

