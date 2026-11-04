// ESP8266 dual-servo UART controller (Arduino) - 115200 version
// Logical convention:
//  - 0 = forward, 90 = HOME, 180 = back
//  - MAX_LOGICAL_ANGLE = 150 (safety cap)
//  - left servo: use logical as-is
//  - right servo: actual = 180 - logical

#include <Arduino.h>
#include <Servo.h>

#define BAUD_RATE 115200
#define LEFT_SERVO_PIN 5    // D1 (GPIO5) - change as needed
#define RIGHT_SERVO_PIN 4   // D2 (GPIO4) - change as needed
#define HOME_LOGICAL 90
#define MAX_LOGICAL_ANGLE 165   // Increased from 150 for wider range
#define STEP_DELAY_MS 8
#define HOLD_ATTACH true

Servo leftServo;
Servo rightServo;
int leftAngleActual = HOME_LOGICAL;
int rightAngleActual = HOME_LOGICAL;
volatile bool abortRequested = false;
volatile bool actionRunning = false;

String readLine() {
  static String buf = "";
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      String out = buf;
      buf = "";
      out.trim();
      return out;
    } else {
      buf += c;
      if (buf.length() > 256) buf = buf.substring(buf.length()-256);
    }
  }
  return String();
}

int clampLogical(int l) {
  if (l < 0) return 0;
  if (l > MAX_LOGICAL_ANGLE) return MAX_LOGICAL_ANGLE;
  return l;
}

int servoAngle(bool isRight, int logicalAngle) {
  logicalAngle = clampLogical(logicalAngle);
  // Both legs use same angle - right servo inversion happens at write() call
  return logicalAngle;
}

void stepMoveToBoth(int leftLogicalTarget, int rightLogicalTarget) {
  int tgtL = servoAngle(false, leftLogicalTarget);
  int tgtR = servoAngle(true, rightLogicalTarget);

  int curL = leftAngleActual;
  int curR = rightAngleActual;

  int steps = max(abs(tgtL - curL), abs(tgtR - curR));
  if (steps == 0) return;

  for (int s = 1; s <= steps; ++s) {
    if (abortRequested) { actionRunning = false; return; }
    int nextL = curL + ((tgtL - curL) * s + steps/2) / steps;
    int nextR = curR + ((tgtR - curR) * s + steps/2) / steps;
    leftServo.write(nextL);
    rightServo.write(nextR);  // Use same angle for both - symmetric movement
    leftAngleActual = nextL;
    rightAngleActual = nextR;
    delay(STEP_DELAY_MS);
  }
}

void gotoSequence(const std::vector<std::pair<int,int>>& seq, int pauseMs = 220) {
  actionRunning = true;
  for (auto &p : seq) {
    if (abortRequested) break;
    stepMoveToBoth(p.first, p.second);
    if (abortRequested) break;
    delay(pauseMs);
  }
  actionRunning = false;
}

void sendAck(const String& cmd) {
  Serial.print("ACK:"); Serial.println(cmd);
}
void sendDone(const String& cmd) {
  Serial.print("DONE:"); Serial.println(cmd);
}
void sendInterrupted(const String& cmd) {
  Serial.print("INTERRUPTED:"); Serial.println(cmd);
}

void do_MOVE_FORWARD()      { gotoSequence({{160,160}}); }  // 160 for more aggressive forward
void do_MOVE_BACKWARD()     { gotoSequence({{20,20}}); }    // 20 for more aggressive backward
void do_TURN_LEFT()         { gotoSequence({{20,160}}); }
void do_TURN_RIGHT()        { gotoSequence({{160,20}}); }
void do_STOP()              { stepMoveToBoth(HOME_LOGICAL, HOME_LOGICAL); }
void do_SLOW_DOWN()         { gotoSequence({{115,115}}); }
void do_SPEED_UP()          { gotoSequence({{165,165}}); }  // Increased to 165 for max range
void do_HOLD_POSITION()     { stepMoveToBoth(HOME_LOGICAL, HOME_LOGICAL); }
void do_ADVANCE()           { gotoSequence({{165,165}}); }
void do_RETREAT()           { gotoSequence({{15,15}}); }
void do_ROTATE_LEFT()       { gotoSequence({{135,135}}); }
void do_ROTATE_RIGHT()      { gotoSequence({{45,45}}); }
void do_HALT()              { stepMoveToBoth(HOME_LOGICAL, HOME_LOGICAL); }
void do_STANDBY()           { gotoSequence({{85,85}}); }
void do_NAVIGATING() { while (!abortRequested) { stepMoveToBoth(140,140); if (abortRequested) break; delay(180); stepMoveToBoth(HOME_LOGICAL, HOME_LOGICAL); if (abortRequested) break; delay(180); } }
void do_ADJUST_COURSE()     { gotoSequence({{100,100}}); }
void do_MOVE_CM_10()        { gotoSequence({{115,115}}); }
void do_MOVE_CM_50()        { gotoSequence({{165,165}}); }
void do_TURN_DEGREE_90_LEFT(){ gotoSequence({{140,140}}); }
void do_TURN_DEGREE_90_RIGHT(){gotoSequence({{40,40}}); }
void do_ALIGN()             { gotoSequence({{95,95}}); }
void do_CALIBRATE_MOVE()    { gotoSequence({{75,75}}); }  // Both legs symmetric now
void do_FOLLOW_ME() { while (!abortRequested) { stepMoveToBoth(145,145); if (abortRequested) break; delay(220); stepMoveToBoth(HOME_LOGICAL, HOME_LOGICAL); if (abortRequested) break; delay(220); } }
void do_COME_HERE() { while (!abortRequested) { stepMoveToBoth(50,50); if (abortRequested) break; delay(220); stepMoveToBoth(HOME_LOGICAL, HOME_LOGICAL); if (abortRequested) break; delay(220); } }
void do_GO_AWAY()           { gotoSequence({{165,165}}); }
void do_CIRCLE() { while (!abortRequested) { stepMoveToBoth(160,160); if (abortRequested) break; delay(180); stepMoveToBoth(80,100); if (abortRequested) break; delay(180); } }
void do_PATROL() { while (!abortRequested) { gotoSequence({{145,145}}); if (abortRequested) break; gotoSequence({{35,35}}); if (abortRequested) break; } }
void do_RETURN_HOME()       { stepMoveToBoth(HOME_LOGICAL, HOME_LOGICAL); }
void do_UNKNOWN()           { stepMoveToBoth(HOME_LOGICAL, HOME_LOGICAL); }

void handleCommand(const String& raw) {
  String s = raw;
  s.trim();
  for (size_t i=0;i<s.length();++i) s[i] = toupper((unsigned char)s[i]);
  if (actionRunning) {
    abortRequested = true;
    delay(10);
  }
  abortRequested = false;
  sendAck(s);
  if (s == "MOVE_FORWARD")            do_MOVE_FORWARD();
  else if (s == "MOVE_BACKWARD")      do_MOVE_BACKWARD();
  else if (s == "TURN_LEFT")          do_TURN_LEFT();
  else if (s == "TURN_RIGHT")         do_TURN_RIGHT();
  else if (s == "STOP")               do_STOP();
  else if (s == "SLOW_DOWN")          do_SLOW_DOWN();
  else if (s == "SPEED_UP")           do_SPEED_UP();
  else if (s == "HOLD_POSITION")      do_HOLD_POSITION();
  else if (s == "ADVANCE")            do_ADVANCE();
  else if (s == "RETREAT")            do_RETREAT();
  else if (s == "ROTATE_LEFT")        do_ROTATE_LEFT();
  else if (s == "ROTATE_RIGHT")       do_ROTATE_RIGHT();
  else if (s == "HALT")               do_HALT();
  else if (s == "STANDBY")            do_STANDBY();
  else if (s == "NAVIGATING")         do_NAVIGATING();
  else if (s == "ADJUST_COURSE")      do_ADJUST_COURSE();
  else if (s == "MOVE_CM_10")         do_MOVE_CM_10();
  else if (s == "MOVE_CM_50")         do_MOVE_CM_50();
  else if (s == "TURN_DEGREE_90_LEFT")do_TURN_DEGREE_90_LEFT();
  else if (s == "TURN_DEGREE_90_RIGHT")do_TURN_DEGREE_90_RIGHT();
  else if (s == "ALIGN")              do_ALIGN();
  else if (s == "CALIBRATE_MOVE")     do_CALIBRATE_MOVE();
  else if (s == "FOLLOW_ME")          do_FOLLOW_ME();
  else if (s == "COME_HERE")          do_COME_HERE();
  else if (s == "GO_AWAY")            do_GO_AWAY();
  else if (s == "CIRCLE")             do_CIRCLE();
  else if (s == "PATROL")             do_PATROL();
  else if (s == "RETURN_HOME")        do_RETURN_HOME();
  else if (s == "UNKNOWN")            do_UNKNOWN();
  else {
    Serial.print("ERR:UNKNOWN:");
    Serial.println(raw);
    do_UNKNOWN();
  }

  if (abortRequested) sendInterrupted(s);
  else sendDone(s);
  abortRequested = false;
}

void setup() {
  Serial.begin(BAUD_RATE);
  leftServo.attach(LEFT_SERVO_PIN);
  rightServo.attach(RIGHT_SERVO_PIN);
  leftAngleActual = servoAngle(false, HOME_LOGICAL);
  rightAngleActual = servoAngle(true, HOME_LOGICAL);
  leftServo.write(leftAngleActual);
  rightServo.write(rightAngleActual);  // Same angle for both
  delay(200);
  Serial.print("BAUD:"); Serial.println(BAUD_RATE);
  Serial.println("READY");
}

void loop() {
  String cmd = readLine();
  if (cmd.length()) {
    handleCommand(cmd);
    if (HOLD_ATTACH) {
      leftServo.write(leftAngleActual);
      rightServo.write(rightAngleActual);  // Same angle for both
    }
  }
  delay(10);
}
