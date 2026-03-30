#ifndef RADAR_SYSTEM_H
#define RADAR_SYSTEM_H

#include <Arduino.h>
#include <ESP32Servo.h>
#include <cmath>

#define SCAN_MIN 15
#define SCAN_MAX 165
#define SCAN_STEP 3
#define SCAN_DELAY 80
#define ANGLE_SLOTS ((SCAN_MAX - SCAN_MIN) / SCAN_STEP + 1)
#define SERVO_SETTLE_TIME 50
#define SERVO_NOISE_WINDOW 40

#define SERVO_PIN 25
#define TRIG_PIN 5
#define ECHO_PIN 18

struct RadarBaseline {
  int distance[ANGLE_SLOTS];
  int history[ANGLE_SLOTS][3];
  int historyIndex[ANGLE_SLOTS];
  bool initialized;
  bool frozen;
};

class RadarSystem {
private:
  Servo radarServo;
  RadarBaseline baseline;
  
  int currentAngle;
  int direction;
  unsigned long lastServoMove;
  
  int getAngleIndex(int angle);
  
public:
  RadarSystem();
  
  void initialize();
  void performBaselineScan();
  
  int getDistance();
  void setServoPosition(int angle);
  void updateServoPosition();
  
  float calculateRadarAnomaly(int angle, int distance);
  float calculateRadarConfidence(int angle, int distance, int measurements[]);
  
  bool canUpdateBaseline(int angle, int distance);
  void updateBaseline(int angle, int distance);
  void freezeBaseline();
  void unfreezeBaseline();
  bool isBaselineFrozen() const;
  bool isBaselineInitialized() const;
  
  int getCurrentAngle() const;
  bool isServoNoiseActive() const;
  unsigned long getLastServoMove() const;
};

#endif
