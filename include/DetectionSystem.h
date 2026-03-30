#ifndef DETECTION_SYSTEM_H
#define DETECTION_SYSTEM_H

#include <Arduino.h>
#include <cmath>

enum SystemState {
  STATE_IDLE,
  STATE_SUSPECT,
  STATE_VERIFY,
  STATE_ALARM,
  STATE_COOLDOWN
};

#define LED_PIN 2
#define COOLDOWN_DURATION 10000
#define ALARM_TIMEOUT 60000

struct SensorConfidence {
  float radar;
  float audio;
  float radarFiltered;
  float audioFiltered;
};

struct ScoringStateData {
  float totalScore;
  float radarScore;
  float audioScore;
  unsigned long lastDecayUpdate;
  unsigned long eventHistory[10];
  int eventHistoryIndex;
};

struct OccupiedZone {
  int centerAngle;
  int angleSpread;
  int avgDistance;
  unsigned long lastSeen;
  int detectionCount;
  bool active;
};

class DetectionSystem {
private:
  SystemState currentState;
  SystemState previousState;
  unsigned long stateEntryTime;
  unsigned long alarmTriggerTime;
  
  SensorConfidence confidence;
  ScoringStateData scoring;
  OccupiedZone trackedZone;
  
  float saturate(float x, float k);
  void printStateTransitionDebug(SystemState from, SystemState to);
  void updateLED();
  
public:
  DetectionSystem();
  
  void addRadarEvent(float intensity);
  void addAudioEvent(float intensity);
  void updateScore();
  void applyDecay();
  
  void updateStateMachine();
  void handleStateIdle();
  void handleStateSuspect();
  void handleStateVerify();
  void handleStateAlarm();
  void handleStateCooldown();
  
  void updateTrackedZone(int angle, int distance);
  bool trackZone();
  
  SystemState getCurrentState() const;
  float getTotalScore() const;
  float getRadarScore() const;
  float getAudioScore() const;
  bool isAlarmActive() const;
  
  void setRadarConfidence(float conf);
  void setAudioConfidence(float conf);
  float getRadarConfidenceFiltered() const;
  float getAudioConfidenceFiltered() const;
  
  void freezeBaseline();
  void unfreezeBaseline();
  bool shouldFreezeBaseline() const;
  
  void printCurrentStatus();
  void printDetectionEvent(const char* type, float intensity, float confidence, float scoreAdded);
};

#endif
