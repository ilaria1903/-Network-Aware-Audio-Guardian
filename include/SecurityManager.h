#ifndef SECURITY_MANAGER_H
#define SECURITY_MANAGER_H

#include <Arduino.h>
#include <EEPROM.h>
#include <cstring>

// ==================== SECURITY STATES ====================
enum SecurityState {
  SECURITY_DISARMED,      // Sistem dezactivat
  SECURITY_ARMING,        // Așteptă parolă
  SECURITY_ARMED,         // Sistem activ
  SECURITY_THREAT,        // Amenințare detectată
  SECURITY_ALARM          // Alarmă confirmată
};

// ==================== PASSWORD MANAGEMENT ====================
#define PASSWORD_LENGTH 6
#define PASSWORD_EEPROM_ADDRESS 0
#define MAX_PASSWORD_ATTEMPTS 3
#define PASSWORD_TIMEOUT 30000

// ==================== SECURITY STATUS ====================
struct SecurityStatus {
  SecurityState currentState;
  SecurityState previousState;
  unsigned long stateEntryTime;
  unsigned long lastStateChangeTime;
  
  char passwordAttempt[PASSWORD_LENGTH + 1];
  int passwordAttemptIndex;
  int failedAttempts;
  unsigned long passwordTimeoutStart;
  bool passwordTimeoutActive;
  
  unsigned long disarmTime;
  bool disarmSuccessful;
};

struct PasswordData {
  char password[PASSWORD_LENGTH + 1];
  bool isSet;
};

// ==================== SECURITY MANAGER CLASS ====================
class SecurityManager {
private:
  PasswordData passwordData;
  SecurityStatus secStatus;
  
  void printSecurityStateName(SecurityState state);
  
public:
  SecurityManager();
  
  void initializeEEPROM();
  void setNewPassword(const char* newPassword);
  bool verifyPassword(const char* attempt);
  void addCharToPassword(char c);
  void clearPasswordAttempt();
  
  bool isDisarmSuccessful();
  void resetDisarmFlag();
  bool isPasswordTimeoutActive();
  unsigned long getPasswordTimeoutRemaining();
  
  void setSecurityState(SecurityState newState);
  SecurityState getSecurityState() const;
  SecurityState getPreviousSecurityState() const;
  
  void printSecurityStateChange();
  void printCurrentSecurityStatus();
};

#endif
