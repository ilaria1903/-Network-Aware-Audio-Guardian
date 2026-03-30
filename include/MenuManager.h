#ifndef MENU_MANAGER_H
#define MENU_MANAGER_H

#include <Arduino.h>
#include "SecurityManager.h"

enum MenuMode {
  MENU_MAIN,
  MENU_ARMING,
  MENU_DISARMING,
  MENU_PASSWORD_SET
};

class MenuManager {
private:
  SecurityManager* secManager;
  MenuMode currentMode;
  unsigned long lastMenuActionTime;
  bool inMenu;
  
  static const int MENU_TIMEOUT = 60000;
  
  void printMainMenu();
  void printPasswordPrompt(const char* title);
  void handleMainMenuInput(char input);
  void handlePasswordInput(char input);
  
public:
  MenuManager(SecurityManager* manager);
  
  void initialize();
  void handleSerialInput(char input);
  void updateMenuTimeout();
  
  bool isInMenu() const;
  MenuMode getCurrentMenuMode() const;
};

#endif
