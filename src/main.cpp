#include <Arduino.h>
#include <ESP32Servo.h>
#include "arduinoFFT.h"
#include <EEPROM.h>

// ==================== CONFIGURARE PINI ====================
const int SERVO_PIN = 25;
const int TRIG_PIN = 5;
const int ECHO_PIN = 18;
const int SOUND_SENSOR_PIN = 36;
const int LED_PIN = 2;

// ==================== PARAMETRI RADAR ====================
const int SCAN_MIN = 15;
const int SCAN_MAX = 165;
const int SCAN_STEP = 3;
const int SCAN_DELAY = 80;
const int ANGLE_SLOTS = (SCAN_MAX - SCAN_MIN) / SCAN_STEP + 1;
static int anomalyCount[ANGLE_SLOTS] = {0};
static int normalCount[ANGLE_SLOTS] = {0};

// ==================== PARAMETRI AUDIO ====================
#define SAMPLES 128
#define SAMPLING_FREQUENCY 8000
const int BASELINE_SAMPLES = 200;
const float THRESHOLD_MULTIPLIER = 1.0;

const int VOICE_LOW = 300;
const int VOICE_HIGH = 3400;
const int WHISPER_LOW = 1000;
const int WHISPER_HIGH = 4000;
const int TRAFFIC_HIGH = 500;

const float VOICE_ENERGY_THRESHOLD = 0.25;
const float WHISPER_RATIO_THRESHOLD = 0.20;
const float TRAFFIC_REJECT_THRESHOLD = 0.4;

unsigned long whisperStartTime = 0;
bool whisperActive = false;

// ==================== PARAMETRI TIMING ====================
const int SERVO_SETTLE_TIME = 50;
const int SERVO_NOISE_WINDOW = 40;
const int AUDIO_SAMPLE_TIME = 50;
const int AUDIO_FFT_TIME = 150;
const int COOLDOWN_DURATION = 10000;
const int ALARM_TIMEOUT = 60000;

// ==================== EEPROM ====================
#define EEPROM_SIZE 64
#define EEPROM_PASSWORD_ADDR 0
#define EEPROM_PASSWORD_LEN 16
#define EEPROM_MAGIC_ADDR 30
#define EEPROM_MAGIC_VALUE 0xA5

// ==================== SYSTEM MODE ====================
enum SystemMode {
  MODE_DISARMED,
  MODE_ARMED
};

SystemMode systemMode = MODE_DISARMED;

// ==================== MENIU STATE ====================
enum MenuState {
  MENU_IDLE,
  MENU_WAITING_PASSWORD,
  MENU_WAITING_NEW_PASSWORD,
  MENU_WAITING_CONFIRM_PASSWORD,
  MENU_WAITING_DISARM_PASSWORD
};

MenuState menuState = MENU_IDLE;
String menuInput = "";
String pendingNewPassword = "";

char storedPassword[EEPROM_PASSWORD_LEN + 1] = "1234";  // parola default

// ==================== FSM STATES ====================
enum SystemState {
  STATE_IDLE,
  STATE_SUSPECT,
  STATE_VERIFY,
  STATE_ALARM,
  STATE_COOLDOWN
};

// ==================== STRUCTURI DATE ====================
struct RadarBaseline {
  int distance[ANGLE_SLOTS];
  int history[ANGLE_SLOTS][3];
  int historyIndex[ANGLE_SLOTS];
  bool initialized;
  bool frozen;
};

struct SensorConfidence {
  float radar;
  float audio;
  float radarFiltered;
  float audioFiltered;
};

struct OccupiedZone {
  int centerAngle;
  int angleSpread;
  int avgDistance;
  unsigned long lastSeen;
  int detectionCount;
  bool active;
};

struct AudioState {
  int baselineNoise;
  int dynamicThreshold;
  int noiseStdDev;
  unsigned long lastValidSample;
};

struct ScoringState {
  float totalScore;
  float radarScore;
  float audioScore;
  unsigned long lastDecayUpdate;
  unsigned long eventHistory[10];
  int eventHistoryIndex;
};

struct AudioProximity {
  bool isLocalSound;
  bool isPersistentLocal;
  float signalStrength;
  int peakValue;
};

// ==================== OBIECTE GLOBALE ====================
Servo radarServo;
ArduinoFFT<double> FFT = ArduinoFFT<double>();

RadarBaseline baseline = {.initialized = false, .frozen = false};
SensorConfidence confidence = {0.5, 0.5, 0.5, 0.5};
OccupiedZone trackedZone = {0, 15, 0, 0, 0, false};
AudioState audioState = {0};
ScoringState scoring = {0.0, 0.0, 0.0, 0, {0}, 0};
AudioProximity audioProximity = {false, false, 0.0, 0};

SystemState currentState = STATE_IDLE;
int currentAngle = SCAN_MIN;
int direction = 1;
unsigned long lastServoMove = 0;
unsigned long stateEntryTime = 0;
unsigned long alarmTriggerTime = 0;

unsigned int samplingPeriod;
double vReal[SAMPLES];
double vImag[SAMPLES];

bool loggingEnabled = true;

// ==================== DECLARAȚII FUNCȚII ====================
int getDistance();
void performBaselineScan();
int getAngleIndex(int angle);
bool canUpdateBaseline(int angle, int distance);
void updateBaseline(int angle, int distance);
float calculateRadarAnomaly(int angle, int distance);
float calculateRadarConfidence(int angle, int distance, int measurements[]);
bool isServoHealthy();

void calibrateAudioBaseline();
void recalibrateAudio();
bool isSoundPresent();
String analyzeSoundType();
float calculateBandEnergy(int freqLow, int freqHigh);
float calculateAudioConfidence();
bool isServoNoiseActive();

void addRadarEvent(float intensity);
void addAudioEvent(float intensity);
void updateScore();
void applyDecay();
float saturate(float x, float k);
void logScoreState();

void updateStateMachine();
void handleStateIdle();
void handleStateSuspect();
void handleStateVerify();
void handleStateAlarm();
void handleStateCooldown();

void updateTrackedZone(int angle, int distance);
bool trackZone();

void setServoPosition(int angle);
bool verifyServoPosition(int targetAngle);
void printSystemStatus();
void printStateTransition(SystemState from, SystemState to);
void printDetectionEvent(String type, float intensity, float confidence, float scoreAdded);
void printScoreBar();
void printCurrentStatus();

// ==================== DECLARAȚII FUNCȚII MENIU ====================
void loadPasswordFromEEPROM();
void savePasswordToEEPROM(const char* pwd);
void printMainMenu();
void printDisarmPrompt();
void handleSerialMenu();
void doArm();
void doDisarm();

// ==================== IMPLEMENTARE EEPROM & PAROLA ====================

void loadPasswordFromEEPROM() {
  EEPROM.begin(EEPROM_SIZE);
  uint8_t magic = EEPROM.read(EEPROM_MAGIC_ADDR);
  if (magic == EEPROM_MAGIC_VALUE) {
    for (int i = 0; i < EEPROM_PASSWORD_LEN; i++) {
      storedPassword[i] = (char)EEPROM.read(EEPROM_PASSWORD_ADDR + i);
    }
    storedPassword[EEPROM_PASSWORD_LEN] = '\0';
    // Trim la primul null
    for (int i = 0; i < EEPROM_PASSWORD_LEN; i++) {
      if (storedPassword[i] == '\0') break;
    }
    Serial.print("✓ Parolă încărcată din EEPROM: ");
    for (int i = 0; i < (int)strlen(storedPassword); i++) Serial.print('*');
    Serial.println();
  } else {
    // Prima rulare - salvăm parola default
    savePasswordToEEPROM("1234");
    Serial.println("✓ Parolă default setată: 1234");
  }
}

void savePasswordToEEPROM(const char* pwd) {
  EEPROM.begin(EEPROM_SIZE);
  int len = strlen(pwd);
  if (len > EEPROM_PASSWORD_LEN) len = EEPROM_PASSWORD_LEN;
  for (int i = 0; i < len; i++) {
    EEPROM.write(EEPROM_PASSWORD_ADDR + i, (uint8_t)pwd[i]);
  }
  for (int i = len; i < EEPROM_PASSWORD_LEN; i++) {
    EEPROM.write(EEPROM_PASSWORD_ADDR + i, 0);
  }
  EEPROM.write(EEPROM_MAGIC_ADDR, EEPROM_MAGIC_VALUE);
  EEPROM.commit();
  strncpy(storedPassword, pwd, EEPROM_PASSWORD_LEN);
  storedPassword[EEPROM_PASSWORD_LEN] = '\0';
}

void printMainMenu() {
  Serial.println("\n╔═════════════════════════════════════════════╗");
  if (systemMode == MODE_DISARMED) {
    Serial.println("║  SISTEM:  DEZARMAT                        ║");
  } else {
    Serial.println("║  SISTEM:  ARMAT                           ║");
  }
  Serial.println("╠═════════════════════════════════════════════╣");
  Serial.println("║  Comenzi disponibile:                       ║");
  if (systemMode == MODE_DISARMED) {
    Serial.println("║  [arm]          - Armează sistemul          ║");
  } else {
    Serial.println("║  [disarm]       - Dezarmează (necesită pwd) ║");
  }
  Serial.println("║  [setpassword]  - Schimbă parola            ║");
  Serial.println("║  [menu]         - Afișează acest meniu       ║");
  Serial.println("╚═════════════════════════════════════════════╝\n");
}

void printDisarmPrompt() {
  Serial.println("\n┌─────────────────────────────────────────────┐");
  Serial.println("│   DEZARMARE - Introdu parola:             │");
  Serial.println("└─────────────────────────────────────────────┘");
  Serial.print("> ");
}

void doArm() {
  if (systemMode == MODE_ARMED) {
    Serial.println("  Sistemul este deja armat.");
    return;
  }
  
  systemMode = MODE_ARMED;
  
  // Reset complet stare detecție
  scoring.totalScore = 0;
  scoring.radarScore = 0;
  scoring.audioScore = 0;
  scoring.lastDecayUpdate = millis();
  currentState = STATE_IDLE;
  stateEntryTime = millis();
  baseline.frozen = false;
  trackedZone.active = false;
  
  Serial.println("\n╔═════════════════════════════════════════════╗");
  Serial.println("║   SISTEM ARMAT                            ║");
  Serial.println("║  Calibrare audio în curs...                 ║");
  Serial.println("╚═════════════════════════════════════════════╝\n");
  
  calibrateAudioBaseline();
  Serial.print("✓ Audio threshold: ");
  Serial.println(audioState.dynamicThreshold);
  
  Serial.println("\n Învățare baseline radar (3 cicluri)...");
  Serial.println("  Stai liniștit!\n");
  delay(2000);
  
  performBaselineScan();
  
  Serial.println("\n Sistem armat - Monitorizare activă\n");
}

void doDisarm() {
  if (systemMode == MODE_DISARMED) {
    Serial.println("  Sistemul este deja dezarmat.");
    return;
  }
  
  systemMode = MODE_DISARMED;
  
  // Oprire completă detecție
  currentState = STATE_IDLE;
  scoring.totalScore = 0;
  scoring.radarScore = 0;
  scoring.audioScore = 0;
  baseline.frozen = false;
  trackedZone.active = false;
  digitalWrite(LED_PIN, LOW);
  
  Serial.println("\n╔═════════════════════════════════════════════╗");
  Serial.println("║   SISTEM DEZARMAT                         ║");
  Serial.println("║  Monitorizare oprită.                       ║");
  Serial.println("╚═════════════════════════════════════════════╝\n");
  
  printMainMenu();
}

void handleSerialMenu() {
  while (Serial.available()) {
    char c = Serial.read();
    
    // Echo caracter (mascat cu * la introducere parole)
    bool isPasswordInput = (menuState == MENU_WAITING_PASSWORD ||
                            menuState == MENU_WAITING_NEW_PASSWORD ||
                            menuState == MENU_WAITING_CONFIRM_PASSWORD ||
                            menuState == MENU_WAITING_DISARM_PASSWORD);
    
    if (c == '\r') continue;

if (c != '\n') {
  menuInput += c;
  continue;
}

if (isPasswordInput) {
  for (int i = 0; i < (int)menuInput.length(); i++) Serial.print('*');
}
Serial.println();
    menuInput.trim();
    
    // -------- MENIU IDLE - așteptăm comenzi --------
    if (menuState == MENU_IDLE) {
      String cmd = menuInput;
      cmd.toLowerCase();
      
      if (cmd == "arm") {
        if (systemMode == MODE_ARMED) {
          Serial.println("  Sistemul este deja armat.");
          printMainMenu();
        } else {
          doArm();
          printMainMenu();
        }
      }
      else if (cmd == "disarm") {
        if (systemMode == MODE_DISARMED) {
          Serial.println("  Sistemul este deja dezarmat.");
          printMainMenu();
        } else {
          menuState = MENU_WAITING_DISARM_PASSWORD;
          printDisarmPrompt();
        }
      }
      else if (cmd == "setpassword") {
        if (systemMode == MODE_ARMED) {
          // Cere parola veche mai întâi
          menuState = MENU_WAITING_PASSWORD;
          Serial.println("\n Introdu parola curentă:");
          Serial.print("> ");
        } else {
          // În modul dezarmat nu cerem parola veche
          menuState = MENU_WAITING_NEW_PASSWORD;
          Serial.println("\n Introdu parola nouă (max 16 caractere):");
          Serial.print("> ");
        }
      }
      else if (cmd == "menu") {
        printMainMenu();
      }
      else if (cmd.length() > 0) {
        Serial.print(" Comandă necunoscută: ");
        Serial.println(menuInput);
        printMainMenu();
      }
    }
    
    // -------- WAITING: parola veche (pentru setpassword când e armat) --------
    else if (menuState == MENU_WAITING_PASSWORD) {
      if (menuInput == String(storedPassword)) {
        menuState = MENU_WAITING_NEW_PASSWORD;
        Serial.println("✓ Parolă corectă.");
        Serial.println("\n Introdu parola nouă (max 16 caractere):");
        Serial.print("> ");
      } else {
        Serial.println(" Parolă greșită! Operațiune anulată.");
        menuState = MENU_IDLE;
        printMainMenu();
      }
    }
    
    // -------- WAITING: parolă nouă --------
    else if (menuState == MENU_WAITING_NEW_PASSWORD) {
      if (menuInput.length() == 0) {
        Serial.println(" Parola nu poate fi goală!");
        menuState = MENU_IDLE;
        printMainMenu();
      } else if (menuInput.length() > EEPROM_PASSWORD_LEN) {
        Serial.print(" Parola prea lungă! Max ");
        Serial.print(EEPROM_PASSWORD_LEN);
        Serial.println(" caractere.");
        menuState = MENU_IDLE;
        printMainMenu();
      } else {
        pendingNewPassword = menuInput;
        menuState = MENU_WAITING_CONFIRM_PASSWORD;
        Serial.println("\n Confirmă parola nouă:");
        Serial.print("> ");
      }
    }
    
    // -------- WAITING: confirmare parolă nouă --------
    else if (menuState == MENU_WAITING_CONFIRM_PASSWORD) {
      if (menuInput == pendingNewPassword) {
        savePasswordToEEPROM(menuInput.c_str());
        Serial.println(" Parolă salvată cu succes în EEPROM!");
        menuState = MENU_IDLE;
        pendingNewPassword = "";
        printMainMenu();
      } else {
        Serial.println(" Parolele nu coincid! Operațiune anulată.");
        menuState = MENU_IDLE;
        pendingNewPassword = "";
        printMainMenu();
      }
    }
    
    // -------- WAITING: parolă dezarmare --------
    else if (menuState == MENU_WAITING_DISARM_PASSWORD) {
      if (menuInput == String(storedPassword)) {
        menuState = MENU_IDLE;
        doDisarm();
      } else {
        Serial.println(" Parolă greșită! Dezarmare refuzată.");
        menuState = MENU_IDLE;
        // Dacă era alarmă, continuă
        if (currentState == STATE_ALARM) {
          Serial.println(" Alarma continuă!");
          Serial.println("Tastează [disarm] pentru a încerca din nou.");
        } else {
          printMainMenu();
        }
      }
    }
    
    menuInput = "";
  }
}

// ==================== IMPLEMENTĂRI FUNCȚII PRINT ====================

void printStateTransition(SystemState from, SystemState to) {
  Serial.println("\n╔═══════════════════════════════════════════════╗");
  Serial.print("║  TRANZIȚIE: ");
  
  switch(from) {
    case STATE_IDLE: Serial.print("IDLE       "); break;
    case STATE_SUSPECT: Serial.print("SUSPECT    "); break;
    case STATE_VERIFY: Serial.print("VERIFY     "); break;
    case STATE_ALARM: Serial.print("ALARM      "); break;
    case STATE_COOLDOWN: Serial.print("COOLDOWN   "); break;
  }
  
  Serial.print(" → ");
  
  switch(to) {
    case STATE_IDLE: 
      Serial.println("IDLE          ║");
      Serial.println("║   Normal - Monitorizare pasivă              ║");
      break;
    case STATE_SUSPECT: 
      Serial.println("SUSPECT       ║");
      Serial.println("║    CEVA SUSPECT - Verificare activă         ║");
      break;
    case STATE_VERIFY: 
      Serial.println("VERIFY        ║");
      Serial.println("║   VERIFICARE INTENSIVĂ - Tracking activ    ║");
      break;
    case STATE_ALARM: 
      Serial.println("ALARM         ║");
      Serial.println("║   INTRUZIUNE CONFIRMATĂ        ║");
      Serial.println("║  Tastează [disarm] + parolă pentru a opri    ║");
      break;
    case STATE_COOLDOWN: 
      Serial.println("COOLDOWN      ║");
      Serial.println("║  ⏸  Pauză după alarmă                       ║");
      break;
  }
  
  Serial.println("╚═══════════════════════════════════════════════╝\n");
}

void printDetectionEvent(String type, float intensity, float confidence, float scoreAdded) {
  Serial.print("┌─ ");
  
  if (type == "RADAR") {
    Serial.print(" RADAR");
  } else {
    Serial.print(" AUDIO");
  }
  
  Serial.println(" DETECTARE ─────────────────");
  Serial.print("│  Intensitate: ");
  Serial.print(intensity, 1);
  Serial.print(" | Încredere: ");
  Serial.print(confidence * 100, 0);
  Serial.println("%");
  Serial.print("│  Scor adăugat: +");
  Serial.print(scoreAdded, 2);
  Serial.print(" → Total: ");
  Serial.println(scoring.totalScore, 1);
  Serial.println("└────────────────────────────────────────");
}

void printScoreBar() {
  Serial.print("[");
  
  int bars = (int)(scoring.totalScore * 3);  // Scalare pentru display
  bars = constrain(bars, 0, 30);
  
  for (int i = 0; i < 30; i++) {
    if (i < bars) {
      if (i < 8) Serial.print("▓");
      else if (i < 16) Serial.print("▓");
      else if (i < 24) Serial.print("▓");
      else Serial.print("█");
    } else {
      Serial.print("░");
    }
  }
  
  Serial.print("] ");
  Serial.print(scoring.totalScore, 1);
  Serial.print("/10");
  
  if (scoring.totalScore < 0.8) {
    Serial.println("  NORMAL");
  } else if (scoring.totalScore < 2.5) {
    Serial.println("   SUSPECT");
  } else if (scoring.totalScore < 5.0) {
    Serial.println("  VERIFICARE");
  } else {
    Serial.println("  ALARMĂ!");
  }
}

void printCurrentStatus() {
  Serial.println("\n┏━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┓");
  Serial.print("┃  STARE CURENTĂ: ");
  
  switch(currentState) {
    case STATE_IDLE:
      Serial.println("IDLE (Normal)              ┃");
      Serial.println("┃  LED: Stins                           ┃");
      break;
      
    case STATE_SUSPECT:
      Serial.println("SUSPECT (Activat)          ┃");
      Serial.println("┃  LED: Clipire lentă  (500ms)         ┃");
      Serial.println("┃  Baseline: ÎNGHEȚAT                   ┃");
      break;
      
    case STATE_VERIFY:
      Serial.println("VERIFY (Verificare)        ┃");
      Serial.println("┃  LED: Clipire rapidă  (200ms)        ┃");
      Serial.println("┃  Baseline: ÎNGHEȚAT                   ┃");
      break;
      
    case STATE_ALARM: {
      Serial.println("ALARM (Intruziune!)        ┃");
      Serial.println("┃  LED: Constant aprins                 ┃");
      Serial.println("┃   INTRUZIUNE CONFIRMATĂ             ┃");
      unsigned long alarmDuration = (millis() - alarmTriggerTime) / 1000;
      Serial.print("┃  Durată alarmă: ");
      Serial.print(alarmDuration);
      Serial.println(" secunde                  ┃");
      Serial.println("┃  Tastează [disarm] pentru dezarmare     ┃");
      break;
    }
    
    case STATE_COOLDOWN:
      Serial.println("COOLDOWN (Pauză)           ┃");
      break;
  }
  
  Serial.println("┃                                         ┃");
  Serial.print("┃  Scor: ");
  printScoreBar();
  Serial.println("┃                                         ┃");
  Serial.print("┃  Radar: ");
  Serial.print(scoring.radarScore, 1);
  Serial.print(" | Audio: ");
  Serial.print(scoring.audioScore, 1);
  Serial.println("                     ┃");
  Serial.println("┗━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┛\n");
}

// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(SOUND_SENSOR_PIN, INPUT);
  pinMode(LED_PIN, OUTPUT);
  
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  radarServo.setPeriodHertz(50);
  radarServo.attach(SERVO_PIN, 500, 2400);
  
  samplingPeriod = round(1000000 * (1.0 / SAMPLING_FREQUENCY));
  
  // Blink startup
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(150);
    digitalWrite(LED_PIN, LOW);
    delay(150);
  }
  
  // Încarcă parola din EEPROM
  loadPasswordFromEEPROM();
  
  Serial.println("\n╔═════════════════════════════════════════════╗");
  Serial.println("║  SISTEM DETECTARE INTRUZIUNI v2.3 FIXED   ║");
  Serial.println("║   Threshold-uri optimizate                ║");
  Serial.println("║   Baseline management îmbunătățit         ║");
  Serial.println("║   Weight-uri echilibrate                  ║");
  Serial.println("║   Modul ARM/DISARM cu parolă EEPROM       ║");
  Serial.println("╚═════════════════════════════════════════════╝\n");
  
  // Pornire în mod DEZARMAT
  systemMode = MODE_DISARMED;
  Serial.println(" Sistemul pornit în mod DEZARMAT.");
  
  scoring.lastDecayUpdate = millis();
  
  printMainMenu();
}

// ==================== LOOP PRINCIPAL ====================
void loop() {
  static unsigned long lastScan = 0;
  static unsigned long lastStatusPrint = 0;

  // ====================================================
  // Procesare meniu serial (întotdeauna activ)
  // ====================================================
  handleSerialMenu();

  // ====================================================
  // Dacă sistemul e DEZARMAT, nu rulăm logica de detecție
  // ====================================================
  if (systemMode == MODE_DISARMED) {
    digitalWrite(LED_PIN, LOW);
    delay(50);
    return;
  }

  // ====================================================
  // De aici în jos: codul original NEATINS
  // ====================================================

  updateStateMachine();
  
  // ========== DETECȚIE PROXIMITATE AUDIO ==========
  static unsigned long lastProximityCheck = 0;
  static int localSoundCounter = 0;
  
  if (!isServoNoiseActive() && millis() - lastProximityCheck >= 300) {
    lastProximityCheck = millis();
    
    int maxReading = 0;
    int avgReading = 0;
    int peakCount = 0;
    
    for (int i = 0; i < 20; i++) {
      int reading = analogRead(SOUND_SENSOR_PIN);
      if (reading > maxReading) maxReading = reading;
      avgReading += reading;
      
      if (reading > audioState.baselineNoise + 300) peakCount++;
      
      delayMicroseconds(200);
    }
    
    avgReading /= 20;
    
    audioProximity.peakValue = maxReading;
    audioProximity.signalStrength = (float)(maxReading - audioState.baselineNoise) / 3000.0;
    audioProximity.signalStrength = constrain(audioProximity.signalStrength, 0.0, 1.0);
    
    int dynamicPeakThreshold = max(audioState.baselineNoise * 8, audioState.baselineNoise + 800);
    int dynamicSpikeThreshold = audioState.baselineNoise + 500;
    
    if (maxReading > dynamicPeakThreshold && peakCount >= 5) {
      audioProximity.isLocalSound = true;
      localSoundCounter++;
    } else if ((maxReading - avgReading) > dynamicSpikeThreshold) {
      audioProximity.isLocalSound = true;
      localSoundCounter++;
    } else {
      audioProximity.isLocalSound = false;
      localSoundCounter = 0;
    }
    
    audioProximity.isPersistentLocal = (localSoundCounter >= 3);
  }
  
  // Debug audio
  static unsigned long lastAudioDebug = 0;
  if (millis() - lastAudioDebug >= 1000) {
    lastAudioDebug = millis();
    
    int currentAudio = analogRead(SOUND_SENSOR_PIN);
    Serial.print(" Audio RAW: ");
    Serial.print(currentAudio);
    Serial.print(" | Baseline: ");
    Serial.print(audioState.baselineNoise);
    Serial.print(" | Threshold: ");
    Serial.print(audioState.dynamicThreshold);
    Serial.print(" | Local: ");
    Serial.print(audioProximity.isPersistentLocal ? "PERSISTENT " : (audioProximity.isLocalSound ? "DA " : "NU"));
    Serial.print(" | Servo: ");
    Serial.println(isServoNoiseActive() ? "BLOCAT " : "OK ✓");
  }
  
  applyDecay();
  
  // Debug scor
  static unsigned long lastScoreDebug = 0;
  if (millis() - lastScoreDebug >= 500) {
    lastScoreDebug = millis();
    
    if (scoring.totalScore > 0) {
      Serial.print("Scor: ");
      Serial.print(scoring.totalScore, 1);
      Serial.print(" (R:");
      Serial.print(scoring.radarScore, 1);
      Serial.print(" A:");
      Serial.print(scoring.audioScore, 1);
      Serial.print(") | Stare: ");
      
      switch(currentState) {
        case STATE_IDLE: Serial.println("IDLE"); break;
        case STATE_SUSPECT: Serial.println("SUSPECT "); break;
        case STATE_VERIFY: Serial.println("VERIFY "); break;
        case STATE_ALARM: Serial.println("ALARM "); break;
        case STATE_COOLDOWN: Serial.println("COOLDOWN"); break;
      }
    }
  }

  // Verificare audio în IDLE
  if (!isServoNoiseActive() && currentState == STATE_IDLE) {
    static unsigned long lastIdleAudioCheck = 0;
    if (millis() - lastIdleAudioCheck >= 200) {
      lastIdleAudioCheck = millis();
      if (isSoundPresent()) {
        String soundType = analyzeSoundType();
        if (soundType == "VOCE" || soundType == "SOAPTA" || soundType == "BATAIE") {
          float audioConf = calculateAudioConfidence();
          confidence.audioFiltered = 0.7 * confidence.audioFiltered + 0.3 * audioConf;
          
          float intensity = 0;
          if (soundType == "VOCE") intensity = 5.0;
          else if (soundType == "SOAPTA") intensity = 8.0;
          else if (soundType == "BATAIE") intensity = 10.0;
          
          addAudioEvent(intensity);
          
          Serial.print(" Audio IDLE: ");
          Serial.print(soundType);
          Serial.print(" | Confidence: ");
          Serial.println(audioConf, 2);
        }
      }
    }
  }

  // ==================== SCANARE RADAR ====================
  if (millis() - lastScan >= SCAN_DELAY) {
    lastScan = millis();

    setServoPosition(currentAngle);
    lastServoMove = millis();
    delay(SERVO_SETTLE_TIME);

    int distance = getDistance();

    if (distance > 0) {
        int index = getAngleIndex(currentAngle);
        float anomaly = calculateRadarAnomaly(currentAngle, distance);
        
        if (anomaly > 0 || (millis() % 3000 < 100)) {
            Serial.print(" Unghi:");
            Serial.print(currentAngle);
            Serial.print("° | Baseline:");
            Serial.print(baseline.distance[index]);
            Serial.print(" | Current:");
            Serial.print(distance);
            Serial.print(" | Diff:");
            Serial.print(baseline.distance[index] - distance);
            Serial.print(" | Anomaly:");
            Serial.print(anomaly, 1);
            Serial.print(" | Frozen:");
            Serial.println(baseline.frozen ? "DA" : "NU");
        }

        static int strongMotionCounter = 0;

        if (anomaly >= 15.0) {
            strongMotionCounter++;
        } else {
            strongMotionCounter = 0;
        }

        if (strongMotionCounter >= 2) {
            Serial.println(" ALARM DIRECT - Mișcare mare confirmată");
            currentState = STATE_ALARM;
            alarmTriggerTime = millis();
            strongMotionCounter = 0;
        }

        baseline.history[index][baseline.historyIndex[index] % 3] = distance;
        baseline.historyIndex[index]++;

        int m0 = baseline.history[index][0];
        int m1 = baseline.history[index][1];
        int m2 = baseline.history[index][2];
        int measurements[3] = {m0, m1, m2};

        static int radarConfirmCounter = 0;

        if (anomaly > 0) {
            radarConfirmCounter++;
        } else {
            radarConfirmCounter = 0;
        }

        if (radarConfirmCounter >= 1) {
            float radarConf = calculateRadarConfidence(currentAngle, distance, measurements);
            confidence.radarFiltered = 0.7 * confidence.radarFiltered + 0.3 * radarConf;

            addRadarEvent(anomaly);

            if (anomaly > 4.0) {
                baseline.frozen = true;
                Serial.println(" Baseline ÎNGHEȚAT - mișcare detectată");
            }

            if (currentState == STATE_VERIFY) {
                updateTrackedZone(currentAngle, distance);
            }

            radarConfirmCounter = 0;
        }

        if (!baseline.frozen && canUpdateBaseline(currentAngle, distance)) {
            updateBaseline(currentAngle, distance);
        }
    }

    currentAngle += direction * SCAN_STEP;

    if (currentAngle >= SCAN_MAX) {
        currentAngle = SCAN_MAX;
        direction = -1;
    }
    else if (currentAngle <= SCAN_MIN) {
        currentAngle = SCAN_MIN;
        direction = 1;
    }
  }
  
  if ((currentState == STATE_SUSPECT || currentState == STATE_VERIFY) && 
      millis() - lastStatusPrint >= 5000) {
    printCurrentStatus();
    lastStatusPrint = millis();
  }
  
  delay(10);

  //  BASELINE UNFREEZE LOGIC
  static unsigned long stableIdleStart = 0;

  if (currentState == STATE_IDLE && scoring.totalScore < 0.1) {
      if (stableIdleStart == 0) {
          stableIdleStart = millis();
      }

      if (millis() - stableIdleStart > 15000) {
          if (baseline.frozen) {
              baseline.frozen = false;
              Serial.println(" Baseline DEZGHEȚAT după liniște prelungită");
          }
      }
  } else {
      stableIdleStart = 0;
      if (currentState != STATE_IDLE) {
          baseline.frozen = true;
      }
  }
}

// ==================== IMPLEMENTARE RADAR ====================

int getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  
  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  int distance = duration * 0.034 / 2;
  
  if (distance == 0 || distance > 400) return -1;
  return distance;
}

void performBaselineScan() {
  for (int cycle = 0; cycle < 3; cycle++) {
    Serial.print("Ciclu ");
    Serial.print(cycle + 1);
    Serial.println("/3");
    
    for (int angle = SCAN_MIN; angle <= SCAN_MAX; angle += SCAN_STEP) {
      setServoPosition(angle);
      lastServoMove = millis();
      delay(100);
      
      int distance = getDistance();
      if (distance > 0) {
        int index = getAngleIndex(angle);
        if (cycle == 0) {
          baseline.distance[index] = distance;
        } else {
          baseline.distance[index] =
            (baseline.distance[index] * cycle + distance) / (cycle + 1);
        }
        Serial.print(".");
        if (angle % 30 == 0) Serial.println();
      }
      delay(30);
    }
    Serial.println();
  }
  baseline.initialized = true;
  Serial.println("✓ Baseline învățat");
}

int getAngleIndex(int angle) {
  return (angle - SCAN_MIN) / SCAN_STEP;
}

float calculateRadarAnomaly(int angle, int distance) {
    if (angle < 20 || angle > 160) return 0.0;
    if (!baseline.initialized) return 0.0;

    int index = getAngleIndex(angle);
    int baselineDist = baseline.distance[index];

    int diff = baselineDist - distance;
    if (diff <= 0) return 0.0;

    float avg = 0;
    for (int i = 0; i < 3; i++) {
        avg += baseline.history[index][i];
    }
    avg /= 3.0;

    float variance = 0;
    for (int i = 0; i < 3; i++) {
        variance += pow(baseline.history[index][i] - avg, 2);
    }
    variance /= 3.0;

    float deviation = sqrt(variance);
    if (deviation < 2.0) deviation = 2.0;

    float zScore = diff / deviation;

    if (zScore < 1.5) return 0.0;
    if (zScore < 2.5) return 8.0;
    if (zScore < 4.0) return 12.0;
    return 18.0;
}

float calculateRadarConfidence(int angle, int distance, int measurements[]) {
  float avg = (measurements[0] + measurements[1] + measurements[2]) / 3.0;
  float variance = 0;
  for (int i = 0; i < 3; i++) {
    variance += pow(measurements[i] - avg, 2);
  }
  variance /= 3.0;
  float consistency = 1.0 - constrain(variance / 100.0, 0.0, 1.0);
  
  int index = getAngleIndex(angle);
  float anomalyMag = abs(baseline.distance[index] - distance) / 100.0;
  anomalyMag = constrain(anomalyMag, 0.0, 1.0);
  
  float proximity = 1.0 - (distance / 400.0);
  
  float servoHealth = isServoHealthy() ? 1.0 : 0.3;
  
  return constrain((consistency + anomalyMag + proximity) / 3.0 * servoHealth, 0.0, 1.0);
}

bool isServoHealthy() {
  return true;
}

bool canUpdateBaseline(int angle, int distance) {
  if (currentState != STATE_IDLE) return false;
  if (baseline.frozen) return false;
  if (scoring.totalScore > 0.1) return false;
  
  int index = getAngleIndex(angle);
  
  baseline.history[index][baseline.historyIndex[index] % 3] = distance;
  baseline.historyIndex[index]++;
  
  if (baseline.historyIndex[index] < 3) return false;
  
  int sum = 0;
  for (int i = 0; i < 3; i++) {
    sum += baseline.history[index][i];
  }
  int avg = sum / 3;
  
  int maxDev = 0;
  for (int i = 0; i < 3; i++) {
    maxDev = max(maxDev, abs(baseline.history[index][i] - avg));
  }
  
  return (maxDev < 10);
}

void updateBaseline(int angle, int distance) {
  int index = getAngleIndex(angle);
  baseline.distance[index] = (baseline.distance[index] * 19 + distance) / 20;
}

void setServoPosition(int angle) {
  radarServo.write(angle);
}

bool verifyServoPosition(int targetAngle) {
  return true;
}

// ==================== IMPLEMENTARE AUDIO ====================

void calibrateAudioBaseline() {
  long sum = 0;
  long sumSquares = 0;
  
  for (int i = 0; i < BASELINE_SAMPLES; i++) {
    int reading = analogRead(SOUND_SENSOR_PIN);
    sum += reading;
    sumSquares += (long)reading * reading;
    delay(15);
  }
  
  audioState.baselineNoise = sum / BASELINE_SAMPLES;
  long variance = (sumSquares / BASELINE_SAMPLES) - 
                  (audioState.baselineNoise * audioState.baselineNoise);
  audioState.noiseStdDev = sqrt(variance);

  if (audioState.baselineNoise < 20) {
    audioState.baselineNoise = 20;
  }
  
  audioState.dynamicThreshold = audioState.baselineNoise + 
                                max(20, (int)(audioState.noiseStdDev * THRESHOLD_MULTIPLIER));
}

void recalibrateAudio() {
  long sum = 0;
  long sumSquares = 0;
  
  for (int i = 0; i < 50; i++) {
    int reading = analogRead(SOUND_SENSOR_PIN);
    sum += reading;
    sumSquares += (long)reading * reading;
    delay(10);
  }
  
  int newBaseline = sum / 50;
  long variance = (sumSquares / 50) - (newBaseline * newBaseline);
  int newStdDev = sqrt(variance);
  
  audioState.baselineNoise = (audioState.baselineNoise * 3 + newBaseline) / 4;
  audioState.noiseStdDev = (audioState.noiseStdDev * 3 + newStdDev) / 4;
  audioState.dynamicThreshold = audioState.baselineNoise + 
                                max(20, (int)(audioState.noiseStdDev * THRESHOLD_MULTIPLIER));
}

bool isServoNoiseActive() {
  return (millis() - lastServoMove < SERVO_NOISE_WINDOW);
}

bool isSoundPresent() {
  if (isServoNoiseActive()) return false;
  
  int sampleCount = 20;
  int aboveThreshold = 0;
  int maxReading = 0;
  int totalEnergy = 0;
  
  for (int i = 0; i < sampleCount; i++) {
    int reading = analogRead(SOUND_SENSOR_PIN);
    if (reading > maxReading) maxReading = reading;
    
    int energy = abs(reading - audioState.baselineNoise);
    totalEnergy += energy;
    
    if (reading > audioState.dynamicThreshold - 5) {
      aboveThreshold++;
    }
    delayMicroseconds(600);
  }
  
  int avgEnergy = totalEnergy / sampleCount;
  return (avgEnergy > 2 && aboveThreshold >= 2);
}

String analyzeSoundType() {
  if (isServoNoiseActive()) return "ZGOMOT";
  
  for (int i = 0; i < SAMPLES; i++) {
    unsigned long microsecondsStart = micros();
    vReal[i] = analogRead(SOUND_SENSOR_PIN);
    vImag[i] = 0;
    while (micros() - microsecondsStart < samplingPeriod) {}
  }
  
  FFT.windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.complexToMagnitude(vReal, vImag, SAMPLES);
  
  float trafficEnergy = calculateBandEnergy(20, TRAFFIC_HIGH);
  float voiceEnergy = calculateBandEnergy(VOICE_LOW, VOICE_HIGH);
  float whisperEnergy = calculateBandEnergy(WHISPER_LOW, WHISPER_HIGH);
  float highFreqEnergy = calculateBandEnergy(2000, 4000);
  
  float totalEnergy = 0;
  for (int i = 0; i < SAMPLES / 2; i++) {
    totalEnergy += vReal[i];
  }
  
  if (totalEnergy < 15.0) return "ZGOMOT";

  int rawCheck = analogRead(SOUND_SENSOR_PIN);
  if (rawCheck < audioState.dynamicThreshold - 20) return "ZGOMOT";

  float voiceRatio = voiceEnergy / totalEnergy;
  float whisperRatio = whisperEnergy / totalEnergy;
  float trafficRatio = trafficEnergy / totalEnergy;
  float highFreqRatio = highFreqEnergy / totalEnergy;

  if (trafficRatio > TRAFFIC_REJECT_THRESHOLD) return "ZGOMOT";
  if (trafficRatio > 0.25 && highFreqRatio < 0.15) return "ZGOMOT";

  if (whisperRatio > 0.18 && whisperEnergy > 20.0) return "SOAPTA";
  if (voiceRatio > 0.18 && voiceEnergy > 20.0) return "VOCE";
  if (voiceRatio > 0.12 && trafficRatio < 0.5) return "BATAIE";

  return "ZGOMOT";
}

float calculateBandEnergy(int freqLow, int freqHigh) {
  float energy = 0;
  int binLow = (freqLow * SAMPLES) / SAMPLING_FREQUENCY;
  int binHigh = (freqHigh * SAMPLES) / SAMPLING_FREQUENCY;
  binLow = constrain(binLow, 0, SAMPLES / 2);
  binHigh = constrain(binHigh, 0, SAMPLES / 2);
  
  for (int i = binLow; i <= binHigh; i++) {
    energy += vReal[i];
  }
  return energy;
}

float calculateAudioConfidence() {
  int maxReading = 0;

  for (int i = 0; i < 20; i++) {
    int r = analogRead(SOUND_SENSOR_PIN);
    if (r > maxReading) maxReading = r;
    delayMicroseconds(500);
  }

  float snr = (maxReading - audioState.baselineNoise) / 100.0;
  snr = constrain(snr, 0.0, 1.0);

  return snr;
}

// ==================== SCORING ENGINE ====================

void addRadarEvent(float intensity) {
  float saturatedIntensity = saturate(intensity, 0.5);
  float scoreAdded = saturatedIntensity * confidence.radarFiltered;
  
  scoring.radarScore += scoreAdded;
  updateScore();
  
  scoring.eventHistory[scoring.eventHistoryIndex % 10] = millis();
  scoring.eventHistoryIndex++;
  
  printDetectionEvent("RADAR", intensity, confidence.radarFiltered, scoreAdded);
}

void addAudioEvent(float intensity) {
  if (currentState == STATE_IDLE && !audioProximity.isLocalSound) {
    Serial.println(" Audio ignorat - sunet distant/slab");
    return;
  }
  
  if (audioProximity.isPersistentLocal) {
    intensity += 4.0;
    Serial.print(" SUNET LOCAL PERSISTENT - Strength: ");
    Serial.println(audioProximity.signalStrength, 2);
  }
  
  static unsigned long lastAudioEventTime = 0;
  unsigned long cooldownTime = audioProximity.isPersistentLocal ? 300 : 500;
  
  if (millis() - lastAudioEventTime < cooldownTime) {
    return;
  }
  lastAudioEventTime = millis();
  
  float saturatedIntensity = saturate(intensity, 0.3);
  float scoreAdded = saturatedIntensity * confidence.audioFiltered;
  
  scoring.audioScore += scoreAdded;
  updateScore();
  
  scoring.eventHistory[scoring.eventHistoryIndex % 10] = millis();
  scoring.eventHistoryIndex++;
  
  printDetectionEvent("AUDIO", intensity, confidence.audioFiltered, scoreAdded);
}

void updateScore() {
  float radarWeight = 0.70;
  float audioWeight = 0.30;
  
  if (audioProximity.isPersistentLocal && scoring.audioScore > 4.0) {
    radarWeight = 0.55;
    audioWeight = 0.45;
  }
  
  scoring.totalScore = scoring.radarScore * radarWeight + 
                       scoring.audioScore * audioWeight;
  
  int recentEvents = 0;
  unsigned long now = millis();
  for (int i = 0; i < 10; i++) {
    if (now - scoring.eventHistory[i] < 2000) {
      recentEvents++;
    }
  }
  
  if (recentEvents >= 3) {
    scoring.totalScore += 1.0;
  }
  if (recentEvents >= 5) {
    scoring.totalScore += 1.5;
  }
}

void applyDecay() {
  unsigned long now = millis();
  float deltaTime = (now - scoring.lastDecayUpdate) / 1000.0;
  scoring.lastDecayUpdate = now;
  
  if (deltaTime > 0.1) {
    float decayRate;

    if (currentState == STATE_VERIFY) {
      decayRate = 0.2;
    }
    else if (currentState == STATE_SUSPECT) {
      decayRate = 0.05;
    }
    else {
      decayRate = 0.4;
    }
    
    scoring.totalScore -= decayRate * deltaTime;
    scoring.radarScore -= decayRate * deltaTime;
    scoring.audioScore -= decayRate * deltaTime;
    
    scoring.totalScore = max(0.0f, scoring.totalScore);
    scoring.radarScore = max(0.0f, scoring.radarScore);
    scoring.audioScore = max(0.0f, scoring.audioScore);
  }
}

float saturate(float x, float k) {
  return 1.0 - exp(-k * x);
}

void logScoreState() {
  if (currentState == STATE_IDLE && scoring.totalScore == 0) return;
  
  if (currentState == STATE_SUSPECT || currentState == STATE_VERIFY) {
    Serial.print("  ");
    printScoreBar();
  }
}

// ==================== FSM ====================

void updateStateMachine() {
  SystemState prevState = currentState;
  
  switch (currentState) {
    case STATE_IDLE:
      handleStateIdle();
      break;
    case STATE_SUSPECT:
      handleStateSuspect();
      break;
    case STATE_VERIFY:
      handleStateVerify();
      break;
    case STATE_ALARM:
      handleStateAlarm();
      break;
    case STATE_COOLDOWN:
      handleStateCooldown();
      break;
  }
  
  if (prevState != currentState) {
    stateEntryTime = millis();
    printStateTransition(prevState, currentState);
    printCurrentStatus();
  }
}

void handleStateIdle() {
  digitalWrite(LED_PIN, LOW);
  
  if (scoring.totalScore >= 0.8) {
    currentState = STATE_SUSPECT;
  }
}

void handleStateSuspect() {
  if ((millis() / 500) % 2 == 0) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
  }
  
  baseline.frozen = true;
  
  if (scoring.totalScore >= 2.5) {
    currentState = STATE_VERIFY;
  } else if (scoring.totalScore < 0.5) {
    currentState = STATE_IDLE;
    baseline.frozen = false;
  }
}

void handleStateVerify() {
  if ((millis() / 200) % 2 == 0) {
    digitalWrite(LED_PIN, HIGH);
  } else {
    digitalWrite(LED_PIN, LOW);
  }
  
  baseline.frozen = true;
  
  if (trackedZone.active) {
    trackZone();
  }
  
  bool canAlarm = false;
  
  if (scoring.radarScore > 2.5 && scoring.totalScore >= 5.0) {
    canAlarm = true;
    Serial.println(" Alarmă: MIȘCARE CONFIRMATĂ");
  }
  else if (scoring.totalScore >= 7.0 && scoring.radarScore > 1.5) {
    canAlarm = true;
    Serial.println(" Alarmă: RADAR + AUDIO confirmat");
  }
  else if (scoring.audioScore > 8.0 &&
           audioProximity.isPersistentLocal &&
           audioProximity.signalStrength > 0.7 &&
           scoring.totalScore >= 10.0) {
    canAlarm = true;
    Serial.println(" Alarmă: SUNET LOCAL PERSISTENT puternic");
  }
  
  if (canAlarm) {
    currentState = STATE_ALARM;
    alarmTriggerTime = millis();
  } else if (scoring.totalScore < 1.5) {
    currentState = STATE_SUSPECT;
  }
}

void handleStateAlarm() {
  digitalWrite(LED_PIN, HIGH);
  
  // Afișează reminder dezarmare periodic
  static unsigned long lastDisarmReminder = 0;
  if (millis() - lastDisarmReminder >= 10000) {
    lastDisarmReminder = millis();
    Serial.println(" ALARMĂ ACTIVĂ! Tastează [disarm] și introdu parola pentru a dezarma.");
  }
  
  if (millis() - alarmTriggerTime > ALARM_TIMEOUT) {
    Serial.println("\n Alarm timeout - reset automat");
    currentState = STATE_COOLDOWN;
  }
}

void handleStateCooldown() {
  digitalWrite(LED_PIN, LOW);
  
  if (millis() - stateEntryTime > COOLDOWN_DURATION) {
    scoring.totalScore = 0;
    scoring.radarScore = 0;
    scoring.audioScore = 0;
    baseline.frozen = false;
    trackedZone.active = false;
    currentState = STATE_IDLE;
    Serial.println("\n Cooldown complete - sistem reset\n");
  }
}

// ==================== TRACKING ====================

void updateTrackedZone(int angle, int distance) {
  if (!trackedZone.active) {
    trackedZone.centerAngle = angle;
    trackedZone.angleSpread = 15;
    trackedZone.avgDistance = distance;
    trackedZone.lastSeen = millis();
    trackedZone.detectionCount = 1;
    trackedZone.active = true;
  } else {
    if (abs(angle - trackedZone.centerAngle) <= trackedZone.angleSpread) {
      trackedZone.avgDistance = (trackedZone.avgDistance + distance) / 2;
      trackedZone.lastSeen = millis();
      trackedZone.detectionCount++;
    }
  }
}

bool trackZone() {
  if (!trackedZone.active) return false;
  
  if (millis() - trackedZone.lastSeen > 3000) {
    trackedZone.active = false;
    Serial.println(" Tracking lost");
    return false;
  }
  
  if (trackedZone.detectionCount >= 3) {
    addRadarEvent(5.0);
    trackedZone.detectionCount = 0;
  }
  
  return true;
}

void printSystemStatus() {
  Serial.println("\n═══ SYSTEM STATUS ═══");
  logScoreState();
  Serial.print("Baseline frozen: ");
  Serial.println(baseline.frozen ? "YES" : "NO");
  Serial.print("Tracked zone active: ");
  Serial.println(trackedZone.active ? "YES" : "NO");
  Serial.println("═══════════════════\n");
}