#ifndef AUDIO_SYSTEM_H
#define AUDIO_SYSTEM_H

#include <Arduino.h>
#include "arduinoFFT.h"
#include <cmath>
#include <cstring>

#define SAMPLES 128
#define SAMPLING_FREQUENCY 8000
#define BASELINE_SAMPLES 200
#define THRESHOLD_MULTIPLIER 1.0

#define VOICE_LOW 300
#define VOICE_HIGH 3400
#define WHISPER_LOW 1000
#define WHISPER_HIGH 4000
#define TRAFFIC_HIGH 500

#define SOUND_SENSOR_PIN 36

struct AudioStateData {
  int baselineNoise;
  int dynamicThreshold;
  int noiseStdDev;
  unsigned long lastValidSample;
};

struct AudioProximityData {
  bool isLocalSound;
  bool isPersistentLocal;
  float signalStrength;
  int peakValue;
};

class AudioSystem {
private:
  AudioStateData audioState;
  AudioProximityData audioProximity;
  
  unsigned int samplingPeriod;
  double vReal[SAMPLES];
  double vImag[SAMPLES];
  ArduinoFFT<double> FFT;
  
  float calculateBandEnergy(int freqLow, int freqHigh);
  
public:
  AudioSystem();
  
  void initialize();
  void calibrateBaseline();
  void recalibrateAudio();
  
  bool isSoundPresent();
  String analyzeSoundType();
  float calculateAudioConfidence();
  
  void updateProximityDetection();
  bool isLocalSoundDetected() const;
  bool isPersistentLocalSound() const;
  float getSignalStrength() const;
  
  int getBaselineNoise() const;
  int getDynamicThreshold() const;
  bool isServoNoiseActive(unsigned long lastServoMove) const;
  
  const AudioProximityData& getProximityData() const;
};

#endif
