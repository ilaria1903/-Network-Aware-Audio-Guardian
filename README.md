#  Ultrasonic Radar + Audio Intrusion Detection System

## General Description

This project implements an embedded security system capable of detecting intrusions in an enclosed space using two complementary sensing modalities: a rotating ultrasonic radar and a sound sensor with FFT spectral analysis. The system runs on an ESP32 and features a 5-state finite state machine (FSM), a fuzzy scoring engine, and an arm/disarm system protected by a password stored in EEPROM.

At startup the system is **DISARMED**. The user arms it via a serial menu, at which point the system performs an audio baseline calibration and three full radar sweeps to learn the environment. From that point on, any deviation from the learned spatial baseline or any detected voice/knock/whisper triggers a scoring mechanism that progressively escalates through suspect → verify → alarm states.

---

## 1. Bill of Materials (BOM)

| Component | Details | Qty |
|---|---|---|
| ESP32 Dev Board | Main microcontroller, dual-core 240 MHz | 1 |
| Ultrasonic Sensor | HC-SR04, range ~2–400 cm | 1 |
| Sound Sensor | EE-46-1, Barnabas Robotics | 1 |
| Servo Motor | Standard 5V servo, 0–180° rotation | 1 |
| Ultrasonic Sensor Mount | Bracket attached to servo horn (makes the radar assembly) | 1 |
| LED | 5mm (status indicator) | 1 |
| Capacitor | 100 µF electrolytic (power decoupling) | 1 |
| Breadboard | Full or half size | 1 |
| Jumper Wires | Male-to-male, male-to-female | ~20 |
| USB Cable | For programming and power | 1 |

---

## 2. Tutorial Source

This project is **not based on a specific tutorial**. The radar scanning concept (servo + ultrasonic) is a well-known hobbyist pattern, but the detection logic, scoring engine, FSM architecture, audio FFT analysis, baseline management, and arm/disarm system with EEPROM password are all original designs developed specifically for this project.

---

## 3. The Five Questions

### Q1 — What is the system boundary?

The system boundary is a **room or enclosed space** covered by the 180° arc of the ultrasonic radar. Physically, the boundary is defined by what the HC-SR04 can see from its mounting position: a semicircular detection zone from 15° to 165° (servo angle), up to approximately 150–200 cm in practice. The sound sensor has no directional boundary — it picks up audio from the entire environment around the device. The system has no network output; all interaction happens over serial (USB). The physical boundary of the device itself is the breadboard assembly sitting in a fixed position in the room.

### Q2 — Where does intelligence live?

All intelligence lives **on the ESP32** — there is no cloud, no external server, and no companion app. The ESP32 runs:

- **Spatial memory**: a learned baseline of distances per angle slot (51 slots across 150°), updated with an exponential moving average when the environment is stable
- **Anomaly detection**: a z-score calculation per angle that compares current distance readings against the baseline's recent history
- **Audio classification**: a 128-point FFT over the microphone signal, with band energy ratios used to distinguish voice (300–3400 Hz), whisper (1000–4000 Hz), knock, and background noise
- **Scoring engine**: a weighted fuzzy score combining radar and audio confidence, with saturation, decay, and persistence bonuses
- **FSM**: five-state machine (IDLE → SUSPECT → VERIFY → ALARM → COOLDOWN) with threshold-based transitions
- **EEPROM password management**: arm/disarm authentication stored across reboots

### Q3 — What is the hardest technical problem?

The hardest technical problem was **calibration and score formula design** — specifically, making the system sensitive enough to catch real intrusions while remaining stable against normal environmental noise.

There are several layers to this problem:

**Radar baseline drift.** The baseline must be learned initially and then frozen as soon as anything moves. If it updates during movement, it learns the intruder and stops detecting them. The solution was an explicit `frozen` flag that prevents baseline updates the moment any anomaly is detected, and only unfreezes after 15 seconds of quiet in IDLE state with near-zero score.

**Servo motor acoustic interference.** The servo generates vibrations and electrical noise that contaminates the microphone signal. A `SERVO_NOISE_WINDOW` of 40 ms suppresses all audio processing immediately after each servo movement.

**Z-score instability at startup.** When the history buffer for an angle slot has fewer than 3 samples, variance is zero and division produces garbage. A minimum deviation floor of 2.0 cm was enforced to prevent false positives on startup.

**Audio classification.** Raw amplitude alone is insufficient — a truck passing outside and a whispered voice can produce similar peak values. FFT band energy ratios were used to classify sounds, with empirically tuned thresholds. Whispers remain the weakest point (see limitations).

**Score decay balance.** A score that decays too fast misses slow-moving intruders; too slow and background noise accumulates into false alarms. Different decay rates per state (0.4/s in IDLE, 0.05/s in SUSPECT, 0.2/s in VERIFY) were tuned through repeated live testing.

### Q4 — What is the minimum demo?

The minimum viable demo consists of:

1. Powering on the device and connecting via serial monitor at 115200 baud
2. Typing `arm` — the system performs audio calibration and 3 radar baseline sweeps (~30 seconds)
3. Walking into the radar's detection zone — the system escalates IDLE → SUSPECT → VERIFY → ALARM, with the LED changing at each stage (off → slow blink → fast blink → solid on)
4. Typing `disarm` and entering the password (`1234` by default) to silence the alarm
5. Optionally typing `setpassword` to change the password, which persists across reboots via EEPROM

This demonstrates spatial learning, multi-modal detection, FSM escalation, and secure arm/disarm in under 2 minutes.

### Q5 — Why is this not just a tutorial?

**Original detection logic.** No tutorial was followed. The combination of z-score radar anomaly detection, FFT audio classification, a fuzzy weighted scoring engine, and a multi-state FSM with freeze/unfreeze baseline management is an original architecture designed from scratch for this specific problem.

**Non-trivial calibration.** Most hobbyist radar tutorials simply threshold distance against a fixed value. This system builds a statistical model of the environment (rolling average + variance per angle slot), computes z-scores, and adapts dynamically. The score formula with saturation (`1 - e^(-kx)`), per-state decay rates, and persistence bonuses is not something found in any tutorial.

**Multi-modal sensor fusion.** Combining ultrasonic and audio data into a single unified score, with dynamic weight adjustment based on sound proximity detection, is an original design decision.

**ARM/DISARM security layer.** The password-protected arm/disarm system with EEPROM persistence, masked serial input, and state-aware menu (disarm available even during active alarm) adds a layer of practical usability not present in any reference material.

---

## 4. How the System Works — Technical Deep Dive

### Radar Baseline Learning

At arm time, the servo sweeps from 15° to 165° in 3° steps (51 angle slots) for **3 full cycles**. At each angle, the HC-SR04 fires an ultrasonic pulse and the echo time is converted to distance in cm. The baseline distance per slot is a running average across the 3 cycles:
```
baseline[i] = (baseline[i] × cycle + new_reading) / (cycle + 1)
```

After these 3 sweeps, the system has a learned spatial map of the room — every angle slot knows how far the nearest surface is.

### Radar Anomaly Detection (Z-Score)

During normal operation, for each servo position the system computes:

1. **Difference**: `diff = baseline_distance - current_distance`. Only positive differences matter — something getting *closer* than baseline is an intrusion. Something moving further away is ignored.
2. **Local variance**: the last 3 readings for that angle slot are kept in a rolling history buffer. Mean and variance are computed to get a local standard deviation (floored at 2.0 cm).
3. **Z-score**: `zScore = diff / deviation`
4. **Anomaly score mapping**:
   - zScore < 1.5 → no anomaly (0.0)
   - zScore 1.5–2.5 → moderate (8.0)
   - zScore 2.5–4.0 → strong (12.0)
   - zScore > 4.0 → very strong (18.0)

If two consecutive readings produce an anomaly ≥ 15.0, the system jumps directly to ALARM, bypassing the FSM escalation.

### Audio Baseline Calibration

At arm time, **200 audio samples** are taken with 15 ms spacing. The system computes:

- `baselineNoise`: arithmetic mean (ambient DC level of the microphone)
- `noiseStdDev`: standard deviation of the samples
- `dynamicThreshold`: `baselineNoise + max(20, stdDev × 1.0)`

This threshold adapts to the room's ambient noise floor automatically — a noisier room gets a higher threshold.

### Audio Classification (FFT)

When a sound event is detected, 128 samples are collected at 8000 Hz and processed through an FFT (Hamming window). Band energies are extracted:

- **Traffic band**: 20–500 Hz (low rumble, outside noise)
- **Voice band**: 300–3400 Hz (normal speech)
- **Whisper band**: 1000–4000 Hz (high-frequency breath noise)
- **High-frequency band**: 2000–4000 Hz

Classification rules (applied in order):
1. Total energy < 15.0 → NOISE
2. Traffic band ratio > 0.4 → NOISE
3. Traffic > 0.25 AND high-freq < 0.15 → NOISE
4. Whisper ratio > 0.18 AND whisper energy > 20 → WHISPER
5. Voice ratio > 0.18 AND voice energy > 20 → VOICE
6. Voice ratio > 0.12 AND traffic < 0.5 → KNOCK
7. Otherwise → NOISE

### Scoring Engine

The system maintains three scores: `radarScore`, `audioScore`, and `totalScore`.

When a radar anomaly is detected:
```
saturatedIntensity = 1 - e^(-0.5 × intensity)
scoreAdded = saturatedIntensity × radarConfidence
radarScore += scoreAdded
```

The saturation function (`1 - e^(-kx)`) prevents a single very strong reading from dominating the score — diminishing returns above a certain intensity. Audio events use the same formula with `k = 0.3`.

**Total score** is a weighted combination:
```
totalScore = radarScore × 0.70 + audioScore × 0.30
```
If a persistent local sound is detected and `audioScore > 4.0`, weights shift to 0.55 / 0.45.

A **persistence bonus** adds +1.0 if ≥3 events occurred in the last 2 seconds, and +1.5 more if ≥5 events — rewarding sustained, consistent detections over noise spikes.

**Score decay** runs every loop cycle:
- IDLE: −0.4 points/second
- SUSPECT: −0.05 points/second
- VERIFY: −0.2 points/second

### State Machine
```


**IDLE**: LED off. Baseline updates allowed. Transitions to SUSPECT if `totalScore ≥ 0.8`.

**SUSPECT**: LED slow blink (500 ms). Baseline frozen. Transitions to VERIFY if `score ≥ 2.5`, back to IDLE if `score < 0.5`.

**VERIFY**: LED fast blink (200 ms). Baseline frozen. Active zone tracking. Transitions to ALARM if:
- `radarScore > 2.5` AND `totalScore ≥ 5.0`, OR
- `totalScore ≥ 7.0` AND `radarScore > 1.5`, OR
- `audioScore > 8.0` AND persistent local sound AND `signalStrength > 0.7` AND `totalScore ≥ 10.0`

**ALARM**: LED solid on. Reminder printed every 10 seconds. User must type `disarm` + correct password to exit. Auto-resets to COOLDOWN after 60 seconds.

**COOLDOWN**: LED off. 10-second pause. All scores reset, baseline unfrozen, returns to IDLE.

```

## 5. Known Limitations

**180° coverage only.** The servo sweeps from 15° to 165°, leaving the rear 180° arc completely unmonitored. An intruder approaching from behind the device will not be detected by radar (though audio may still trigger).

**Limited range.** The HC-SR04 has a maximum specified range of 400 cm, but accuracy degrades significantly beyond 150–200 cm in a furnished room.

**Whisper detection unreliability.** Whispers have low amplitude and their spectral signature overlaps with certain ambient noises (HVAC, distant traffic). The FFT classification can miss whispers in moderately noisy environments, or occasionally misclassify ambient noise as a whisper. This remains an open calibration problem.

**Fixed mounting required.** The device must remain completely stationary after arming. Any accidental bump corrupts the baseline and may trigger false alarms until the baseline unfreezes and relearns after 15 seconds of quiet.


## 6. Future Work

The most significant planned improvement is **remote monitoring and control via smartphone**. Currently, all interaction with the system happens through a serial USB connection, which requires physical proximity to a computer. The goal is to replace this with a mobile interface accessible from anywhere.

Concretely, this would involve:

**Wi-Fi connectivity.** The ESP32 already has built-in Wi-Fi, so no additional hardware is needed. The plan is to run a lightweight web server or MQTT client directly on the ESP32, connecting it to the local network.

**Mobile interface.** A simple web app (or a platform like Blynk / Home Assistant) would allow the user to arm and disarm the system from their phone, using the same password authentication currently implemented over serial. The masked password input and EEPROM persistence would carry over directly.

**Push notifications.** When the system transitions to ALARM state, it would send a push notification to the user's phone with the detection type (radar movement, voice, whisper, knock) and the score at the time of trigger — giving context, not just a raw alert.

**Remote disarm.** After receiving the alert, the user would be able to disarm the system directly from the phone notification or app, entering their password remotely. This mirrors the current `disarm` serial command but over Wi-Fi.

This addition would transform the system from a standalone local device into a practical home security tool usable without being physically present at a computer.
