# Network-Aware Audio Guardian
*A multi-sensor embedded intrusion detection system with local intelligence and remote monitoring*

---

## Project Overview

**Network-Aware Audio Guardian** is an embedded intrusion detection system designed to detect nearby intrusions while minimizing false alarms caused by external or ambient noise.

The system combines **local audio processing** with **multi-sensor fusion** (motion and proximity sensing) to make robust decisions in real time. All critical logic runs **locally on the embedded device**, ensuring correct operation even without network connectivity.

A secondary networking layer is used only for **monitoring, configuration, and optional remote audio listening (VoIP-like functionality)**.

The core design philosophy is:

> **Local intelligence first. Networking is supportive, not critical.**

---

## Key Features

- Local audio event detection using a microphone
- Voice vs non-voice sound classification
- Multi-sensor fusion (audio + motion + proximity)
- State-based behavior (IDLE → ARMING → ARMED → ALERT)
- Local alarm indication (LED / servo / speaker)
- Optional remote monitoring and configuration
- Optional audio streaming during alerts (VoIP-like)


---

## Bill of Materials (BOM)

| Component | Description |
|--------|-------------|
| ESP32 Development Board | Main controller (audio processing + Wi-Fi) |
| I2S Microphone (INMP441 / SPH0645) | Digital audio input |
| PIR Motion Sensor | Human motion detection |
| Ultrasonic Sensor (HC-SR04) | Proximity / distance estimation |
| LEDs | System state indication |
| Optional Servo Motor | Physical alert / visual feedback |
| Optional Speaker + Amplifier | Local alarm or audio playback |
| Breadboard & Jumper Wires | Prototyping |
| USB Cable / 5V Power Supply | Power |

---

## Tutorial Source

This project **does not follow a single tutorial**.

It is an original design inspired by:
- embedded audio processing principles
- intrusion detection system architectures
- multi-sensor fusion techniques
- IoT networking patterns

No end-to-end reference implementation exists for this system.

---

## Changes to Existing Work

**Not applicable.**  
This project is not a modification of an existing tutorial or repository.

---

# Course Questions

---

## Q1 — What is the system boundary?

**Inside the system:**
- ESP32 firmware
- audio signal processing
- sensor fusion logic
- state machine logic
- local decision-making and alarm triggering

**Outside the system:**
- remote dashboard or monitoring client
- optional cloud logging
- user interface for configuration

The system must continue to operate correctly **even if all networking functionality is disabled**.

---

## Q2 — Where does intelligence live?

All intelligence lives **locally on the ESP32**.

This includes:
- audio feature extraction
- voice vs noise classification
- proximity estimation
- sensor correlation
- state-dependent decision logic

The networking layer is informational only and does not make decisions.

---

## Q3 — What is the hardest technical problem?

The hardest technical problem is **reducing false alarms caused by external or ambient noise**.

This is addressed through:
- contextual audio analysis
- proximity heuristics
- correlation with independent sensors (PIR, ultrasonic)
- system state awareness (arming vs armed)

---

## Q4 — What is the minimum demo?

The minimum working demo consists of:

1. System enters *arming* mode with an exit delay
2. After arming, a nearby sound combined with motion triggers an alert
3. LED or servo indicates alarm state
4. Event data is logged and viewable remotely (optional)

No cloud connectivity or audio streaming is required for the minimum demo.

---

## Do We Need an ESP32?

**Yes, an ESP32 is required.**

Reasons:
- real-time audio processing capability
- native I2S support for digital microphones
- sufficient CPU performance for feature extraction
- integrated Wi-Fi for monitoring and audio streaming
- deterministic behavior suitable for embedded systems

Alternatives such as Arduino Uno are insufficient for audio + networking, while Raspberry Pi is overkill and less reliable for live demos.

---

## Summary

This project demonstrates a **locally intelligent embedded security system** that uses multi-sensor fusion and contextual audio processing to detect nearby intrusions, with networking used only for monitoring and interaction rather than control.

---


