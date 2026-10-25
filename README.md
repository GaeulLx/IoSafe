# IoSafe 🛡️

> A wearable student safety prototype designed to provide a discreet way for students to send an emergency alert and share their location.

## 📌 Overview

IoSafe is an IoT-based student safety system developed to address situations such as bullying, harassment, or emergencies where students may be unable or afraid to seek help openly.

The system combines a wearable ESP32 device with sensors, GPS, and a cloud backend to allow an emergency alert to be triggered and communicated to a designated guardian or authority.

## 🎯 Problem

Students in boarding or residential environments may face situations where they cannot safely make a phone call or openly ask for help.

IoSafe explores how a small wearable device could provide a faster and more discreet way to request assistance.

## ⚙️ How It Works

```text
Student
   │
   ▼
[IoSafe Wearable]
   │
   ├── SOS / Touch Input
   ├── Motion Detection
   └── GPS Location
   │
   ▼
[ESP32]
   │
   ▼
[Cloud Backend]
   │
   ▼
[Web Dashboard]
   │
   ├── Guardian
   └── College Authority
```

When the student activates the SOS function:

1. The ESP32 detects the emergency trigger.
2. The device obtains the student's current location using GPS.
3. The device sends the relevant data to the backend.
4. The backend can update the web platform.
5. A designated guardian or authority can respond to the alert.

## 🔧 Hardware

- ESP32-C3 Mini
- GPS NEO-6M
- MPU6050
- Touch sensor
- LiPo battery
- Power button

## 💻 Technologies

- Arduino / C++
- ESP32
- Firebase
- GPS
- IoT
- Embedded Systems
- Real-time data communication

## 👨‍💻 My Contribution

I worked mainly on the technical implementation of the IoT device and backend integration.

My contributions included:

- Programming the ESP32 using Arduino/C++
- Integrating the GPS module
- Integrating the MPU6050 motion sensor
- Implementing the SOS trigger
- Connecting the device with Firebase
- Testing sensor and communication functionality
- Troubleshooting hardware and software integration

## 📸 Prototype

> Add photos of the physical prototype here.

## 🚧 Current Status

**Prototype / Academic Project**

The current repository contains the ESP32 firmware and documentation of the prototype. The system was developed as a proof of concept and has not been deployed as a production-level emergency service.

## 🔮 Future Improvements

- Smaller wearable enclosure
- Improved battery management
- More reliable connectivity
- Push notifications
- Guardian/warden authentication
- Emergency alert history
- Better GPS accuracy
- Offline emergency handling
- Security and privacy improvements

## 📚 What I Learned

This project gave me hands-on experience with embedded programming, sensors, GPS, cloud services, and system integration. It also taught me that building an IoT solution involves much more than writing code — hardware reliability, connectivity, data security, and user experience all need to be considered.

---

**Project:** IoSafe  
**Type:** IoT / Student Safety Prototype  
**Platform:** ESP32  
**Status:** Prototype
