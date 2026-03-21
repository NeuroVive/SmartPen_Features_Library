# NeuroVive: Intrinsic Feature Extraction Library (C++)

![Version](https://img.shields.io/badge/version-1.0.0-blue.svg)
![Platform](https://img.shields.io/badge/platform-ESP32--S3%20%7C%20Android%20%7C%20iOS-green.svg)
![Language](https://img.shields.io/badge/language-C++17-orange.svg)

## Overview
It is a high-performance C++ signal processing engine designed for the early detection of Parkinson's Disease. It transforms raw smart-pen sensor data into a high-dimensional feature vector (354 features) using advanced kinematic, statistical, and non-linear dynamical analysis.

This library is optimized for **embedded deployment (ESP32-S3)** and seamless integration with **Mobile Applications (Flutter/Dart via FFI)**.

---

## Directory Structure
```text
NeuroVive-Library/
├── include/
│   └── SmartPen_Features.h    
├── src/
│   └── SmartPen_Features.cpp  
├── tests/
│   └── main.cpp               
├── .gitignore 
├── Makefile
└── README.md                  
````

---

## Key Features & Signal Layers

The library processes raw signals (X, Y, Pressure, Azimuth, Altitude) through **10 specialized layers** to extract **354 unique features**:

| **Layer** | **Feature Category**  | **Count** | **Description**                                      |
| --------- | --------------------- | --------- | ---------------------------------------------------- |
| **L1**    | Raw Signal Statistics | 55        | Mean, StdDev, Skewness, Kurtosis for raw sensors.    |
| **L2-L5** | Kinematic Dynamics    | 176       | Velocity, Acceleration, and Jerk (Full & Axis-wise). |
| **L6**    | Temporal Features     | 7         | On-surface vs In-air timing, stroke frequency.       |
| **L7-L9** | Intrinsic Features    | 66        | EMD (IMF) Analysis, Shannon & Renyi Entropy.         |
| **L10**   | Teager-Kaiser Energy  | 50        | Instantaneous energy profiles of the movement.       |

---

## Technical Integration (For Flutter Developers)

This library exports a C-compatible API for easy integration with **Dart FFI**.

## Core Function: `compute_features`

C++

```
float* compute_features(
    const float* x, const float* y, 
    const float* pressure, const float* azimuth, 
    const float* altitude, const float* acc_x, 
    const float* acc_y, 
    int32_t n_samples, 
    int32_t* out_size
);
```

## Usage Workflow:

1. **Allocate:** Send raw arrays (minimum 150 samples) to the function.
    
2. **Process:** The library returns a pointer to a `float` array of size 354.
    
3. **Clean-up:** **CRITICAL!** You must call `free_features(ptr)` from Dart to prevent memory leaks on the device.
    

---

## Building & Testing

To verify the library on your local machine (Ubuntu/Windows/Mac):

Bash

```
# Compile the test suite
g++ -I./include src/SmartPen_Features.cpp tests/main.cpp -o neuro_test

# Run tests
./neuro_test
```

---

## Roadmap

- [x] v1.0.0: Core Feature Extraction (354 features).
    
- [ ] v1.1.0: Memory optimization for ESP32-S3 (SRAM efficiency).
    
- [ ] v1.2.0: Hardware acceleration using ESP-DSP (SIMD).
    
