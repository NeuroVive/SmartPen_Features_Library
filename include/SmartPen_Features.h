#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif

// Constants
#define PEN_FEATURES_COUNT       354
#define PEN_STATIS_COUNT         11
#define PEN_SAMPLING_RATE        150.0f
#define PEN_DT                   (1.0f / PEN_SAMPLING_RATE)
#define PEN_MIN_SAMPLES          150
#define PEN_PRESSURE_THRESHOLD   0.05f
#define PEN_ACCEL_TO_MM          9810.0f // 9.81 m/s^2 to mm/s^2

// Main API
// Notice: x and y are REMOVED. We now compute them internally.
float* compute_features(
    const float* acc_x,        // [IN] Acceleration X from IMU
    const float* acc_y,        // [IN] Acceleration Y from IMU
    const float* acc_z,        // [IN] Acceleration Z from IMU
    const float* pressure,     // [IN] Pressure from FSR
    const float* azimuth,      // [IN] Orientation
    const float* altitude,     // [IN] Orientation
    int32_t      n_samples,    // [IN] Number of samples
    int32_t*     out_size      // [OUT] Will be 354
);

void free_features(float* ptr);
const char* SmartPen_features_version(void);
const char* SmartPen_features_last_error(void);

// Helper functions
void compute_statistical_single(const float* signal, int32_t n, float* out);
void compute_button_status(const float* pressure, int32_t n, uint8_t* out);

#ifdef __cplusplus
}
#endif