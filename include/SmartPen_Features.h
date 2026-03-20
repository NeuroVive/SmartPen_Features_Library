#pragma once // guard
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif
// coding

// Constants
#define  PEN_FEATURES_COUNT 354
#define  PEN_STATIS_COUNT   11
#define  PEN_SAMPLING_RATE  150.0f
#define  PEN_DT             (1.0F / PEN_SAMPLING_RATE)
#define  PEN_MIN_SAMPLES    150  
#define  PEN_PRESSURE_THRESHOLD 0.05f





// Error codes
typedef enum{
            PEN_OK          = 0,
            PEN_ERR_NULL    = 1,
            PEN_ERR_SAMPLES = 2,
            PEN_ERR_MEMORY  = 3,
}PenStatus;



// Main Function (API)
float* compute_features(
    const float* x,        // pointer to array from PMW3901
    const float* y,        // pointer to array from PMW3901
    const float* pressure, // pointer to array from FSR
    const float* azimuth,  // pointer to array from MPU6050
    const float* altitude, // pointer to array from MPU6050
    const float* acc_x,    // pointer to array from MPU6050
    const float* acc_y,    // pointer to array from MPU6050
    int32_t      n_samples, // no.of samples of each array
    int32_t*     out_size  // [out] will be set to 354 on success (354)
);
// @param ptr : pointer to float array returned by compute_function.
/* This function must be called from the host language (dart) 
 * after the 354 features have been processed to prevent memory leaks in 
 * the NeuroVive diagnostic system.
*/
void free_features(float *ptr);

// ── Utilities ─────────────────────────────────────────────
const char* SmartPen_features_version(void);
const char* SmartPen_features_last_error(void);


// ----------------------------
// Helper functions for Testing
// ----------------------------

void compute_statistical_single(
    const float* signal, // array (in)
    int32_t     n, // no. of elements (in)
    float* out // out 11 statical value (max, min, mean, median, ....)
);

void compute_button_status(
    const float* pressure, // array of pressure (in)
    int32_t      n,        // no of elements (in)
    uint8_t*     out       // status_values (out)
    );



#ifdef __cplusplus
}
#endif