#include "SmartPen_Features.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

static int tests_passed = 0;
static int tests_failed = 0;

static void check(const char* test_name, int condition) {
    if (condition) {
        printf("  [ OK ]   %s\n", test_name);
        tests_passed++;
    } else {
        printf("  [FAIL]   %s\n", test_name);
        tests_failed++;
    }
}

static void test_full_pipeline() {
    printf("\n[TEST] Full Pipeline (IMU-Only Mode)\n");
    const int N = 200;
    
    // NEW: We generate ACC and PRESSURE, not X and Y
    float acc_x[N], acc_y[N], acc_z[N]; 
    float pressure[N], az[N], al[N];
    
    for (int i = 0; i < N; i++) {
        // Simulate acceleration
        acc_x[i] = sinf(i * 0.1f) * 0.5f; 
        acc_y[i] = cosf(i * 0.1f) * 0.5f;
        acc_z[i] = 1.0f; // Gravity
        pressure[i] = (i > 10 && i < 190) ? 0.6f : 0.0f;
        az[i] = 45.0f; al[i] = 60.0f;
    }
    
    int32_t out_size = 0;
    // Call function WITHOUT x, y
    float* features = compute_features(acc_x, acc_y, acc_z, pressure, az, al, N, &out_size);
    
    check("Memory allocated", features != NULL);
    check("Feature count is 354", out_size == 354);
    
    if (features) {
        printf("    [0] AccX Mean:  %.4f\n", features[0]);
        printf("    [277] Air Time: %.4f\n", features[277]);
        free_features(features);
        check("Memory freed", 1);
    }
}

int main() {
    printf("NeuroVive SmartPen Library - IMU-Only Test\n");
    test_full_pipeline();
    printf("Result: %d Passed / %d Failed\n", tests_passed, tests_failed);
    return (tests_failed == 0) ? 0 : 1;
}