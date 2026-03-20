// ============================================================
// main.cpp -- NeuroVive SmartPen Feature Library Tests
// ============================================================

#include "SmartPen_Features.h"
#include <stdio.h>
#include <math.h>
#include <string.h>

#ifdef _WIN32
#include <windows.h>
#endif

// ── Test Counters ─────────────────────────────────────────────
static int tests_passed = 0;
static int tests_failed = 0;

// ── Test Helpers ──────────────────────────────────────────────

static void check(const char* test_name, int condition)
{
    if (condition)
    {
        // استخدام [ OK ] بدلاً من الرموز الغريبة لضمان التوافق
        printf("  [ OK ]   %s\n", test_name);
        tests_passed++;
    }
    else
    {
        printf("  [FAIL]   %s\n", test_name);
        tests_failed++;
    }
}

// مقارنة الأرقام العشرية مع نسبة سماحية (Tolerance)
static int float_near(float a, float b, float tol)
{
    return fabsf(a - b) < tol;
}

// ── Test 1: Utility Functions ─────────────────────────────────
static void test_utilities()
{
    printf("\n[TEST 1] Utility Functions\n");

    check("Version string is not NULL", 
          SmartPen_features_version() != NULL);

    check("Version string matches 1.0.0", 
          strcmp(SmartPen_features_version(), "1.0.0") == 0);

    check("Last error string is accessible", 
          SmartPen_features_last_error() != NULL);
}

// ── Test 2: Null Pointer Validation ──────────────────────────
static void test_null_validation()
{
    printf("\n[TEST 2] Null Pointer Validation\n");

    int32_t out_size = 0;
    // اختبار تمرير مؤشرات فارغة (NULL) للتأكد من صمود المكتبة ضد الانهيار
    float* result = compute_features(
        NULL, NULL, NULL, NULL,
        NULL, NULL, NULL,
        150, &out_size
    );

    check("Returns NULL on invalid input", result == NULL);
    check("Output size stays 0 on error",  out_size == 0);
}

// ── Test 3: Minimum Samples Validation ───────────────────────
static void test_min_samples()
{
    printf("\n[TEST 3] Minimum Samples Validation\n");

    float dummy[10] = {0};
    int32_t out_size = 0;

    // إرسال 10 عينات فقط (أقل من الحد الأدنى 150)
    float* result = compute_features(
        dummy, dummy, dummy, dummy,
        dummy, dummy, dummy,
        10, &out_size
    );

    check("Fails if samples < 150 (Buffer protection)", result == NULL);
}

// ── Test 4: Statistical Single Function ──────────────────────
static void test_statistical_single()
{
    printf("\n[TEST 4] Statistical Feature Extraction\n");

    // إشارة اختبارية بسيطة: {1, 2, 3, 4, 5}
    float signal[5] = {1.0f, 2.0f, 3.0f, 4.0f, 5.0f};
    float out[11]   = {0};

    compute_statistical_single(signal, 5, out);

    check("Mean calculation (Expected 3.0)",     float_near(out[0], 3.0f, 0.001f));
    check("Median calculation (Expected 3.0)",   float_near(out[1], 3.0f, 0.001f));
    check("Variance calculation (Expected 2.0)", float_near(out[2], 2.0f, 0.001f));
    check("Max value detection (Expected 5.0)",  float_near(out[4], 5.0f, 0.001f));
}

// ── Test 5: Button Status Detection ──────────────────────────
static void test_button_status()
{
    printf("\n[TEST 5] Pen Contact Status (Pressure Threshold)\n");

    float   pressure[5] = {0.0f, 0.02f, 0.1f, 0.5f, 0.0f};
    uint8_t status[5]   = {0};

    compute_button_status(pressure, 5, status);

    check("Pressure 0.0 -> In-Air (0)",    status[0] == 0);
    check("Pressure 0.02 -> In-Air (0)",   status[1] == 0);
    check("Pressure 0.1 -> On-Surface (1)", status[2] == 1);
    check("Pressure 0.5 -> On-Surface (1)", status[3] == 1);
}

// ── Test 6: Full Feature Extraction Pipeline ──────────────────
static void test_full_pipeline()
{
    printf("\n[TEST 6] Full Pipeline (354 Features)\n");

    const int N = 200;
    float x[N], y[N], pressure[N], az[N], al[N], ax[N], ay[N];

    // توليد إشارات اصطناعية تحاكي حركة القلم
    for (int i = 0; i < N; i++) {
        x[i] = 100.0f + i;
        y[i] = 100.0f + sinf(i * 0.1f) * 10.0f;
        pressure[i] = (i > 10 && i < 190) ? 0.6f : 0.0f;
        az[i] = 45.0f; al[i] = 60.0f;
        ax[i] = 0.01f; ay[i] = 0.01f;
    }

    int32_t out_size = 0;
    float* features = compute_features(x, y, pressure, az, al, ax, ay, N, &out_size);

    check("Memory allocated successfully", features != NULL);
    check("Feature count is exactly 354", out_size == 354);

    if (features) {
        // التحقق من سلامة الأرقام (ليست NaN أو Infinity)
        int clean = 1;
        for (int i = 0; i < out_size; i++) {
            if (isnan(features[i]) || isinf(features[i])) clean = 0;
        }
        check("No corruption (NaN/Inf) found", clean);

        // عينة من النتائج للمعاينة
        printf("\n    --- Feature Samples ---\n");
        printf("    [0]   X Mean:      %.4f\n", features[0]);
        printf("    [11]  Y Mean:      %.4f\n", features[11]);
        printf("    [277] In-Air Time: %.4f\n", features[277]);
        printf("    [282] Shannon Ent: %.4f\n", features[282]);

        free_features(features);
        check("Memory freed safely", 1);
    }
}

// ── Test 7: Free NULL Safety ──────────────────────────────────
static void test_free_null()
{
    printf("\n[TEST 7] Stability Check\n");
    free_features(NULL);
    check("free_features(NULL) does not crash", 1);
}

// ── Main Entry Point ──────────────────────────────────────────
int main()
{
    // ضبط ترميز الشاشة للويندوز لدعم UTF-8 (يمنع الرموز الغريبة)
#ifdef _WIN32
    SetConsoleOutputCP(65001);
#endif

    printf("================================================\n");
    printf("   NeuroVive SmartPen Library - Testing Suite   \n");
    printf("   Library Version: %s\n", SmartPen_features_version());
    printf("================================================\n");

    test_utilities();
    test_null_validation();
    test_min_samples();
    test_statistical_single();
    test_button_status();
    test_full_pipeline();
    test_free_null();

    printf("\n================================================\n");
    printf("   FINAL RESULTS: %d Passed / %d Failed\n", tests_passed, tests_failed);

    if (tests_failed == 0) {
        printf("   STATUS: SUCCESS - All modules stable.\n");
    } else {
        printf("   STATUS: FAILURE - Review log above.\n");
    }
    printf("================================================\n");

    return (tests_failed == 0) ? 0 : 1;
}