#include "SmartPen_Features.h"
#include <cmath>
#include <cstring>
#include <algorithm>
#include <vector>
#include <cstdlib>

using namespace std;

// ============================================================
// NEW: IMU-Based Position Estimation (ZUPT)
// Replaces Optical Sensor inputs
// ============================================================
static void compute_imu_position(
    const float* acc_x,
    const float* acc_y,
    const float* pressure,
    int32_t      n_samples,
    float*       out_x,
    float*       out_y)
{
    float pos_x = 0.0f;
    float pos_y = 0.0f;
    float vel_x = 0.0f;
    float vel_y = 0.0f;
    
    // Simple gravity compensation factor (approximate)
    // In a full system, we'd rotate using Azimuth/Altitude here.
    // For MVP, we assume acc_x/y contain the linear motion mostly.
    
    for (int i = 0; i < n_samples; i++)
    {
        bool pen_down = (pressure[i] > PEN_PRESSURE_THRESHOLD);
        
        if (!pen_down)
        {
            // ZUPT: When pen is in air, force velocity to 0
            // This kills the drift!
            vel_x = 0.0f;
            vel_y = 0.0f;
            // Position holds its last value or resets (here we hold)
        }
        else
        {
            // Integration: Velocity = Velocity + Acc * dt
            // Note: acc is in 'g', convert to mm/s^2 roughly by scaling
            // Assuming input acc is in 'g': 1g = 9810 mm/s^2
            float ax_mm = acc_x[i] * PEN_ACCEL_TO_MM;
            float ay_mm = acc_y[i] * PEN_ACCEL_TO_MM;
            
            vel_x += ax_mm * PEN_DT;
            vel_y += ay_mm * PEN_DT;
            
            // Optional: Clamp velocity to prevent explosion
            if (vel_x > 5000.0f) vel_x = 5000.0f;
            if (vel_y > 5000.0f) vel_y = 5000.0f;
            
            // Integration: Position = Position + Velocity * dt
            pos_x += vel_x * PEN_DT;
            pos_y += vel_y * PEN_DT;
        }
        
        out_x[i] = pos_x;
        out_y[i] = pos_y;
    }
}

// ============================================================
// SECTION 1: Statistical Helper Functions
// ============================================================
static float calc_median_sorted(const vector<float>& sorted)
{
    int n = (int)sorted.size();
    if (n == 0) return 0.0f;
    if (n % 2 == 0)
    {
        return ((sorted[(n/2)-1] + sorted[n/2])) * 0.5f;
    }
    // if the size is odd
    return sorted[n/2];
}

static float calc_percentile_sorted(
    const vector<float>& sorted,
    float                p)
{
    if (sorted.empty()) return 0.0f;
    int n = (int)sorted.size();
    float idx = p * (n-1);
    int lo = (int)idx;
    int hi = lo + 1;
    if (hi >= n) return sorted[n-1];
    float frac = idx - lo;
    return sorted[lo] * (1.0f - frac) + sorted[hi] * frac;
}

void compute_statistical_single(
    const float* signal,
    int32_t      n,
    float*       out)
{
    // input validation
    if (!signal || n <= 0 || !out)
    {
        // check the valid out pointer
        // set 11 * size(float) by zeros to avoid crashing
        if (out) memset(out, 0, (PEN_STATIS_COUNT * sizeof(float)));
        return;
    }
    
    vector<float> sorted(signal, signal + n);
    sort(sorted.begin(), sorted.end());
    
    // 1- mean
    double sum = 0.0;
    for (int i = 0; i < n; i++) sum += signal[i];
    float mean = (float)(sum / n);
    
    // 2- median
    float median = calc_median_sorted(sorted);
    
    // 3- variance
    double var_sum = 0.0;
    for (int i = 0; i < n; i++)
    {
        var_sum += (signal[i] - mean) * (signal[i] - mean);
    }
    float variance = (float)(var_sum / n);
    
    // 4- standard deviation
    float std_dev = sqrtf(variance);
    
    // 5- max and min
    float max_val = sorted.back();
    float min_val = sorted.front();
    
    // 6- percentiles
    float p1  = calc_percentile_sorted(sorted, 0.01f);
    float p99 = calc_percentile_sorted(sorted, 0.99f);
    
    // 7- displacement between 99th and 1st percentiles
    float displacement = p99 - p1;
    
    // 8- skewness
    float skewness = 0.0f;
    if (std_dev > 1e-10f)
    {
        double skewness_sum = 0.0;
        for (int i = 0; i < n; i++)
        {
            double item = ((signal[i] - mean) / std_dev);
            skewness_sum += item * item * item;
        }
        skewness = (float)(skewness_sum / n);
    }
    
    // 9- kurtosis
    float kurtosis = 0.0f;
    if (std_dev > 1e-10f)
    {
        double kurtosis_sum = 0.0;
        for (int i = 0; i < n; i++)
        {
            double item = ((signal[i] - mean) / std_dev);
            kurtosis_sum += item * item * item * item;
        }
        kurtosis = (float)(kurtosis_sum / n) - 3.0f; // excess kurtosis
    }
    
    // Output the results in the provided output array
    out[0]  = mean;
    out[1]  = median;
    out[2]  = variance;
    out[3]  = std_dev;
    out[4]  = max_val;
    out[5]  = min_val;
    out[6]  = p1;
    out[7]  = p99;
    out[8]  = displacement;
    out[9]  = skewness;
    out[10] = kurtosis;
}

// ============================================================
// SECTION 2: Kinematic Derivation
// ============================================================
static void compute_displacement(
    const float* x,
    const float* y,
    int32_t      n,
    float*       out)
{
    for (int i = 0; i < n-1; i++)
    {
        float dx = x[i+1] - x[i];
        float dy = y[i+1] - y[i];
        out[i] = sqrtf(dx*dx + dy*dy); // Euclidean distance
    }
}

static void compute_axis_disp(
    const float* axis,
    int32_t      n,
    float*       out)
{
    for (int i = 0; i < n-1; i++)
    {
        out[i] = fabsf(axis[i+1] - axis[i]);
    }
}

static void compute_velocity(
    const float* disp,
    int32_t      n,
    float*       out)
{
    for (int i = 0; i < n-1; i++)
    {
        out[i] = disp[i] / PEN_DT;
    }
}

static void compute_acceleration(
    const float* velocity,
    int32_t      n,
    float*       out)
{
    for (int i = 0; i < n-1; i++)
    {
        out[i] = (velocity[i+1] - velocity[i]) / PEN_DT;
    }
}

static void compute_jerk(
    const float* acc,
    int32_t      n,
    float*       out)
{
    for (int i = 0; i < n-1; i++)
    {
        out[i] = (acc[i+1] - acc[i]) / PEN_DT;
    }
}

// ============================================================
// SECTION 3: Button Status & Temporal Features
// ============================================================
void compute_button_status(
    const float* pressure,
    int32_t      n,
    uint8_t*     out)
{
    for (int i = 0; i < n; i++)
    {
        out[i] = (pressure[i] > PEN_PRESSURE_THRESHOLD) ? 1 : 0;
    }
}

static void compute_temporal_features(
    const float*   pressure,
    const uint8_t* button_status,
    int32_t        n,
    float*         out)
{
    int pressure_changes = 0;
    int strokes_num      = 0;
    int in_air_count     = 0;
    int on_surface_count = 0;
    
    // how many stroke?
    for (int i = 1; i < n; i++)
    {
        uint8_t prev = (pressure[i-1] > PEN_PRESSURE_THRESHOLD) ? 1 : 0;
        uint8_t curr = (pressure[i]   > PEN_PRESSURE_THRESHOLD) ? 1 : 0;
        
        // changes in pressure
        if (prev != curr) pressure_changes++;
        
        // stroke => transition from air to surface
        if (button_status[i-1] == 0 && button_status[i] == 1)
        {
            strokes_num++;
        }
    }
    
    // air and surface times
    for (int i = 0; i < n; i++)
    {
        if (button_status[i] == 0) in_air_count++;
        else                       on_surface_count++;
    }
    
    float overall_time     = n * PEN_DT;
    float in_air_time      = in_air_count * PEN_DT;
    float on_surface_time  = on_surface_count * PEN_DT;
    float ratio_in_air     = (overall_time > 0) ? (in_air_time / overall_time) : 0.0f;
    float ratio_on_surface = (overall_time > 0) ? (on_surface_time / overall_time) : 0.0f;
    
    // outputs
    out[0] = (float)pressure_changes;
    out[1] = (float)strokes_num;
    out[2] = in_air_time;
    out[3] = on_surface_time;
    out[4] = ratio_in_air;
    out[5] = ratio_on_surface;
    out[6] = overall_time;
}

// ============================================================
// SECTION 4: Entropy & Energy Features
// ============================================================
static float compute_shannon_entropy(
    const float* signal,
    int32_t      n)
{
    if (n <= 0) return 0.0f;
    
    const int BINS = 32;
    float max_val = *max_element(signal, signal + n);
    float min_val = *min_element(signal, signal + n);
    float range   = max_val - min_val;
    
    if (range < 1e-10f) return 0.0f; // no entropy
    
    vector<int> hist(BINS, 0); // to calc frequency
    
    for (int i = 0; i < n; i++)
    {
        int bin = (int)((signal[i] - min_val) / range * (BINS - 1));
        bin = std::max(0, std::min(BINS - 1, bin));
        hist[bin]++;
    }
    
    float entropy = 0.0f;
    for (int b = 0; b < BINS; b++)
    {
        if (hist[b] > 0)
        {
            float p = (float)hist[b] / n;
            entropy += p * log2f(1.0f / p);
        }
    }
    return entropy;
}

static float compute_renyi_entropy(
    const float* signal,
    int32_t      n,
    float        alpha)
{
    if (n <= 0 || fabsf(alpha - 1.0f) < 1e-6f) return 0.0f;
    
    const int BINS = 32;
    float min_val = *min_element(signal, signal + n);
    float max_val = *max_element(signal, signal + n);
    float range   = max_val - min_val;
    
    if (range < 1e-10f) return 0.0f;
    
    vector<int> hist(BINS, 0);
    
    for (int i = 0; i < n; i++)
    {
        int bin = (int)((signal[i] - min_val) / range * (BINS - 1));
        bin = std::max(0, std::min(BINS - 1, bin));
        hist[bin]++;
    }
    
    float sum_p_alpha = 0.0f;
    for (int b = 0; b < BINS; b++)
    {
        if (hist[b] > 0)
        {
            float p = (float)hist[b] / n;
            sum_p_alpha += powf(p, alpha);
        }
    }
    
    if (sum_p_alpha < 1e-10f) return 0.0f;
    return log2f(sum_p_alpha) / (1.0f - alpha);
}

static float compute_conventional_energy(
    const float* signal,
    int32_t      n)
{
    if (n <= 0) return 0.0f;
    
    double sum = 0.0;
    for (int i = 0; i < n; i++)
    {
        sum += (double)signal[i] * signal[i]; // using double to avoid overflow
    }
    return (float)(sum / n);
}

static float compute_teager_kaiser_energy(
    const float* signal,
    int32_t      n)
{
    if (n < 3) return 0.0f;
    
    double sum = 0.0;
    for (int i = 1; i < n - 1; i++)
    {
        double tke = (double)(signal[i] * signal[i]) - 
                     (double)(signal[i-1] * signal[i+1]);
        sum += tke;
    }
    return (float)(sum / (n - 2));
}

static float compute_snr_conventional(
    const float* signal,
    int32_t      n)
{
    if (n <= 0) return 0.0f;
    
    float energy = compute_conventional_energy(signal, n);
    
    // حساب الـ noise كـ variance
    double sum = 0.0;
    for (int i = 0; i < n; i++) sum += signal[i];
    float mean = (float)(sum / n);
    
    double noise_sum = 0.0;
    for (int i = 0; i < n; i++)
    {
        double d = signal[i] - mean;
        noise_sum += d * d;
    }
    float noise = (float)(noise_sum / n);
    
    if (noise < 1e-10f) return 0.0f;
    return energy / noise;
}

static float compute_snr_teager_kaiser(
    const float* signal,
    int32_t      n)
{
    if (n < 3) return 0.0f;
    
    float tke = compute_teager_kaiser_energy(signal, n);
    
    // noise = variance
    double sum = 0.0;
    for (int i = 0; i < n; i++) sum += signal[i];
    float mean = (float)(sum / n);
    
    double noise_sum = 0.0;
    for (int i = 0; i < n; i++)
    {
        double d = signal[i] - mean;
        noise_sum += d * d;
    }
    float noise = (float)(noise_sum / n);
    
    if (noise < 1e-10f) return 0.0f;
    return fabsf(tke) / noise;
}

// ============================================================
// SECTION 5: Intrinsic Features (EMD approx)
// ============================================================
static void compute_imf_approx(
    const float* signal,
    int32_t      n,
    float*       imf_out)
{
    if (n < 4)
    {
        memcpy(imf_out, signal, n * sizeof(float));
        return;
    }
    
    // حساب الـ local mean كـ moving average
    vector<float> local_mean(n, 0.0f);
    int window = std::max(3, n / 20);
    
    for (int i = 0; i < n; i++)
    {
        int start = std::max(0, i - window/2);
        int end   = std::min(n - 1, i + window/2);
        double sum = 0.0;
        for (int j = start; j <= end; j++) sum += signal[j];
        local_mean[i] = (float)(sum / (end - start + 1));
    }
    
    // IMF = signal - local_mean
    for (int i = 0; i < n; i++)
    {
        imf_out[i] = signal[i] - local_mean[i];
    }
}

static void compute_intrinsic_features_single(
    const float* signal,
    int32_t      n,
    float*       out)
{
    std::vector<float> imf(n);
    compute_imf_approx(signal, n, imf.data());
    
    out[0] = compute_shannon_entropy(imf.data(), n);
    out[1] = compute_renyi_entropy(imf.data(), n, 2.0f);
    out[2] = compute_renyi_entropy(imf.data(), n, 3.0f);
    out[3] = compute_conventional_energy(imf.data(), n);
    out[4] = compute_teager_kaiser_energy(imf.data(), n);
}

// ============================================================
// SECTION 6: Main Feature Extraction Function (REVISED)
// ============================================================
float* compute_features(
    const float* acc_x,
    const float* acc_y,
    const float* acc_z,
    const float* pressure,
    const float* azimuth,
    const float* altitude,
    int32_t      n_samples,
    int32_t*     out_size)
{
    // ── Validation ──────────────────────────────────────────
    if (!acc_x || !acc_y || !acc_z || !pressure || 
        !azimuth || !altitude || !out_size)
    {
        return nullptr;
    }
    
    if (n_samples < PEN_MIN_SAMPLES)
    {
        return nullptr;
    }
    
    *out_size = 0;
    
    // ── Allocate output ─────────────────────────────────────
    float* features = new (std::nothrow) float[PEN_FEATURES_COUNT];
    if (!features) return nullptr;
    
    memset(features, 0, PEN_FEATURES_COUNT * sizeof(float));
    
    // --- STEP 1: Compute X, Y from IMU Data ---
    vector<float> pos_x(n_samples), pos_y(n_samples);
    compute_imu_position(
        acc_x, acc_y, pressure, n_samples, 
        pos_x.data(), pos_y.data());
    
    // --- STEP 2: Compute Kinematics ---
    int n1 = n_samples - 1;
    int n2 = n1 - 1;
    int n3 = n2 - 1;
    
    vector<float> disp(n1), vel(n1), acc(n2), jrk(n3);
    vector<float> x_disp(n1), x_vel(n1), x_acc(n2), x_jrk(n3);
    vector<float> y_disp(n1), y_vel(n1), y_acc(n2), y_jrk(n3);
    
    // Compute from Calculated Position
    compute_displacement(pos_x.data(), pos_y.data(), n_samples, disp.data());
    compute_velocity(disp.data(), n1, vel.data());
    compute_acceleration(vel.data(), n1, acc.data());
    compute_jerk(acc.data(), n2, jrk.data());
    
    // Compute Axis Specific
    compute_axis_disp(pos_x.data(), n_samples, x_disp.data());
    compute_velocity(x_disp.data(), n1, x_vel.data());
    compute_acceleration(x_vel.data(), n1, x_acc.data());
    compute_jerk(x_acc.data(), n2, x_jrk.data());
    
    compute_axis_disp(pos_y.data(), n_samples, y_disp.data());
    compute_velocity(y_disp.data(), n1, y_vel.data());
    compute_acceleration(y_vel.data(), n1, y_acc.data());
    compute_jerk(y_acc.data(), n2, y_jrk.data());
    
    // --- STEP 3: Extract 354 Features ---
    int idx = 0;
    
    // ============================================================
    // LAYER 1: Raw Signal Statistics [0 → 54]
    // 5 signals × 11 stats = 55 features
    // ============================================================

    compute_statistical_single(acc_x, n_samples, features + idx); idx += 11;
    compute_statistical_single(acc_y, n_samples, features + idx); idx += 11;
    compute_statistical_single(pressure, n_samples, features + idx); idx += 11;
    compute_statistical_single(azimuth, n_samples, features + idx); idx += 11;
    compute_statistical_single(altitude, n_samples, features + idx); idx += 11;
    
    // ============================================================
    // LAYER 2: Full Kinematic [55 → 98]
    // 4 signals × 11 stats = 44 features
    // ============================================================
    compute_statistical_single(disp.data(), n1, features + idx); idx += 11;
    compute_statistical_single(vel.data(), n1, features + idx); idx += 11;
    compute_statistical_single(acc.data(), n2, features + idx); idx += 11;
    compute_statistical_single(jrk.data(), n3, features + idx); idx += 11;
    
    // ============================================================
    // LAYER 3: X-axis [99 → 142]
    // 4 signals × 11 stats = 44 features
    // ============================================================
    compute_statistical_single(x_disp.data(), n1, features + idx); idx += 11;
    compute_statistical_single(x_vel.data(), n1, features + idx); idx += 11;
    compute_statistical_single(x_acc.data(), n2, features + idx); idx += 11;
    compute_statistical_single(x_jrk.data(), n3, features + idx); idx += 11;
    
    // ============================================================ 
    // LAYER 4: Y-axis [143 → 186]
    // 4 signals × 11 stats = 44 features
    // ============================================================
    compute_statistical_single(y_disp.data(), n1, features + idx); idx += 11;
    compute_statistical_single(y_vel.data(), n1, features + idx); idx += 11;
    compute_statistical_single(y_acc.data(), n2, features + idx); idx += 11;
    compute_statistical_single(y_jrk.data(), n3, features + idx); idx += 11;
    
    // ============================================================
    // LAYER 5: Signed Stats [187 → 274] (simplified copy for MVP)
    // ============================================================
    memcpy(features + idx, features + 99, 44 * sizeof(float)); idx += 44;
    memcpy(features + idx, features + 143, 44 * sizeof(float)); idx += 44;
    
    // ============================================================
    // LAYER 6: Temporal [275 → 281]
    // ============================================================
    vector<uint8_t> btn(n_samples);
    compute_button_status(pressure, n_samples, btn.data());
    
    float temporal[7];
    compute_temporal_features(pressure, btn.data(), n_samples, temporal);
    for (int i = 0; i < 7; i++) features[idx++] = temporal[i];
    
    // ============================================================
    // LAYER 7-10:   Entropy/Energy/SNR/Intrinsic
    // ============================================================
    const float* entropy_signals[6] = {
        pos_x.data(), pos_y.data(), 
        disp.data(), vel.data(), acc.data(), jrk.data()
    };
    const int entropy_sizes[6] = {n_samples, n_samples, n1, n1, n2, n3};
    
    // Shannon [282 → 287]
    for (int s = 0; s < 6; s++)
    {
        features[idx++] = compute_shannon_entropy(
            entropy_signals[s], entropy_sizes[s]);
    }
    
    // Renyi α=2 [288 → 293]
    for (int s = 0; s < 6; s++)
    {
        features[idx++] = compute_renyi_entropy(
            entropy_signals[s], entropy_sizes[s], 2.0f);
    }
    
    // Renyi α=3 [294 → 299]
    for (int s = 0; s < 6; s++)
    {
        features[idx++] = compute_renyi_entropy(
            entropy_signals[s], entropy_sizes[s], 3.0f);
    }
    
    // Conventional Energy [300 → 305]
    for (int s = 0; s < 6; s++)
    {
        features[idx++] = compute_conventional_energy(
            entropy_signals[s], entropy_sizes[s]);
    }
    
    // Teager-Kaiser Energy [306 → 311]
    for (int s = 0; s < 6; s++)
    {
        features[idx++] = compute_teager_kaiser_energy(
            entropy_signals[s], entropy_sizes[s]);
    }
    
    // SNR Conventional [312 → 317]
    for (int s = 0; s < 6; s++)
    {
        features[idx++] = compute_snr_conventional(
            entropy_signals[s], entropy_sizes[s]);
    }
    
    // SNR Teager-Kaiser [318 → 323]
    for (int s = 0; s < 6; s++)
    {
        features[idx++] = compute_snr_teager_kaiser(
            entropy_signals[s], entropy_sizes[s]);
    }
    
    // Intrinsic EMD [324 → 353]
    for (int s = 0; s < 6; s++)
    {
        float intrinsic[5];
        compute_intrinsic_features_single(
            entropy_signals[s], entropy_sizes[s], intrinsic);
        for (int f = 0; f < 5; f++)
        {
            features[idx++] = intrinsic[f];
        }
    }
    
    // ── Verify index ────────────────────────────────────────
    if (idx != PEN_FEATURES_COUNT)
    {
        delete[] features;
        return nullptr;
    }
    
    *out_size = PEN_FEATURES_COUNT;
    return features;
}

// ============================================================
// SECTION 7: Utility Functions
// ============================================================
void free_features(float* ptr)
{
    delete[] ptr;
}

const char* SmartPen_features_version(void)
{
    return "2.0.0-IMU-Only";
}

const char* SmartPen_features_last_error(void)
{
    return "No Error";
}