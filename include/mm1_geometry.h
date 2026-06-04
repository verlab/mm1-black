#pragma once

#include <math.h>

/**
 * MM1-BLACK — laser range to survey distance at bottom or top reference.
 *
 * Bottom: D = D_laser + (146 - trim) mm   (add offset to raw laser)
 * Top:    D = D_laser - (2.25 - trim) mm   (subtract from raw laser)
 *
 * Trim shifts laser + IMU X references together (negative trim → more toward bottom).
 *   IMU arm = |imu_x_base + trim|
 */

#ifndef MM1_LZR_BOTTOM_ADD_MM
#define MM1_LZR_BOTTOM_ADD_MM (146.0f)
#endif

#ifndef MM1_LZR_TOP_SUB_MM
#define MM1_LZR_TOP_SUB_MM (2.25f)
#endif

#ifndef MM1_IMU_X_BOTTOM_MM
#define MM1_IMU_X_BOTTOM_MM (-75.254f)
#endif

#ifndef MM1_IMU_X_TOP_MM
#define MM1_IMU_X_TOP_MM (72.856f)
#endif

#ifndef MM1_TRIM_LIMIT_MM
#define MM1_TRIM_LIMIT_MM (300.0f)
#endif

#ifndef MM1_LASER_BASE_OFFSET_MM
#define MM1_LASER_BASE_OFFSET_MM (2.25f)
#endif
#ifndef MM1_M11_BASE_OFFSET_MM
#define MM1_M11_BASE_OFFSET_MM (148.11f)
#endif
#ifndef MM1_IMU_TO_M11_X_MM
#define MM1_IMU_TO_M11_X_MM (75.25f)
#endif

static inline float mm1_imu_x_base_mm(int proj_top)
{
    return proj_top ? MM1_IMU_X_TOP_MM : MM1_IMU_X_BOTTOM_MM;
}

static inline float mm1_imu_arm_mm(int proj_top, float trim_mm)
{
    const float x = mm1_imu_x_base_mm(proj_top) + trim_mm;
    return fabsf(x);
}

/** Effective laser delta (mm): + = add to reading, - = subtract from reading. */
static inline float mm1_laser_delta_mm(int proj_top, float trim_mm)
{
    if (proj_top)
        return -(MM1_LZR_TOP_SUB_MM - trim_mm);
    return MM1_LZR_BOTTOM_ADD_MM - trim_mm;
}

/** Raw laser (m) → distance at active projection (m). */
static inline float mm1_distance_at_ref_m(float laser_m, int proj_top, float trim_mm)
{
    if (!isfinite(laser_m))
        return laser_m;
    const float d = laser_m + mm1_laser_delta_mm(proj_top, trim_mm) * 0.001f;
    return (d > 0.f) ? d : 0.f;
}
