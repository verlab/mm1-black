#pragma once

#include <math.h>

/**
 * MM1-BLACK mechanical reference (M11 survey base).
 * Distances in millimetres along the laser beam; IMU offset in body frame (-X).
 *
 * Survey distance at M11 base:
 *   D_m11 = D_laser_raw - (LASER_BASE_OFFSET_MM + M11_BASE_OFFSET_MM)
 *
 * IMU orientation is expressed at the M11 pivot (body -X from IMU chip).
 */

#ifndef MM1_LASER_BASE_OFFSET_MM
#define MM1_LASER_BASE_OFFSET_MM (2.25f)
#endif

#ifndef MM1_M11_BASE_OFFSET_MM
#define MM1_M11_BASE_OFFSET_MM (148.11f)
#endif

/** Laser + M11 mechanical sum (2.25 + 148.11 mm) — fixed in firmware. */
#ifndef MM1_LZR_M11_BASE_SUM_MM
#define MM1_LZR_M11_BASE_SUM_MM (150.36f)
#endif

#ifndef MM1_IMU_TO_M11_X_MM
/** IMU chip → M11 base, along body -X (metres in helpers). */
#define MM1_IMU_TO_M11_X_MM (75.25f)
#endif

static inline float mm1_laser_distance_correction_m(void)
{
    return MM1_LZR_M11_BASE_SUM_MM * 0.001f;
}

/** Raw laser range (m) → distance from M11 base along the beam (m). */
static inline float mm1_distance_at_m11_m(float laser_m)
{
    if (!isfinite(laser_m))
        return laser_m;
    const float d = laser_m - mm1_laser_distance_correction_m();
    return (d > 0.f) ? d : 0.f;
}
