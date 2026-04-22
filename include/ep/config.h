#ifndef EP_CONFIG_H
#define EP_CONFIG_H

// Physical Constants
#define PIXELS_PER_METER 50.0f
#define GRAVITY_MS2      9.80665f

// Simulation Timing
#define TARGET_FPS       60.0f
#define FIXED_DT         (1.0f / TARGET_FPS)

// World Dimensions (The fix for your error)
#define WORLD_WIDTH_M    16.0f  // 800px / 50
#define WORLD_HEIGHT_M   11.0f  // 550px / 50

// Unit Conversion Macros
#define METERS_TO_PIXELS(m) ((m) * PIXELS_PER_METER)
#define PIXELS_TO_METERS(p) ((p) / PIXELS_PER_METER)

#endif