#pragma once

#if defined(TEL_DRIVE_ONLY) && !defined(TEL_WIRED_MANUAL_ONLY)
#error "The partial-hardware benches are wired-manual only"
#endif
#if defined(TEL_MANUAL_AXES_TEST) && !defined(TEL_DRIVE_ONLY)
#error "The manual-axes bench builds on the drive bench"
#endif
#if defined(TEL_MANUAL_FLYWHEEL_TEST) && !defined(TEL_MANUAL_AXES_TEST)
#error "The flywheel bench requires the manual-axes bench"
#endif

namespace BuildProfile {

#ifdef TEL_WIRED_MANUAL_ONLY
constexpr bool WIRED_MANUAL_ONLY = true;  // no wireless control, no Auto, no auto paths
#else
constexpr bool WIRED_MANUAL_ONLY = false;
#endif

#ifdef TEL_DRIVE_ONLY
constexpr bool PARTIAL_HARDWARE_BENCH = true;
#else
constexpr bool PARTIAL_HARDWARE_BENCH = false;
#endif

#ifdef TEL_MANUAL_AXES_TEST
constexpr bool BENCH_HAS_SHOOTER_AXES = true;
#else
constexpr bool BENCH_HAS_SHOOTER_AXES = false;
#endif

#ifdef TEL_MANUAL_FLYWHEEL_TEST
constexpr bool BENCH_HAS_FLYWHEEL = true;
#else
constexpr bool BENCH_HAS_FLYWHEEL = false;
#endif

constexpr bool DRIVES_SHOOTER_AXES = !PARTIAL_HARDWARE_BENCH || BENCH_HAS_SHOOTER_AXES;
constexpr bool DRIVES_FLYWHEEL = !PARTIAL_HARDWARE_BENCH || BENCH_HAS_FLYWHEEL;
constexpr bool DRIVES_FEED_TOWER = !PARTIAL_HARDWARE_BENCH;
constexpr bool DRIVES_ART = !PARTIAL_HARDWARE_BENCH;

}
