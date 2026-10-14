#pragma once

// Which hardware this build drives. The production env defines none of the
// TEL_* bench flags; the wired bench envs in platformio.ini add them:
//
//   env                       flags                                   drives
//   megaatmega2560            (none)                                  everything
//   mega_wired_manual_test    WIRED_MANUAL_ONLY                       everything, USB only
//   mega_drive_conveyor_test  + DRIVE_CONVEYOR_ONLY                   chassis + conveyor
//   mega_manual_axes_test     + MANUAL_AXES_TEST + MANUAL_FLYWHEEL_TEST  + turret, elevation, flywheel
//
// This is the only place that tests the macros. Everything else reads the
// constants below, so a bench restriction is one `if`, not an #ifdef.

#if defined(TEL_DRIVE_CONVEYOR_ONLY) && !defined(TEL_WIRED_MANUAL_ONLY)
#error "The partial-hardware benches are wired-manual only"
#endif
#if defined(TEL_MANUAL_AXES_TEST) && !defined(TEL_DRIVE_CONVEYOR_ONLY)
#error "The manual-axes bench builds on the drive+conveyor bench"
#endif
#if defined(TEL_MANUAL_FLYWHEEL_TEST) && !defined(TEL_MANUAL_AXES_TEST)
#error "The flywheel bench requires the manual-axes bench"
#endif

namespace BuildProfile {

#ifdef TEL_WIRED_MANUAL_ONLY
constexpr bool WIRED_MANUAL_ONLY = true;  // no wireless control, no Auto, L stays manual
#else
constexpr bool WIRED_MANUAL_ONLY = false;
#endif

#ifdef TEL_DRIVE_CONVEYOR_ONLY
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

// What actually gets driven.
constexpr bool DRIVES_SHOOTER_AXES = !PARTIAL_HARDWARE_BENCH || BENCH_HAS_SHOOTER_AXES;
constexpr bool DRIVES_FLYWHEEL = !PARTIAL_HARDWARE_BENCH || BENCH_HAS_FLYWHEEL;
// The feed tower is never wired on a partial bench.
constexpr bool DRIVES_FEED_TOWER = !PARTIAL_HARDWARE_BENCH;

}  // namespace BuildProfile
