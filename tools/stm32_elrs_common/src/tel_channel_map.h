#ifndef TEL_CHANNEL_MAP_H
#define TEL_CHANNEL_MAP_H

/* SUPERSEDED for the robot bridge (2026-10-11): the robot-side bridge and
   tools/stm32_ground_control now use the channel table in
   src/IO/RcControlMap.h (9 channels, "RC,..." line). This header is kept
   only for the older tools/stm32_elrs_ground_bridge, which has not been
   updated to that table.

   Original note: CRSF channel-index assignment shared by the ground-bridge
   firmware (writes these channels from its own control inputs) and the
   robot-bridge firmware (reads them back out and forwards them to the
   Mega as a "CH,..." line -- see stm32_elrs_robot_bridge/src/ch_line_format.h
   for that line's exact field order, which follows this same assignment).
   This is a fresh assignment for the ELRS/CRSF link, NOT a continuation of
   the retired SBUS channel numbering (ch0/ch1/ch2/ch3/ch8) used before
   this project moved off SBUS -- see docs/codex-handoff.md. The Mega-side
   MechLink class documents this same mapping again at its getters. */
#define TEL_CH_DRIVE_FORWARD 0u
#define TEL_CH_DRIVE_TURN    1u
#define TEL_CH_MECHANISM     2u
#define TEL_CH_AUXILIARY     3u /* Chassis starting-side left/right switch */
#define TEL_CH_FIRE          4u /* Shooter semi-auto fire-confirm button */
#define TEL_CH_MODE          5u /* OperatorMode: full-auto/semi-auto/full-manual */

#endif
