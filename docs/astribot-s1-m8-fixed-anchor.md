# Astribot S1 M8 dual-arm fixed-anchor trial

This profile records the current Quest wrist frames and current robot desired
end-effector poses as fixed calibration anchors. Runtime commands are six-DoF
increments from those anchors. Tracking recovery never replaces the anchors.

## Start

Compact validated real-hardware profile:

```bash
/usr/bin/python3 -m \
  stardust_wuji_quest3_pc_retargeting.tools.run_control_pc_supervisor \
  --run-m8-fixed-anchor-real \
  --host 0.0.0.0 --port 9001 \
  --arm both --mapping-mode relative \
  --m8-position-scale 2.0 \
  --m8-rotation-scale 1.0
```

The profile switch explicitly selects real dual-arm hardware and applies the
fixed-anchor, init-joint recovery, orientation, tracking recovery, control
takeover, expanded workspace, 2 m/s speed, waiver/bundled confirmation, and
interactive-console settings shown in the full command below. Torso, chassis,
head, and real hand control remain disabled.

Equivalent expanded command:

```bash
/usr/bin/python3 -m \
  stardust_wuji_quest3_pc_retargeting.tools.run_control_pc_supervisor \
  --host 0.0.0.0 --port 9001 \
  --arm both --mapping-mode relative \
  --enable-real-arm \
  --enable-m8-fixed-anchor \
  --enable-m8-init-recovery \
  --confirm-m8-init-recovery \
  --m8-init-recovery-duration-sec 4.0 \
  --allow-control-takeover \
  --m8-max-linear-speed-mps 2.0 \
  --confirm-m8-very-high-speed \
  --m8-position-scale 1.5 \
  --m8-workspace-limit-m 2.0 \
  --confirm-m8-expanded-workspace \
  --m8-position-alpha 1.0 \
  --m8-orientation-alpha 1.0 \
  --enable-m8-orientation \
  --m8-rotation-scale 1.0 \
  --m8-max-angular-speed-rad-s 3.0 \
  --m8-hand-reacquire-timeout-sec 5.0 \
  --m8-hand-reacquire-stable-frames 12 \
  --m8-engage-stable-frames 12 \
  --m8-engage-timeout-sec 5.0 \
  --m8-engage-hold-sec 0.25 \
  --m8-engage-soft-start-sec 0.50 \
  --m8-arm-wrist-invalid-grace-frames 2 \
  --m8-anchor-reacquire-linear-speed-mps 0.10 \
  --m8-anchor-reacquire-linear-accel-mps2 0.30 \
  --m8-anchor-reacquire-angular-accel-rad-s2 1.50 \
  --m8-anchor-reacquire-max-position-error-m 0.20 \
  --m8-orientation-reacquire-speed-rad-s 0.5 \
  --m8-orientation-reacquire-max-error-rad 1.57 \
  --m8-waiver logs/m7_hardware_audit/m8_operator_waiver.yaml \
  --accept-m8-risk-bundle M8_ACCEPT_ALL_AUTHORIZED_RISKS \
  --interactive
```

Use the Quest page through the Orin gateway with explicit VR hand tracking:

```text
http://127.0.0.1:8443/?xr=vr
```

## Calibrate and start

Keep both wrists in comfortable neutral poses and keep the robot stationary:

```text
teleop> E
```

`E`/`e` is the short alias for the mode-aware `engage` command. In this
fixed-anchor relative profile it collects 12 stable HMD/wrist frames, freezes
the current HMD yaw as the operator direction, averages both wrist anchors,
and reads both robot current Cartesian poses. It then holds the current robot
poses for 0.25 seconds and applies a 0.50-second smoothstep ramp to position and
orientation before entering `RUNNING`. HMD pitch/roll are not mapped, and later
head motion does not move the arms; the frozen operator direction changes only
on the next `E`.

The older two-step calibration commands are no longer exposed to the operator.
Absolute mode continues to use its separate multi-frame absolute calibration
internally and also starts automatically after validation.

`P`/`p` is the short alias for `pause`; `R`/`r` is the short alias for
`recover-init`; `S`/`s` is the short alias for `status`.

An operator `P` freezes the last accepted dual-arm Cartesian target and keeps
sending that exact target at the configured SDK rate. WebXR frames are not
consumed while paused. This preserves the vendor online-controller cadence and
avoids reconstructing the WBC/IK path on the next `E`. Confirm this with
`pause_hold_active=true`, increasing `paused_hold_cycles`, and an approximately
constant `sdk_last_call_interval_ms` in `S`.

After an operator pause, use `E` again. Keep the HMD and both wrists still until
`engage_state` changes from `STABILIZING` through `SOFT_START` to `IDLE` and
`teleop_state` becomes `RUNNING`. Standalone `start` is not an operator command.

## Tracking recovery

If either hand is lost, both arms hold. If both hands return within the configured
timeout, candidates are computed from the original anchors.

- Arm tracking uses the independent `arm_wrists` channel. The original full-hand
  `hands.valid` and 25-joint payload are unchanged for dexterous-hand consumers.
- Up to 2 isolated invalid wrist frames during catch-up hold the arms without
  restarting recovery. A longer wrist loss starts a new recovery attempt.
- Recovery waits for 12 consecutive valid tracking frames by default.
- Only errors at or below 0.002 m and 0.02 rad bypass catch-up.
- Larger accepted errors enter `FIXED_ANCHOR_CATCHUP`.
- Catch-up is limited to 0.10 m/s and 0.5 rad/s, with linear and angular
  acceleration ramps of 0.30 m/s² and 1.50 rad/s².
- Catch-up must remain within 0.002 m and 0.02 rad for 10 frames before normal
  limits resume.
- The original anchors remain unchanged after catch-up.
- Errors above 0.20 m or 1.57 rad, or candidates outside the configured arm
  workspaces, enter `ALIGNMENT_REQUIRED` without automatic recovery motion.

During `FIXED_ANCHOR_CATCHUP`, the status reports `teleop_state=PAUSED`, but the
robot intentionally follows the bounded recovery trajectory. `pause` or the
physical E-stop aborts it.

Diagnostics distinguish `hand_tracking` from `arm_wrist_tracking` and report
`reacquire_candidate_positions_m`, `reacquire_workspace_violations`, loss-event
counts, catch-up interruptions, and successful recovery counts.

A discontinuous wrist pose can occur without WebXR ever publishing an invalid
wrist frame. If the normal input-jump filter rejects such a frame, fixed-anchor
mode now converts that rejection into the same stabilized recovery flow instead
of remaining silently in `HOLD`. `last_arm_filter_rejections` and
`hand_reacquire_trigger_reason` identify this path in `status`.

`position_alpha=1.0` and `orientation_alpha=1.0` disable application-side pose
smoothing. Alpha values above 1 are intentionally rejected because they are
extrapolation gains and can overshoot or oscillate.

For experimental response compensation, use bounded position lead rather than
alpha above 1:

```text
--m8-position-lead-sec 0.02
--m8-max-position-lead-m 0.03
--confirm-m8-position-lead
```

Start with lead disabled (`0.0`). The authorized maxima are 0.05 s and 0.05 m.

The optional 2 m workspace profile expands the left arm to `[0,2] x [0,2] x
[0,2]` and the right arm to `[0,2] x [-2,0] x [0,2]` in the chassis frame. It
keeps the ground, rear, and arm-midline guards. The automatic recovery error
gate remains 0.20 m; larger offsets require `clutch-resume` instead of commanding
a long automatic catch-up trajectory.

## Position-only clutch

If the operator reaches an uncomfortable position limit, pause and run:

```text
teleop> clutch-resume
```

This replaces only the translation anchors with the current hand and robot
positions and resumes in the same control-thread operation. The original
wrist-to-end-effector orientation anchors remain unchanged. A WebXR session,
reference-space, or revision change makes the fixed anchor invalid; run
`engage` (`E`) again rather than using the clutch.

If the preserved orientation anchor produces more than the normal start gate
(0.35 rad) but no more than the automatic recovery gate (1.57 rad),
`clutch-resume` now accepts the command and starts bounded orientation catch-up.
Only errors above the recovery gate require `engage` (`E`) to replace the
orientation anchor.

## Explicit return-to-init recovery

When tracking recovery or clutch alignment cannot be completed, request the
recorded init-joint recovery. The command pauses first as one atomic operation:

```text
teleop> R
```

The command uses only the `astribot_arm_left` and `astribot_arm_right` desired
joint rows recorded in the vendor SDK `init_joints.md`. It does not command the
chassis, torso, head, grippers, or Wuji hands. The SDK checks joint limits,
moves both arms as one blocking batch over the configured duration, verifies
the resulting joint error, clears the old fixed anchors, and remains `PAUSED`.
It does not require wrist tracking during the return motion and does not
automatically recalibrate or restart teleoperation.

After the exact joint move, `R` waits for five consecutive low-velocity and
low-acceleration samples before changing control type. It then performs the
single joint-to-Cartesian transition inside `R`, using the measured current
Cartesian pose, and continuously holds that pose while paused. A mild
control-type transition may therefore be felt during `R`; the following `E`
only replaces the HMD/wrist anchors and should not introduce another mode
transition or vibration. After the hold is established, `R` reads the arm
joints again and verifies that Cartesian takeover did not move away from the
recorded unique init-joint posture. Any driver error, failed settle gate, or
post-handoff joint error faults closed and requires a process restart.

The command is unavailable in `ESTOP` or `FAULT`, but does not require a
separate preceding `P`. After the arms stop at the init joint pose, restore
stable wrist tracking, choose a comfortable hand pose, and enter `E` to create
fresh anchors and start again.
