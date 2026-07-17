# Astribot S1 M8 dual-arm full-absolute trial

This trial maps both Quest wrists to absolute robot positions and orientations
inside the calibrated WebXR reference space. It is separate from the relative
checkpoint `fca6526` and requires a fresh runtime calibration after every
supervisor start, WebXR session change, reference-space change, or recenter.

## Start the supervisor

Source the Astribot and ROS environments, keep the physical E-stop in hand,
unload both arms, clear the workspace, and start:

```bash
/usr/bin/python3 -m \
  stardust_wuji_quest3_pc_retargeting.tools.run_control_pc_supervisor \
  --host 0.0.0.0 --port 9001 \
  --arm both --mapping-mode absolute \
  --enable-real-arm \
  --enable-m8-full-absolute \
  --allow-control-takeover \
  --m8-max-linear-speed-mps 1.0 \
  --m8-position-scale 1.5 \
  --m8-position-alpha 1.0 \
  --enable-m8-orientation \
  --m8-rotation-scale 1.0 \
  --m8-max-angular-speed-rad-s 3.0 \
  --m8-hand-reacquire-timeout-sec 5.0 \
  --m8-absolute-reacquire-linear-speed-mps 0.10 \
  --m8-absolute-reacquire-max-position-error-m 0.20 \
  --m8-orientation-reacquire-speed-rad-s 0.5 \
  --m8-orientation-reacquire-max-error-rad 1.57 \
  --absolute-calibration-report logs/m8_absolute_calibration/report.yaml \
  --m8-waiver logs/m7_hardware_audit/m8_operator_waiver.yaml \
  --accept-m8-risk-bundle M8_ACCEPT_ALL_AUTHORIZED_RISKS \
  --interactive
```

The supervisor starts in `IDLE/HOLD` and sends no pose target before a valid
absolute calibration and explicit `start`.

## Runtime calibration

Enter the Quest session and verify that HMD, left hand, and right hand tracking
are all valid. Stand in the intended operator location. Hold the HMD, both
wrists, and the robot stationary, with each wrist in a comfortable neutral
pose, then run the unified engage command:

```text
teleop> E
```

The default calibration has a 3-second countdown followed by 1.5 seconds of
sampling and requires at least 60 valid samples. It checks HMD stability, wrist
position/orientation stability, robot stability, desired/current robot error,
and WebXR session/reference-space consistency. No arm target is sent while
calibration is collecting.

The command performs calibration and starts automatically only after validation.
Proceed only when `status` reports:

```json
{
  "mapping_mode": "absolute",
  "calibration_state": "VALID",
  "teleop_state": "RUNNING",
  "hand_tracking": {"left": true, "right": true}
}
```

Start tests with a 5 mm wrist translation and a small wrist rotation, one arm
at a time. `E` creates fresh position, orientation, operator-frame, and
tool-alignment anchors each time; there is no separate operator calibration
command.

## Tracking recovery

If either hand is lost while fresh HMD/WebXR frames continue, both arms hold.
When both hands return within 5 seconds, the supervisor evaluates the absolute
pose candidates without immediately sending them.

- Errors at or below 0.02 m and 0.15 rad resume through the normal filters.
- Larger accepted errors enter `ABSOLUTE_POSE_CATCHUP`.
- Catch-up is limited to 0.10 m/s and 0.5 rad/s, independently of the normal
  1.0 m/s and 3.0 rad/s teleoperation limits.
- Both absolute position and orientation converge to the current hand targets;
  no relative re-anchor is applied.
- Position candidates above 0.20 m, orientation candidates above 1.57 rad, or
  candidates outside either arm workspace enter `ALIGNMENT_REQUIRED` and send
  no automatic recovery motion.

During `ABSOLUTE_POSE_CATCHUP`, position and orientation recovery motion is
intentional even though `teleop_state` is shown as `PAUSED`. Use `pause` or the
physical E-stop to abort. Normal teleoperation resumes only after both arms stay
within 0.01 m and 0.087 rad for five valid frames.

If recovery times out or alignment is required, restore both hands and either
move them close to the calibrated absolute targets or enter `E` again while
everything is stationary. A WebXR session/reference-space change
always invalidates calibration and requires a new calibration.
