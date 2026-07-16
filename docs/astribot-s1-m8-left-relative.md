# Astribot S1 M8 left/dual-arm Relative acceptance

M8 is limited to left-arm or dual-arm operation, Relative mapping, optional
explicitly authorized orientation control, and the operator waiver in
`logs/m7_hardware_audit/m8_operator_waiver.yaml`. The waiver is self-attested
and does not change the M7 report or its `m8_permitted` value. The real adapter
sends only arm names with `use_wbc=false` and `add_default_torso=false`; torso,
chassis, and head command groups remain locked. Dual-arm commands are sent as
one ordered left/right SDK batch.

## Environment

Every Control PC terminal must use the system Python and source all three setups:

```bash
cd /home/zxc/Desktop/wuji/wuji-teleop/wuji-retargeting
conda deactivate
source /home/zxc/cenyj/astribot_sdk/astribot_sdk_ros2-master/env.sh
source /home/zxc/cenyj/astribot_sdk/astribot_sdk_ros2-master/install/setup.sh
source /home/zxc/Desktop/wuji/wuji-teleop/wujihandros2/install/setup.bash
```

Stop WebUI, VR control, joystick control, and every other Astribot SDK client.
`ros2 service list | rg '^/astribot/control_rights$'` must produce no output.
Keep the physical E-stop in hand, unload the arms, clear the workspace, and keep
the robot stationary in safe mode.

## Dry-run gate

Run the mapping replay before each real session:

```bash
/usr/bin/python3 -m \
  stardust_wuji_quest3_pc_retargeting.sim.dryrun_arm_validation \
  --config configs/arm/s1_quest3_default.yaml \
  --mapping-replay-only \
  --output-dir logs/m8_preflight_dryrun
```

Proceed only when `passed: true`. A Quest tracking dry-run must then use the
normal supervisor without `--enable-real-arm`; confirm fresh dual-hand/HMD
tracking, correct axes, recenter, pause, tracking-loss pause, and disconnect
pause before opening a real session.

## Real supervisor

Start the Control PC supervisor only after the dry-run gate:

```bash
/usr/bin/python3 -m \
  stardust_wuji_quest3_pc_retargeting.tools.run_control_pc_supervisor \
  --host 0.0.0.0 --port 9001 \
  --arm both --mapping-mode relative \
  --enable-real-arm \
  --allow-control-takeover \
  --m8-max-linear-speed-mps 1.0 \
  --m8-position-scale 1.5 \
  --m8-hand-reacquire-timeout-sec 5.0 \
  --m8-position-alpha 1.0 \
  --enable-m8-orientation \
  --m8-rotation-scale 1.0 --m8-max-angular-speed-rad-s 3.0 \
  --enable-m8-absolute-orientation-reacquire \
  --m8-orientation-reacquire-speed-rad-s 0.5 \
  --m8-orientation-reacquire-max-error-rad 1.57 \
  --m8-waiver logs/m7_hardware_audit/m8_operator_waiver.yaml \
  --accept-m8-risk-bundle M8_ACCEPT_ALL_AUTHORIZED_RISKS \
  --interactive
```

`--accept-m8-risk-bundle M8_ACCEPT_ALL_AUTHORIZED_RISKS` replaces the five
interactive M8 confirmation phrases. It is valid only when the waiver contains
the same bundled-confirmation token. The explicit real-arm, takeover, speed,
orientation, and waiver options remain required, and all technical safety gates
remain active. An omitted or incorrect token fails closed; without this option,
the original interactive confirmation flow remains available.

The bundled confirmation authorizes the vendor `high_control_rights=True`
handoff from the permanently resident Web backend. The robot must already be
stationary because the handoff stops any motion owned by the prior controller.
Initialization may acquire control rights, but the supervisor remains IDLE/HOLD
and sends no pose target.
If the vendor reports another controller, control rights are false, robot mode
is not `safe`, or the interface is not alive, stop without entering motion.

Keep the Quest wrist stationary, establish fresh tracking, then issue the atomic
Relative command:

```text
teleop> engage
```

`engage` rereads the current robot desired pose, recenters, resets filters, and
enters RUNNING in one control-thread operation. This avoids the hand-motion gap
between separate `recenter` and `start` commands. The separate commands remain
available for diagnostics.
For the first dual-arm run, keep both wrists stationary during `engage`. Move
only the left wrist by 0.005 m, then `pause`; repeat with only the right wrist,
then test simultaneous motion. With position scale 1.5, a 0.01 m Quest wrist
translation requests approximately 0.015 m at the robot before workspace and
speed filtering. After the initial low-speed acceptance,
the normal M8 real-mode speed limit is 0.20 m/s. The separately confirmed
high-speed mode permits up to 1.00 m/s. Position alpha is configurable in
`(0, 1]`; 1.0 disables application-layer position smoothing. Orientation is
opt-in. The CLI no longer contains the previous fixed 0.70 / 1.00 rad/s bounds;
the current waiver authorizes rotation scale 1.00 and angular speed 3.00 rad/s.
Rotation scale remains in `(0, 1]` because 1.0 already applies the full Quest
rotation delta. Requested angular speed must be finite and positive, then is
bounded by the active waiver.
The mapped input-jump gate is 0.03 m, and the start jump gate is 0.01 m. Do not
begin with a 0.10 m operator motion. At position scale 1.5, a single-frame
Quest translation above approximately 0.02 m reaches the mapped 0.03 m jump
gate. Position scale is configurable and the current waiver authorizes up to
2.0; the 1.0 m/s speed limit still applies after scaling.
Use `engage` again before each resumed run.

Test order: left only, right only, both together, forward/back, left/right,
up/down, tracking loss, pause, and WebSocket disconnect. Any unexpected
direction, continued motion, control-right loss, stale-input behavior, or SDK
fault requires immediate pause or physical E-stop. Right-arm-only and Absolute
mode remain outside this waiver. In dual-arm mode, loss of either hand's
tracking first holds both arms without sending new targets. If both hands return
within `--m8-hand-reacquire-timeout-sec`, three consecutive valid frames cause
an automatic position-only Relative re-anchor. The position re-anchor absorbs
large hand translation accumulated outside the tracking volume, while the
orientation alignment established by the original `engage` remains unchanged.

With `--enable-m8-absolute-orientation-reacquire`, a recovered wrist orientation
within 0.15 rad resumes directly. Larger errors up to
`--m8-orientation-reacquire-max-error-rad` enter `ORIENTATION_CATCHUP`: arm
positions remain fixed while each arm follows the shortest quaternion path at
`--m8-orientation-reacquire-speed-rad-s`. Position control resumes only after
both orientation errors remain below 0.087 rad for five valid frames. Errors
above the automatic maximum enter `ALIGNMENT_REQUIRED` and do not command a
rotation. The default maximum 1.57 rad is approximately 90 degrees.

If the hand-reacquire timeout expires, the pause becomes latched and the
operator must restore tracking and use `engage`. The grace period applies only
to hand tracking; WebSocket staleness, HMD loss, WebXR session loss, or any
reference-space/session revision change retains the fast fail-closed pause and
requires a new `engage` orientation alignment.

SDK calls above the 10 ms control period are counted as deadline misses. An
occasional call below 50 ms remains diagnostic; three consecutive calls above
50 ms fault the loop. Status exposes SDK call p50/p95/max latency.
