# Astribot S1 M7 hardware safety audit

## Status

M7 is **INCOMPLETE** until every onsite scenario in `logs/m7_hardware_audit/report.yaml` is marked `COMPLETE`. Static source inspection and mock tests do not prove physical Orin behavior.

M8-M10 remain prohibited. Do not connect Quest3 tracking, start Relative/Absolute teleoperation, move home, reset joints, or send a target different from the current desired pose.

## Mandatory onsite conditions

Before every command that uses `--enable-live-sdk`:

- An operator holds the physical E-stop and knows how to use it.
- The robot workspace is clear and both arms carry no object.
- The robot is stationary and in safe mode.
- No Web/VR/SDK controller is active.
- The control PC has the intended ROS2/Fast DDS environment.
- Stop immediately if control rights, robot state, desired/current pose, or physical behavior is unclear.

The audit CLI prompts for the exact phrase `M7 READY PHYSICAL ESTOP` before constructing `Astribot`. SDK construction itself is not read-only: the installed SDK calls `acquire_control_rights()` during initialization. If the vendor SDK asks whether it should forcibly acquire another user's control rights, the M7 tool always answers with an empty string and never sends `yes`. This does **not** prove that rights remain unclaimed: the installed SDK has an exception fallback that may create the control-rights service and mark itself as owning control. Treat the live `get_control_rights_status()` snapshot and onsite behavior as authoritative.

## SDK findings

The source hashes and line references are captured in the machine-readable report. The inspected SDK currently:

- uses `/astribot/control_rights` for control ownership;
- may mark itself as owning control in an exception fallback even without a `yes` response;
- creates a 0.1 second heartbeat timer and publishes `[16000000]` while control rights and driver feedback are valid;
- makes `shutdown()` request control transfer, then destroys the heartbeat timer and ROS node;
- has its `atexit.register(self.shutdown)` line disabled;
- has a `__del__()` path that is not equivalent to `shutdown()`;
- may call `os._exit(0)` after repeated missing driver feedback.

Therefore normal exit, Ctrl+C, `SIGKILL`, and network loss must be tested separately. Never infer one from another.

## Preparation

Use a fresh terminal and source the vendor environment exactly as installed onsite:

```bash
cd /home/zxc/Desktop/wuji/wuji-teleop/wuji-retargeting
conda deactivate
source /home/zxc/cenyj/astribot_sdk/astribot_sdk_ros2-master/env.sh
source /home/zxc/cenyj/astribot_sdk/astribot_sdk_ros2-master/install/setup.sh

test "$ROS_DOMAIN_ID" = "25"
test "$RMW_IMPLEMENTATION" = "rmw_fastrtps_cpp"
test "$ROBOT_TYPE" = "S1"
```

Initialize or refresh the static report. This command does not import the SDK:

```bash
python3 -m stardust_wuji_quest3_pc_retargeting.tools.run_astribot_m7_audit \
  --report logs/m7_hardware_audit/report.yaml inspect
```

Before any live confirmation, verify Python dependencies and SDK import in a disposable subprocess. This does not construct `Astribot` and cannot acquire control rights:

```bash
python3 -m stardust_wuji_quest3_pc_retargeting.tools.run_astribot_m7_audit \
  --report logs/m7_hardware_audit/report.yaml preflight
```

Do not run `live-read` unless preflight reports `ready_for_live_confirmation: true`.

Check progress at any time:

```bash
python3 -m stardust_wuji_quest3_pc_retargeting.tools.run_astribot_m7_audit \
  --report logs/m7_hardware_audit/report.yaml status
```

An `INCOMPLETE` status exits with code 2 by design.

Show the single next gated onsite action without importing or initializing the SDK:

```bash
python3 -m stardust_wuji_quest3_pc_retargeting.tools.run_astribot_m7_audit \
  --report logs/m7_hardware_audit/report.yaml next-step
```

While M7 is incomplete, `next-step` also exits with code 2 by design.

## Scenario 1: read-only SDK session

This still acquires or negotiates control rights during SDK construction. It sends no Cartesian target.

In terminal 1:

```bash
python3 -m stardust_wuji_quest3_pc_retargeting.tools.run_astribot_m7_audit \
  --report logs/m7_hardware_audit/report.yaml \
  live-read --enable-live-sdk
```

Observe and record whether the robot stopped, restarted, held, or otherwise changed when control rights were acquired. Verify both desired/current poses are finite and the reported mode is `safe`.

The live-read command pauses before shutdown. While terminal 1 remains open, run the required read-only capture in terminal 2:

```bash
python3 -m stardust_wuji_quest3_pc_retargeting.tools.run_astribot_m7_audit \
  --report logs/m7_hardware_audit/report.yaml \
  ros-capture --scenario sdk_read_only --phase during
```

Return to terminal 1 only after the capture finishes, then type its requested `M7 OBSERVATION VERIFIED` phrase to allow normal SDK shutdown.

After the command exits and the physical/Orin behavior has been verified:

```bash
python3 -m stardust_wuji_quest3_pc_retargeting.tools.run_astribot_m7_audit \
  --report logs/m7_hardware_audit/report.yaml \
  record-observation --scenario sdk_read_only \
  --disposition safe \
  --observation "REPLACE WITH THE ACTUAL ONSITE OBSERVATION"
```

## Scenario 2: exact desired static hold

The command has two flags and two separate confirmations. Immediately before the only send, it rereads both desired poses. It refuses if:

- control rights are false;
- robot alive is false;
- robot mode is not exactly `safe`;
- desired/current position error exceeds 0.02 m;
- desired changes between the preflight read and final read;
- control rights, live robot status, or safe mode changes after the final desired reread;
- any pose or quaternion is invalid.

The 0.02 m threshold is a hard maximum. `--max-desired-current-error-m` may only tighten it; larger, negative, NaN, or infinite values fail before any pose read or command.

```bash
python3 -m stardust_wuji_quest3_pc_retargeting.tools.run_astribot_m7_audit \
  --report logs/m7_hardware_audit/report.yaml \
  static-hold --enable-live-sdk --enable-static-hold
```

The only permitted call is one dual-arm `set_cartesian_pose()` batch with the final desired pose copied without modification, `control_way="filter"`, `use_wbc=False`, and `add_default_torso=True`.

Then record the observed desired/current and physical behavior:

```bash
python3 -m stardust_wuji_quest3_pc_retargeting.tools.run_astribot_m7_audit \
  --report logs/m7_hardware_audit/report.yaml \
  ros-capture --scenario static_hold --phase after
```

```bash
python3 -m stardust_wuji_quest3_pc_retargeting.tools.run_astribot_m7_audit \
  --report logs/m7_hardware_audit/report.yaml \
  record-observation --scenario static_hold \
  --disposition safe \
  --observation "REPLACE WITH THE ACTUAL ONSITE OBSERVATION"
```

## Scenario 3: normal shutdown

Both `live-read` and `static-hold` call `astribot_interface.shutdown()` in `finally`, but the required `normal_exit` runtime evidence is the clean shutdown of the read-only `live-read` session. Observe control rights, desired/current feedback, heartbeat/control nodes, and physical arm behavior after that process exits.

```bash
python3 -m stardust_wuji_quest3_pc_retargeting.tools.run_astribot_m7_audit \
  --report logs/m7_hardware_audit/report.yaml \
  ros-capture --scenario normal_exit --phase after
```

```bash
python3 -m stardust_wuji_quest3_pc_retargeting.tools.run_astribot_m7_audit \
  --report logs/m7_hardware_audit/report.yaml \
  record-observation --scenario normal_exit \
  --disposition safe \
  --observation "REPLACE WITH THE ACTUAL NORMAL-SHUTDOWN OBSERVATION"
```

## Scenario 4: Ctrl+C

Take the before capture:

```bash
python3 -m stardust_wuji_quest3_pc_retargeting.tools.run_astribot_m7_audit \
  --report logs/m7_hardware_audit/report.yaml \
  ros-capture --scenario ctrl_c --phase before
```

```bash
python3 -m stardust_wuji_quest3_pc_retargeting.tools.run_astribot_m7_audit \
  --report logs/m7_hardware_audit/report.yaml \
  monitor --scenario ctrl_c --enable-live-sdk
```

After several snapshots, press Ctrl+C once. The tool records the signal and calls SDK shutdown. Verify the actual robot and Orin behavior, then record it:

The monitor restores Python's default SIGINT handler after SDK initialization because
ROS 2 replaces it. A single terminal Ctrl+C must therefore enter the monitor's
`KeyboardInterrupt` cleanup path and print that SDK shutdown returned.

```bash
python3 -m stardust_wuji_quest3_pc_retargeting.tools.run_astribot_m7_audit \
  --report logs/m7_hardware_audit/report.yaml \
  ros-capture --scenario ctrl_c --phase after
```

```bash
python3 -m stardust_wuji_quest3_pc_retargeting.tools.run_astribot_m7_audit \
  --report logs/m7_hardware_audit/report.yaml \
  record-observation --scenario ctrl_c \
  --disposition safe \
  --observation "REPLACE WITH THE ACTUAL CTRL-C OBSERVATION"
```

## Scenario 5: process SIGKILL

First capture the ROS graph and one sample of each endpoint feedback topic. The command is read-only but requires onsite confirmation:

```bash
python3 -m stardust_wuji_quest3_pc_retargeting.tools.run_astribot_m7_audit \
  --report logs/m7_hardware_audit/report.yaml \
  ros-capture --scenario process_kill --phase before
```

Start the monitor:

```bash
python3 -m stardust_wuji_quest3_pc_retargeting.tools.run_astribot_m7_audit \
  --report logs/m7_hardware_audit/report.yaml \
  monitor --scenario process_kill --enable-live-sdk
```

The monitor first records a live snapshot and refuses to continue unless control rights are held, the robot is alive, and mode is exactly `safe`. It then requires the exact phrase `M7 READY MANUAL SIGKILL` before printing its PID. In a second terminal, only after that confirmation:

```bash
kill -9 PID_PRINTED_BY_THE_MONITOR
```

`SIGKILL` cannot execute Python `finally`, `__del__`, or SDK shutdown. Observe the physical robot and Orin graph before starting any replacement SDK client. Then capture the after state:

First record that the manual kill actually occurred. This requires the exact phrase `M7 SIGKILL PERFORMED AND OBSERVED`. The command also checks the recorded monitor PID and Linux process start identity; it refuses while the original monitor process instance is still running:

```bash
python3 -m stardust_wuji_quest3_pc_retargeting.tools.run_astribot_m7_audit \
  --report logs/m7_hardware_audit/report.yaml \
  record-failure-event --scenario process_kill
```

Then capture the after state; its timestamp must be later than the verified failure event:

```bash
python3 -m stardust_wuji_quest3_pc_retargeting.tools.run_astribot_m7_audit \
  --report logs/m7_hardware_audit/report.yaml \
  ros-capture --scenario process_kill --phase after
```

Finally record the onsite observation. The report refuses to complete this scenario unless both captures exist:

```bash
python3 -m stardust_wuji_quest3_pc_retargeting.tools.run_astribot_m7_audit \
  --report logs/m7_hardware_audit/report.yaml \
  record-observation --scenario process_kill \
  --disposition safe \
  --observation "REPLACE WITH THE ACTUAL SIGKILL OBSERVATION"
```

## Scenario 6: control-PC network disconnect

The tool never disables a network interface. First capture the before state:

```bash
python3 -m stardust_wuji_quest3_pc_retargeting.tools.run_astribot_m7_audit \
  --report logs/m7_hardware_audit/report.yaml \
  ros-capture --scenario network_disconnect --phase before
```

Start the monitor:

```bash
python3 -m stardust_wuji_quest3_pc_retargeting.tools.run_astribot_m7_audit \
  --report logs/m7_hardware_audit/report.yaml \
  monitor --scenario network_disconnect --enable-live-sdk
```

The monitor first requires a valid live snapshot with control rights, robot alive, and mode exactly `safe`, then requires the exact phrase `M7 READY PHYSICAL NETWORK DISCONNECT`. Only after that prompt, physically disconnect the approved control-PC network cable. Keep the physical E-stop ready. Observe how quickly control rights, heartbeat, desired/current feedback, and physical arm behavior change. Reconnect the cable only after the operator declares it safe. If the monitor exits or errors, do not assume shutdown reached Orin.

After reconnection, record that the physical disconnect actually occurred. This requires the exact phrase `M7 NETWORK DISCONNECT PERFORMED AND OBSERVED`:

```bash
python3 -m stardust_wuji_quest3_pc_retargeting.tools.run_astribot_m7_audit \
  --report logs/m7_hardware_audit/report.yaml \
  record-failure-event --scenario network_disconnect
```

Capture the after state only after recording that event, then record the actual observation:

```bash
python3 -m stardust_wuji_quest3_pc_retargeting.tools.run_astribot_m7_audit \
  --report logs/m7_hardware_audit/report.yaml \
  ros-capture --scenario network_disconnect --phase after
```

If the monitor terminal is still running, return to it and press Ctrl+C once so the tool records the post-reconnection shutdown result.

The installed SDK may instead call `os._exit(0)` after repeated missing driver feedback. If the monitor exited by itself, do **not** claim that SDK shutdown ran. Verify and record that the original monitor process instance is absent; this requires the exact phrase `M7 MONITOR PROCESS ABSENT AFTER DISCONNECT`:

```bash
python3 -m stardust_wuji_quest3_pc_retargeting.tools.run_astribot_m7_audit \
  --report logs/m7_hardware_audit/report.yaml \
  record-monitor-absence
```

Use exactly one of these outcomes: recorded Ctrl+C/shutdown result when the monitor remains alive, or verified process absence when it self-exited. Then record the actual observation:

```bash
python3 -m stardust_wuji_quest3_pc_retargeting.tools.run_astribot_m7_audit \
  --report logs/m7_hardware_audit/report.yaml \
  record-observation --scenario network_disconnect \
  --disposition safe \
  --observation "REPLACE WITH THE ACTUAL NETWORK-DISCONNECT OBSERVATION"
```

## Completion gate

M7 is complete only when `status` prints:

```text
status: COMPLETE
completion_reasons: []
m8_permitted: true
```

The status command recalculates completion from full SDK snapshots, exact confirmations, static-hold command contents, process identity evidence, recorded runtime actions, confirmed onsite observations, and at least one valid confirmed ROS capture containing successful service-list plus all four endpoint `topic info` and one-shot `topic echo` commands for every required phase. Editing YAML status fields or setting only `capture.valid: true` cannot bypass this gate. Failed capture attempts remain in the audit history and may be followed by a successful retry.

`--disposition safe` in the examples is not a target value. Select `safe`, `unsafe`, or `unknown` strictly from the onsite physical observation. Six measured scenarios can make `status: COMPLETE`, but any `unsafe` or `unknown` result keeps `m8_permitted: false` and M8 prohibited.

Once a scenario is `COMPLETE`, its observation and disposition are immutable in that report. If an onsite result was entered incorrectly, preserve the report as audit history and start a new report file; do not edit or overwrite the prior conclusion.

Until both `status: COMPLETE` and `m8_permitted: true`, do not begin M8 single-arm motion. The machine-readable report must retain timestamps, environment, SDK source hashes, all SDK snapshots, before/after ROS captures, confirmations, literal onsite observations, and dispositions.
