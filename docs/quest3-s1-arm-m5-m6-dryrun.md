# Quest3 → Astribot S1 arm teleoperation M5-M6 dry-run

M5-M6 is mock-only. The entry points reject `--enable-real-arm` and do not import or initialize the Astribot SDK. M7-M10 hardware validation remains deferred.

## CLI

Start the WebSocket supervisor in its default `IDLE` state:

```bash
python3 -m stardust_wuji_quest3_pc_retargeting.tools.run_control_pc_supervisor \
  --arm both \
  --mapping-mode relative \
  --interactive
```

The interactive prompt accepts:

```text
recenter
absolute-calibrate
cancel-calibration
invalidate-calibration
calibration-status
mode relative
mode absolute
start
pause
stop
estop
reset
```

Commands may also be queued at startup with repeated `--command`, for example `--command status`. Startup never calibrates, arms, or starts automatically.

The mock WebXR sender reads the supervisor's return channel continuously to prevent WebSocket backpressure. `control_state` replies are rate-limited to approximately 5 Hz; command results remain immediate.

Relative mode requires a fresh frame and `recenter` before `start`. After a Relative pause, a new `recenter` is required. Absolute calibration performs countdown, multi-frame sampling, quality checks, workspace/jump checks, and finishes in `ARMED`; it never starts or sends an arm target automatically. Absolute resume reruns freshness, session/revision, workspace, and jump gates.

## Control panel

```bash
python3 -m stardust_wuji_quest3_pc_retargeting.tools.run_control_pc_panel \
  --arm both
```

Tkinter is imported only when this entry point opens the panel. A headless installation can run the CLI without Tkinter. The panel buttons only submit commands to the same thread-safe supervisor API used by the CLI. Closing the panel queues `pause` by default, controlled by `control_panel.pause_on_close` in `configs/services/control_pc_default.yaml`.

The panel entry point also starts the same Quest WebSocket receiver on `--host/--port` in a background asyncio thread; Tkinter remains on the process main thread.

The red warning is intentional: software E-Stop does not replace the physical E-Stop.

## Ten-minute report

The completed mock run is stored under `logs/m6_dryrun_validation/`:

- `report.yaml`: pass/fail gates, timing percentiles, injected delays, mapping checks, and event timestamps.
- `telemetry.csv`: sampled target pose, speed, frame age, safety state, and deadline counters.
- `telemetry.svg`: dependency-free target-speed plot.
- `mapping_replay.yaml`: Relative and Absolute results for the same non-zero wrist trajectory, checked against the configured axis transform and per-axis scale.

Re-run it with:

```bash
python3 -m stardust_wuji_quest3_pc_retargeting.sim.dryrun_arm_validation \
  --duration-sec 600 \
  --input-rate-hz 60 \
  --output-dir logs/m6_dryrun_validation
```

The authoritative speed limit in the report is measured against control-loop monotonic tick time, the same `dt` used by `ArmSafetyFilter`. The sampled and SDK-return wall-clock speed fields are diagnostic endpoint estimates and can contain scheduler/call-return jitter.
