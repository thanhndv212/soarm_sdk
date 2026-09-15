# Changelog

All notable changes to `soarm-sdk` are documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Changed

- **`soarm-calibrate` renamed to `soarm-reconfigure`.** It has never
  touched the tick↔URDF-frame calibration this package's *other*
  `soarm-*calibrat*` commands do — it writes servo EEPROM registers (IDs,
  angle limits, speed, torque, baud), the same operation as the
  dashboard's Reconfigure tab — but sharing the word "calibrate" with
  them read as if it did. The module moved with it:
  `soarm_sdk.cli.calibrate` → `soarm_sdk.cli.reconfigure`.

### Removed

- **`soarm-seed-calibration`**, and `soarm_sdk.calibration.seed_from_lerobot`
  it wrapped. Seeding a calibration from an existing lerobot calibration
  file's travel ranges measures whatever produced that file, not this
  arm — and it was never more than a rough, unvalidated starting point
  regardless (direction signs still assumed, not measured). The one
  remaining way to build a calibration is `soarm-calibrate-rom`, which
  measures the arm's own hard stops, or the guided
  `soarm-dashboard-calibration` workflow built on top of it. Existing
  code that imported `seed_from_lerobot` directly has no replacement —
  `seed_from_travel` (the generic function it thinly wrapped) is
  unaffected.

### Added

- **`soarm-dashboard-setup` now loads every tab** (Start Up, Homing Wizard,
  Reconfigure, Command Panel, PID Tuning, Monitor, Recorder) — what the
  bare `soarm-dashboard` script used to load. That bare script is removed:
  every dashboard now has its own named script (`soarm-dashboard-setup`,
  `soarm-dashboard-calibration`), so there is no default you have to
  already know about to reach. `examples/viser_dashboard.py` is likewise
  replaced by `examples/setup_dashboard.py` and the new
  `examples/calibration_dashboard.py`, one per script, for running from a
  checkout without installing.

- **The Calibration tab is now four tabs, worked in order, not one tab of
  six stacked folders.** *Tolerances*, *Signs*, *Travel*, *Zeros* — split by
  which acceptance row each owns (`TAB_STAGES`), each ending in its own
  Review & save showing that tab's own rows plus the full record, since
  Save writes the whole file and can refuse on a row owned by a tab you
  have not opened. Replaces `build_calibration_panel` with
  `build_calibration_panels` (plural); the CLI registers all four.

- **A translucent reference-pose ghost in the 3-D view**
  (`soarm_sdk.dashboard.fk.load_ghost_meshes` / `pose_meshes`,
  `DashboardApp.show_ghost`). Shows the pose an operator is being asked to
  physically match — set from the pose's own configuration, never from
  servo readings — instead of leaving "fold the arm flat against itself"
  as prose to interpret. Toggled from the Zeros tab; hidden by default and
  shares the mirror's link geometry via `add_mesh_simple` (the only Viser
  mesh call that takes `opacity`).

- **Direction signs are six per-joint dropdowns** (`SIGN_UNCHECKED` /
  `SIGN_OK` / `SIGN_INVERTED`), replacing a free-text "what did you check"
  field. Selecting *Opposite* flips that joint's sign **immediately** —
  `JointCalibration.with_direction_flipped()` — so the mirror reverses
  while the operator is still on that joint, without waiting for a
  Confirm click. A flip drops any pose-anchored provenance and resets
  `validated`, since the zero was solved under the sign being abandoned.

- **Withdraw an unsupported pose claim** (`with_claim_withdrawn()`,
  `_unsupported_pose_claims`). A joint labelled `reference_pose` when the
  recorded witness does not constrain it is a false claim, and on this
  arm's own file four of six joints carried one. Downgrade-only by
  construction — it can remove a claim, never manufacture one — so it
  cannot be used to make an unverified zero look verified.

- **Measuring travel in the dashboard now actually feeds the ROM
  acceptance row.** Both the automatic sweep and the manual recorder
  funnel through one `on_endpoints` callback into
  `rom_endpoint_samples` — previously written only by the standalone
  `soarm-calibrate-rom`, so the dashboard's own travel controls could
  never satisfy the row they exist to feed, however carefully they were
  used. Also writes the measured stops onto the joints' own
  `tick_min`/`tick_max` (`JointCalibration.with_travel`), and flags a
  joint whose recorded span crosses the encoder's 4095/0 wrap — that span
  is the encoder's range, not the joint's, and no calibration zero can
  fix it; `soarm-calibrate-rom --recentre` can.

- **Discard one recorded ROM pass** without re-measuring the rest. A
  stalled sweep or a mis-click counts as real evidence otherwise and drags
  the whole joint's repeatability number with it — one degenerate
  `min == max` sample turned a joint that swept cleanly twice into
  "repeatable to 2215 ticks". Validated against the current list at click
  time (label and index), so a pass recorded in between is never silently
  discarded by a stale position.

- **A per-joint Reset button** beside each joint's Record Min/Max in the
  manual recorder, clearing just that joint's two recorded values —
  previously the only way to undo a bad entry was to overwrite it.
  Record/Reset now sit directly under that joint's own live-value label
  instead of in one eighteen-button block below a shared table.

- **`soarm_sdk.calibration.limits.effective_limits`**: accepted ROM travel
  now *replaces* a model's declared joint limits for `ServoRobot` and for
  `soarm_tamp`'s planner, rather than always being intersected with them —
  intersection silently keeps the more conservative number even once the
  travel is trustworthy, which is what clamped a planned trajectory on 55%
  of its waypoints. Gated on `measured_is_trusted` (the ROM acceptance row
  passing: repeated, non-simulated, within tolerance), because an
  unaccepted sweep can be the *encoder's* range on a wrapped joint —
  handing that to a planner unconditionally would be worse than staying
  conservative.

- Suggested starting values for the acceptance-tolerance form (3 deg pose
  repeatability, 3 deg model deviation, 30 tick ROM repeatability) —
  starting points to review and commit, never a fallback: an unrecorded
  calibration still has no tolerances, and the acceptance row still
  blocks on it.

### Fixed

- **`soarm_sdk.kinematics.urdf_fk.MEMBERS` measured the wrong line.** A
  member's pitch was the chord between two *joint-frame origins*, not the
  member's own body axis — on the SO-101 those origins sit ~14 deg off the
  upper arm's axis and ~3 deg off the forearm's. Every reference pose here
  had been solved to level that chord, so `folded_flat` rendered with its
  upper arm visibly sloped while reporting itself level, and was 11.75 deg
  from its own defining constraint (the two links resting face to face).
  `MEMBERS` now carries each member's body axis (from its shell mesh's
  oriented bounding box); `folded_flat` re-solves to the clean
  `(0, -pi/2, +pi/2, 0, 0, 0)` and is now inside the URDF's elbow limit
  (it previously overshot by 9.4 deg and rendered self-intersecting).
  `LEVEL` turns out to be the URDF's own zero after all — the
  `rezero_from_pose` docstring this module was written to correct was
  right the first time. **Any zero pinned against the previous pose values
  is off by that offset** (~14 deg on `shoulder_lift`, ~16 deg on
  `elbow_flex`) and should be re-pinned.

- **`wrist_roll`'s assumed direction sign, in every "no info supplied"
  fallback.** `soarm_sdk.calibration.frame.DEFAULT_DIRECTION_SIGN_OVERRIDES`,
  `conversions.SOARM100_DIRECTION_SIGNS`, and `configs/so101.yaml`'s
  `hardware.direction_signs` now all assume `wrist_roll = -1`, the other
  five `+1` — previously all six defaulted to `+1`. These are still
  assumptions, not certifications: `seed_from_travel` returns
  `validated=False` regardless of which sign it used, and Step 2's live
  check is what actually confirms it. (This default was itself revised
  mid-investigation from a live check contradicting the indirect
  hard-stop-landing method that first established it — see the Sep-14
  entry in a real arm's `notes["superseded_validated_by"]`.)

- **Fine-alignment sliders ran a flat +-45 deg on every joint.** Matched no
  joint on this arm — short of the gripper's jaw travel, nowhere near
  `wrist_roll`'s -157..+163, and symmetric on a joint (the gripper) that
  is not. Each slider is now bounded by that joint's own URDF travel
  (`_alignment_range`).

- **A refused Save was indistinguishable from a successful one.** Its
  reply went only to a status line built above Step 1 — several screens
  above the Save button at the bottom — and multi-line messages (the
  refusal is a markdown table) were wrapped in `*emphasis*`, which
  renders literal asterisks around broken rows. A result now appears
  directly beside every tab's Save button as well.

- A long provenance note (`validated_by`) was printed on screen verbatim —
  956 characters on this arm's own file, longer than every other word on
  the tab combined. Reduced to a headline plus a "+N more notes" count;
  the full text stays in the calibration file, and pipe-joined records are
  counted rather than run together into one sentence about neither.

### Changed

- Prose across all four calibration tabs trimmed by roughly two thirds —
  reasoning that belongs in a docstring moved there; only what prevents a
  mistake stayed on screen.

- **Guided calibration acceptance pipeline.** `CalibrationPipeline` records
  fail-closed evidence for explicit per-arm tolerances, physical
  direction-sign verification, repeated ROM endpoints, named-pose zero
  provenance, and pose repeatability. The new
  `soarm-dashboard-calibration` Viser entry point keeps this workflow in the
  SDK; TAMP consumes its accepted result read-only.

- **The background bus thread now reads the whole telemetry block.** The
  sync-read group widened from 4 bytes (position + speed) to the full
  read-only SRAM span, addresses 56-70: position, speed, load, voltage,
  temperature, status flags, moving, and current — still **one transaction
  per tick**. On the wire that is 1.40 ms against 0.74 ms for 6 servos at
  1 Mbaud, so load and current now cost 0.66 ms/tick instead of the twelve
  per-servo round trips they used to need. Bus bandwidth was never the
  constraint here; per-transaction USB turnaround is.
- `ServoHealth` (`soarm_sdk.robot.types`, re-exported from `soarm_sdk`) and
  `ServoHardwareInterface.get_servo_health()` — the servo-frame diagnostic
  half of that block (load %, current mA, voltage V, temperature °C, status
  flags, moving), all sharing the single timestamp of the read they came
  from. `JointState` keeps the joint-frame control quantities.
- Named protocol constants for the block and its sign-magnitude decode:
  `STS_TELEMETRY_START`, `STS_TELEMETRY_LENGTH`, `STS_{POSITION,SPEED,LOAD,
  CURRENT}_SIGN_BIT`, and the LSB scale factors.
- **A telemetry tap on the bus thread.** `ServoHardwareInterface.subscribe()`
  returns a `TelemetryStream` — a bounded, drop-oldest queue receiving one
  `ServoSample` per successful bus tick. The bus thread never calls consumer
  code: it appends and returns. That thread also issues servo writes, so a
  slow consumer there would delay commands to the arm. With no subscribers
  nothing is built, so subscribing is the switch that turns telemetry on.
- **`ServoSample` carries commanded and measured state together**, under one
  timestamp taken next to the wire, so tracking error is a subtraction
  (`sample.tracking_error_rad()`) rather than a join across two logs with
  different clocks. `seq` counts bus ticks rather than published samples: a
  gap tells a consumer a read failed, distinguishing "the arm did not move"
  from "we missed the sample".
- **Sinks and a recorder thread** (`soarm_sdk.robot.telemetry_sinks`):
  `TelemetryRecorder` drains a stream on its own thread and fans out to
  `JsonlSink` (one JSON object per line, buffered — the same file-as-channel
  shape `soarm_tamp` uses for `live.jsonl`) and `RerunSink` (per-joint scalar
  series). A sink that raises is logged and dropped rather than retried, so
  one broken sink cannot cost the others their data. `rerun-sdk` is not a
  dependency — it is the new `[telemetry]` extra, imported at construction.

- **The dashboard can consume the telemetry stream instead of polling**
  (`soarm-dashboard-setup --stream`, off by default while it beds in). One
  `ServoHardwareInterface` holds the port open and the dashboard drains its
  samples, which removes the two costs the legacy loop paid every cycle:
  reopening the serial port each iteration, and twelve per-servo round trips
  for temperature and current every fifth poll — measured at 7.36 ms against a
  2.09 ms full-block sync-read carrying the same fields. Health data now
  arrives every tick rather than every fifth poll. The dashboard connects with
  `torque_on_start=False`: opening a browser tab must never energise the arm.
- **`ServoHardwareInterface.lend_bus()`** — pause the bus thread and hand the
  caller the live servo handle. The port is exclusive, so the dashboard's 19
  `ctx.bus()` call sites (EEPROM writes, servo ID changes, one-off diagnostics)
  cannot open their own connection while the interface holds it; they borrow
  this one and are otherwise unchanged. Measured hand-over on hardware: 0.01 ms,
  because the lock is free during the inter-tick sleep.
- **`soarm_sdk.diagnostics`** — `measure_backlash()` drives a joint to one
  target from below and from above and reports the hysteresis gap against
  within-direction scatter, so a gap smaller than the repeat noise is reported
  as insignificant rather than as a result. `measure_droop()` sweeps a joint and
  records settled position, load and current together from the same samples.
  Its steady-state error is documented as a **lower bound** on true deflection:
  compliance downstream of the encoder is invisible to the servo, and closing
  that gap needs an external reference.

- **The 3-D view rendered raw ticks against a nominal zero.** `update_fk` mapped
  every joint with `ticks_to_radians(ticks)` — tick 2048 is zero, no direction
  signs — ignoring the arm's saved calibration entirely. On the arm here that
  put the main joints 16-30 degrees out, the gripper 74 degrees out, and turned
  `wrist_roll` the wrong way, which reads as the model and the robot
  disagreeing when only the view was uncalibrated. It now loads
  `~/.soarm_sdk/calibration.json` (override with `--calibration`) and maps each
  joint through its own measured zero and sign; with no calibration present it
  still runs but says so instead of silently rendering wrong.
- **Defaults moved from the SO-100 to the SO-101 revision** — the arm this
  workspace actually has. The dashboard URDF is now
  `SO-ARM100/Simulation/SO101/so101_new_calib.urdf`, and `load_robot_config()`
  defaults to `so101` rather than `soarm100` (`soarm100` stays loadable by name;
  the two carry identical limits today and differ only in name and
  description). This is not cosmetic: the revisions do not share a zero
  convention — SO100 puts `shoulder_lift` at `[0, 3.5]` and `elbow_flex` at
  `[-3.1416, 0]` while SO101 centres both at `[-1.745, 1.745]` and
  `[-1.69, 1.69]`. The saved calibration is SO101-framed (confirmed from its
  recorded `span_ratio`), so feeding its radians into the SO100 model was wrong
  by more than a radian on those joints.

- **`rezero_from_pose()`** — pin joint zeros to a configuration you can verify
  physically, instead of inferring them from travel endpoints.
  `seed_from_travel` assumes, in its own words, that measured travel and the
  URDF's limits "describe the same mechanical hard stops". On this arm they do
  not: `span_ratio` runs 0.96-1.34, because the URDF's limits are conservative
  software limits while the real travel is wider. Stretching one onto the other
  misplaces every zero by a share of the disagreement — which is the standing
  "URDF/travel span mismatch" that also blocks `soarm_tamp`'s `execute.py`.
  The new path takes the zero from a held reference pose and carries measured
  travel through untouched, so `reachable_rad` still reports the true hard
  stops and `span_ratio` survives as the record that they disagree.

  Confirmed on the arm (2026-09-13): re-zeroing against a level-verified pose
  — upper arm vertical, forearm and gripper axis horizontal, no yaw — moved
  `shoulder_lift` by only 3.07 degrees, while a clean `seed_from_travel`
  re-seed wanted to move it 15.5 degrees the other way. The hand-levelled zero
  had been right and the seeding method wrong. With that calibration and the
  SO101 model, the 3-D mirror tracks the physical arm.

- **A span mismatch now only impeaches the zeros it actually informed.**
  `JointCalibration` gained `zero_source`, and `suspect` is no longer a bare
  restatement of `span_ratio`: it means "measured travel disagrees with the
  URDF *and* this joint's zero was inferred from those same limits". A zero
  pinned by `rezero_from_pose` never touched them, so the same mismatch says
  only that the URDF is conservative about travel — reported through the new
  `span_mismatch` / `span_mismatch_joints`, not through `suspect`. Files
  written before this field default to `unknown`, which is treated as
  limits-derived, so nothing silently becomes trusted.

### Changed

- **The Calibration tab now reads in the order the work is done.** Its
  folders were emitted 1, 1, 0, 2, 3, 4, 4, 5: two steps numbered 1, two
  numbered 4, and the prerequisite numbered 0 sitting *below* the step that
  depends on it. The numbers matched neither each other, nor the acceptance
  record's rows, nor the sequence of the work. Every folder was correct on
  its own, which is why it survived — nothing was broken, it was only
  impossible to follow. The tab is now one linear pass: banners, a *Live arm
  state* panel that is deliberately not a step, then Steps 1-6 (tolerances,
  direction signs, ROM, pose zeros, optional nudge, review and save).
  `CalibrationReport.as_markdown()` labels its rows with those same step
  numbers, so a BLOCKED row names a folder that exists. Tests assert the
  ordering, since nothing about it fails loudly.
- **The status line moved above the steps.** Every button in the tab writes
  to one `status_md`, which was created last — so pressing the tolerances
  button in the first folder printed the reply several screens down, past
  every other step. It and the unsaved-drift and out-of-limits banners now
  sit above Step 1, where they apply to everything below them.
- **The acceptance record moved next to the Save it gates**, from mid-panel
  into Step 6, so the reason Save refused and the button that refused are on
  one screen.
- **The acceptance record renders while disconnected.** `_on_tick` returned
  early with no arm attached, freezing the record on "waiting for
  calibration" — the one panel that says what remains to be done was blank
  until the thing it grades was live. Calibration-derived sections now
  refresh unconditionally; only the live-reading sections wait for a
  connection.
- `_build_homing` takes `heading=False`, so the ROM controls embedded in
  Step 3 no longer nest a second `## Homing Wizard` title inside a numbered
  folder.
- The `calibrate` skill's procedure cites the tab's step numbers per stage,
  and documents the optional nudge it had omitted.

### Fixed

- **`ReadLoad` and `ReadCurrent` truncated their registers.** Both issued
  1-byte reads against 2-byte sign-magnitude registers — `PRESENT_LOAD`
  (60-61, magnitude in bits 0-9 and direction in bit 10) and
  `PRESENT_CURRENT` (69-70, 6.5 mA per LSB, signed at bit 15). Every value
  above 255 LSB wrapped and every direction was lost: a full-scale 100.0%
  load read as 23.2%, and 1950 mA read as 286 mA. Both now read two bytes
  and decode the sign bit their register actually uses. These are the two
  registers that matter most for diagnosing backlash and load-dependent
  elasticity, so nothing that consumed them was trustworthy.

  Verified on a real SO-101 (2026-09-13): driving `shoulder_pan` — the one
  gravity-neutral joint, so its load is unbiased — through +/-40 ticks swung
  load symmetrically to -6.4%/+6.4%, setting bit 10 while bits 11-15 stayed
  clear throughout. A sign at bit 15 would have decoded those same words as
  >100% load, which never appeared. `PRESENT_CURRENT` was never observed
  negative, so its bit-15 sign is unexercised; the decode is safe either way,
  since 6.5 mA per LSB puts bit 15 at 213 A and no real reading can reach it.
- **`JointState.efforts` was always zeros.** `_cached_currents_mA` was
  allocated and returned but never written, because the sync read only
  covered position and speed — while both the class docstring and
  `get_robot_joint_state` documented efforts as motor current in mA. It is
  now populated from the widened read. No package in `soarm-ws` consumed
  `efforts`, so nothing downstream was silently wrong.
- **`start()` could lurch the arm.** `torque_on_start` defaults to `True`, and
  `start()` enabled torque with a raw register write while the position cache
  still held its `TICK_ZERO` seed — so the servos chased whatever stale goal
  sat in their SRAM from a previous session. `_apply_pending_torque` had always
  parked goals at the measured pose first, for exactly this reason; `start()`
  now does the same, via the shared `_park_goals_at_measured()`. It also takes
  a priming read before anything can act on the cache (up to 3 attempts), and
  **refuses to enable torque** if no state could be read, rather than parking
  against the placeholder zero pose. This matters because the arm is normally
  left at an arbitrary pose between sessions, never at its kinematic zero.
- **`GroupSyncRead.isAvailable` did not check the requested field.** It
  compared the response length against `data_length + 1` regardless of where
  in the block the field sat, so it answered correctly for a field at the
  front and wrongly for every field behind it — letting `getData` walk off
  the end of a truncated response with an `IndexError`. It now checks that
  the field's own byte range was actually received. Latent until now: the
  only sync-read group was 4 bytes wide, and the parser returns whole blocks
  or nothing.

- **`soarm-reconfigure --list-ports` ran a reconfiguration afterwards.**
  (Named `soarm-calibrate` at the time this was fixed.) The flag
  is a query, but `main()` printed the ports and then fell through into
  `run_calibration()`, which opens `--device` — default `/dev/ttyUSB0` —
  and raised `SerialException` on any machine that only wanted to know
  which ports exist. The list also printed twice, since `run_calibration`
  prints it again for callers that pass the flag in a `Namespace`. It now
  prints once and returns 0. Covered by `tests/test_reconfigure_cli.py`,
  which also pins that the early return is scoped to the flag and that a
  bad configuration still exits 2 with a message rather than a traceback.
- **The enforced joint limits were in the wrong frame for two joints, and
  would have silently truncated real trajectories.**
  `configs/soarm100.yaml` declared `shoulder_lift` offset by −π/2 and
  `elbow_flex` by +π/2 relative to the URDF, while the other four matched
  it. The joint *travel* was right — only the zero it was measured from was
  wrong. Inert while nothing read those numbers; 0.2.0 began enforcing them
  on every write, at which point a planner commanding URDF-frame angles had
  **55% of a trajectory (200 of 366 commands) clamped at `shoulder_lift`**,
  which on hardware means the arm quietly stops following the plan. Limits
  in both configs are now taken from
  `SO-ARM100/Simulation/SO101/so101_new_calib.urdf`.
- **A clamp counter that cried wolf at arithmetic.** Clamps were counted
  with `atol=1e-9`, so a planner planning right up to a joint limit — whose
  waypoints round a hair past it, by 0.03 of an encoder tick in real
  trajectories here — registered as limit hits. An excursion smaller than
  half a tick cannot change what the servo does, and is no longer counted.
  Clamping itself is unchanged.

### Fixed

- **The Homing Wizard's Apply wrote the homing offset with the wrong sign.**
  The servo reports `raw - STS_OFS`, so centring a joint's swept midpoint on
  `target_ref` needs `zero - target_ref`; `setup.py` wrote `target_ref - zero`,
  which moved the joint's frame away from centre by exactly what should have
  brought it back — and silently, since nothing reads the offset afterwards.
  The angle limits it wrote alongside were already correct and are unchanged
  in effect. The arithmetic now lives in
  `soarm_sdk.calibration.recentre.centring_offset`, with the sign pinned by a
  test rather than by a comment. Apply also now refuses to write an offset for
  a joint whose sweep looks wrapped, instead of deriving one from a range that
  is the encoder's rather than the joint's.
- **Torque could not be released.** `ServoHardwareInterface` had no torque
  control at all, so anything needing a back-driveable arm — checking a
  calibration's direction signs by hand, releasing a servo that has tripped
  its overload protection against a stop — had no option but cutting the
  supply, which takes the bus down with it. `set_torque()` /
  `disable_torque()` / `enable_torque()` are queued onto the bus thread that
  owns the port (a caller-thread write would interleave with its sync-read
  packets) and block until applied. Re-enabling parks the goal at the
  measured pose first, so an arm moved by hand while limp does not lurch, and
  a command queued before going limp is discarded rather than executed on the
  way back up.
- **A ROM sweep measures a different joint frame than the runtime reads, so
  every calibration derived from one was out by that joint's homing offset.**
  While a joint is driving in wheel mode the STS3215 reports the *raw*
  encoder, with `STS_OFS` not applied; in servo mode it reports
  `raw - STS_OFS`. The position visibly jumps by exactly the offset the
  moment wheel-mode motion starts (measured: +903 on `shoulder_lift`, whose
  offset was 903) and jumps back on the return to servo mode. `run_rom_sweep`
  records min/max of the reported position, so its travel range is in the raw
  frame while `ServoRobot` reads the offset-applied one. On the arm here that
  displaced four of six joint zeros by 139–880 ticks (0.2–1.4 rad).
- **A sweep's min/max cannot measure a joint whose travel crosses 4095/0.**
  The reported position wraps, so the recorded range is the encoder's
  (`0..4095`), not the joint's — `shoulder_lift` measured 4095 ticks against a
  URDF travel of 2275, a span ratio of 1.80. Travel is now measured by
  accumulating displacement, treating a jump over half the encoder as a wrap,
  and anchoring the total to a settled servo-mode reading. The same joint then
  measured 2449 ticks, ratio 1.076.
- **A position read taken straight after a mode switch or an EEPROM write can
  still be in the previous frame**, which silently mis-anchors an entire
  joint's calibration. Reads that anchor a measurement now wait for
  consecutive agreeing samples.

### Added

- **`soarm-calibrate-rom` — measure an arm's travel and write its
  calibration in one command.** The pieces existed but nothing joined
  them: the ROM sweep lived behind the dashboard's Homing Wizard (which
  writes servo EEPROM and no calibration file), and the only producer of
  `~/.soarm_sdk/calibration.json` was `soarm-seed-calibration`, which
  borrows travel ranges from a lerobot file rather than measuring them.
  This CLI runs `run_rom_sweep` against the hardware, feeds the result to
  `seed_from_travel`, and saves — so an arm with no lerobot calibration, or
  one that has been re-assembled, can be calibrated from the hard stops
  themselves. Joint names, servo IDs and URDF limits all come from the
  robot config, so the ordering cannot drift from `ServoRobot`'s. Records
  the raw sweep under `notes["sweep"]`, flags any direction that timed out
  instead of stalling, and — like the seeding CLI — writes
  `validated: false`, because a travel range still cannot settle the
  direction signs. `examples/calibrate_rom.py` runs it from a checkout.
- **`seed_from_travel(direction_signs=...)`** — seed a joint whose direction
  has actually been measured. The sign is not a cosmetic flag over the same
  zero: it decides which end of the measured travel is the URDF's *lower*
  limit, so it changes the zero the two endpoints agree on. Flipping only the
  field leaves the joint mirrored about the wrong point — on this arm's
  `wrist_roll` that is a 63.5-tick (0.097 rad) error, small enough to look
  plausible and be wrong everywhere.
- **`--joints` sweeps part of an arm and merges the result into the existing
  calibration**, so a heavy arm can be done one joint at a time with the
  numbers checked between runs. Joints never swept are listed under
  `notes["incomplete"]` and the file is short a joint until they are, which
  `soarm_tamp.conventions.check_ready` already refuses to drive.
- **`soarm_sdk.calibration.recentre` and `--recentre`** — move a servo's
  homing offset so its travel is centred at tick 2048 and reset its angle
  limits to the measured stops. Required for a joint whose travel straddles
  the encoder wrap, which no linear tick-to-radian mapping can describe.
  Note the sign: the servo reports `raw - STS_OFS`, so the offset moves the
  reported frame *opposite* to the intuitive direction; the wrong sign moves
  a joint's frame away from centre by exactly what should have brought it
  back. `notes["recentred"]` keeps the previous offset so the change is
  reversible.
- **`ServoRobot` enforces the arm's measured travel, not just the config.**
  `effective_joint_limits()` intersects the declared limits with
  `RobotCalibration.reachable_limits()` when a calibration is supplied, so
  the physical hard stops always win on the tight side — a config is a
  model's opinion about a joint, the measured range is where the mechanism
  actually stops. Without a calibration, behaviour is unchanged.
- `JointCalibration.reachable_rad` / `RobotCalibration.reachable_limits()` —
  measured tick travel expressed in URDF radians, ordered correctly for a
  negative direction sign.
- `configs/so101.yaml` — the revision physically present in this workspace.
  Same limits as `soarm100.yaml`; prefer it for real-arm work.

### Changed

- **`joint_names` in both configs now use the URDF's names**
  (`shoulder_pan`, `shoulder_lift`, ...) instead of the servo/assembly
  labels (`Rotation`, `Pitch`, ...), which matched neither URDF. The SDK
  already used URDF names in `dashboard/fk.py`, and lerobot, soarm_mjlab and
  soarm_tamp all use them too, so this was the odd one out. The servo labels
  survive as per-line comments, which is where the ID mapping actually
  belongs. Nothing read these strings, only their count.

## [0.3.0] - 2026-09-12

### Removed

**The legacy top-level import paths deprecated in 0.2.0 are gone.** Each was
a re-export shim over its new home; they warned on import for one release and
have now been deleted:

| Removed | Use instead |
|---|---|
| `soarm_sdk.port_handler` | `soarm_sdk.protocol.port_handler` |
| `soarm_sdk.protocol_packet_handler` | `soarm_sdk.protocol.packet_handler` |
| `soarm_sdk.group_sync_read` | `soarm_sdk.protocol.group_sync_read` |
| `soarm_sdk.group_sync_write` | `soarm_sdk.protocol.group_sync_write` |
| `soarm_sdk.sts` | `soarm_sdk.protocol.sts` |
| `soarm_sdk.scscl` | `soarm_sdk.protocol.scscl` |
| `soarm_sdk.stservo_def` | `soarm_sdk.protocol.registers` |
| `soarm_sdk.types` | `soarm_sdk.robot.types` |
| `soarm_sdk.interfaces` | `soarm_sdk.robot.interfaces` |
| `soarm_sdk.hardware_interface` | `soarm_sdk.robot.hardware` |
| `soarm_sdk.servo_robot` | `soarm_sdk.robot.servo` |
| `soarm_sdk.frame_calibration` | `soarm_sdk.calibration.frame` |
| `soarm_sdk.seed_calibration` | `soarm_sdk.calibration.seed` |
| `python -m soarm_sdk.seed_calibration` | `soarm-seed-calibration` |

**The top-level namespace is unaffected**, and remains the recommended entry
point: `from soarm_sdk import ServoRobot, RobotCalibration, Pose, ...` works
exactly as before. Only direct submodule imports of the old paths break, and
only if you had not already moved them — `ImportError` /
`ModuleNotFoundError` names the module, and the table above gives the
replacement.

Also removed: `tests/test_deprecated_shims.py`, which existed only to pin the
shims' behaviour.

Nothing in this workspace needed changing for this release: `soarm_tamp` and
`m5teleop` moved to the canonical paths in 0.2.0 and pass untouched against
0.3.0.

## [0.2.0] - 2026-09-12

The package reorganization. Every pre-0.2.0 import path still works here —
the legacy top-level modules are deprecation shims — so this release is
adopt-at-your-own-pace. They are removed in 0.3.0.

### Changed — package reorganization

The flat 20-module top-level namespace is now grouped by layer, each with
a real subpackage. Every old import path still works (see "Deprecated"
below), so this is additive for existing code, but new code should use
the new paths:

- `soarm_sdk.protocol` — the Feetech wire protocol (`port_handler`,
  `packet_handler` [was `protocol_packet_handler`], `group_sync_read`,
  `group_sync_write`, `sts`, `scscl`, `registers` [was `stservo_def`]).
- `soarm_sdk.bus` — port discovery/diagnostics (`discovery`, was the flat
  `bus.py`) plus servo EEPROM configuration planning (`servo_config`,
  **moved from `soarm_sdk.calibration`** — see below).
- `soarm_sdk.robot` — the `RobotInterface`/`Robot` abstraction layer
  (`types`, `interfaces`, `base`, `hardware`, `servo`), plus a new `null`
  backend (see "Added").
- `soarm_sdk.calibration` — **now the URDF-frame mapping package**
  (`frame`, was `frame_calibration.py`; `seed`, was `seed_calibration.py`),
  plus a new `rom_sweep` module (see "Added").
- `soarm_sdk.kinematics` — URDF loading + forward kinematics, split out of
  `dashboard/fk.py` so it has no Viser dependency.
- `soarm_sdk.cli` — the calibration TUI and dashboard launchers, moved out
  of `examples/` into the installable package (see "Added").

**Breaking:** `soarm_sdk.calibration` (servo EEPROM reconfiguration —
`OperationPlan`, `apply_plan`, `parse_range`, ...) moved to
`soarm_sdk.bus.servo_config` / `soarm_sdk.bus`. The name `calibration` was
freed up because it was doing double duty for two unrelated concepts —
servo register configuration vs. the tick-to-URDF-frame mapping — and the
latter (`frame_calibration.py`) wasn't even reachable from the top-level
`soarm_sdk` namespace. Everything reachable via `from soarm_sdk import
...` (`OperationPlan`, `apply_plan`, `run_calibration`) is unaffected;
only a direct `from soarm_sdk.calibration import ...` submodule import
needs updating, and only for the servo-config names.

### Deprecated

The following top-level modules are now thin re-export shims over their new
subpackage location, and **emit a `DeprecationWarning` on import naming
their replacement**. They still work — no import using them is broken — but
they are **scheduled for removal in 0.3.0**:

| Deprecated | Use instead |
|---|---|
| `soarm_sdk.port_handler` | `soarm_sdk.protocol.port_handler` |
| `soarm_sdk.protocol_packet_handler` | `soarm_sdk.protocol.packet_handler` |
| `soarm_sdk.group_sync_read` | `soarm_sdk.protocol.group_sync_read` |
| `soarm_sdk.group_sync_write` | `soarm_sdk.protocol.group_sync_write` |
| `soarm_sdk.sts` | `soarm_sdk.protocol.sts` |
| `soarm_sdk.scscl` | `soarm_sdk.protocol.scscl` |
| `soarm_sdk.stservo_def` | `soarm_sdk.protocol.registers` |
| `soarm_sdk.types` | `soarm_sdk.robot.types` |
| `soarm_sdk.interfaces` | `soarm_sdk.robot.interfaces` |
| `soarm_sdk.hardware_interface` | `soarm_sdk.robot.hardware` |
| `soarm_sdk.servo_robot` | `soarm_sdk.robot.servo` |
| `soarm_sdk.frame_calibration` | `soarm_sdk.calibration.frame` |
| `soarm_sdk.seed_calibration` | `soarm_sdk.calibration.seed` |
| `python -m soarm_sdk.seed_calibration` | `soarm-seed-calibration` |

Importing from the top-level `soarm_sdk` namespace is **not** deprecated and
never warns — `from soarm_sdk import ServoRobot, RobotCalibration, ...` stays
the recommended entry point.

Python hides `DeprecationWarning` by default outside `__main__`; run with
`python -W default::DeprecationWarning` (or under pytest, which shows them)
to see which call sites still need updating.

Nothing inside the package, its tests, or any package in this workspace
imports through a shim any more — `tests/test_deprecated_shims.py` asserts
that importing `soarm_sdk` emits no deprecation warnings of its own, so the
only warnings you can see are from your own code.

### Added

- `soarm_sdk.robot.NullRobot` — an in-memory `Robot` implementation with
  no hardware behind it, satisfying `RobotInterface` completely. Useful
  for testing SDK-consuming code, CI, and any call site that wants "a
  robot" without one connected. Note it is *not* a drop-in for every
  existing `dry_run` flag in the workspace: soarm_tamp's `execute.py`
  defers its `soarm_sdk` import specifically so `--dry-run` stays
  runnable inside the HPP planning container, where the SDK isn't
  installed at all — using `NullRobot` there would reintroduce the
  import it is avoiding.
- `soarm_sdk.robot.LeRobotRobot` — the same servo bus driven through
  lerobot's `SOFollower` instead of this SDK's protocol stack, behind the
  same `RobotInterface`. m5teleop had a hand-rolled wrapper (`ArmInterface`)
  that did not implement the interface, so teleop code could not be pointed
  at a planner's robot, a simulation, or `NullRobot` without a rewrite.
  lerobot stays an optional dependency (`pip install soarm-sdk[lerobot]`),
  imported lazily in `connect()`; a `follower_factory` hook makes the
  backend testable without it. Note it converts lerobot's *normalized*
  degrees to radians and nothing more — that is not the URDF frame a
  planner speaks; see `calibration/frame.py`.
- **Config-driven gripper API** on the `Robot` base class — `set_gripper()`,
  `toggle_gripper()`, `gripper_is_open`, `gripper_index`, driven by an
  optional `gripper:` block (`joint_index`, `open_rad`, `closed_rad`) now
  present in `configs/soarm100.yaml`. m5teleop and soarm_tamp each carried
  their own `GRIPPER_OPEN_DEG`/`CLOSED_DEG` constants and their own "which
  joint is the jaw" assumption; this makes it a property of the robot.
- `soarm_sdk.trajectory.resample()` — linear waypoint interpolation
  bounding the per-joint step between consecutive commands. Generalizes a
  pattern reimplemented per-caller around planned/recorded paths (e.g.
  soarm_tamp's waypoint-manifest executor).
- `soarm_sdk.calibration.rom_sweep` (`run_rom_sweep`, `simulate_rom_sweep`)
  — the range-of-motion sweep `seed_from_travel()` consumes as input,
  extracted from the Homing Wizard dashboard panel so it's testable and
  usable without a browser. The panel is now a thin GUI wrapper over it.
- `soarm_sdk.robot.config.validate_robot_config()` — checks a robot config
  dict's required keys, types, and array lengths up front. `Robot.__init__`
  now raises one readable `ConfigError` naming every problem, instead of a
  bare `KeyError` (or a silent shape mismatch) surfacing later from
  whichever accessor happens to touch the bad field first.
- `soarm_sdk.kinematics` — URDF loading (`load_urdf`) and forward
  kinematics (`link_transforms`) with no dependency on a viewer, so a
  planner or a headless test can use FK without pulling in Viser.
- `[project.scripts]`: `soarm-calibrate`, `soarm-dashboard`,
  `soarm-dashboard-setup`, `soarm-seed-calibration` — the calibration CLI
  and dashboard launchers are now real console scripts (`pip install
  soarm-sdk` puts them on `$PATH`), not just scripts you run from a
  checkout. The `examples/*.py` scripts still work identically as thin
  launchers over the same code.
- `frame_calibration.py` — the mapping between raw servo ticks and the
  URDF's joint frame, which nothing in this workspace previously had.
  `ServoHardwareInterface` always accepted `zero_offsets`/`direction_signs`
  but no code ever produced them, so the package config's static 2048 was
  in effect a guess. `seed_from_travel()` estimates them from measured
  travel plus the URDF's joint limits, records the resulting uncertainty
  per joint, and persists a `RobotCalibration` marked `validated: false`
  until a physical check confirms the direction signs — which a travel
  range cannot determine, since it says how far a joint moves and not
  which end is which.
- `seed_calibration.py` — CLI that seeds a calibration from an existing
  lerobot calibration file, offline, no hardware. It reads only the travel
  ranges; `homing_offset` is deliberately ignored because lerobot writes it
  into servo EEPROM, so the ticks this SDK reads already include it.
- **Joint-limit enforcement on the write path.** `joint_limits_lower/upper`
  had been declared in `configs/soarm100.yaml` and exposed via
  `get_joint_limits()` since the beginning, but nothing read them when
  commanding — the only clamp was `radians_to_ticks` pinning to [0, 4095],
  which is protocol validity, not safety.
- **Per-step clamping** (`max_step_rad`), bounding how far a joint may be
  commanded from its last *measured* position, so a lagging joint cannot
  accumulate an ever-larger jump. Both clamps count rather than fail
  silently; `limit_clamps` and `step_clamps` are public so a caller can
  assert on them after a run.
- `ServoRobot` now accepts `calibration`, `max_step_rad` and
  `enforce_limits`, and passes the config's declared joint limits through
  by default.

- `tests/` — first test suite for the package: `test_conversions.py`,
  `test_protocol_packet_handler.py`, `test_servo_config.py` (originally
  `test_calibration.py`, renamed alongside the module move above),
  `test_bus.py`. Covers unit conversions, packet framing/checksum logic,
  servo-config planning, and bus discovery control flow with fake
  port/packet-handler doubles — no real hardware required to run them.
- `tests/test_null_robot.py`, `tests/test_trajectory.py`,
  `tests/test_robot_config.py`, `tests/test_rom_sweep.py` — coverage for
  the additions above.
- `py.typed` marker, so consumers get real type checking instead of `Any`.
- `LICENSE` (MIT) file, matching the license already declared in
  `pyproject.toml`.
- `hatch run test` script (`pytest`), and `[tool.pytest.ini_options]`
  `testpaths` so plain `pytest` works from the package root.
- `.github/workflows/ci.yml` — lint (`ruff check src tests`) and test
  (`pytest`) on push/PR, matrixed over Python 3.9–3.12.
- This file.

### Fixed

- **`run_calibration(args)` crashed with `ModuleNotFoundError` when
  `--list-ports` was passed** (so `soarm-calibrate --list-ports` was broken).
  The reorganization moved this module under `bus/`, which silently changed
  what its function-local `from .bus import print_ports` resolved to. Being a
  lazy import inside a rarely-taken branch, nothing caught it until every
  relative import in the package was resolved against the filesystem; now
  covered by a regression test.
- **A `TYPE_CHECKING` import in `robot/hardware.py` pointed at
  `soarm_sdk.robot.frame_calibration`**, which does not exist. Invisible at
  runtime (the branch never executes) but wrong for any type checker.
- Removed unused imports in `calibration.py` (now `bus/servo_config.py`)
  (`Iterable`, `discover_servos`, `COMM_SUCCESS`) flagged by
  `ruff check src`, which had never been run in CI.
- `[project.urls]` pointed at the `fullstack-manip` repo (this package's
  origin before it was split out); now points at `soarm_sdk`'s own repo.

## [0.1.0] - 2026-05-10

Initial release: Feetech STS/SCS servo protocol (`PortHandler`,
`ProtocolPacketHandler`, `sts`, `scscl`, `GroupSyncRead`/`GroupSyncWrite`),
`bus.py` port discovery/diagnostics helpers, `calibration.py` operation
planning, `conversions.py` unit conversions, and the `examples/` calibration
CLI and Viser dashboard.
