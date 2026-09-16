# PID Auto-Tuning — Design & Implementation Plan

Status: implemented (`soarm_sdk/tuning/` + the dashboard's Auto-Tune
section), with two known deviations from the original plan and one item
deferred — see "Implementation status" at the end. This document is the
source of truth for the `tuning/` module; update it as the implementation
deviates from the plan rather than letting the two drift apart.

## Current state (as of this plan)

- **Gains** live in EEPROM: `STS_P_COEF`/`STS_D_COEF`/`STS_I_COEF` (addrs
  21/22/23, 1 byte each, defaults 32/32/0). Writing requires
  `STS_LOCK` unlock -> write -> lock.
- The dashboard's **PID Tuning** tab
  (`src/soarm_sdk/dashboard/panels/pid.py`) already reads/writes gains and
  runs a manual step test with a live chart and rise/overshoot/settling
  metrics — but it is manual only: a human sets gains, reads the chart, and
  decides. No search loop, no acceptance gate, no safety abort.
- It does **not** use the real telemetry system — it polls `ReadPos` itself
  at 20 Hz. `src/soarm_sdk/robot/telemetry.py` already exposes a
  subscribable `TelemetryStream` of `ServoSample` (position, velocity,
  load %, current mA, voltage, temperature, moving flag, commanded goal,
  and tracking error) sourced from the bus thread's own read — richer
  signal, no duplicate polling loop, and the same object `JsonlSink` /
  `RerunSink` already know how to log.
- **No current/thermal safety watchdog exists anywhere.**
  `ServoHardwareInterface._apply_safety()` only clamps position and step
  size; nothing watches current or temperature during a commanded move.
  This gap matters independent of auto-tuning.
- The step-target slider in the PID tab is **unbounded** (raw 0–4095
  ticks), not clamped to the arm's validated ROM
  (`calibration.limits.effective_limits`).
- The calibration module (`calibration/pipeline.py`) has the right template
  to copy: `AcceptanceTolerances` (frozen dataclass, validates its own
  fields), `CalibrationReport` / `StageResult` (`ready` = all stages pass),
  and a `validated: False` gate that downstream consumers
  (`soarm_tamp.execute`) are entitled to refuse to act on.

## Design

New module, sibling to `calibration/`: `soarm_sdk/tuning/`

```
tuning/
  step_test.py   # run one step, return a StepResponse (from TelemetryStream)
  metrics.py     # StepResponse -> rise/overshoot/settle/steady-state-error/
                 #   oscillation-count/peak current & temperature
  acceptance.py  # PIDAcceptanceCriteria + PIDTuningReport (mirrors calibration/pipeline.py)
  search.py      # auto-tune algorithm(s), pure functions over an injected run_trial()
  watchdog.py    # concurrent telemetry consumer: abort + restore-last-good-gains
  provenance.py  # persist algorithm/criteria/before-after gains next to calibration.json
```

Everything above `step_test.py` talks to a `run_trial(p, d, i) -> StepResponse`
callback, not to hardware directly. That is what makes `search.py` and
`watchdog.py` testable with zero hardware: inject a synthetic
second-order-plant simulator in tests, inject the real hardware step test in
the dashboard.

### Validation signal

**Primary: position step response**, reusing the existing test but sourced
from `TelemetryStream` so velocity/current/temperature come for free, for the
same trial, at no extra bus traffic. Metrics (`metrics.py`, extending the
existing `_compute_step_metrics`):

- Rise time (10→90%), overshoot %, settling time (±2% band) — already
  implemented, relocate and reuse.
- **Steady-state error** (final tracking error after settling) — new.
- **Oscillation count** (zero-crossings of error after first settle) — new;
  distinguishes "underdamped but converges" from "sustained ringing."
- **Peak current (mA) and peak temperature rise (°C)** during the trial —
  new; the safety half of the signal, not just the performance half.

Secondary, optional (not built first): a small-amplitude relay/square-wave
test (repeated steps) to check for limit-cycling at candidate gains before
trusting a single-step result.

### Acceptance criteria (`acceptance.py`)

```python
@dataclass(frozen=True)
class PIDAcceptanceCriteria:
    overshoot_max_pct: float
    settling_time_max_s: float
    steady_state_error_max_ticks: int
    oscillation_count_max: int
    current_peak_max_mA: float
    temperature_max_C: float
    repeat_trials: int = 3   # must pass this many consecutive step tests
```

`PIDTuningReport` mirrors `CalibrationReport`: one `StageResult` per
criterion, `ready` = all pass, plus `validated: bool` that starts `False`
and only flips after `repeat_trials` consecutive passes.

### Tuning strategy

The onboard controller is a fixed-point embedded PID with 1-byte integer
gains (0–254) and an unknown internal update law/scaling — not a plant to
model or single-step, and not classic enough for Ziegler–Nichols
relay-feedback math to transfer cleanly. Treat it as a **black box**:

- **Primary: bounded coordinate descent / pattern search over (P, D, I).**
  Start from the servo's *current* gains (read, don't assume factory
  defaults). Vary one gain at a time by a step size, run a trial, accept if
  cost improves, halve the step size on repeated failure to improve
  (Hooke–Jeeves-style pattern search). Cost combines the metrics above with
  heavy penalties for any safety-relevant breach (current/temp/oscillation),
  steering the search away from dangerous regions rather than only
  rejecting them after the fact.
- **Search bounds are conservative**, not the register's full 0–254 range —
  e.g. start with ±32 around the current value, expand only if the search
  wants to walk to a bound.
- **Ziegler–Nichols as an optional seed, not the tuner**: a short P-only
  sweep to find the onset of oscillation can seed the coordinate descent's
  starting point. Nice-to-have, not required for v1.
- Skip Bayesian optimization / CMA-ES for v1 — 3 integer parameters in a
  bounded box is exactly what coordinate descent is good at, and far easier
  to reason about and test than a BO loop.

### Timeout, tolerance, and safety gates

- **Per-trial timeout**: reuse the existing `Duration (s)` bound
  (0.5–10 s, default ~3 s). A trial that never crosses 90% of the step
  within the timeout is scored as a hard fail, not "n/a", so the search
  moves away from it.
- **Watchdog runs concurrently with every trial** on the same
  `TelemetryStream`: aborts the trial (torque off or hold, not just stop
  the loop) and restores the last known-good gains if current or
  temperature crosses the configured max, or oscillation count blows past a
  hard ceiling above `oscillation_count_max`.
- **Global search timeout/trial budget**: cap total trials (e.g. 30) and
  wall-clock time; on exhaustion, report best-found gains plus the full
  report rather than silently picking something.
- **EEPROM writes are gated, not automatic.** P/D/I are EEPROM-only, so
  every trial is an EEPROM write (unlock/write/lock). Feetech EEPROM
  endurance (~100k cycles) makes a 30-trial search a non-issue but rules
  out strategies wanting thousands of trials. Final gains are written as
  "accepted" only after `repeat_trials` consecutive passes, and only when a
  human clicks "Accept & Write" — the search never auto-commits.
- **Step targets are bounded by the arm's validated `effective_limits`**,
  not raw ticks; refuse to run for a joint whose calibration isn't
  `validated: True`.

### Implementation plan (incremental)

1. Extract & generalize the step test out of `pid.py` into
   `tuning/step_test.py`, rebased on `TelemetryStream`. Add the missing
   metrics (steady-state error, oscillation count, peak current/temp) in
   `metrics.py`. Pure functions, hardware-free to unit test.
2. Add the watchdog (`watchdog.py`): current/temperature/oscillation abort
   + gain-restore, independent of tuning — closes a real safety gap in
   `hardware.py` today.
3. Add `PIDAcceptanceCriteria` / `PIDTuningReport` (`acceptance.py`),
   following the `AcceptanceTolerances` / `CalibrationReport` shape.
4. Add the search algorithm (`search.py`) as `run_trial()`-injected
   coordinate descent, unit-tested against a synthetic 2nd-order discrete
   plant (mass-spring-damper + dead time).
5. Wire into the dashboard: an "Auto-Tune" section in the existing PID tab,
   background thread (same pattern as `_do_step`), live trial log, step
   target bound to `effective_limits`, and an explicit "Accept & Write"
   button gated on `report.ready`.
6. Persist provenance (`provenance.py`) next to
   `~/.soarm_sdk/calibration.json` — algorithm, criteria used, before/after
   gains, timestamp, servo id, pass/fail history.

### Test plan (hardware-free — no test may need a connected arm)

- `metrics.py` against synthetic time/position/current/temp arrays.
- `search.py` against a synthetic plant simulator: converges to known-good
  gains within the trial budget; never proposes gains outside configured
  bounds.
- `watchdog.py` against fabricated `ServoSample` sequences that spike
  current/temp/oscillation: abort fires and last-good gains are restored,
  using this repo's existing fake port/packet-handler doubles for the
  EEPROM restore write.
- `acceptance.py`: `PIDTuningReport.ready` truth table; `validated` only
  flips after `repeat_trials` consecutive passes (regression test for a
  single lucky pass being insufficient).

### Real-hardware validation protocol

1. Read and record current gains + one baseline step response per joint
   before touching anything — the rollback point.
2. Joint order: non-gravity-loaded first (gripper, wrist roll), then
   gravity-loaded joints last (shoulder_lift, elbow_flex) once the watchdog
   and bounds are trusted.
3. First pass per joint is supervised — operator within reach of
   power/E-stop — until the watchdog's abort path has been observed to work
   on that joint at least once.
4. Conservative search bounds on hardware (e.g. ±32 around current gains)
   even if simulator tests used wider bounds — real gear backlash/friction
   is not in the synthetic plant.
5. After acceptance passes, re-verify in context: replay a real teleop or
   `soarm_tamp` trajectory segment on the tuned joint, since coupled-chain
   dynamics can differ from an isolated single-joint step test.
6. Mark `validated: True` only after step 5, and log the provenance file.

## Implementation status

`soarm_sdk/tuning/` (`metrics.py`, `step_test.py`, `acceptance.py`,
`watchdog.py`, `gains_io.py`, `search.py`, `provenance.py`) is implemented
and unit-tested (`tests/test_tuning_*.py`, `tests/test_pid_panel.py`), and
wired into the dashboard's PID Tuning tab (Safety limits, Step Response,
Auto-Tune sections). All hardware-free per this repo's convention.

Three deviations from the plan above, each deliberate:

- **Sampling is not `TelemetryStream`-based.** The step test and the
  search's trials read position/current/temperature directly
  (`_read_sample` in `dashboard/panels/pid.py`: `ReadPos` + `ReadCurrent` +
  `ReadTemperature`, three round trips), not via
  `ServoHardwareInterface.subscribe()`. This tunes exactly one servo at a
  time, so the per-transaction overhead `GroupSyncRead` exists to avoid
  doesn't apply, and it keeps auto-tune working identically whether or not
  the dashboard's opt-in `--stream` mode is active. `tuning/step_test.py`'s
  `read_sample` callback is intentionally source-agnostic, so wiring it to
  a `TelemetryStream` instead is a future drop-in change if a
  multi-joint or higher-rate use case shows up.
- **The watchdog is a per-sample check inside the polling loop**
  (`tuning/watchdog.py`'s `Watchdog`/`check_sample`, called from
  `run_step_test`'s `watchdog=` argument), not a second concurrent thread.
  Reaction time is one poll interval either way, so a separate thread would
  add complexity without adding safety margin.
- **The search is two phases, not "N consecutive passing trials during
  descent."** Phase 1 (`coordinate_descent_tune`) converges by cost with no
  repeat-trial bookkeeping; phase 2 re-runs only the converged winner up to
  `repeat_trials` times. This matches the acceptance criteria's actual
  intent — confirm one candidate is reliable — rather than requiring
  several different candidates in a row to each individually pass, which
  the original phrasing could be read as.

**Deferred, not yet implemented:** bounding the Step Response target (and
auto-tune's commanded target) to the joint's validated calibration
(`RobotCalibration.joints[i].tick_min/tick_max`, gated on
`RobotCalibration.validated`). The slider remains the full 0–4095 raw-tick
range today. Follow the `calibrate` skill's joint-by-name lookup
(`SOARM100_JOINT_NAMES[sid - 1]`, as `dashboard/panels/monitor.py` already
does) to add this gate before running unattended/wide-bound searches on
hardware.
