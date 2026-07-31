# RammsAccess — Alternative-Access Input for RAMMS (BCI, HID, AT devices)

Plan of record for interfacing alternative input devices — OpenBCI Galea,
Neuralink (via its user-facing control surface), and conventional assistive
technology (sip-and-puff, head arrays, switches, eye gaze, adaptive
joysticks) — with the RAMMS simulator now and the physical robot later.
Drafted 2026-07-30.

## Architecture: device hub outside the engine, thin consumer inside

```
 devices                    ramms-access hub (Python)              consumers
┌──────────┐   BrainFlow   ┌──────────────────────────┐   intent  ┌─────────────────┐
│ Galea    │──────────────▶│ adapters → decoders →    │  stream   │ RAMMS (UE)      │
│ HID/ptr  │──────────────▶│ calibration → INTENTS    │──UDP─────▶│ RammsAccess-    │
│ sip-puff │──────────────▶│                          │  JSON v1  │ InputComponent  │
│ ...      │               │ optional: LSL record tap │     │     └─────────────────┘
└──────────┘               └──────────────────────────┘     └────▶ real robot bridge
                                                                   (ROS2 node, later)
```

Why this split:
- **Signal processing lives in Python**, outside the engine: BrainFlow, MNE,
  sklearn/torch decoders, calibration UIs all iterate at pip speed and must
  not require an engine rebuild — and the SAME hub must eventually drive the
  physical robot with the sim entirely absent.
- **The engine consumes intents, not signals.** The UE side is one component
  that maps a normalized intent stream onto the controllers that already
  exist: `URammsDifferentialDriveController::SetDriveInput` (the wheelchair
  joystick abstraction), `UKinovaGen3ControllerComponent::
  ApplyEndEffectorTeleopInput` (EE teleop rates), gripper open/close/toggle.
  Decoders improve without touching the robot-facing contract.
- **The intent schema IS the product.** Devices and decoders churn; the
  versioned intent contract is what both the sim and the future robot bridge
  depend on.

## Intent protocol v1

Transport: **UDP, JSON, latest-wins**, default port **30040** (RC=30010,
RMSS=30030, URLab=5555-5559 are taken). Control streams want lossy
latest-wins semantics — a late control packet is worse than a dropped one —
and UDP keeps both ends dependency-free (UE has Json/Sockets built in; the
hub's core is pure stdlib). msgpack/ZMQ can slot in later behind the same
schema if a higher-rate channel is ever needed; LSL is deliberately kept on
the hub side as an optional *recording* tap, not the control path.

```json
{"v": 1, "t": 1769812345.123, "src": "galea", "seq": 42,
 "drive": {"x": -0.2, "y": 0.8},
 "ee": {"lin": [0, 0, 0.5], "ang": [0, 0, 0]},
 "events": ["gripper_toggle"],
 "conf": 0.85}
```

- `drive`: wheelchair joystick semantics, x=turn y=forward, −1..1.
- `ee`: normalized end-effector rates (linear xyz, angular pitch/yaw/roll).
- `events` (discrete, processed in order): `estop`, `resume`, `stop`,
  `gripper_open`, `gripper_close`, `gripper_toggle`, `sync_target`.
- `conf`: decoder confidence 0..1; the consumer scales continuous rates by
  it (low-confidence BCI output moves the robot slowly, not wrongly).

**Safety (non-negotiable for AT use):** the consumer runs a watchdog —
no valid packet within `WatchdogTimeoutMs` (default 250) → drive zeroed,
EE rates zeroed (arm holds pose). `estop` latches everything zero until
`resume`. The same rules must carry to the robot bridge verbatim.

## Device feasibility

| Device | Path | Feasibility |
|---|---|---|
| **OpenBCI Galea** (in hand) | BrainFlow (`GALEA_BOARD`, WiFi streaming): EEG+EMG+EDA+PPG+eye | **Near-term real.** v0: EMG-driven control (jaw clench L/R → turn, sustained clench → forward; blink pairs → events) — robust, low-latency, minimal calibration. v1: SSVEP/motor-imagery EEG decoding for discrete commands; eye channel for pointing. The adapter skeleton ships now; decoder work is iterative lab time with the headset. |
| **Neuralink** | No public SDK. A Link user's output surface is standard **cursor/HID control** on their paired device. | **Pragmatic path:** a first-class *pointer/HID adapter* (any OS-level pointer or gamepad → intents). That makes "Neuralink support" = the user pointing at a virtual joystick pad / dwell-clicking commands — identical machinery also covers head mice, eye-gaze pointers, and every HID-class AT device. If the collaboration yields something richer (raw or intermediate access), it becomes another adapter behind the same schema. |
| **Sip-and-puff, switches, head arrays** | HID/joystick/serial | Trivial adapters over the same base class; mostly mapping + debounce logic. |
| **Gamepad/keyboard** | evdev/pygame/pynput | Ships now as the dev/test adapter (also the demo driver for CI). |

## Repository layout (recommendation: ONE new repo)

`rammp-org/ramms-access`, submoduled at `Plugins/RammsAccess` — the URLab
pattern: a UE plugin repo that carries its non-engine tooling with it.

```
RammsAccess.uplugin
Source/RammsAccess/            # UE module: intent receiver + mapping
python/                        # the hub: pip install -e python/
  src/ramms_access/
    intents.py transport.py    # schema + UDP publisher (stdlib only)
    adapters/ (base, demo, pointer_hid, galea, ...)
    cli.py                     # ramms-access <adapter> [...]
doc/PLAN.md                    # this file
```

One repo keeps schema, hub, and consumer in lockstep (schema changes land
atomically). The hub stays usable robot-side without UE via
`pip install "git+…ramms-access#subdirectory=python"`. A future ROS2 bridge
node can live in `python/` too or graduate to its own repo when the robot
integration starts in earnest.

## Milestones

1. **Vertical slice (scaffolded now):** UE component + hub + demo adapter —
   `ramms-access demo --pattern circle` drives the chair in PIE; keyboard
   adapter proves interactive latency; watchdog/estop verified.
2. **Pointer/HID adapter:** OS pointer + gamepad → virtual-joystick and
   dwell-command mapping. This is the Neuralink-ready path and immediately
   useful for head-mouse/eye-gaze users.
3. **Galea online:** BrainFlow session management, signal-quality display,
   EMG calibration routine (record neutral/clench baselines → thresholds),
   EMG control of drive + gripper. First real BCI-in-the-loop demo.
4. **EEG decoding:** SSVEP or motor-imagery discrete commands with
   confidence-scaled rates; recorded sessions (LSL tap) feed offline decoder
   training — dovetails with the parallel-sim data-collection plan.
5. **Robot bridge:** ROS2 node consuming the identical intent stream;
   safety cert of watchdog/estop path on hardware.

## Open questions

- Repo name: `ramms-access` (assistive "alternative access" framing) vs
  `ramms-bci` (narrower). Plan assumes `ramms-access`.
- Galea unit specifics (channel count/firmware) — verify against BrainFlow
  board id on first connect.
- Neuralink collaboration: what surface the user can actually share (screen
  control only? intermediate decoder outputs?) — determines whether they get
  the pointer adapter or a dedicated one.
