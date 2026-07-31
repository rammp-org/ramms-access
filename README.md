# RammsAccess

Alternative-access input for RAMMS: BCI (OpenBCI Galea), pointer/HID devices
(the Neuralink-compatible path, head mice, eye gaze), and conventional
assistive switches — normalized into one versioned intent stream consumed by
the simulator today and the physical robot later.

Full design + device feasibility + milestones: [doc/PLAN.md](doc/PLAN.md).

## Layout

- `Source/RammsAccess/` — UE plugin module. `URammsAccessInputComponent`
  listens for intents (UDP JSON v1, port 30040) and maps them onto the
  robot's existing controllers (differential drive, Kinova EE teleop,
  gripper), with a safety watchdog (silence → stop) and estop latch.
- `python/` — the device hub. Core is stdlib-only; device support via
  extras: `pip install -e "python[all]"`.

## Quickstart (vertical slice)

1. Add `URammsAccessInputComponent` to `BP_Mebot_Ramms` (next to the drive /
   Kinova / gripper controllers). PIE.
2. `pip install -e Plugins/RammsAccess/python`
3. `ramms-access demo --pattern circle` — the chair drives in a circle.
   Ctrl-C → estop → the chair stops. Kill -9 it instead → watchdog stops the
   chair within 250 ms.
4. `ramms-access pointer` — drive with the OS cursor (click = gripper).
5. Galea: `ramms-access galea --calibrate`, then `ramms-access galea`.
