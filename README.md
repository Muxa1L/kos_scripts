# kos_scripts

A collection of [kOS](https://github.com/KSP-KOS/KOS) scripts for Kerbal Space Program.

---

## `launch.ks` – Ascent to Orbit

Fully-automated gravity-turn ascent from the pad to a circular orbit,
with optional rendezvous launch-window support.

### Usage

```kerboscript
// Ascend to a 100 km circular orbit
run launch(100000).

// Ascend to 200 km and time the launch to rendezvous with "Kerbal Station"
run launch(200000, "Kerbal Station").
```

### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `p_targetAlt` | `100000` | Target circular-orbit altitude in metres |
| `p_targetName` | `""` | Exact name of an in-orbit vessel to rendezvous with (leave blank for altitude-only flight) |

### Features

* **Configurable target altitude** – any circular orbit altitude you choose.
* **Gravity-turn ascent** – linear pitch programme from 90° at pad to ~5° at
  50 km, then coasts to apoapsis on prograde.
* **Auto-staging** – stages automatically when all active engines flame out
  (with a 2-second cooldown to prevent double-staging).
* **Target-vessel rendezvous launch window** – when a vessel name is supplied
  the script:
  1. Looks up the vessel in the active vessel list.
  2. Calculates the optimal phase angle (accounting for estimated ascent
     time and a Hohmann transfer leg if the target orbit is higher).
  3. Uses `warpto()` to skip forward in time to the window, stopping
     `CFG_WARP_LEAD` seconds before launch.
  4. Adjusts the launch heading to match the target's orbital inclination.
* **Circularisation burn** – uses a maneuver node and executes it at
  apoapsis, throttling down for the last 10 m/s to avoid overshoot.
* **Post-flight report** – prints final apoapsis × periapsis and, when a
  target vessel is set, the residual phase angle to the target.

### Configuration constants (top of file)

| Constant | Default | Meaning |
|----------|---------|---------|
| `CFG_TURN_START_ALT` | `500 m` | Altitude at which pitch-over begins |
| `CFG_TURN_END_ALT` | `50,000 m` | Altitude where gravity turn completes |
| `CFG_TURN_END_PITCH` | `5°` | Final pitch angle at end of turn |
| `CFG_DEFAULT_HEADING` | `90°` | Launch azimuth (90 = due east) |
| `CFG_CIRC_ACCURACY` | `0.5 m/s` | Δv threshold to end circularisation burn |
| `CFG_WARP_LEAD` | `30 s` | How early to stop time-warp before the window |
| `CFG_ASCENT_EST` | `380 s` | Estimated ascent duration used in window calc |
| `CFG_STAGE_COOLDOWN` | `2 s` | Minimum time between staging events |

### Notes

* Designed for Kerbin / KSC.  For other bodies adjust `CFG_TURN_END_ALT`
  and `CFG_ASCENT_EST` to match the atmosphere thickness and gravity.
* The rendezvous launch window targets the correct *phase angle* but does
  not match the orbital *plane* beyond a simple heading adjustment.  For
  targets in highly-inclined orbits a dedicated plane-change burn after
  circularisation may be needed.
* After ascent is complete, use a dedicated rendezvous script to close the
  remaining gap and dock.
