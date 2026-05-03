// ============================================================
// launch.ks  –  Ascent to Orbit
// kOS script for Kerbal Space Program
//
// PARAMETERS
//   p_targetAlt  : Target circular-orbit altitude in metres.
//                  Default: 100 000 m (100 km).
//   p_targetName : Exact name of a vessel already in orbit to
//                  rendezvous with.  Default: "" (altitude only).
//
// USAGE
//   run launch(100000).                   // ascend to 100 km
//   run launch(200000, "Kerbal Station"). // rendezvous launch
//
// NOTES
//   • Designed for Kerbin launches from KSC, but works on any
//     body with an atmosphere (adjust CFG as needed).
//   • If a target vessel is given the script calculates the next
//     phase-angle window and uses time-warp to wait for it.
//   • The circularisation burn is created as a maneuver node and
//     executed automatically.
// ============================================================

parameter p_targetAlt  is 100000.
parameter p_targetName is "".

// ── Configuration ─────────────────────────────────────────────
local CFG_TURN_START_ALT  is 500.    // m   – altitude to start pitch-over
local CFG_TURN_END_ALT    is 50000.  // m   – altitude where turn completes
local CFG_TURN_END_PITCH  is 5.      // °   – final pitch at end of turn
local CFG_DEFAULT_HEADING is 90.     // °   – heading from north (90 = east)
local CFG_CIRC_ACCURACY   is 0.5.    // m/s – Δv threshold to end circ burn
local CFG_WARP_LEAD       is 30.     // s   – stop warp this early before window
local CFG_ASCENT_EST      is 380.    // s   – estimated ascent duration (Kerbin LKO)
local CFG_STAGE_COOLDOWN  is 2.      // s   – minimum gap between staging events

// ── Derived constants ─────────────────────────────────────────
local FULL_CIRCLE_DEG   is 360.      // degrees in a full circle
local MIN_ANG_VEL       is 0.00001.  // deg/s – threshold to treat angular velocities as equal

// ── State ─────────────────────────────────────────────────────
local g_tgt       is 0.
local g_hasTgt    is false.
local g_hdg       is CFG_DEFAULT_HEADING.
local g_stageTime is 0.   // universal time of last staging event

// ── Resolve target vessel ──────────────────────────────────────
if p_targetName <> "" {
    list vessels in vlist.
    for v in vlist {
        if v:name = p_targetName {
            set g_tgt    to v.
            set g_hasTgt to true.
            set target   to v.
            break.
        }
    }
    if not g_hasTgt {
        print "WARNING: vessel '" + p_targetName + "' not found.".
        print "         Proceeding to altitude only.".
    }
}

// ── Banner ────────────────────────────────────────────────────
clearscreen.
print "=== kOS Ascent to Orbit ===".
print "Target altitude : " + fmtKm(p_targetAlt).
if g_hasTgt {
    print "Rendezvous tgt  : " + g_tgt:name.
    print "Target orbit    : " + fmtKm(g_tgt:apoapsis) +
          " x " + fmtKm(g_tgt:periapsis).
    // Adjust heading to match target inclination
    set g_hdg to calcLaunchAzimuth(g_tgt:inclination).
    print "Launch heading  : " + round(g_hdg, 1) + "deg".
}

// ── Launch window ─────────────────────────────────────────────
if g_hasTgt {
    print "".
    print "Computing launch window...".
    local waitSec is calcLaunchWindow().
    local windowUT is time:seconds + waitSec.
    print "Launch window in: " + fmtTime(waitSec).

    if waitSec > CFG_WARP_LEAD + 5 {
        print "Time-warping to launch window...".
        warpto(windowUT - CFG_WARP_LEAD).
        wait until time:seconds >= windowUT - CFG_WARP_LEAD.
        set warp to 0.
    }

    // Wait out any remaining seconds at 1x
    wait until time:seconds >= windowUT.
}

// ── Pre-launch ────────────────────────────────────────────────
sas off.
rcs off.
lock throttle to 0.
lock steering to heading(g_hdg, 90).

print "".
print "Pre-launch checks complete.".
countdown(5).

// ── Ignition ──────────────────────────────────────────────────
print "IGNITION!".
lock throttle to 1.0.
stage.

// ── Gravity-turn ascent ───────────────────────────────────────
until ship:apoapsis >= p_targetAlt {

    // Pitch program: linear from 90° to CFG_TURN_END_PITCH
    local pitch is calcPitch(ship:altitude).
    lock steering to heading(g_hdg, pitch).

    // Throttle: back off as apoapsis approaches target to avoid overshoot
    local apoFrac is ship:apoapsis / p_targetAlt.
    if apoFrac > 0.95 {
        lock throttle to max(0.05, 1.0 - (apoFrac - 0.95) * 20.0).
    } else {
        lock throttle to 1.0.
    }

    // Stage if engines have flamed out
    checkStage().

    // Status line
    print "Alt " + fmtKm(ship:altitude) +
          "  Apo " + fmtKm(ship:apoapsis) +
          "  Pitch " + round(pitch, 0) + "deg      " at (0, 14).

    wait 0.
}

lock throttle to 0.
print "".
print "Target apoapsis reached - coasting.".

// ── Coast to apoapsis ─────────────────────────────────────────
lock steering to prograde.

if eta:apoapsis > 60 {
    print "Warping to apoapsis...".
    warpto(time:seconds + eta:apoapsis - 45).
    wait until eta:apoapsis < 50.
    set warp to 0.
}

// ── Circularisation ───────────────────────────────────────────
execCirc().

// ── Done ──────────────────────────────────────────────────────
lock throttle to 0.
unlock steering.
sas on.
rcs on.

print "".
print "=== ASCENT COMPLETE ===".
print "Final orbit: " + fmtKm(ship:apoapsis) + " x " + fmtKm(ship:periapsis).

if g_hasTgt {
    local pa is phaseAngle(
        ship:position - body:position,
        g_tgt:position - body:position
    ).
    print "Phase angle to target: " + round(pa, 1) + "deg".
    print "(Use a rendezvous script to close the remaining gap.)".
}


// ============================================================
// FUNCTION DEFINITIONS
// ============================================================

// ── Formatting ───────────────────────────────────────────────

function fmtKm {
    parameter m.
    return round(m / 1000, 1) + " km".
}

function fmtTime {
    parameter s.
    set s to max(0, s).
    local h   is floor(s / 3600).
    local m   is floor(mod(s, 3600) / 60).
    local sec is round(mod(s, 60)).
    return h + "h " + m + "m " + sec + "s".
}

function countdown {
    parameter n.
    from { local i is n. } until i <= 0 step { set i to i - 1. } do {
        print "T-" + i + "     ".
        wait 1.
    }
}

// ── Ascent helpers ───────────────────────────────────────────

// Linearly-interpolated pitch (degrees above horizon) for gravity turn
function calcPitch {
    parameter alt.
    if alt < CFG_TURN_START_ALT { return 90. }
    if alt > CFG_TURN_END_ALT   { return CFG_TURN_END_PITCH. }
    local frac is (alt - CFG_TURN_START_ALT) /
                  (CFG_TURN_END_ALT - CFG_TURN_START_ALT).
    return 90 - (90 - CFG_TURN_END_PITCH) * frac.
}

// Stage if all engines have flamed out, with cooldown guard
function checkStage {
    if ship:maxthrust < 0.01 and
       stage:number > 0 and
       time:seconds > g_stageTime + CFG_STAGE_COOLDOWN {
        print "Staging (stage " + stage:number + ")...".
        stage.
        set g_stageTime to time:seconds.
    }
}

// ── Orbital mechanics ────────────────────────────────────────

// Speed of a circular orbit at the given altitude above the current body
function circVel {
    parameter alt.
    return sqrt(body:mu / (body:radius + alt)).
}

// Period of a circular orbit at the given altitude
function orbPeriod {
    parameter alt.
    return 2 * constant:pi * sqrt((body:radius + alt)^3 / body:mu).
}

// Signed angle from pos1 to pos2 in degrees.
// Positive = pos2 is ahead of pos1 in the direction of the body's rotation.
function phaseAngle {
    parameter pos1, pos2.
    local angle is vang(pos1, pos2).
    if vdot(vcrs(pos1, pos2), body:angularvel) < 0 {
        set angle to -angle.
    }
    return angle.
}

// Launch heading (degrees from north) needed to reach the given inclination,
// corrected for the body's surface rotation.
function calcLaunchAzimuth {
    parameter tgtInc.
    local lat is ship:latitude.
    // Inclination cannot be less than the latitude of the launch site
    local inc is max(abs(tgtInc), abs(lat)).
    local sinAz is cos(inc) / cos(lat).
    set sinAz to max(-1.0, min(1.0, sinAz)).
    // arcsin returns [-90, 90]; for eastward prograde launches this maps
    // directly to the compass heading (0 = N, 90 = E).
    return arcsin(sinAz).
}

// Seconds until the next optimal launch window for a rendezvous with g_tgt.
//
// Theory: the target vessel moves faster than the launch site rotates.
// We need the target to be slightly *behind* us at launch so it catches
// up to our orbit-insertion point during ascent.
//
// desiredPhase = −(ω_tgt − ω_site) × flightTime
//
// We then solve for how long to wait for the current phase to reach
// desiredPhase given the relative angular rate.
function calcLaunchWindow {
    local tgt    is g_tgt.
    local tgtAlt is (tgt:apoapsis + tgt:periapsis) / 2.

    // Angular velocities in deg/s
    local tgtAngVel  is FULL_CIRCLE_DEG / orbPeriod(tgtAlt).
    local siteAngVel is FULL_CIRCLE_DEG / body:rotationperiod.
    local relAngVel  is tgtAngVel - siteAngVel.  // > 0 for typical LKO targets

    // Total estimated flight time: ascent + Hohmann transfer (if target is higher)
    local flightTime is CFG_ASCENT_EST.
    if abs(tgtAlt - p_targetAlt) > 5000 {
        local xfrSMA is (body:radius + p_targetAlt + body:radius + tgtAlt) / 2.
        set flightTime to flightTime +
            constant:pi * sqrt(xfrSMA^3 / body:mu).
    }

    // Phase angle the target must have at launch time (negative = target behind us)
    local desiredPhase is -(relAngVel * flightTime).

    // Current phase angle (positive = target is ahead of us)
    local myVec  is ship:position - body:position.
    local tgtVec is tgt:position  - body:position.
    local curPhase is phaseAngle(myVec, tgtVec).

    // How many degrees of phase-angle change do we still need?
    local diff is desiredPhase - curPhase.
    set diff to mod(diff, FULL_CIRCLE_DEG).
    if diff < 0 { set diff to diff + FULL_CIRCLE_DEG. }   // normalise to [0, 360)

    // Edge case: both bodies have effectively the same angular rate
    if abs(relAngVel) < MIN_ANG_VEL { return orbPeriod(tgtAlt). }

    return max(0, diff / relAngVel).
}

// Calculate and execute a prograde circularisation burn at apoapsis.
function execCirc {
    print "Planning circularisation burn...".

    // Δv from vis-viva equation at apoapsis
    local apoR  is body:radius + ship:apoapsis.
    local sma   is (body:radius + ship:apoapsis +
                    body:radius + ship:periapsis) / 2.
    local vApo  is sqrt(body:mu * (2.0 / apoR - 1.0 / sma)).
    local vCirc is circVel(ship:apoapsis).
    local dv    is vCirc - vApo.
    local eta   is eta:apoapsis.

    print "  dv = " + round(dv, 1) + " m/s   ETA " + round(eta) + " s".

    // Add the maneuver node at the current ETA to apoapsis
    local nd is node(time:seconds + eta, 0, 0, dv).
    add nd.

    // Estimated burn duration (half-burn before, half after apoapsis)
    local burnDur is ship:mass * abs(dv) / max(0.01, ship:maxthrust).
    local burnUT  is time:seconds + eta - burnDur / 2.

    // Time-warp to 10 s before ignition
    if burnUT - time:seconds > 15 {
        print "  Warping to burn window...".
        warpto(burnUT - 10).
        wait until time:seconds >= burnUT - 10.
        set warp to 0.
    }
    wait until time:seconds >= burnUT.

    // Point at burn vector and wait for alignment
    lock steering to nd:burnvector.
    print "  Orienting...".
    wait until vang(ship:facing:vector, nd:burnvector) < 1 or
               eta:apoapsis < 5.

    // Execute the burn
    print "  Burning...".
    local initDV is nd:deltav:mag.
    lock throttle to 1.0.

    until nd:deltav:mag < CFG_CIRC_ACCURACY {
        // Throttle down for the last 10 m/s to avoid overshoot
        if nd:deltav:mag < 10 {
            lock throttle to max(0.05, nd:deltav:mag / initDV).
        }
        // Stage if needed during the burn
        checkStage().
        wait 0.
    }

    lock throttle to 0.
    remove nd.
    print "  Done. Orbit: " + fmtKm(ship:apoapsis) +
          " x " + fmtKm(ship:periapsis).
}
