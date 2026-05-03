// ============================================================
// rendezvous_dock.ks  –  Rendezvous and Docking
// kOS script for Kerbal Space Program
//
// USAGE
//   1. Set a vessel as the current TARGET.
//   2. Make sure both vessels have an undocked docking port.
//   3. Start from the same orbit regime / close rendezvous.
//   4. Run:  run rendezvous_dock.
//
// NOTES
//   • The script automatically chooses the nearest free docking-port
//     pair between the active vessel and the selected target vessel.
//   • Uses RCS translation for the rendezvous corridor and docking.
//   • Best used once the target is already nearby (same orbit and
//     relative speed low enough for proximity operations).
// ============================================================

local CFG_SAFE_VEL       is 0.15.  // m/s minimum controller speed cap
local CFG_RVEL_GAIN      is 1.00.  // relative-velocity damping gain
local CFG_POS_GAIN       is 0.08.  // position-error to desired-velocity gain
local CFG_KILL_VEL       is 0.25.  // m/s tolerance for pre-approach stop
local CFG_ALIGN_TOL      is 3.00.  // deg allowed port misalignment
local CFG_LOOP_WAIT      is 0.     // physics-tick loop wait

function clamp {
    parameter v.
    parameter lo.
    parameter hi.
    return max(lo, min(hi, v)).
}

function clamp_mag {
    parameter vec.
    parameter maxMag.
    if vec:mag <= maxMag { return vec. }
    if maxMag <= 0       { return V(0,0,0). }
    return vec:normalized * maxMag.
}

function stop_translation {
    set ship:control:starboard to 0.
    set ship:control:fore to 0.
    set ship:control:top to 0.
}

function translate_world {
    parameter vec.
    local cmd is clamp_mag(vec, 1).
    set ship:control:starboard to cmd * ship:facing:starvector.
    set ship:control:fore to cmd * ship:facing:forevector.
    set ship:control:top to cmd * ship:facing:topvector.
}

function is_free_port {
    parameter port.
    return port:state = "Ready".
}

function collect_free_ports {
    parameter vesselRef.
    local ports is list().
    for port in vesselRef:dockingports {
        if is_free_port(port) {
            ports:add(port).
        }
    }
    return ports.
}

function pick_port_pair {
    parameter ownPorts.
    parameter tgtPorts.
    local bestOwn  is 0.
    local bestTgt  is 0.
    local bestDist is 999999999.

    for ownPort in ownPorts {
        for tgtPort in tgtPorts {
            local dist is (tgtPort:nodeposition - ownPort:nodeposition):mag.
            if dist < bestDist {
                set bestDist to dist.
                set bestOwn to ownPort.
                set bestTgt to tgtPort.
            }
        }
    }

    return list(bestOwn, bestTgt, bestDist).
}

function port_alignment_error {
    parameter ownPort.
    parameter tgtPort.
    return vang(ownPort:portfacing:vector, -tgtPort:portfacing:vector).
}

function relative_velocity_to {
    parameter vesselRef.
    return ship:velocity:orbit - vesselRef:velocity:orbit.
}

function desired_standoff_speed {
    parameter distance.
    parameter maxSpeed.
    return clamp(distance * 0.12, CFG_SAFE_VEL, maxSpeed).
}

function hold_target_alignment {
    parameter tgtPort.
    lock steering to lookdirup(-tgtPort:portfacing:vector, tgtPort:portfacing:upvector).
}

function kill_relative_velocity {
    parameter tgtVessel.
    parameter tol.
    until relative_velocity_to(tgtVessel):mag < tol {
        translate_world(-relative_velocity_to(tgtVessel) * 0.5).
        wait CFG_LOOP_WAIT.
    }
    stop_translation().
}

function fly_to_standoff {
    parameter ownPort.
    parameter tgtPort.
    parameter standoff.
    parameter maxSpeed.
    parameter posTol.
    parameter velTol.

    until false {
        if ownPort:state <> "Ready" or tgtPort:state <> "Ready" { break. }

        local desiredPos is tgtPort:nodeposition + tgtPort:portfacing:vector * standoff.
        local posErr is desiredPos - ownPort:nodeposition.
        local relVel is relative_velocity_to(tgtPort:ship).
        local desiredVel is clamp_mag(
            posErr * CFG_POS_GAIN,
            desired_standoff_speed(posErr:mag, maxSpeed)
        ).
        local cmd is clamp_mag(desiredVel - relVel * CFG_RVEL_GAIN, 1).
        local alignErr is port_alignment_error(ownPort, tgtPort).

        translate_world(cmd).

        if posErr:mag < posTol and relVel:mag < velTol and alignErr < CFG_ALIGN_TOL {
            break.
        }

        wait CFG_LOOP_WAIT.
    }

    stop_translation().
}

function final_dock {
    parameter ownPort.
    parameter tgtPort.

    until ownPort:state <> "Ready" or tgtPort:state <> "Ready" {
        local posErr is tgtPort:nodeposition - ownPort:nodeposition.
        local relVel is relative_velocity_to(tgtPort:ship).
        local desiredVel is clamp_mag(posErr * CFG_POS_GAIN, 0.20).
        local cmd is clamp_mag(desiredVel - relVel * CFG_RVEL_GAIN, 0.35).

        translate_world(cmd).
        wait CFG_LOOP_WAIT.
    }

    stop_translation().
}

function main {
    if not hastarget {
        print "ERROR: Select a vessel target first.".
        return.
    }

    local tgtVessel is target.
    local ownPorts is collect_free_ports(ship).
    local tgtPorts is collect_free_ports(tgtVessel).

    if ownPorts:length = 0 {
        print "ERROR: Active vessel has no free docking ports.".
        return.
    }

    if tgtPorts:length = 0 {
        print "ERROR: Target vessel has no free docking ports.".
        return.
    }

    local pair is pick_port_pair(ownPorts, tgtPorts).
    local ownPort is pair[0].
    local tgtPort is pair[1].

    clearscreen.
    print "=== kOS Rendezvous and Docking ===".
    print "Target vessel : " + tgtVessel:name.
    print "Initial range : " + round(pair[2], 1) + " m".
    print "Own port      : " + ownPort:part:name.
    print "Target port   : " + tgtPort:part:name.

    sas off.
    rcs on.
    lock throttle to 0.
    ownPort:controlfrom().
    hold_target_alignment(tgtPort).

    print "".
    print "Killing relative velocity...".
    kill_relative_velocity(tgtVessel, CFG_KILL_VEL).

    print "Moving to 120 m standoff...".
    fly_to_standoff(ownPort, tgtPort, 120, 6.0, 6.0, 0.8).

    print "Moving to 40 m standoff...".
    fly_to_standoff(ownPort, tgtPort, 40, 2.5, 2.0, 0.4).

    print "Moving to 12 m standoff...".
    fly_to_standoff(ownPort, tgtPort, 12, 0.9, 0.7, 0.20).

    print "Moving to 3 m standoff...".
    fly_to_standoff(ownPort, tgtPort, 3, 0.35, 0.25, 0.12).

    print "Final docking approach...".
    final_dock(ownPort, tgtPort).

    unlock steering.
    unlock throttle.
    stop_translation().

    if ownPort:state = "Ready" and tgtPort:state = "Ready" {
        print "Docking not completed - hold position and check alignment.".
    } else {
        print "Docking complete.".
    }
}

main().
