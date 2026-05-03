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
local CFG_KILL_GAIN      is 0.50.  // controller gain while nulling rel velocity
local CFG_ALIGN_TOL      is 3.00.  // deg allowed port misalignment
local CFG_LOOP_WAIT      is 0.01.  // physics-tick loop wait
local CFG_SPEED_DIST_K   is 0.12.  // approach-speed increase per metre of error
local CFG_STANDOFF_FAR   is 120.   // m first hold point on the docking axis
local CFG_STANDOFF_MID   is 40.    // m second hold point on the docking axis
local CFG_STANDOFF_NEAR  is 12.    // m third hold point on the docking axis
local CFG_STANDOFF_FINAL is 3.     // m final hold point before soft dock
local CFG_SPEED_FAR      is 6.0.   // m/s max speed for far standoff leg
local CFG_SPEED_MID      is 2.5.   // m/s max speed for mid standoff leg
local CFG_SPEED_NEAR     is 0.9.   // m/s max speed for near standoff leg
local CFG_SPEED_FINAL    is 0.35.  // m/s max speed for final standoff leg
local CFG_POSTOL_FAR     is 6.0.   // m position tolerance at far standoff
local CFG_POSTOL_MID     is 2.0.   // m position tolerance at mid standoff
local CFG_POSTOL_NEAR    is 0.7.   // m position tolerance at near standoff
local CFG_POSTOL_FINAL   is 0.25.  // m position tolerance at final standoff
local CFG_VELTOL_FAR     is 0.8.   // m/s relative-speed tolerance at far standoff
local CFG_VELTOL_MID     is 0.4.   // m/s relative-speed tolerance at mid standoff
local CFG_VELTOL_NEAR    is 0.20.  // m/s relative-speed tolerance at near standoff
local CFG_VELTOL_FINAL   is 0.12.  // m/s relative-speed tolerance at final standoff
local CFG_DOCK_VEL       is 0.20.  // m/s max desired velocity during soft dock
local CFG_DOCK_CMD       is 0.35.  // max translation command during soft dock

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

function zero_translation_controls {
    set ship:control:starboard to 0.
    set ship:control:fore to 0.
    set ship:control:top to 0.
}

function translate_world {
    parameter vec.
    local cmd is clamp_mag(vec, 1).
    set ship:control:starboard to vdot(cmd, ship:facing:starvector).
    set ship:control:fore to vdot(cmd, ship:facing:forevector).
    set ship:control:top to vdot(cmd, ship:facing:topvector).
}

function is_free_port {
    parameter port.
    return (port:state = "Ready").
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
    local bestOwn  is ownPorts[0].
    local bestTgt  is tgtPorts[0].
    local bestDist is (bestTgt:nodeposition - bestOwn:nodeposition):mag.

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
    return clamp(distance * CFG_SPEED_DIST_K, CFG_SAFE_VEL, maxSpeed).
}

function hold_target_alignment {
    parameter tgtPort.
    lock steering to lookdirup(-tgtPort:portfacing:vector, tgtPort:portfacing:upvector).
}

function kill_relative_velocity {
    parameter tgtVessel.
    parameter tol.
    until relative_velocity_to(tgtVessel):mag < tol {
        translate_world(-relative_velocity_to(tgtVessel) * CFG_KILL_GAIN).
        wait CFG_LOOP_WAIT.
    }
    zero_translation_controls.
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

    zero_translation_controls.
}

function final_dock {
    parameter ownPort.
    parameter tgtPort.

    until ownPort:state <> "Ready" or tgtPort:state <> "Ready" {
        local posErr is tgtPort:nodeposition - ownPort:nodeposition.
        local relVel is relative_velocity_to(tgtPort:ship).
        local desiredVel is clamp_mag(posErr * CFG_POS_GAIN, CFG_DOCK_VEL).
        local cmd is clamp_mag(desiredVel - relVel * CFG_RVEL_GAIN, CFG_DOCK_CMD).

        translate_world(cmd).
        wait CFG_LOOP_WAIT.
    }

    zero_translation_controls.
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
    ownPort:controlfrom.
    hold_target_alignment(tgtPort).

    print "".
    print "Killing relative velocity...".
    kill_relative_velocity(tgtVessel, CFG_KILL_VEL).

    print "Moving to " + round(CFG_STANDOFF_FAR, 0) + " m standoff...".
    fly_to_standoff(ownPort, tgtPort, CFG_STANDOFF_FAR, CFG_SPEED_FAR, CFG_POSTOL_FAR, CFG_VELTOL_FAR).

    print "Moving to " + round(CFG_STANDOFF_MID, 0) + " m standoff...".
    fly_to_standoff(ownPort, tgtPort, CFG_STANDOFF_MID, CFG_SPEED_MID, CFG_POSTOL_MID, CFG_VELTOL_MID).

    print "Moving to " + round(CFG_STANDOFF_NEAR, 0) + " m standoff...".
    fly_to_standoff(ownPort, tgtPort, CFG_STANDOFF_NEAR, CFG_SPEED_NEAR, CFG_POSTOL_NEAR, CFG_VELTOL_NEAR).

    print "Moving to " + round(CFG_STANDOFF_FINAL, 0) + " m standoff...".
    fly_to_standoff(ownPort, tgtPort, CFG_STANDOFF_FINAL, CFG_SPEED_FINAL, CFG_POSTOL_FINAL, CFG_VELTOL_FINAL).

    print "Final docking approach...".
    final_dock(ownPort, tgtPort).

    unlock steering.
    unlock throttle.
    zero_translation_controls.

    if ownPort:state = "Ready" and tgtPort:state = "Ready" {
        print "Docking not completed - hold position and check alignment.".
    } else {
        print "Docking complete.".
    }
}

main.
