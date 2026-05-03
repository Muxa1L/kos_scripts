// ============================================================
// autoland_orbit_v3_generic.ks   v3.0
// ============================================================
// Generic Orbital-to-Target Propulsive Landing Autopilot.
// SpaceX-style retropropulsive descent from orbit to a
// predefined surface target.
//
// ── PHASE SEQUENCE ──────────────────────────────────────────
//   1. PLANE_ALIGN   Correct orbit plane to pass over target
//   2. DEORBIT       Retrograde burn to aim at landing zone
//   3. COAST         Free-fall with iterative impact prediction
//   4. ENTRY_BURN    High-altitude deceleration (atmo only)
//   5. LANDING_BURN  Main suicide / hover-slam burn
//   6. TERMINAL      Lateral homing + soft touchdown
//
// ── MODES ───────────────────────────────────────────────────
//   AUTO  : all guidance gains derived from vessel & body state
//           each update tick.  Recommended for first use.
//   TUNED : explicit PID gains (Section A3) you can dial in
//           for a specific vessel / body combination.
//
// ── REQUIREMENTS ────────────────────────────────────────────
//   • kOS v1.4+ CPU on the vessel
//   • Stable prograde orbit above the target body
//   • Engines capable of retrograde (bottom-mounted) thrust
//   • Sufficient ΔV for deorbit + powered landing
//   • RCS optional but improves plane-change accuracy
//
// ── ACCURACY NOTE ───────────────────────────────────────────
//   Sub-1 m touchdown is a design goal, not a hard guarantee.
//   Actual delivery accuracy depends on vessel TWR, engine
//   throttle response, body rotation rate, terrain, and
//   physics fidelity.  Typical results:
//     Open-loop delivery (no terminal phase): 50 – 500 m
//     With terminal guidance enabled:           5 – 50 m
//
// ── QUICK START ─────────────────────────────────────────────
//   1. Edit TGT_LAT / TGT_LNG in Section A2 below.
//   2. Place the vessel in a stable orbit.
//   3. Run:  SWITCH TO 0.  RUN autoland_orbit_v3_generic.
//   4. Leave CFG_MODE as "AUTO" for a first attempt.
// ============================================================

CLEARSCREEN.
PRINT "autoland_orbit_v3_generic.ks  v3.0  loading...".


// ============================================================
// SECTION A — USER CONFIGURATION
// ============================================================

// ── A1.  Controller mode ─────────────────────────────────────
// "AUTO"  → parameters derived from vessel & body each tick.
//           Start here; works on most vessels without changes.
// "TUNED" → uses the explicit gains in Section A3.
//           Tune after observing AUTO-mode behaviour.
LOCAL CFG_MODE IS "AUTO".

// ── A2.  Target landing coordinates  ── EDIT THESE ──────────
LOCAL TGT_LAT IS  -0.0972.   // Latitude  (degrees; positive = North, negative = South)
LOCAL TGT_LNG IS -74.5577.   // Longitude (degrees; positive = East,  negative = West)
// Example above: KSC launch-pad area on Kerbin.
// Replace with your own landing-pad coordinates.

// ── A3.  TUNED-mode gains  (ignored when CFG_MODE = "AUTO") ─
//
// Lateral guidance — steers the vessel horizontally toward the
// target during the powered landing burn.
//   _KP : steering tilt (fraction) per metre of horizontal error
//   _KD : steering tilt (fraction) per m/s of lateral speed
//   _KI : steering tilt (fraction) per m·s of integrated error
LOCAL T_LAT_KP IS 0.008.
LOCAL T_LAT_KD IS 0.045.
LOCAL T_LAT_KI IS 0.0001.

// Throttle control during the landing burn.
// Applied to error = (target_vspeed − actual_vspeed)  [m/s].
//   _KP : throttle fraction per m/s of vertical-speed error
//   _KD : throttle fraction per m/s² of error rate of change
LOCAL T_THR_KP IS 0.10.
LOCAL T_THR_KD IS 0.07.

// Terminal descent speed target (m/s, NEGATIVE = downward).
LOCAL T_VTERM IS -1.5.

// AGL altitude (m) at which to arm the landing-burn ignition.
// Set to 0 to let the script auto-compute from vessel TWR.
LOCAL T_BURN_ALT_M IS 0.

// ── A4.  Hard limits  ────────────────────────────────────────
LOCAL CFG_MIN_THR IS 0.05.   // minimum throttle while engines on
LOCAL CFG_MAX_THR IS 1.00.   // maximum throttle
LOCAL CFG_HOVER_M IS 50.     // AGL (m) to enter terminal phase
LOCAL CFG_TOUCH_V IS 2.0.    // max acceptable touchdown speed (m/s)


// ============================================================
// SECTION B — INTERNAL STATE  (do not edit)
// ============================================================
LOCAL _lat_i  IS 0.   LOCAL _lat_ep IS 0.
LOCAL _lng_i  IS 0.   LOCAL _lng_ep IS 0.
LOCAL _thr_ep IS 0.
LOCAL _prev_t IS TIME:SECONDS.


// ============================================================
// SECTION C — UTILITY FUNCTIONS
// ============================================================

// Timestamped console message
FUNCTION clog {
    PARAMETER msg.
    PRINT "[T+" + ROUND(TIME:SECONDS, 1) + "] " + msg.
}

// Clamp value v into the closed interval [lo, hi]
FUNCTION clamp {
    PARAMETER v.  PARAMETER lo.  PARAMETER hi.
    RETURN MAX(lo, MIN(hi, v)).
}

// Sign of x: returns +1 for x >= 0, -1 for x < 0
FUNCTION sgn {
    PARAMETER x.
    IF x < 0 { RETURN -1. }
    RETURN 1.
}

// Altitude above terrain in metres (AGL)
FUNCTION get_agl {
    RETURN SHIP:ALTITUDE - SHIP:GEOPOSITION:TERRAINHEIGHT.
}

// Local gravitational acceleration at current altitude (m/s²)
FUNCTION get_grav {
    LOCAL r IS SHIP:BODY:RADIUS + SHIP:ALTITUDE.
    RETURN SHIP:BODY:MU / (r * r).
}

// Net upward deceleration the engines can provide (m/s²).
// Positive  → engines can overcome gravity (can hover/decelerate).
// Negative  → thrust-to-weight < 1 (cannot hover on this body).
FUNCTION get_net_decel {
    LOCAL a_thr IS SHIP:AVAILABLETHRUST / SHIP:MASS * 1000.
    RETURN a_thr - get_grav().
}

// Vessel position vector relative to body centre (m)
FUNCTION r_vec {
    RETURN SHIP:POSITION - SHIP:BODY:POSITION.
}

// Orbit angular-momentum unit vector (orbit normal, right-hand rule)
FUNCTION h_hat {
    RETURN VCRS(r_vec(), SHIP:VELOCITY:ORBIT):NORMALIZED.
}

// Radially-outward unit vector at the ship's current position
FUNCTION up_hat {
    RETURN r_vec():NORMALIZED.
}

// Local north unit vector.
// Uses the body's rotation axis (angular velocity) as the north pole.
// Falls back to a cross with V(1,0,0) at the geographic poles.
FUNCTION north_hat {
    LOCAL up  IS up_hat().
    LOCAL bav IS SHIP:BODY:ANGULARVEL.
    IF bav:MAG < 1e-8 { RETURN VCRS(up, V(1,0,0)):NORMALIZED. }
    LOCAL n IS VXCL(up, bav:NORMALIZED).
    IF n:MAG < 0.01   { RETURN VCRS(up, V(1,0,0)):NORMALIZED. }
    RETURN n:NORMALIZED.
}

// Local east unit vector (perpendicular to up and north)
FUNCTION east_hat {
    RETURN VCRS(up_hat(), north_hat()).
}

// Great-circle distance from the ship's current geopos to the target (m)
FUNCTION tgt_dist_m {
    LOCAL a  IS SHIP:GEOPOSITION.
    LOCAL b  IS LATLNG(TGT_LAT, TGT_LNG).
    LOCAL R  IS SHIP:BODY:RADIUS.
    LOCAL sd IS SIN((b:LAT - a:LAT) / 2).
    LOCAL sl IS SIN((b:LNG - a:LNG) / 2).
    LOCAL hv IS sd*sd + COS(a:LAT)*COS(b:LAT)*sl*sl.
    RETURN 2 * ARCSIN(MIN(1, SQRT(hv))) * CONSTANT:DEGTORAD * R.
}

// Horizontal world-space vector from ship to target, projected
// onto the local tangent plane (perpendicular to up_hat).
FUNCTION tgt_horiz_err {
    LOCAL tp IS LATLNG(TGT_LAT, TGT_LNG):ALTITUDEPOSITION(SHIP:ALTITUDE).
    RETURN VXCL(up_hat(), tp - SHIP:POSITION).
}

// Rodrigues rotation formula: rotate vector v by deg degrees
// around unit axis k.  Pure trigonometry, no quaternion needed.
FUNCTION rodrigues {
    PARAMETER v.
    PARAMETER k.
    PARAMETER deg.
    LOCAL ct IS COS(deg).
    LOCAL st IS SIN(deg).
    RETURN v * ct + VCRS(k, v) * st + k * VDOT(k, v) * (1 - ct).
}


// ============================================================
// SECTION D — IMPACT PREDICTION
//
// Uses POSITIONAT() (patched-conic orbit propagation; no thrust)
// to find when the vessel will cross the body's mean-surface
// radius.  Returns ETA in seconds, or -1 if not found.
//
// Body rotation correction: POSITIONAT returns inertial-frame
// positions.  We un-rotate by the body's rotation during the
// flight time so that BODY:GEOPOSITIONOF gives a body-fixed
// landing geoposition.
//
// CAVEAT: POSITIONAT ignores aerodynamic drag.  Use during
//         vacuum coast only; accuracy degrades in atmosphere.
// ============================================================

FUNCTION impact_eta {
    LOCAL R    IS SHIP:BODY:RADIUS.
    LOCAL dt   IS 8.                              // coarse scan step (s)
    LOCAL tmax IS MIN(SHIP:ORBIT:PERIOD, 86400).  // cap at 1 orbit or 1 day
    LOCAL t    IS dt.

    UNTIL t > tmax {
        LOCAL pos IS POSITIONAT(SHIP, TIME:SECONDS + t) - SHIP:BODY:POSITION.
        IF pos:MAG < R + 500 {
            // Narrow down with binary search (24 iterations → < 0.01 s error)
            LOCAL lo IS t - dt.
            LOCAL hi IS t.
            FROM { LOCAL i IS 0. } UNTIL i >= 24 STEP { SET i TO i + 1. } DO {
                LOCAL tm IS (lo + hi) / 2.
                LOCAL pm IS POSITIONAT(SHIP, TIME:SECONDS + tm) - SHIP:BODY:POSITION.
                IF pm:MAG > R { SET lo TO tm. }
                ELSE           { SET hi TO tm. }
            }
            RETURN (lo + hi) / 2.
        }
        SET t TO t + dt.
    }
    RETURN -1.  // no surface crossing found within the search window
}

// Predicted impact geoposition, corrected for body rotation.
// Returns a LATLNG object on the body's surface.
FUNCTION impact_geo {
    LOCAL eta IS impact_eta().
    IF eta < 0 { RETURN SHIP:GEOPOSITION. }  // fallback: nowhere found

    // Position of the vessel at impact time in the inertial frame
    LOCAL p_iner IS POSITIONAT(SHIP, TIME:SECONDS + eta) - SHIP:BODY:POSITION.

    // Un-rotate by the body rotation that will accumulate over eta seconds.
    // Body rotation axis ≈ direction of angular velocity vector.
    LOCAL omega IS 360 / SHIP:BODY:ROTATIONPERIOD.  // deg/s
    LOCAL rot   IS omega * eta.                      // total degrees rotated at impact
    LOCAL ax    IS SHIP:BODY:ANGULARVEL:NORMALIZED.
    LOCAL p_fix IS rodrigues(p_iner, ax, -rot).      // now in body-fixed frame

    // Convert body-fixed position vector to surface geoposition
    RETURN SHIP:BODY:GEOPOSITIONOF(p_fix + SHIP:BODY:POSITION).
}

// Great-circle distance between two LATLNG objects (m)
FUNCTION geo_dist {
    PARAMETER a.
    PARAMETER b.
    LOCAL R  IS SHIP:BODY:RADIUS.
    LOCAL sd IS SIN((b:LAT - a:LAT) / 2).
    LOCAL sl IS SIN((b:LNG - a:LNG) / 2).
    LOCAL hv IS sd*sd + COS(a:LAT)*COS(b:LAT)*sl*sl.
    RETURN 2 * ARCSIN(MIN(1, SQRT(hv))) * CONSTANT:DEGTORAD * R.
}


// ============================================================
// PHASE 1 — ORBITAL PLANE ALIGNMENT
//
// Goal: rotate the orbit plane so the ground track will pass
//       over (or close to) the target lat/lng.
//
// Algorithm:
//   The "desired" orbit plane must contain the body centre and
//   the target surface position vector.  Its normal is:
//       desired_n = normalise( ship_pos × target_pos )
//
//   We burn perpendicular to the prograde direction (normal /
//   anti-normal) at the ascending or descending node of the
//   desired plane, whichever arrives sooner.
//
//   ΔV = 2 · v_orb · sin(Δi / 2),  executed as a finite burn.
//
// Note: this is a first-order correction.  A residual ≤ 1° is
//       acceptable; the coast phase will correct range errors.
// ============================================================

FUNCTION do_plane_align {
    clog("=== PHASE 1: PLANE ALIGN ===").

    // Skip if already on a suborbital arc — no stable orbit to correct.
    IF SHIP:ORBIT:PERIAPSIS < 0 {
        clog("Suborbital trajectory — plane align skipped.").
        RETURN.
    }

    // Desired orbit plane normal
    LOCAL tgt_r  IS LATLNG(TGT_LAT, TGT_LNG):POSITION - SHIP:BODY:POSITION.
    LOCAL des_n  IS VCRS(r_vec(), tgt_r):NORMALIZED.
    LOCAL cur_n  IS h_hat().
    LOCAL err    IS VANG(cur_n, des_n).
    clog("Plane error: " + ROUND(err, 2) + " deg").

    IF err < 0.5 {
        clog("Plane already aligned — skipping.").
        RETURN.
    }

    // ΔV for the plane change at current orbital speed
    LOCAL v_orb IS SHIP:VELOCITY:ORBIT:MAG.
    LOCAL dv    IS 2 * v_orb * SIN(err / 2).
    clog("Plane-change dV: " + ROUND(dv, 1) + " m/s").

    // ── Find the time to the ascending node of the desired plane ───
    // "Latitude" in the desired plane frame: sin(φ) = r_hat · des_n.
    // AN is where φ crosses zero going positive.
    // For a circular orbit the angular travel to AN ≈ -φ (mod 360).
    LOCAL sin_phi IS clamp(VDOT(r_vec():NORMALIZED, des_n), -1, 1).
    LOCAL phi_deg IS ARCSIN(sin_phi).
    LOCAL period  IS SHIP:ORBIT:PERIOD.
    LOCAL to_an   IS MOD(-phi_deg + 360, 360).       // degrees ahead to AN
    LOCAL to_dn   IS MOD(to_an + 180, 360).          // degrees ahead to DN
    LOCAL t_an    IS to_an / 360 * period.
    LOCAL t_dn    IS to_dn / 360 * period.

    LOCAL t_node    IS t_an.
    LOCAL node_name IS "AN".
    IF t_dn < t_an { SET t_node TO t_dn.  SET node_name TO "DN". }
    clog("Node: " + node_name + " in " + ROUND(t_node, 0) + " s").

    // Burn duration (constant-thrust estimate)
    LOCAL a_max IS SHIP:AVAILABLETHRUST / SHIP:MASS * 1000.
    IF a_max < 0.1 { clog("WARNING: no thrust — aborting plane change."). RETURN. }
    LOCAL burn_dur  IS dv / a_max.
    LOCAL t_ign_abs IS TIME:SECONDS + t_node - burn_dur / 2.

    // Time-warp to 60 s before ignition if the node is far away
    IF t_ign_abs - TIME:SECONDS > 90 {
        clog("Warping to burn window...").
        WARPTO(t_ign_abs - 60).
        WAIT UNTIL TIME:SECONDS >= t_ign_abs - 65.
    }

    // ── Compute burn direction ─────────────────────────────────────
    // The ideal plane-change burn is perpendicular to the velocity and
    // aimed at shrinking the angle between cur_n and des_n.
    // The required change in the orbit normal is (des_n − cur_n).
    // We project this onto the plane perpendicular to the prograde
    // direction (VXCL removes the prograde component) to get the
    // actual burn vector — this is the standard plane-change direction.
    FUNCTION plane_steer {
        LOCAL v_hat IS SHIP:VELOCITY:ORBIT:NORMALIZED.
        LOCAL dn    IS h_hat().          // current orbit normal (updated live)
        LOCAL delta IS des_n - dn.       // desired change in orbit normal
        LOCAL bd    IS VXCL(v_hat, delta).  // remove prograde component
        IF bd:MAG < 1e-6 { RETURN LOOKDIRUP(VCRS(up_hat(), v_hat), up_hat()). }
        RETURN LOOKDIRUP(bd:NORMALIZED, up_hat()).
    }

    SAS OFF.
    RCS ON.
    LOCK throttle TO 0.
    LOCK steering  TO plane_steer().

    WAIT UNTIL TIME:SECONDS >= t_ign_abs.
    clog("Plane-change burn START").
    LOCK throttle TO 1.0.

    // Burn until plane error < 0.5°, engines flame out, or timeout.
    // Timeout = 2× the estimated burn duration (generous safety margin).
    LOCAL burn_start IS TIME:SECONDS.
    LOCAL timeout    IS burn_dur * 2 + 10.
    UNTIL VANG(h_hat(), des_n) < 0.5 {
        LOCK steering TO plane_steer().
        IF SHIP:AVAILABLETHRUST <= 0 { BREAK. }
        IF TIME:SECONDS - burn_start > timeout {
            clog("WARNING: plane-change burn timeout — aborting.").
            BREAK.
        }
        WAIT 0.
    }

    LOCK throttle TO 0.
    UNLOCK throttle.
    UNLOCK steering.
    clog("Plane-change done.  Residual: " + ROUND(VANG(h_hat(), des_n), 2) + " deg").
}


// ============================================================
// PHASE 2 — DEORBIT BURN
//
// Lowers periapsis to intercept the target landing zone.
//
// Target periapsis altitude:
//   Atmospheric body → 65 % of atmosphere height (allows
//     aerodynamic braking before the powered phase)
//   Airless body     → 5 km above the surface (direct burn)
//
// Burn-point timing:
//   After the burn the vessel travels half an ellipse to
//   periapsis.  The body rotates eastward by Δlng during that
//   transit.  We therefore burn at:
//       burn_lng = TGT_LNG + 180° + Δlng   (mod 360°)
//   A 3° downrange overshoot is added so the terminal phase
//   always has a forward correction to make (more controllable).
//
// Deorbit ΔV is from the Vis-Viva equation.
// ============================================================

FUNCTION do_deorbit {
    clog("=== PHASE 2: DEORBIT ===").

    LOCAL body   IS SHIP:BODY.
    LOCAL R      IS body:RADIUS.
    LOCAL mu     IS body:MU.
    LOCAL r_cur  IS R + SHIP:ALTITUDE.
    LOCAL v_cur  IS SHIP:VELOCITY:ORBIT:MAG.

    // Choose target periapsis altitude
    LOCAL peri_alt IS 5000.
    IF body:ATM:EXISTS {
        SET peri_alt TO MAX(body:ATM:HEIGHT * 0.65, 15000).
        clog("Atmospheric body — peri target: " + ROUND(peri_alt/1000, 1) + " km").
    } ELSE {
        clog("Airless body — peri target: 5 km AGL").
    }

    LOCAL r_peri IS R + peri_alt.
    LOCAL sma    IS (r_cur + r_peri) / 2.

    // ΔV at current position (Vis-Viva: v_new is speed on deorbit ellipse)
    LOCAL v_new IS SQRT(mu * (2 / r_cur - 1 / sma)).
    LOCAL dv    IS v_cur - v_new.   // positive → retrograde burn needed
    clog("Deorbit dV: " + ROUND(dv, 1) + " m/s").

    // Transit time from burn point to periapsis = half the ellipse period
    LOCAL t_tran IS CONSTANT:PI * SQRT(sma^3 / mu).
    clog("Transit time: " + ROUND(t_tran, 0) + " s").

    // Body rotation during transit (degrees eastward)
    LOCAL d_lng IS 360 * t_tran / body:ROTATIONPERIOD.

    // Target burn longitude (body-fixed)
    // peri ends up at: burn_lng + 180° − d_lng  (mod 360°)
    // So: burn_lng = TGT_LNG + 180° + d_lng   (+3° overshoot)
    LOCAL overshoot IS 3.
    LOCAL burn_lng  IS MOD(TGT_LNG + 180 + d_lng + overshoot + 720, 360).
    clog("Burn longitude: " + ROUND(burn_lng, 2) + " deg").

    // ── Wait for burn window ───────────────────────────────────────
    clog("Waiting for deorbit window...").
    SAS OFF.
    LOCK throttle TO 0.
    LOCK steering  TO RETROGRADE.

    LOCAL in_win IS FALSE.
    UNTIL in_win {
        LOCAL cur_lng  IS MOD(SHIP:GEOPOSITION:LNG + 360, 360).
        // Signed difference: positive = burn_lng is ahead of us
        LOCAL diff IS MOD(burn_lng - cur_lng + 540, 360) - 180.

        IF      ABS(diff) < 1.0  { SET in_win TO TRUE. }
        ELSE IF ABS(diff) > 40   { SET WARP TO 3. }   // 50x warp when far
        ELSE IF ABS(diff) > 10   { SET WARP TO 1. }   // 5x when near
        ELSE                     { SET WARP TO 0. }    // real-time in window
        WAIT 0.
    }
    SET WARP TO 0.
    WAIT UNTIL WARP = 0.

    // ── Execute deorbit burn ───────────────────────────────────────
    // Aim retrograde; wait a moment for the autopilot to settle
    LOCK steering TO RETROGRADE.
    WAIT 5.

    clog("Deorbit burn START").
    LOCAL v_start IS SHIP:VELOCITY:ORBIT:MAG.
    LOCK throttle TO CFG_MAX_THR.

    UNTIL (v_start - SHIP:VELOCITY:ORBIT:MAG) >= dv {
        IF SHIP:AVAILABLETHRUST <= 0 { BREAK. }
        WAIT 0.
    }

    LOCK throttle TO 0.
    UNLOCK throttle.
    clog("Deorbit burn DONE.  Periapsis: " + ROUND(SHIP:ORBIT:PERIAPSIS/1000, 1) + " km").

    // Keep retrograde attitude during the coast that follows
    LOCK steering TO RETROGRADE.
}


// ============================================================
// PHASE 3 — COAST
//
// Free-fall from the deorbit burn to the entry interface (for
// atmospheric bodies) or to the landing-burn arming altitude
// (for airless bodies).
//
// Every ~15 s the script checks the predicted impact geopos
// using impact_geo() and compares it with the target.  If the
// downrange or cross-range error exceeds COAST_CORR_THRESH,
// a small correction burn is executed.
//
// During the coast, time warp (10×) is used to skip dead time.
// Warp is cancelled before each correction burn.
// ============================================================

LOCAL COAST_CORR_THRESH IS 5000.  // m — correct if error exceeds this
LOCAL COAST_CORR_MAXDV  IS 30.    // m/s — cap on each correction burn

FUNCTION do_coast {
    clog("=== PHASE 3: COAST ===").
    LOCAL body IS SHIP:BODY.
    LOCAL done IS FALSE.
    LOCK throttle TO 0.
    LOCK steering  TO RETROGRADE.

    UNTIL done {
        // ── Exit condition ─────────────────────────────────────────
        IF body:ATM:EXISTS {
            IF SHIP:ALTITUDE < body:ATM:HEIGHT {
                SET done TO TRUE.
                clog("Atmosphere interface reached — exiting coast.").
            }
        } ELSE {
            // Airless: exit when approaching landing-burn altitude
            LOCAL arm_alt IS landing_burn_alt().
            IF get_agl() < arm_alt * 1.5 {
                SET done TO TRUE.
                clog("Approaching landing-burn altitude — exiting coast.").
            }
        }

        // ── Impact prediction & correction ─────────────────────────
        IF NOT done {
            LOCAL eta IS impact_eta().
            IF eta < 0 {
                clog("Impact prediction: no surface crossing found yet.").
            } ELSE {
                LOCAL imp  IS impact_geo().
                LOCAL tgt  IS LATLNG(TGT_LAT, TGT_LNG).
                LOCAL dist IS geo_dist(imp, tgt).
                clog("Predicted impact error: " + ROUND(dist/1000, 2) + " km  (ETA " + ROUND(eta, 0) + " s)").

                IF dist > COAST_CORR_THRESH AND SHIP:AVAILABLETHRUST > 0 {
                    coast_correction(imp).
                }
            }
        }

        // Short warp between checks — cancel if already done
        IF NOT done {
            SET WARP TO 2.   // 10× — safe during vacuum coast
            WAIT 15.
            SET WARP TO 0.
        }
        WAIT 0.
    }

    SET WARP TO 0.
    WAIT UNTIL WARP = 0.
    LOCK throttle TO 0.
}

// Small correction burn that shifts the predicted impact toward
// the target.  Applies at most COAST_CORR_MAXDV m/s of ΔV.
FUNCTION coast_correction {
    PARAMETER imp_gp.   // current predicted impact LATLNG

    SET WARP TO 0.
    WAIT UNTIL WARP = 0.
    clog("Coast correction burn...").

    // Convert to body-centred direction vectors
    LOCAL v_imp IS imp_gp:POSITION           - SHIP:BODY:POSITION.
    LOCAL v_tgt IS LATLNG(TGT_LAT, TGT_LNG):POSITION - SHIP:BODY:POSITION.

    // Correction direction: perpendicular to orbit velocity, pointing
    // from the predicted impact toward the desired impact (the target).
    LOCAL imp_hat  IS v_imp:NORMALIZED.
    LOCAL tgt_hat  IS v_tgt:NORMALIZED.
    LOCAL corr_ax  IS VCRS(imp_hat, tgt_hat):NORMALIZED.
    LOCAL corr_dir IS VCRS(corr_ax, SHIP:VELOCITY:ORBIT:NORMALIZED):NORMALIZED.

    // Scale ΔV with angular error, capped at COAST_CORR_MAXDV
    LOCAL ang    IS VANG(imp_hat, tgt_hat).
    LOCAL dv_c   IS MIN(COAST_CORR_MAXDV, ang * 60).
    LOCAL a_max  IS SHIP:AVAILABLETHRUST / SHIP:MASS * 1000.
    IF a_max < 0.1 { clog("No thrust for correction."). RETURN. }
    LOCAL dur IS dv_c / a_max.

    LOCK steering TO LOOKDIRUP(corr_dir, up_hat()).
    WAIT MIN(5, dur * 2).                              // allow orientation

    LOCAL t_end IS TIME:SECONDS + dur.
    LOCK throttle TO 0.15.   // fixed low throttle for precise correction burn
    WAIT UNTIL TIME:SECONDS >= t_end.
    LOCK throttle TO 0.

    clog("Correction done.  Resuming coast.").
    LOCK steering TO RETROGRADE.
}


// ============================================================
// PHASE 4 — ENTRY BURN  (atmospheric bodies only)
//
// On bodies with a significant atmosphere the vessel may enter
// at high speed (> ENTRY_SPD_LIM m/s surface-relative).  This
// phase performs a brief retrograde burn to reduce speed and
// keep aerodynamic heating/load manageable.
//
// If entry speed is already below the limit, the burn is
// skipped and the script simply waits for aerobraking to slow
// the vessel to a safe altitude for the landing burn.
//
// For bodies without an atmosphere this phase is a no-op.
// ============================================================

LOCAL ENTRY_SPD_LIM  IS 2500.  // m/s surface speed above which we burn
LOCAL ENTRY_THR_FRAC IS 0.50.  // throttle fraction for entry burn

FUNCTION do_entry_burn {
    IF NOT SHIP:BODY:ATM:EXISTS {
        clog("=== PHASE 4: ENTRY BURN — skipped (no atmosphere) ===").
        RETURN.
    }

    clog("=== PHASE 4: ENTRY BURN ===").

    IF SHIP:VELOCITY:SURFACE:MAG > ENTRY_SPD_LIM {
        clog("Entry speed " + ROUND(SHIP:VELOCITY:SURFACE:MAG, 0) +
             " m/s — burning to < " + ENTRY_SPD_LIM + " m/s.").
        LOCK steering TO SRFRETROGRADE.
        WAIT 3.
        LOCK throttle TO ENTRY_THR_FRAC.
        UNTIL SHIP:VELOCITY:SURFACE:MAG < ENTRY_SPD_LIM {
            IF SHIP:AVAILABLETHRUST <= 0 { BREAK. }
            WAIT 0.
        }
        LOCK throttle TO 0.
        clog("Entry burn done.  Speed: " + ROUND(SHIP:VELOCITY:SURFACE:MAG, 0) + " m/s").
    } ELSE {
        clog("Entry speed " + ROUND(SHIP:VELOCITY:SURFACE:MAG, 0) +
             " m/s — below limit, skipping burn.").
    }

    // Aerobrake passively until we are low enough for the landing burn.
    // We wait until altitude drops to 10 % of atmosphere height OR we
    // are already within 1.5× the landing-burn arming altitude.
    LOCAL exit_alt IS SHIP:BODY:ATM:HEIGHT * 0.10.
    clog("Aerobraking — waiting for < " + ROUND(exit_alt/1000, 1) + " km altitude.").
    LOCK throttle TO 0.
    LOCK steering  TO SRFRETROGRADE.

    UNTIL SHIP:ALTITUDE < exit_alt OR get_agl() < landing_burn_alt() * 1.5 {
        WAIT 1.
    }
    clog("Entry phase complete.  AGL: " + ROUND(get_agl(), 0) + " m").
}


// ============================================================
// LANDING BURN HELPERS
//
// landing_burn_alt() returns the AGL (m) at which the vessel
// must ignite engines to decelerate from current speed to
// near-zero at the target hover altitude.
//
// Formula (constant-thrust, gravity-loss included):
//   h_burn = v² / ( 2 · a_net )
//   where a_net = (Thrust/mass) − g
//
// A 20 % safety margin is added.  TUNED mode can override this
// with a fixed value via T_BURN_ALT_M.
// ============================================================

FUNCTION landing_burn_alt {
    IF CFG_MODE = "TUNED" AND T_BURN_ALT_M > 0 { RETURN T_BURN_ALT_M. }
    LOCAL v IS SHIP:VELOCITY:SURFACE:MAG.
    LOCAL a IS get_net_decel().
    IF a < 1 { RETURN 5000. }    // fallback for low-TWR vessels
    RETURN (v * v / (2 * a)) * 1.2.   // 20 % margin
}


// ============================================================
// PHASE 5 — LANDING BURN
//
// The main retropropulsive braking burn.  Combines:
//
//   (a) Vertical deceleration
//         Throttle is set to produce a "constant-deceleration"
//         profile targeting v = 0 at CFG_HOVER_M AGL.
//         A PD layer corrects deviations from the ideal profile.
//         TUNED mode replaces the auto-gain with T_THR_KP/KD.
//
//   (b) Lateral guidance (both modes)
//         Horizontal position error and lateral velocity are
//         decomposed into north/south and east/west components.
//         A PID controller outputs a small "lean" for the
//         steering direction: tilting the nose slightly toward
//         the target causes the engines to push the vessel in
//         that direction.
//         Lean magnitude is clamped to ≤ 0.25 (≈ 14°) to keep
//         deceleration the dominant use of thrust.
//
// The phase exits to TERMINAL when the vessel is slow (< 15 m/s)
// and low (< CFG_HOVER_M AGL).
// ============================================================

FUNCTION do_landing_burn {
    clog("=== PHASE 5: LANDING BURN ===").

    // ── Wait for arming altitude ───────────────────────────────────
    clog("Waiting for burn-arm altitude...").
    LOCK throttle TO 0.
    LOCK steering  TO SRFRETROGRADE.
    UNTIL get_agl() < landing_burn_alt() {
        WAIT 0.
    }

    clog("IGNITION at AGL " + ROUND(get_agl(), 0) + " m").
    SAS OFF.
    RCS ON.

    // Reset PID state
    SET _thr_ep TO 0.
    SET _lat_i   TO 0.   SET _lat_ep TO 0.
    SET _lng_i   TO 0.   SET _lng_ep TO 0.
    SET _prev_t  TO TIME:SECONDS.

    // ── Main burn loop ─────────────────────────────────────────────
    LOCAL exit_burn IS FALSE.
    UNTIL exit_burn {
        LOCAL dt_now IS TIME:SECONDS - _prev_t.
        IF dt_now <= 0 { SET dt_now TO 0.02. }
        SET _prev_t TO TIME:SECONDS.

        LOCAL agl_now IS get_agl().
        LOCAL vspd    IS SHIP:VERTICALSPEED.   // m/s, negative = downward
        LOCAL spd     IS SHIP:VELOCITY:SURFACE:MAG.
        LOCAL g_now   IS get_grav().
        LOCAL a_max   IS SHIP:AVAILABLETHRUST / SHIP:MASS * 1000.

        // ── Exit check ─────────────────────────────────────────────
        IF agl_now < CFG_HOVER_M AND spd < 15 {
            LOCK throttle TO 0.
            clog("Transitioning to TERMINAL at AGL " + ROUND(agl_now, 1) + " m").
            SET exit_burn TO TRUE.
            BREAK.
        }
        IF SHIP:STATUS = "LANDED" OR SHIP:STATUS = "SPLASHED" {
            SET exit_burn TO TRUE.
            BREAK.
        }

        // ── Throttle command ───────────────────────────────────────
        // Compute the "ideal suicide burn" target vertical speed once,
        // shared by both AUTO and TUNED branches below.
        LOCAL vspd_tgt IS -SQRT(MAX(0, 2 * get_net_decel() *
                                     MAX(agl_now - CFG_HOVER_M, 0.1))).
        LOCAL verr  IS vspd_tgt - vspd.
        LOCAL dverr IS (verr - _thr_ep) / dt_now.
        SET _thr_ep TO verr.

        LOCAL thr IS CFG_MIN_THR.

        IF CFG_MODE = "AUTO" {
            // "Constant deceleration" feed-forward:
            //   deceleration needed ≈ v² / (2 · remaining_distance)
            LOCAL dist_to_hover IS MAX(agl_now - CFG_HOVER_M, 1).
            LOCAL a_needed IS (spd * spd) / (2 * dist_to_hover) + g_now.
            SET thr TO clamp(a_needed / a_max, CFG_MIN_THR, CFG_MAX_THR).
            // PD correction on top of the feed-forward
            SET thr TO clamp(thr + 0.08*verr + 0.05*dverr,
                             CFG_MIN_THR, CFG_MAX_THR).
        } ELSE {
            // TUNED: explicit PD on vertical-speed error only
            SET thr TO clamp(T_THR_KP * verr + T_THR_KD * dverr,
                             CFG_MIN_THR, CFG_MAX_THR).
        }

        // ── Lateral guidance ───────────────────────────────────────
        // Horizontal error and velocity components in N/E frame
        LOCAL herr   IS tgt_horiz_err().
        LOCAL herr_n IS VDOT(herr, north_hat()).   // N/S error  (m, +N)
        LOCAL herr_e IS VDOT(herr, east_hat()).    // E/W error  (m, +E)
        LOCAL v_h    IS VXCL(up_hat(), SHIP:VELOCITY:SURFACE).
        LOCAL vn     IS VDOT(v_h, north_hat()).    // northward speed (m/s)
        LOCAL ve     IS VDOT(v_h, east_hat()).     // eastward speed  (m/s)

        // Select gains
        LOCAL kp IS 0.   LOCAL kd IS 0.   LOCAL ki IS 0.
        IF CFG_MODE = "AUTO" {
            // Scale with altitude: reduce aggressiveness at high altitude
            LOCAL sc IS clamp(1 - agl_now / 3000, 0.05, 1.0).
            SET kp TO 0.007 * sc.
            SET kd TO 0.040 * sc.
            SET ki TO 0.00008 * sc.
        } ELSE {
            SET kp TO T_LAT_KP.
            SET kd TO T_LAT_KD.
            SET ki TO T_LAT_KI.
        }

        // Integrate position error (with anti-windup clamp)
        SET _lat_i TO clamp(_lat_i + herr_n * dt_now, -500, 500).
        SET _lng_i TO clamp(_lng_i + herr_e * dt_now, -500, 500).

        // PID lean fractions (clamped to ≤ ±0.25 to keep decel dominant)
        LOCAL lean_n IS clamp(kp*herr_n - kd*vn + ki*_lat_i, -0.25, 0.25).
        LOCAL lean_e IS clamp(kp*herr_e - kd*ve + ki*_lng_i, -0.25, 0.25).

        // Blend: start from surface-retrograde direction, add lateral lean.
        // Tilting the nose toward the target causes the bottom engines to
        // produce a thrust component in that direction.
        LOCAL base_dir IS -SHIP:VELOCITY:SURFACE:NORMALIZED.
        LOCAL steer_v  IS (base_dir
                         + north_hat() * lean_n
                         + east_hat()  * lean_e):NORMALIZED.

        LOCK steering TO LOOKDIRUP(steer_v, up_hat()).
        LOCK throttle TO thr.

        // ── Console HUD ────────────────────────────────────────────
        PRINT "=== LANDING BURN ===" AT(0, 5).
        PRINT "AGL     : " + ROUND(agl_now,     1) + " m   " AT(0, 6).
        PRINT "Speed   : " + ROUND(spd,          1) + " m/s " AT(0, 7).
        PRINT "V-speed : " + ROUND(vspd,         2) + " m/s " AT(0, 8).
        PRINT "H-err   : " + ROUND(herr:MAG,     1) + " m   " AT(0, 9).
        PRINT "Throttle: " + ROUND(thr * 100,    1) + " %   " AT(0, 10).

        WAIT 0.
    }

    LOCK throttle TO 0.
    UNLOCK throttle.
    clog("Landing burn complete.  AGL: " + ROUND(get_agl(), 1) +
         " m  |  speed: " + ROUND(SHIP:VELOCITY:SURFACE:MAG, 1) + " m/s").
}


// ============================================================
// PHASE 6 — TERMINAL CORRECTION & TOUCHDOWN
//
// Fine-guides the vessel to the exact target coordinates from
// the hover altitude (CFG_HOVER_M AGL).
//
// Sequence:
//   (a) Null all horizontal velocity, holding altitude at
//       CFG_HOVER_M AGL with a throttle PD loop.
//   (b) Translate slowly toward the target while maintaining
//       altitude (velocity capped at TERM_XLATE_V m/s).
//   (c) Once over the target within TERM_OVER_THRESH metres,
//       begin the final vertical descent at T_VTERM m/s.
//   (d) Shut down on SHIP:STATUS = "LANDED" or speed < CFG_TOUCH_V
//       below 1 m AGL.
//
// This phase uses proportional–derivative control because the
// manoeuvres are small, slow, and require gentle damping.
// ============================================================

LOCAL TERM_OVER_THRESH IS 3.     // m — "close enough" to begin descent
LOCAL TERM_XLATE_V     IS 2.5.   // m/s — max lateral translation speed

FUNCTION do_terminal {
    clog("=== PHASE 6: TERMINAL ===").
    SAS OFF.
    RCS ON.
    SET _thr_ep TO 0.
    SET _prev_t TO TIME:SECONDS.

    UNTIL SHIP:STATUS = "LANDED" OR SHIP:STATUS = "SPLASHED" {

        LOCAL dt_now IS TIME:SECONDS - _prev_t.
        IF dt_now <= 0 { SET dt_now TO 0.02. }
        SET _prev_t TO TIME:SECONDS.

        LOCAL agl_now   IS get_agl().
        LOCAL vspd      IS SHIP:VERTICALSPEED.
        LOCAL v_h       IS VXCL(up_hat(), SHIP:VELOCITY:SURFACE).
        LOCAL horiz_spd IS v_h:MAG.
        LOCAL herr      IS tgt_horiz_err().
        LOCAL hd        IS herr:MAG.        // horizontal distance to target (m)
        LOCAL g_now     IS get_grav().
        LOCAL a_max     IS SHIP:AVAILABLETHRUST / SHIP:MASS * 1000.

        // ── Vertical speed target ──────────────────────────────────
        LOCAL vspd_tgt IS 0.
        IF hd < TERM_OVER_THRESH AND horiz_spd < 0.5 {
            // Over the target — begin final descent
            SET vspd_tgt TO T_VTERM.
        } ELSE {
            // Hold altitude: small vertical corrections to stay near hover alt
            LOCAL alt_err IS CFG_HOVER_M - agl_now.
            SET vspd_tgt TO clamp(alt_err * 0.5, -2.0, 2.0).
        }

        // ── Throttle PD for altitude hold ─────────────────────────
        LOCAL verr  IS vspd_tgt - vspd.
        LOCAL dverr IS (verr - _thr_ep) / dt_now.
        // Feed-forward: exact thrust to cancel gravity
        LOCAL thr_ff IS g_now / a_max.
        LOCAL thr    IS clamp(thr_ff + 0.10*verr + 0.05*dverr,
                              CFG_MIN_THR, CFG_MAX_THR).
        SET _thr_ep TO verr.

        // ── Lateral guidance ───────────────────────────────────────
        // Desired horizontal velocity: proportional to distance, capped
        LOCAL hd_v IS V(0,0,0).
        IF hd > 0.1 {
            SET hd_v TO herr:NORMALIZED * MIN(hd * 0.30, TERM_XLATE_V).
        }
        LOCAL lat_err_v IS hd_v - v_h.       // velocity we need to add

        // Gain selection
        LOCAL kp_t IS 0.04.
        LOCAL kd_t IS 0.12.
        IF CFG_MODE = "TUNED" {
            SET kp_t TO T_LAT_KP * 5.
            SET kd_t TO T_LAT_KD * 2.5.
        }

        // Lean vector: tilt the nose toward the correction direction
        LOCAL lean    IS lat_err_v * kp_t - v_h * kd_t.
        LOCAL lean_mg IS lean:MAG.
        LOCAL lean_cl IS V(0,0,0).
        IF lean_mg > 0.001 {
            SET lean_cl TO lean * (MIN(lean_mg, 0.25) / lean_mg).
        }

        // Steer: mostly up_hat with a small lean
        LOCAL steer IS (up_hat() + lean_cl):NORMALIZED.
        LOCK steering TO LOOKDIRUP(steer, up_hat()).
        LOCK throttle TO thr.

        // ── Console HUD ────────────────────────────────────────────
        PRINT "=== TERMINAL GUIDANCE ===" AT(0, 5).
        PRINT "AGL       : " + ROUND(agl_now,   1) + " m    " AT(0, 6).
        PRINT "H-dist    : " + ROUND(hd,         1) + " m    " AT(0, 7).
        PRINT "V-speed   : " + ROUND(vspd,       2) + " m/s  " AT(0, 8).
        PRINT "H-speed   : " + ROUND(horiz_spd,  2) + " m/s  " AT(0, 9).

        // ── Touchdown detection ────────────────────────────────────
        IF agl_now < 1.0 AND ABS(vspd) < CFG_TOUCH_V {
            clog("TOUCHDOWN.").
            BREAK.
        }

        WAIT 0.
    }

    // ── Shutdown ───────────────────────────────────────────────────
    LOCK throttle TO 0.
    UNLOCK throttle.
    UNLOCK steering.
    SAS ON.
    GEAR ON.

    LOCAL miss IS tgt_dist_m().
    clog("=== LANDING COMPLETE ===").
    clog("Miss distance : " + ROUND(miss, 1) + " m").
    clog("Final position: " + ROUND(SHIP:GEOPOSITION:LAT, 5) + " N  " +
                              ROUND(SHIP:GEOPOSITION:LNG, 5) + " E").
}


// ============================================================
// MAIN EXECUTION SEQUENCE
// ============================================================

clog("====================================================").
clog("  autoland_orbit_v3_generic  v3.0  STARTING").
clog("  Mode    : " + CFG_MODE).
clog("  Target  : " + TGT_LAT + " N  " + TGT_LNG + " E").
clog("  Body    : " + SHIP:BODY:NAME).
clog("  Orbit   : " + ROUND(SHIP:ORBIT:PERIAPSIS/1000, 1) + " km  x  " +
                      ROUND(SHIP:ORBIT:APOAPSIS/1000,  1) + " km").
clog("====================================================").

// Sanity checks
IF SHIP:AVAILABLETHRUST < 1 {
    clog("WARNING: no engine thrust detected.  Check engine staging/activation.").
}
IF SHIP:ORBIT:PERIAPSIS < 0 {
    clog("NOTE: already on suborbital trajectory — skipping deorbit phase.").
}

// Initial state
LOCK throttle TO 0.
SAS OFF.
WAIT 0.5.

// ── Phase sequence ─────────────────────────────────────────────
do_plane_align().

// Skip deorbit if already suborbital
IF SHIP:ORBIT:PERIAPSIS > 0 {
    do_deorbit().
}

do_coast().
do_entry_burn().
do_landing_burn().
do_terminal().

clog("Script finished.").
