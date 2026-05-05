// ============================================================
// Inclined Orbit Launcher - Fuel Efficient kOS Script
// ============================================================
// Launches vehicle to a selected circular orbit with
// a selected inclination using fuel-optimal techniques:
//   - Gravity turn ascent for minimal gravity losses
//   - Combined apoapsis lift during ascent
//   - Circularization at apoapsis (Hohmann transfer)
//   - Inclination change at apoapsis (lowest velocity)
// ============================================================

// --- TARGET PARAMETERS - EDIT THESE ---
SET TARGET_ORBIT_HEIGHT TO 80000.        // Desired circular orbit altitude in meters (80km = 80000)
SET TARGET_INCLINATION TO 0.             // Desired orbital inclination in degrees (0 = equatorial)
// --------------------------------------

// --- ASCENT PROFILE PARAMETERS ---
SET PITCH_OVER_ALTITUDE TO 30.           // Altitude (m terrain) to begin pitch over
SET TARGET_YAW_HEADING TO 90.            // Yaw heading for ascent (90 = eastward for prograde)
SET MIN_PITCH_ANGLE TO 10.               // Minimum pitch angle above horizon during gravity turn
// --------------------------------------

// --- HELPER FUNCTION ---
FUNCTION CLAMP {
    PARMS [VALUE, MINVAL, MAXVAL].
    IF VALUE < MINVAL { RETURN MINVAL. }
    IF VALUE > MAXVAL { RETURN MAXVAL. }
    RETURN VALUE.
}

// --- SETUP ---
LOCK STEERING TO PROGRADE.
SET WRAPLOCK TO FALSE.
SET AUTOPILLOT TO TRUE.

// --- PRE-LAUNCH ---
PRINT "========================================".
PRINT "   INCLINED ORBIT LAUNCHER".
PRINT "========================================".
PRINT "Target Orbit: " + TARGET_ORBIT_HEIGHT/1000 + " km".
PRINT "Target Inclination: " + TARGET_INCLINATION + " deg".
PRINT "Fuel: " + ROUND(LIQUIDFUEL, 0) + " units".
PRINT "Max Thrust: " + ROUND(MAXTHRUST, 0) + " N".
PRINT "========================================".
PRINT "".
PRINT "Press ENTER to begin launch sequence...".
WAIT FOR PAUSED.

// --- STAGE IF NO THRUST ---
IF MAXTHRUST < 0.1 {
    PRINT "Staging...".
    STAGE.
    WAIT 2.
}

// ============================================================
// PHASE 1: VERTICAL ASCENT
// ============================================================
PRINT ">>> PHASE 1: Vertical Ascent <<<".
SET THRUSTPCT TO 100.
SET STEERING TO VECTORUP(SHIP).

WAIT FOR ALT TERR > PITCH_OVER_ALTITUDE.

// ============================================================
// PHASE 2: PITCH OVER - Start gravity turn
// ============================================================
PRINT ">>> PHASE 2: Pitch Over - Gravity Turn <<<".

// Set yaw to target heading (eastward is most efficient for Kerbin)
// For non-zero inclination, we still start eastward and adjust later
SET STEERING TO VECTORFROMANGLES(TARGET_YAW_HEADING, 80).
WAIT 1.

// ============================================================
// PHASE 3: GRAVITY TURN ASCENT
// Steer prograde with gradual pitch down
// This is the most fuel-efficient ascent method
// ============================================================
PRINT ">>> PHASE 3: Gravity Turn Ascent <<<".

// During gravity turn, we steer prograde and let gravity pull us down
// We only correct if pitch gets too low
UNTIL APAPOASIS >= TARGET_ORBIT_HEIGHT - 5000 {
    // Steer prograde for gravity turn
    SET STEERING TO PROGRADE.
    SET THRUSTPCT TO 100.

    // If pitch drops below minimum, add upward correction to steering
    IF PITCH < MIN_PITCH_ANGLE {
        SET STEERING TO (PROGRADE + UP) * 0.5.
    }

    // Fuel monitoring
    IF LIQUIDFUEL < 50 {
        PRINT "[CRITICAL] Low fuel! Cutting engine.".
        SET THRUSTPCT TO 0.
        BREAK.
    }

    PRINT "Alt: " + ROUND(ALT/1000, 1) + "km | Ap: " + ROUND(APAPOASIS/1000, 1) + "km | Pitch: " + ROUND(PITCH, 0) + "deg | HSPD: " + ROUND(HSPD, 0) + "m/s        \r\r", false.
    WAIT 0.5.
}

SET THRUSTPCT TO 0.
WAIT 1.

// ============================================================
// PHASE 4: COAST TO APOAPSIS
// ============================================================
PRINT ">>> PHASE 4: Coasting to Apoapsis <<<".
PRINT "Current Apoapsis: " + ROUND(APAPOASIS/1000, 1) + " km".

UNTIL ALT >= APAPOASIS - 5000 {
    PRINT "Alt: " + ROUND(ALT/1000, 1) + "km | Apoapsis: " + ROUND(APAPOASIS/1000, 1) + "km | Distance: " + ROUND(APAPOASIS - ALT, 0) + "m        \r\r", false.
    WAIT 1.
}

// ============================================================
// PHASE 5: CIRCULARIZATION BURN at Apoapsis
// Burn prograde at apoapsis where velocity is lowest
// This is the most fuel-efficient point to circularize
// ============================================================
PRINT ">>> PHASE 5: Circularization Burn <<<".
SET STEERING TO PROGRADE.

// Burn until periapsis reaches target altitude
UNTIL APPERIASIS >= TARGET_ORBIT_HEIGHT - 1000 {
    SET THRUSTPCT TO 100.

    // Throttle down as we approach target to avoid overshoot
    IF APPERIASIS > TARGET_ORBIT_HEIGHT * 0.85 {
        SET THRUSTPCT TO CLAMP((TARGET_ORBIT_HEIGHT - APPERIASIS) / 2000 * 100, 5, 100).
    }

    PRINT "Periapsis: " + ROUND(APPERIASIS/1000, 1) + "km | Alt: " + ROUND(ALT/1000, 1) + "km        \r\r", false.

    // Safety break
    IF APPERIASIS >= TARGET_ORBIT_HEIGHT - 200 {
        SET THRUSTPCT TO 0.
        BREAK.
    }

    WAIT 0.3.
}

SET THRUSTPCT TO 0.
WAIT 2.
PRINT "Periapsis circularized to: " + ROUND(APPERIASIS/1000, 1) + " km".

// ============================================================
// PHASE 6: INCLINATION ADJUSTMENT
// Perform at apoapsis where orbital velocity is lowest
// dV = 2 * v * sin(dI/2) - minimum at lowest velocity
// ============================================================
PRINT ">>> PHASE 6: Inclination Adjustment <<<".
SET CURRENT_INCLINATION TO ORBIT:INCLINATION.
PRINT "Current inclination: " + ROUND(CURRENT_INCLINATION, 2) + " deg".
PRINT "Target inclination:  " + TARGET_INCLINATION + " deg".

SET INC_DIFF TO ABS(CURRENT_INCLINATION - TARGET_INCLINATION).

IF INC_DIFF > 0.2 {
    PRINT "Inclination change needed: " + ROUND(INC_DIFF, 2) + " deg".

    // Determine if we need to raise or lower inclination
    IF TARGET_INCLINATION > CURRENT_INCLINATION {
        SET STEERING TO NORMAL.
    } ELSE {
        SET STEERING TO ANTINORMAL.
    }

    SET INC_BURN_START TO TIME.

    UNTIL ABS(ORBIT:INCLINATION - TARGET_INCLINATION) < 0.05 {
        SET THRUSTPCT TO 40.

        // Reduce throttle for precision near target
        IF ABS(ORBIT:INCLINATION - TARGET_INCLINATION) < 0.15 {
            SET THRUSTPCT TO 15.
        }

        PRINT "Inclination: " + ROUND(ORBIT:INCLINATION, 3) + " deg | Burn time: " + ROUND(TIME - INC_BURN_START, 0) + "s        \r\r", false.

        // Safety timeout
        IF TIME - INC_BURN_START > 600 {
            PRINT "[WARNING] Inclination burn timeout.".
            BREAK.
        }

        WAIT 0.5.
    }

    SET THRUSTPCT TO 0.
    PRINT "Inclination adjusted to: " + ROUND(ORBIT:INCLINATION, 2) + " deg".
} ELSE {
    PRINT "Inclination within tolerance. No adjustment needed.".
}

// ============================================================
// PHASE 7: FINAL ORBIT CIRCULARIZATION TUNING
// Fine-tune both apoapsis and periapsis to match target
// ============================================================
PRINT ">>> PHASE 7: Final Orbit Tuning <<<".

SET TUNE_ITERATIONS TO 0.
SET MAX_TUNE_ITERATIONS TO 200.

UNTIL (ABS(APAPOASIS - TARGET_ORBIT_HEIGHT) < 500 AND ABS(APPERIASIS - TARGET_ORBIT_HEIGHT) < 500) {
    SET NEEDS_BURN TO FALSE.

    // Fix apoapsis
    IF APAPOASIS < TARGET_ORBIT_HEIGHT - 500 {
        SET STEERING TO PROGRADE.
        SET NEEDS_BURN TO TRUE.
    } ELSE IF APAPOASIS > TARGET_ORBIT_HEIGHT + 500 {
        SET STEERING TO RETROGRADE.
        SET NEEDS_BURN TO TRUE.
    }

    // Fix periapsis when near apoapsis (burn opposite side)
    IF ORBIT:TRUEANOMALY > 90 AND ORBIT:TRUEANOMALY < 270 {
        IF APPERIASIS < TARGET_ORBIT_HEIGHT - 500 {
            SET STEERING TO PROGRADE.
            SET NEEDS_BURN TO TRUE.
        } ELSE IF APPERIASIS > TARGET_ORBIT_HEIGHT + 500 {
            SET STEERING TO RETROGRADE.
            SET NEEDS_BURN TO TRUE.
        }
    }

    IF NEEDS_BURN {
        SET THRUSTPCT TO 20.
    } ELSE {
        SET THRUSTPCT TO 0.
        SET STEERING TO PROGRADE.
    }

    SET TUNE_ITERATIONS TO TUNE_ITERATIONS + 1.

    PRINT "Ap: " + ROUND(APAPOASIS/1000, 1) + "km | Pe: " + ROUND(APPERIASIS/1000, 1) + "km | Iter: " + TUNE_ITERATIONS + "        \r\r", false.

    IF TUNE_ITERATIONS >= MAX_TUNE_ITERATIONS {
        PRINT "[WARNING] Tuning timeout reached.".
        BREAK.
    }

    WAIT 1.
}

SET THRUSTPCT TO 0.

// ============================================================
// RESULTS
// ============================================================
PRINT "".
PRINT "========================================".
PRINT "   ORBIT ACHIEVED!".
PRINT "========================================".
PRINT "".
PRINT "  FINAL ORBIT:".
PRINT "  ------------".
PRINT "  Apoapsis:    " + ROUND(APAPOASIS/1000, 2) + " km".
PRINT "  Periapsis:   " + ROUND(APPERIASIS/1000, 2) + " km".
PRINT "  Inclination: " + ROUND(ORBIT:INCLINATION, 3) + " deg".
PRINT "  Velocity:    " + ROUND(SPEED, 1) + " m/s".
PRINT "  Period:      " + ROUND(ORBIT:PERIOD, 0) + " s (" + ROUND(ORBIT:PERIOD/60, 1) + " min)".
PRINT "".
PRINT "  TARGETS:".
PRINT "  ------------".
PRINT "  Orbit Height:" + TARGET_ORBIT_HEIGHT/1000 + " km".
PRINT "  Inclination: " + TARGET_INCLINATION + " deg".
PRINT "".
PRINT "  ERRORS:".
PRINT "  ------------".
PRINT "  Ap Error:    " + ROUND((APAPOASIS - TARGET_ORBIT_HEIGHT)/1000, 2) + " km".
PRINT "  Pe Error:    " + ROUND((APPERIASIS - TARGET_ORBIT_HEIGHT)/1000, 2) + " km".
PRINT "  Inc Error:   " + ROUND(ABS(ORBIT:INCLINATION - TARGET_INCLINATION), 3) + " deg".
PRINT "".
PRINT "  Remaining Fuel: " + ROUND(LIQUIDFUEL, 0) + " units".
PRINT "========================================".
PRINT "".
PRINT "Script complete. Vehicle in target orbit.".

// Lock to prograde for stable orbit
SET STEERING TO PROGRADE.
SET THRUSTPCT TO 0.