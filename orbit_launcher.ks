// orbit_launcher.ks
// A fuel-efficient kOS script to launch to a circular orbit with a specific inclination.

// --- CONFIGURATION / INPUTS ---
// You can modify these or pass them as arguments.
SET target_altitude TO 100.
SET target_inclination TO 0.
SET pitch_over_alt TO 1000.
SET gravity_turn_alt TO 5000.

// --- VARIABLES ---
SET vessel TO SHIP.
SET target_orbit TO ORBIT(target_inclination, target_altitude, target_altitude, 0, 0, 0, vessel:body:ref).

PRINT "Starting Launch Sequence...".

// --- PHASE 1: ASCENT ---

PRINT "Phase 1: Vertical Ascent...".
LOCK CONTROL TO HEADING(90, 90).
LOCK THROTTLE TO 1.

// Wait until pitch over altitude
UNTIL vessel:altitude GT pitch_over_alt {
    WAIT 0.1.
}

PRINT "Phase 2: Pitch Over...".
// Start a gradual gravity turn
SET start_pitch TO 90.
SET end_pitch TO 10. // Target pitch for ascent
SET start_alt TO pitch_over_alt.
SET end_alt TO gravity_turn_alt.

UNTIL vessel:altitude GT gravity_turn_alt {
    SET current_alt TO vessel:altitude.
    SET progress TO (current_alt - start_alt) / (end_alt - start_alt).
    IF progress < 0 { SET progress TO 0. }
    IF progress > 1 { SET progress TO 1. }
    
    SET target_pitch TO start_pitch - (progress * (start_pitch - end_pitch)).
    
    // Use HEADING to control pitch and keep heading towards prograde-ish
    // For a simple script, we'll just adjust the pitch of the heading
    LOCK CONTROL TO HEADING(0, target_pitch).
    
    WAIT 0.1.
}

PRINT "Phase 3: Sustained Ascent...".
// Continue ascent until apoapsis is near target
UNTIL vessel:apoapsis GT (target_altitude - 500) {
    // Maintain a low pitch for the gravity turn effect
    SET current_pitch TO target_pitch. // From previous loop
    LOCK CONTROL TO HEADING(0, current_pitch).
    
    // Automatic staging
    // Note: In a real scenario, we might want to stage based on fuel or stage number
    // Here we just check if we should stage if we have multiple stages
    IF vessel:stages > 0 {
        STAGE.
        PRINT "Staging...".
        WAIT 2. // Wait for staging animation/separation
    }
    
    IF vessel:available:liquid_fuel < 0.1 {
        PRINT "Warning: Low fuel!".
        BREAK.
    }
    
    WAIT 0.5.
}

// --- PHASE 2: COAST ---

PRINT "Phase 3: Coasting to Apoapsis...".
LOCK THROTTLE TO 0.
// Release control lock to allow other maneuvers or just stay steady
UNLOCK CONTROL.

UNTIL vessel:altitude GT (vessel:apoapsis - 500) {
    WAIT 0.5.
}

// --- PHASE 3: CIRCULARIZATION ---

PRINT "Phase 4: Circularization at Apoapsis...".
// Wait until we are very close to apoapsis
WAIT UNTIL vessel:altitude GT (vessel:apoapsis - 100).

// Lock to prograde for circularization
LOCK CONTROL TO PROGRADE.
LOCK THROTTLE TO 1.

// Burn until periapsis reaches target altitude
UNTIL vessel:periapsis GT (target_altitude - 5) {
    IF vessel:available:liquid_fuel < 0.1 {
        PRINT "Error: Out of fuel during circularization!".
        BREAK.
    }
    WAIT 0.5.
}

LOCK THROTTLE TO 0.
UNLOCK CONTROL.
PRINT "Circularization Complete.".

// --- PHASE 4: INCLINATION CORRECTION ---

PRINT "Phase 5: Inclination Correction (if needed)...".
// This is best done at the node (ascending/descending node)
// For simplicity in this script, we check if inclination is close enough
IF ABS(vessel:inclination - target_inclination) > 0.5 {
    PRINT "Inclination mismatch detected. Manual adjustment recommended or complex node burn required.".
    PRINT "Current Inclination: " + vessel:inclination.
} ELSE {
    PRINT "Inclination is within acceptable range.".
}

PRINT "Launch Mission Complete.".