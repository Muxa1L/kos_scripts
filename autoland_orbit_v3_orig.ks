// ============================================================
// autoland_orbit_v3_generic.ks
//
// Generic orbital-to-target propulsive landing autopilot for kOS.
// Inspired by SpaceX-style landing phases:
//
//   1) plane alignment to target latitude/longitude
//   2) boostback / deorbit correction
//   3) coast with repeated impact prediction
//   4) entry energy management
//   5) landing burn
//   6) terminal touchdown correction
//
// Goals:
//   - predefined target coordinates
//   - generic across celestial bodies
//   - no mandatory tuning in AUTO mode
//   - optional TUNED mode
//
// Limits:
//   - still heuristic
//   - atmospheric prediction is approximate
//   - sub-meter precision everywhere is not guaranteed
// ============================================================

CLEARSCREEN.
PRINT "autoland_orbit_v3_generic.ks booting...".

// ------------------------------------------------------------
// USER CONFIG
// ------------------------------------------------------------

SET MODE TO "AUTO".
// SET MODE TO "TUNED".

SET TARGET_LAT TO 0.000000.
SET TARGET_LNG TO -74.500000.
SET TARGET_HEADING TO 90.

SET ENABLE_PLANE_ALIGN TO TRUE.
SET ENABLE_BOOSTBACK   TO TRUE.
SET ENABLE_ENTRY_BURN  TO TRUE.
SET ENABLE_RCS_FINAL   TO TRUE.

SET MIN_REQUIRED_TWR TO 1.05.
SET ABORT_IF_LOW_TWR TO TRUE.
SET GEAR_DEPLOY_ALT TO 250.
SET HARD_GEAR_ALT TO 80.

SET GOAL_TOUCHDOWN_VS TO -0.5.
SET GOAL_TOUCHDOWN_HS TO 0.6.

SET MAX_THR TO 1.0.
SET MIN_THR TO 0.0.

SET PREDICT_STEP_VAC TO 0.35.
SET PREDICT_STEP_ATM TO 0.20.
SET PREDICT_MAX_TIME TO 5000.

SET T_PLANE_KP TO 0.010.
SET T_PLANE_KD TO 0.024.

SET T_BOOST_CROSS_KP TO 0.00020.
SET T_BOOST_CROSS_KD TO 0.00030.
SET T_BOOST_DOWN_KP  TO 0.00014.
SET T_BOOST_DOWN_KD  TO 0.00022.

SET T_ENTRY_CROSS_KP TO 0.00028.
SET T_ENTRY_CROSS_KD TO 0.00036.
SET T_ENTRY_DOWN_KP  TO 0.00018.
SET T_ENTRY_DOWN_KD  TO 0.00024.

SET T_FINAL_LAT_KP TO 0.014.
SET T_FINAL_LAT_KD TO 0.028.

SET T_VS_KP TO 0.12.
SET T_VS_KD TO 0.21.

SET T_ENTRY_MARGIN TO 1.10.
SET T_LANDING_MARGIN TO 1.06.
SET T_SUICIDE_PAD TO 10.
SET T_UP_BIAS_FINAL TO 2.2.
SET T_UP_BIAS_ENTRY TO 0.25.

// ------------------------------------------------------------
// GLOBALS
// ------------------------------------------------------------

LOCK THROTTLE TO 0.
SAS OFF.
RCS OFF.

SET BODYREF TO BODY.
SET MU TO BODYREF:MU.
SET RADIUS TO BODYREF:RADIUS.
SET HASATM TO BODYREF:HASATM.
SET ROTPERIOD TO BODYREF:ROTATIONPERIOD.
SET TARGET TO LATLNG(TARGET_LAT, TARGET_LNG).

SET DT TO 0.05.
SET PHASE TO "INIT".

SET prevPlaneErr TO 0.
SET prevCrossErr TO 0.
SET prevDownErr TO 0.
SET prevVsErr TO 0.
SET prevFinalCrossErr TO 0.

// ------------------------------------------------------------
// UTILS
// ------------------------------------------------------------

FUNCTION clamp {
    PARAMETER x, lo, hi.
    IF x < lo { RETURN lo. }
    IF x > hi { RETURN hi. }
    RETURN x.
}

FUNCTION safeDiv {
    PARAMETER a, b, fallback.
    IF ABS(b) < 0.000001 { RETURN fallback. }
    RETURN a / b.
}

FUNCTION angleWrap {
    PARAMETER deg.
    LOCAL a IS deg.
    UNTIL a <= 180 { SET a TO a - 360. }.
    UNTIL a >= -180 { SET a TO a + 360. }.
    RETURN a.
}

FUNCTION sqr {
    PARAMETER x.
    RETURN x*x.
}

FUNCTION gravAtAlt {
    PARAMETER alt.
    RETURN MU / ((RADIUS + alt)^2).
}

FUNCTION maxAccel {
    RETURN MAX(0.001, SHIP:AVAILABLETHRUST / SHIP:MASS).
}

FUNCTION maxUpDecel {
    RETURN MAX(0.001, maxAccel() - gravAtAlt(SHIP:ALTITUDE)).
}

FUNCTION twrNow {
    RETURN safeDiv(SHIP:AVAILABLETHRUST, SHIP:MASS * gravAtAlt(SHIP:ALTITUDE), 0).
}

FUNCTION stopDist {
    PARAMETER v, decel.
    RETURN safeDiv(v*v, 2*MAX(0.001, decel), 1e9).
}

FUNCTION bodyRotRate {
    RETURN 2 * CONSTANT():PI / ROTPERIOD.
}

FUNCTION upVec {
    RETURN SHIP:BODY:POSITION:VECTORFROM(SHIP:POSITION):NORMALIZED * -1.
}

FUNCTION surfaceVel {
    RETURN SHIP:VELOCITY:SURFACE.
}

FUNCTION orbitalVel {
    RETURN SHIP:VELOCITY:ORBIT.
}

FUNCTION horizVel {
    LOCAL up IS upVec().
    LOCAL v IS surfaceVel().
    RETURN v - up * VDOT(v, up).
}

FUNCTION horizSpeed {
    RETURN horizVel():MAG.
}

FUNCTION verticalSpeed {
    RETURN SHIP:VERTICALSPEED.
}

FUNCTION totalSpeed {
    RETURN surfaceVel():MAG.
}

FUNCTION radarAlt {
    RETURN MAX(0, SHIP:ALT:RADAR).
}

FUNCTION targetDist {
    RETURN SHIP:GEOPOSITION:DISTANCE(TARGET).
}

FUNCTION targetBearing {
    RETURN SHIP:GEOPOSITION:BEARINGTO(TARGET).
}

FUNCTION localUpAtGeo {
    PARAMETER geo.
    RETURN (geo:POSITION - BODY:POSITION):NORMALIZED.
}

FUNCTION eastVecAt {
    PARAMETER geo.
    LOCAL up IS localUpAtGeo(geo).
    LOCAL ref IS V(0,1,0).
    LOCAL east IS VCRS(ref, up).
    IF east:MAG < 0.0001 {
        SET east TO V(1,0,0).
    } ELSE {
        SET east TO east:NORMALIZED.
    }
    RETURN east.
}

FUNCTION northVecAt {
    PARAMETER geo.
    LOCAL up IS localUpAtGeo(geo).
    LOCAL east IS eastVecAt(geo).
    RETURN VCRS(up, east):NORMALIZED.
}

FUNCTION vectorFromBearingAt {
    PARAMETER geo, bearingDeg.
    LOCAL east IS eastVecAt(geo).
    LOCAL north IS northVecAt(geo).
    RETURN north * COS(bearingDeg * CONSTANT():DEGTORAD) + east * SIN(bearingDeg * CONSTANT():DEGTORAD).
}

FUNCTION lookAtVec {
    PARAMETER dir.
    IF dir:MAG < 0.0001 { RETURN. }
    LOCK STEERING TO LOOKDIRUP(dir:NORMALIZED, SHIP:FACING:TOPVECTOR).
}

FUNCTION bodyAtmosHeight {
    IF HASATM { RETURN BODY:ATM:HEIGHT. }
    RETURN 0.
}

// ------------------------------------------------------------
// MODE-DEPENDENT CONSTANTS
// ------------------------------------------------------------

FUNCTION planeKP {
    IF MODE = "TUNED" { RETURN T_PLANE_KP. }
    RETURN 0.006 + clamp(SHIP:ALTITUDE / 700000, 0, 0.018).
}

FUNCTION planeKD {
    IF MODE = "TUNED" { RETURN T_PLANE_KD. }
    RETURN planeKP() * 2.0.
}

FUNCTION boostCrossKP {
    IF MODE = "TUNED" { RETURN T_BOOST_CROSS_KP. }
    RETURN 0.00015.
}

FUNCTION boostCrossKD {
    IF MODE = "TUNED" { RETURN T_BOOST_CROSS_KD. }
    RETURN 0.00022.
}

FUNCTION boostDownKP {
    IF MODE = "TUNED" { RETURN T_BOOST_DOWN_KP. }
    RETURN 0.00011.
}

FUNCTION boostDownKD {
    IF MODE = "TUNED" { RETURN T_BOOST_DOWN_KD. }
    RETURN 0.00017.
}

FUNCTION entryCrossKP {
    IF MODE = "TUNED" { RETURN T_ENTRY_CROSS_KP. }
    RETURN 0.00020.
}

FUNCTION entryCrossKD {
    IF MODE = "TUNED" { RETURN T_ENTRY_CROSS_KD. }
    RETURN 0.00028.
}

FUNCTION entryDownKP {
    IF MODE = "TUNED" { RETURN T_ENTRY_DOWN_KP. }
    RETURN 0.00014.
}

FUNCTION entryDownKD {
    IF MODE = "TUNED" { RETURN T_ENTRY_DOWN_KD. }
    RETURN 0.00020.
}

FUNCTION finalLatKP {
    IF MODE = "TUNED" { RETURN T_FINAL_LAT_KP. }
    RETURN 0.010.
}

FUNCTION finalLatKD {
    IF MODE = "TUNED" { RETURN T_FINAL_LAT_KD. }
    RETURN 0.020.
}

FUNCTION vsKP {
    IF MODE = "TUNED" { RETURN T_VS_KP. }
    RETURN 0.10 + clamp((twrNow()-1.0)*0.03, 0, 0.05).
}

FUNCTION vsKD {
    IF MODE = "TUNED" { RETURN T_VS_KD. }
    RETURN vsKP() * 1.8.
}

FUNCTION entryMargin {
    IF MODE = "TUNED" { RETURN T_ENTRY_MARGIN. }
    IF HASATM { RETURN 1.08. }
    RETURN 1.03.
}

FUNCTION landingMargin {
    IF MODE = "TUNED" { RETURN T_LANDING_MARGIN. }
    RETURN 1.05 + clamp((2.0 - twrNow())*0.06, 0, 0.18).
}

FUNCTION suicidePad {
    IF MODE = "TUNED" { RETURN T_SUICIDE_PAD. }
    RETURN 8 + clamp(horizSpeed()/25, 0, 20).
}

FUNCTION upBiasEntry {
    IF MODE = "TUNED" { RETURN T_UP_BIAS_ENTRY. }
    RETURN 0.22.
}

FUNCTION upBiasFinal {
    IF MODE = "TUNED" { RETURN T_UP_BIAS_FINAL. }
    RETURN 2.0.
}

// ------------------------------------------------------------
// TARGET-CENTERED LOCAL FRAME
// ------------------------------------------------------------

FUNCTION geoOffsetFromTargetENU {
    PARAMETER geo.
    LOCAL d IS TARGET:DISTANCE(geo).
    LOCAL b IS TARGET:BEARINGTO(geo).
    LOCAL eastErr IS d * SIN(b * CONSTANT():DEGTORAD).
    LOCAL northErr IS d * COS(b * CONSTANT():DEGTORAD).
    RETURN V(eastErr, northErr, 0).
}

FUNCTION localCrossDownFromPredGeo {
    PARAMETER predGeo.
    LOCAL lineBearing IS SHIP:GEOPOSITION:BEARINGTO(TARGET).
    LOCAL forwardW IS vectorFromBearingAt(SHIP:GEOPOSITION, lineBearing).
    LOCAL rightW IS vectorFromBearingAt(SHIP:GEOPOSITION, lineBearing + 90).

    LOCAL offENU IS geoOffsetFromTargetENU(predGeo).
    LOCAL eastT IS eastVecAt(TARGET).
    LOCAL northT IS northVecAt(TARGET).

    LOCAL worldOffset IS eastT * offENU:X + northT * offENU:Y.

    LOCAL crossErr IS VDOT(worldOffset, rightW).
    LOCAL downErr IS VDOT(worldOffset, forwardW).

    RETURN V(crossErr, downErr, 0).
}

// ------------------------------------------------------------
// IMPACT PREDICTOR
// ------------------------------------------------------------

FUNCTION atmosphericDragFactor {
    PARAMETER alt, speed.
    IF NOT HASATM { RETURN 0. }.
    LOCAL h IS bodyAtmosHeight().
    IF h <= 0 { RETURN 0. }.
    LOCAL dens IS clamp(1 - alt / h, 0, 1).
    RETURN dens * MAX(0.15, speed / 120).
}

FUNCTION predictImpactGeo {
    LOCAL geo IS SHIP:GEOPOSITION.
    LOCAL alt IS SHIP:ALTITUDE.

    LOCAL eastSpd IS VDOT(surfaceVel(), eastVecAt(geo)).
    LOCAL northSpd IS VDOT(surfaceVel(), northVecAt(geo)).
    LOCAL upSpd IS VDOT(surfaceVel(), localUpAtGeo(geo)).

    LOCAL t IS 0.
    LOCAL step IS PREDICT_STEP_VAC.
    IF HASATM { SET step TO PREDICT_STEP_ATM. }

    UNTIL alt <= 0 OR t > PREDICT_MAX_TIME {
        LOCAL g IS gravAtAlt(MAX(0,alt)).

        LOCAL hs IS SQRT(eastSpd*eastSpd + northSpd*northSpd).
        LOCAL dragF IS atmosphericDragFactor(alt, hs).

        SET eastSpd TO eastSpd * (1 - dragF * step * 0.020).
        SET northSpd TO northSpd * (1 - dragF * step * 0.020).
        SET upSpd TO upSpd - g * step.
        IF HASATM {
            SET upSpd TO upSpd * (1 - dragF * step * 0.010).
        }

        LOCAL latRad IS geo:LAT * CONSTANT():DEGTORAD.
        LOCAL cosLat IS MAX(0.01, COS(latRad)).

        LOCAL dLatDeg IS (northSpd / RADIUS) * step * CONSTANT():RADTODEG.
        LOCAL dLngDeg IS (eastSpd / (RADIUS * cosLat)) * step * CONSTANT():RADTODEG.
        LOCAL rotDeg IS bodyRotRate() * step * CONSTANT():RADTODEG.

        SET geo TO LATLNG(geo:LAT + dLatDeg, geo:LNG + dLngDeg - rotDeg, BODY).
        SET alt TO alt + upSpd * step.
        SET t TO t + step.
    }

    RETURN geo.
}

// ------------------------------------------------------------
// PLANE ALIGN
// ------------------------------------------------------------

FUNCTION targetPlaneErrorDeg {
    LOCAL targetPos IS TARGET:POSITION - BODY:POSITION.
    LOCAL nOrbit IS SHIP:ORBIT:NORMAL:VECTOR.
    LOCAL s IS VDOT(targetPos:NORMALIZED, nOrbit:NORMALIZED).
    RETURN ARCSIN(clamp(s, -1, 1)) * CONSTANT():RADTODEG.
}

FUNCTION planeAlignBurnDir {
    LOCAL err IS targetPlaneErrorDeg().
    LOCAL n IS SHIP:ORBIT:NORMAL:VECTOR:NORMALIZED.
    IF err > 0 { RETURN -n. }
    RETURN n.
}

FUNCTION doPlaneAlign {
    IF NOT ENABLE_PLANE_ALIGN { RETURN. }

    SET PHASE TO "PLANE_ALIGN".
    PRINT "Phase: plane align".

    UNTIL ABS(targetPlaneErrorDeg()) < 0.05 {
        LOCAL err IS targetPlaneErrorDeg().
        LOCAL derr IS (err - prevPlaneErr) / DT.
        SET prevPlaneErr TO err.

        LOCAL burnDir IS planeAlignBurnDir().
        lookAtVec(burnDir).

        LOCAL thr IS clamp(ABS(err) * planeKP() + ABS(derr) * planeKD(), 0, 0.30).
        LOCK THROTTLE TO thr.

        PRINT "Plane err: " + ROUND(err,4) + " deg  Thr: " + ROUND(thr,3) AT(0,3).
        WAIT DT.
    }

    LOCK THROTTLE TO 0.
    WAIT 0.2.
}

// ------------------------------------------------------------
// BOOSTBACK / DEORBIT
// ------------------------------------------------------------

FUNCTION boostbackGuidanceVec {
    PARAMETER crossErr, downErr, crossRate, downRate.

    LOCAL retro IS -surfaceVel():NORMALIZED.
    LOCAL up IS upVec().

    LOCAL towardTarget IS vectorFromBearingAt(SHIP:GEOPOSITION, targetBearing()).
    LOCAL rightOfTrack IS vectorFromBearingAt(SHIP:GEOPOSITION, targetBearing() + 90).

    LOCAL crossCmd IS -(crossErr * boostCrossKP() + crossRate * boostCrossKD()).
    LOCAL downCmd  IS -(downErr  * boostDownKP()  + downRate  * boostDownKD()).

    LOCAL vec IS retro
              + towardTarget * clamp(downCmd, -0.8, 0.8)
              + rightOfTrack * clamp(crossCmd, -0.8, 0.8)
              + up * upBiasEntry().

    RETURN vec:NORMALIZED.
}

FUNCTION shouldStartBoostback {
    LOCAL pred IS predictImpactGeo().
    LOCAL e IS localCrossDownFromPredGeo(pred).
    IF ABS(e:X) > 1500 { RETURN TRUE. }
    IF ABS(e:Y) > 4000 { RETURN TRUE. }
    IF SHIP:ORBIT:PERIAPSIS < 20000 { RETURN TRUE. }
    RETURN FALSE.
}

FUNCTION doBoostback {
    IF NOT ENABLE_BOOSTBACK { RETURN. }

    SET PHASE TO "BOOSTBACK".
    PRINT "Phase: boostback/deorbit".

    UNTIL SHIP:ORBIT:PERIAPSIS < 0 OR radarAlt() < 25000 {
        LOCAL pred IS predictImpactGeo().
        LOCAL e IS localCrossDownFromPredGeo(pred).

        LOCAL crossErr IS e:X.
        LOCAL downErr IS e:Y.

        LOCAL crossRate IS (crossErr - prevCrossErr) / DT.
        LOCAL downRate IS (downErr - prevDownErr) / DT.

        SET prevCrossErr TO crossErr.
        SET prevDownErr TO downErr.

        LOCAL gvec IS boostbackGuidanceVec(crossErr, downErr, crossRate, downRate).
        lookAtVec(gvec).

        LOCAL thr IS 0.
        IF shouldStartBoostback() {
            IF downErr < -300 {
                SET thr TO clamp(ABS(downErr)/80000 + ABS(crossErr)/150000, 0.05, 0.45).
            } ELSE IF ABS(crossErr) > 5000 {
                SET thr TO clamp(ABS(crossErr)/120000, 0.04, 0.22).
            } ELSE {
                SET thr TO 0.
            }
        }

        LOCK THROTTLE TO thr.

        PRINT "Pred cross: " + ROUND(crossErr,1) + " m  down: " + ROUND(downErr,1) + " m  Thr: " + ROUND(thr,2) AT(0,5).
        PRINT "Periapsis: " + ROUND(SHIP:ORBIT:PERIAPSIS,1) AT(0,6).

        IF ABS(crossErr) < 500 AND ABS(downErr) < 1500 AND SHIP:ORBIT:PERIAPSIS < 5000 {
            BREAK.
        }

        WAIT DT.
    }

    LOCK THROTTLE TO 0.
    WAIT 0.2.
}

// ------------------------------------------------------------
// ENTRY
// ------------------------------------------------------------

FUNCTION shouldStartEntryBurn {
    LOCAL hs IS horizSpeed().
    LOCAL decel IS maxUpDecel().
    LOCAL need IS stopDist(hs, decel) * entryMargin().
    RETURN radarAlt() < need + 2500.
}

FUNCTION entryGuidanceVec {
    LOCAL pred IS predictImpactGeo().
    LOCAL e IS localCrossDownFromPredGeo(pred).

    LOCAL crossErr IS e:X.
    LOCAL downErr IS e:Y.

    LOCAL crossRate IS (crossErr - prevCrossErr) / DT.
    LOCAL downRate IS (downErr - prevDownErr) / DT.

    SET prevCrossErr TO crossErr.
    SET prevDownErr TO downErr.

    LOCAL retro IS -surfaceVel():NORMALIZED.
    LOCAL up IS upVec().

    LOCAL towardTarget IS vectorFromBearingAt(SHIP:GEOPOSITION, targetBearing()).
    LOCAL rightOfTrack IS vectorFromBearingAt(SHIP:GEOPOSITION, targetBearing() + 90).

    LOCAL crossCmd IS -(crossErr * entryCrossKP() + crossRate * entryCrossKD()).
    LOCAL downCmd IS -(downErr * entryDownKP() + downRate * entryDownKD()).

    LOCAL vec IS retro
              + towardTarget * clamp(downCmd, -0.7, 0.7)
              + rightOfTrack * clamp(crossCmd, -0.7, 0.7)
              + up * upBiasEntry().

    RETURN vec:NORMALIZED.
}

FUNCTION doEntryBurn {
    IF NOT ENABLE_ENTRY_BURN { RETURN. }

    SET PHASE TO "ENTRY".
    PRINT "Phase: entry".

    UNTIL shouldStartLandingBurn() OR radarAlt() < 1200 {
        LOCAL gvec IS entryGuidanceVec().
        lookAtVec(gvec).

        LOCAL hs IS horizSpeed().
        LOCAL alt IS radarAlt().
        LOCAL desiredDecel IS safeDiv(hs*hs, 2*MAX(20, alt), 0).
        LOCAL accelNeed IS desiredDecel + gravAtAlt(SHIP:ALTITUDE).
        LOCAL thr IS clamp(safeDiv(accelNeed, maxAccel(), 0), 0, 1).

        IF shouldStartEntryBurn() {
            LOCK THROTTLE TO thr.
        } ELSE {
            LOCK THROTTLE TO 0.
        }

        IF alt < GEAR_DEPLOY_ALT {
            GEAR ON.
        }

        PRINT "Alt: " + ROUND(alt,1) + " HS: " + ROUND(hs,2) + " VS: " + ROUND(verticalSpeed(),2) + " Thr: " + ROUND(thr,2) AT(0,8).
        WAIT DT.
    }

    LOCK THROTTLE TO 0.
    WAIT 0.2.
}

// ------------------------------------------------------------
// LANDING BURN / TERMINAL
// ------------------------------------------------------------

FUNCTION shouldStartLandingBurn {
    LOCAL speed IS totalSpeed().
    LOCAL decel IS maxUpDecel().
    LOCAL need IS stopDist(speed, decel) * landingMargin() + suicidePad().
    RETURN radarAlt() <= need.
}

FUNCTION desiredVerticalSpeed {
    LOCAL alt IS radarAlt().
    IF alt > 1000 { RETURN -18. }.
    IF alt > 500  { RETURN -12. }.
    IF alt > 200  { RETURN -7. }.
    IF alt > 80   { RETURN -4. }.
    IF alt > 30   { RETURN -2.2 }.
    IF alt > 10   { RETURN -1.2 }.
    RETURN GOAL_TOUCHDOWN_VS.
}

FUNCTION finalGuidanceVec {
    LOCAL up IS upVec().
    LOCAL hv IS horizVel().

    LOCAL pred IS predictImpactGeo().
    LOCAL e IS localCrossDownFromPredGeo(pred).
    LOCAL crossErr IS e:X.
    LOCAL crossRate IS (crossErr - prevFinalCrossErr) / DT.
    SET prevFinalCrossErr TO crossErr.

    LOCAL towardTarget IS vectorFromBearingAt(SHIP:GEOPOSITION, targetBearing()).
    LOCAL rightOfTrack IS vectorFromBearingAt(SHIP:GEOPOSITION, targetBearing() + 90).

    LOCAL latCmd IS -(crossErr * finalLatKP() + crossRate * finalLatKD()).
    SET latCmd TO clamp(latCmd, -18, 18).

    LOCAL vec IS up * upBiasFinal().

    IF hv:MAG > 0.1 {
        SET vec TO vec - hv:NORMALIZED * clamp(hv:MAG / 6, 0, 1.6).
    }

    SET vec TO vec + towardTarget * clamp(targetDist()/MAX(1, radarAlt()) * 0.20, 0, 0.8).
    SET vec TO vec + rightOfTrack * (latCmd / 18).

    RETURN vec:NORMALIZED.
}

FUNCTION doLandingBurnAndTouchdown {
    SET PHASE TO "LANDING".
    PRINT "Phase: landing".

    IF ENABLE_RCS_FINAL { RCS ON. }
    GEAR ON.

    UNTIL SHIP:STATUS = "LANDED" OR SHIP:STATUS = "SPLASHED" {
        LOCAL alt IS radarAlt().
        LOCAL hs IS horizSpeed().
        LOCAL vs IS verticalSpeed().

        LOCAL gvec IS finalGuidanceVec().
        lookAtVec(gvec).

        LOCAL targetVS IS desiredVerticalSpeed().
        LOCAL err IS targetVS - vs.
        LOCAL derr IS (err - prevVsErr) / DT.
        SET prevVsErr TO err.

        LOCAL accelCmd IS err * vsKP() + derr * vsKD().
        LOCAL thr IS safeDiv(gravAtAlt(SHIP:ALTITUDE) + accelCmd, maxAccel(), 0).

        IF alt < 20 AND hs > 2 {
            SET thr TO thr + clamp(hs / 15, 0, 0.25).
        }

        IF alt < 5 {
            SET thr TO MAX(thr, 0.08).
        }

        SET thr TO clamp(thr, MIN_THR, MAX_THR).
        LOCK THROTTLE TO thr.

        IF alt < GEAR_DEPLOY_ALT {
            GEAR ON.
        }

        PRINT "Alt: " + ROUND(alt,2) + " HS: " + ROUND(hs,2) + " VS: " + ROUND(vs,2) + " tgtVS: " + ROUND(targetVS,2) + " Thr: " + ROUND(thr,3) AT(0,11).
        PRINT "TargetDist: " + ROUND(targetDist(),2) + " m" AT(0,12).

        WAIT DT.
    }

    LOCK THROTTLE TO 0.
    RCS OFF.
    SAS ON.
    PRINT "Touchdown.".
    PRINT "Final target distance: " + ROUND(targetDist(),3) + " m".
}

// ------------------------------------------------------------
// MAIN
// ------------------------------------------------------------

FUNCTION main {
    PRINT "Mode: " + MODE.
    PRINT "Body: " + BODY:NAME.
    PRINT "Target lat/lng: " + ROUND(TARGET_LAT,6) + " / " + ROUND(TARGET_LNG,6).
    PRINT "TWR: " + ROUND(twrNow(),2).

    IF twrNow() < MIN_REQUIRED_TWR {
        PRINT "WARNING: insufficient TWR for reliable landing.".
        IF ABORT_IF_LOW_TWR {
            RETURN.
        }
    }

    lookAtVec(-orbitalVel():NORMALIZED).
    WAIT 1.

    IF ENABLE_PLANE_ALIGN {
        doPlaneAlign().
    }

    IF ENABLE_BOOSTBACK {
        doBoostback().
    }

    SET PHASE TO "COAST".
    PRINT "Phase: coast".

    UNTIL shouldStartEntryBurn() OR shouldStartLandingBurn() OR radarAlt() < 15000 {
        lookAtVec((-surfaceVel():NORMALIZED + upVec()*0.2):NORMALIZED).
        LOCK THROTTLE TO 0.
        PRINT "Coast alt: " + ROUND(radarAlt(),1) + " targetDist: " + ROUND(targetDist(),1) AT(0,14).
        WAIT 0.2.
    }

    doEntryBurn().
    doLandingBurnAndTouchdown().

    UNLOCK STEERING.
    LOCK THROTTLE TO 0.
    PRINT "Done.".
}

main().