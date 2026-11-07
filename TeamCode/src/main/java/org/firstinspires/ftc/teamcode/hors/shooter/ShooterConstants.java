package org.firstinspires.ftc.teamcode.hors.shooter;

import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * The pollen shooter's hardware name, direction and tuning, for HORS Clutch Teleop and the
 * FtcAutoTune OpMode.
 *
 * Every value is CARRIED OVER from BioHORS (team 33950, MechanismConstants.POLLEN_SHOOTER, as of
 * 2026-11-06): the same robot build, motor and 1:1 flywheel. Re-run "AutoTune Pollen Shooter" on
 * this robot and paste its printed lines over the tuned values below.
 *
 * Provenance labels:
 *   SPEC              from the manufacturer's spec sheet
 *   CARRIED OVER      copied from BioHORS; true for that robot, not yet checked on this one
 *   PLACEHOLDER       made up so the code runs
 */
public final class ShooterConstants {

    private ShooterConstants() {
    }

    /** Hardware map name, the same one Teleop and PIDTest use. */
    public static final String POLLEN_SHOOTER_NAME = "pollenShooter";

    /** REVERSE, as in tests/PIDTest. Flip it here if the wheel spins backwards. */
    public static DcMotorSimple.Direction POLLEN_SHOOTER_DIRECTION = DcMotorSimple.Direction.REVERSE;

    // ===== goBILDA 5000-series 6000 RPM motor (1:1) =====
    // SPEC: 28 encoder counts per motor-shaft revolution; 6000 RPM free speed at 12 V.
    private static final double GOBILDA_6000_TICKS_PER_REV = 28.0; // SPEC; 1:1 flywheel CARRIED OVER

    public static final ShooterController.Config POLLEN_SHOOTER = new ShooterController.Config();

    static {
        POLLEN_SHOOTER.label = "Pollen";
        POLLEN_SHOOTER.ticksPerRev = GOBILDA_6000_TICKS_PER_REV;

        // Default RPM: SET BY HAND (the team, 2026-11-07; was BioHORS's 3050).
        POLLEN_SHOOTER.targetRpm = 3200.0;

        // Pollen: CARRIED OVER. On BioHORS: CALIBRATED FROM ROBOT by FtcAutoTune v1.4.3 (2026-11-05),
        // PD (classic ZN) candidate, Ku 0.019099 Tu 0.2211 s, kF sweep at 13.52 V; kS not measured.
        POLLEN_SHOOTER.ffReferenceVoltage = 12.0;
        POLLEN_SHOOTER.ffReferenceMaxTicksPerSec = 2250.8;
        POLLEN_SHOOTER.kF = 1.0;
        POLLEN_SHOOTER.kS = 0.0;
        // P and D: CARRIED OVER (BioHORS team, 2026-11-06: 1.5x the autotune's, for rapid fire).
        POLLEN_SHOOTER.kP = 0.010700;
        POLLEN_SHOOTER.kI = 0.000000;
        POLLEN_SHOOTER.kD = 0.000296;

        POLLEN_SHOOTER.integralLimit = 50.0;        // CARRIED OVER
        POLLEN_SHOOTER.derivativeAlpha = 0.9;       // CARRIED OVER
        POLLEN_SHOOTER.rpmFilterAlpha = 0.72;       // CARRIED OVER
        POLLEN_SHOOTER.powerSmoothingAlpha = 0.5;   // CARRIED OVER
        POLLEN_SHOOTER.rpmTolerance = 75.0;         // CARRIED OVER
    }

    /** Dpad left / right in HORS Clutch Teleop moves the pollen target by this much, rpm. CARRIED OVER. */
    public static double SHOOTER_TRIM_STEP_RPM = 50.0;

    /** A switched-off shooter still turning faster than this shows "OFF, coasting". Display only. */
    public static double SHOOTER_COAST_SHOW_RPM = 100.0;

    // ===== Shooter encoder check (ShooterController), CARRIED OVER =====
    /** Only judged while the motor is given at least this power. */
    public static double SHOOTER_ENCODER_CHECK_POWER = 0.3;
    /** Below this reading, rpm, the encoder counts as silent. */
    public static double SHOOTER_ENCODER_DEAD_RPM = 50.0;
    /** Silent (or backwards) this long while pushed, seconds, means dead. */
    public static double SHOOTER_ENCODER_DEAD_S = 0.5;
}
