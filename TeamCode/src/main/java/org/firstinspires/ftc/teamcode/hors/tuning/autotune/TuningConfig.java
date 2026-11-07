package org.firstinspires.ftc.teamcode.hors.tuning.autotune;

import com.aaravdhawan25.pidautotuner.ftc.dashboard.AutoTuneDash;

/**
 * Relay-test settings for the FtcAutoTune shooter OpModes (https://ftcautotune.dev/).
 *
 * Adapted from the FtcAutoTune QuickStart's TuningConfig, velocity fields only: the drivetrain is
 * tuned through Pedro, so only the pollen shooter uses this. The motor name, direction, ticks per rev
 * and target RPM are NOT here: the OpMode reads them from hors/shooter/ShooterConstants, so the tuner always
 * spins the same motor, the same way, at the same speed the robot code does.
 *
 * {@code @AutoTuneDash} makes every public static field live-editable on the Pulse dashboard
 * that ships with the library, without a redeploy.
 */
@AutoTuneDash
public class TuningConfig {

    private TuningConfig() {}

    // ---- Relay test -----------------------------------------------------

    /** Bang-bang power, 0..1. The library suggests 0.5-0.8 for flywheels. CARRIED OVER (QuickStart default). */
    public static double RELAY_AMPLITUDE = 0.5;

    /** Oscillation cycles averaged for Ku and Tu. CARRIED OVER (QuickStart default). */
    public static int CYCLES_TO_COLLECT = 6;

    /** First cycles thrown away while the wheel settles. CARRIED OVER (QuickStart default). */
    public static int CYCLES_TO_IGNORE = 2;

    /** Gives up if the relay never oscillates, seconds of OpMode runtime. CARRIED OVER (QuickStart default). */
    public static double RELAY_TEST_TIMEOUT_S = 15.0;

    // ---- Velocity / PIDF ------------------------------------------------

    /** Relay deadband, flywheel RPM. CARRIED OVER (QuickStart default). */
    public static double VELOCITY_HYSTERESIS_RPM = 30;

    /** Open-loop powers for the kF sweep. CARRIED OVER (QuickStart default). */
    public static double[] FEEDFORWARD_TEST_POWERS = {0.5, 0.75, 1.0};

    /** Seconds held at each sweep power before sampling. CARRIED OVER (QuickStart default). */
    public static double FEEDFORWARD_SETTLE_TIME_S = 1.5;

    /** False = PD-only Ziegler-Nichols, kI = 0 (the library's advice for flywheels). */
    public static boolean TUNE_INTEGRAL_TERM = false;
}
