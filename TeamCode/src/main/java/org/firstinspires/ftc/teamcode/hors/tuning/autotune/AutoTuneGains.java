package org.firstinspires.ftc.teamcode.hors.tuning.autotune;

/**
 * Converts FtcAutoTune's gains into ShooterController.Config units, kept free of hardware so it
 * can be unit-tested.
 *
 * FtcAutoTune works in encoder ticks/s with a plain feedforward:
 *   power = kP * errTicks + kD * dErrTicks/dt + kF * targetTicks          (at the sweep's voltage)
 * ShooterController works in flywheel RPM with a voltage-scaled feedforward:
 *   power = kP * errRpm + kD * dErrRpm/dt + kF * targetTicks / (maxTicksPerSec * vBattery / vRef)
 *
 * One RPM is ticksPerRev / 60 ticks/s, so kP and kD scale by that factor. With kF = 1 and
 * vRef = 12 V, the two feedforwards agree at the sweep voltage when
 *   maxTicksPerSec = (1 / kF_autotune) * (12 / vSweep)
 * and from there ShooterController's voltage scaling takes over as the battery drains.
 * FtcAutoTune fits its kF through the origin, so it measures no static friction: kS stays 0.
 */
public final class AutoTuneGains {

    public static final double V_REF = 12.0;

    private AutoTuneGains() {
    }

    /** One gain set in ShooterController.Config units. */
    public static final class Converted {
        public final double kP;
        public final double kI;
        public final double kD;
        /** ShooterController.Config.ffReferenceMaxTicksPerSec, at {@link #V_REF}. */
        public final double ffReferenceMaxTicksPerSec;

        Converted(double kP, double kI, double kD, double ffReferenceMaxTicksPerSec) {
            this.kP = kP;
            this.kI = kI;
            this.kD = kD;
            this.ffReferenceMaxTicksPerSec = ffReferenceMaxTicksPerSec;
        }
    }

    /**
     * @param kP          FtcAutoTune kP, power per (ticks/s) of error
     * @param kI          FtcAutoTune kI, power per (ticks/s * s) of error
     * @param kD          FtcAutoTune kD, power per (ticks/s per s) of error
     * @param kF          FtcAutoTune kF, power per ticks/s of target
     * @param ticksPerRev encoder ticks per flywheel revolution
     * @param vSweep      mean battery voltage during the kF sweep, volts
     */
    public static Converted convert(double kP, double kI, double kD, double kF,
                                    double ticksPerRev, double vSweep) {
        if (!(kF > 0) || !(vSweep > 0) || !(ticksPerRev > 0)) {
            throw new IllegalArgumentException("kF, vSweep and ticksPerRev must be positive");
        }
        double ticksPerSecPerRpm = ticksPerRev / 60.0;
        return new Converted(kP * ticksPerSecPerRpm, kI * ticksPerSecPerRpm, kD * ticksPerSecPerRpm,
                (1.0 / kF) * (V_REF / vSweep));
    }

    /** ticks/s for a flywheel speed in RPM. */
    public static double rpmToTicksPerSec(double rpm, double ticksPerRev) {
        return rpm * ticksPerRev / 60.0;
    }
}
