package org.firstinspires.ftc.teamcode.hors.shooter;


import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.function.DoubleSupplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * One shooter: a single encoder motor under a custom PIDF velocity loop.
 *
 * Ported from BioHORS (team 33950). On this robot only the POLLEN shooter uses it; the nectar
 * shooter stays on the existing FlywheelPIDF. The gains live in a {@link Config} (see
 * ShooterConstants.POLLEN_SHOOTER), and this controller reads them every loop, so live edits take
 * effect without a restart.
 *
 * Spin direction is set on the motor in the OpMode, not here.
 *
 * Encoder check: if the motor is driven at SHOOTER_ENCODER_CHECK_POWER or more for
 * SHOOTER_ENCODER_DEAD_S and the encoder reads under SHOOTER_ENCODER_DEAD_RPM, the encoder (or the
 * motor) is dead. The loop would otherwise wind up to full power forever; from then on the shooter
 * runs on its feedforward alone (the power the target needs, no PID) and {@link #encoderFault()} is
 * true. A backwards encoder (wired or set reversed: it reads NEGATIVE rpm while pushed forward, so
 * the loop pushes harder and harder) is caught the same way and also sets
 * {@link #encoderReversed()}. Both stay latched until the OpMode ends. A wheel that never reads speed is never at target,
 * so the fire interlock keeps the gates shut: a dead motor and a dead encoder look the same.
 */
public class ShooterController {

    /**
     * Per-shooter tuning values. Every value has a provenance label where
     * it is assigned in ShooterConstants; nothing here is measured yet.
     */
    public static class Config {
        public String label;

        /** Encoder ticks per FLYWHEEL revolution (motor ticks x gear ratio). */
        public double ticksPerRev;

        public double kP;
        public double kI;
        public double kD;
        public double kF;
        /**
         * Static-friction power: the power it takes just to keep the flywheel turning, added while
         * the target is above zero. Voltage-compensated like the feedforward. 0 = off.
         */
        public double kS;
        public double integralLimit;
        public double derivativeAlpha;
        public double rpmFilterAlpha;
        public double powerSmoothingAlpha;

        public double targetRpm;
        public double rpmTolerance;

        /**
         * Feedforward reference: motor-encoder ticks/sec at full power at the
         * reference voltage. The encoder is on the motor, so this does not
         * change with an external gear ratio (only ticksPerRev does).
         */
        public double ffReferenceVoltage;
        public double ffReferenceMaxTicksPerSec;
    }

    private final Config config;
    private final DcMotorEx motor;
    private final Telemetry telemetry; // nullable
    private final VoltageSensor voltageSensor; // nullable
    /** A power change smaller than this is not written to the motor (a hub command each). A move
     *  to exactly zero is always written. SET BY HAND: 0.002 of full power, about 12 rpm of
     *  feedforward on a 6000 rpm motor, inside every shooter's rpm tolerance. */
    public static double POWER_WRITE_EPSILON = 0.002;
    private double lastWrittenPower = Double.NaN;
    private DoubleSupplier voltageSource; // nullable: when set, used instead of reading the sensor
    private final ElapsedTime timer = new ElapsedTime();

    // --- Internal state ---
    private double targetRpm;
    private double lastError = 0.0;
    private double integralSum = 0.0;
    private double lastDerivativeEstimate = 0.0;
    private int lastPos = 0;
    private double currentRpm = 0.0;
    private double lastAppliedPower = 0.0;

    private boolean shooterOn = false;
    private double encoderSilentS = 0.0;
    private double encoderBackwardS = 0.0;
    private boolean encoderFault = false;
    private boolean encoderReversed = false;
    private boolean lastAtTarget = false;
    private boolean justReachedTargetFlag = false;

    public ShooterController(Config config, DcMotor motor,
                             Telemetry telemetry, VoltageSensor voltageSensor) {
        if (!(motor instanceof DcMotorEx)) {
            throw new IllegalArgumentException(config.label + " shooter must be a DcMotorEx");
        }
        this.config = config;
        this.motor = (DcMotorEx) motor;
        this.telemetry = telemetry;
        this.voltageSensor = voltageSensor;
        this.targetRpm = config.targetRpm;

        try {
            // Our own PIDF does the velocity control, so the motor runs open
            // loop; getVelocity() still reads the encoder.
            this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        } catch (Exception e) {
            if (telemetry != null) telemetry.addData(config.label + ".init", "config failed: " + e.getMessage());
        }

        timer.reset();
        lastPos = this.motor.getCurrentPosition();
    }

    /** Take the battery voltage from {@code source} (a shared, cached reading) instead of reading
     *  the sensor every update. */
    public void setVoltageSource(DoubleSupplier source) {
        voltageSource = source;
    }

    public String getLabel() {
        return config.label;
    }

    public void setTargetRpm(double rpm) {
        if (rpm != targetRpm) {
            // Reset on setpoint changes so the integral does not carry over.
            integralSum = 0.0;
            lastError = 0.0;
            lastDerivativeEstimate = 0.0;
        }
        targetRpm = rpm;
    }

    public double getTargetRpm() {
        return targetRpm;
    }

    public void adjustTargetRpm(double delta) {
        setTargetRpm(Math.max(0.0, targetRpm + delta));
    }

    /** True when switched on and spinning at {@code fraction} of the target or more. */
    public boolean atLeastFractionOfTarget(double fraction) {
        return shooterOn && targetRpm > 0 && currentRpm >= fraction * targetRpm;
    }

    public boolean isAtTarget() {
        return Math.abs(targetRpm - currentRpm) <= config.rpmTolerance;
    }

    public void toggleShooterOn() {
        shooterOn = !shooterOn;
    }

    public void setShooterOn(boolean on) {
        shooterOn = on;
    }

    public boolean isShooterOn() {
        return shooterOn;
    }

    public double getCurrentRpm() {
        return currentRpm;
    }

    public double getLastAppliedPower() {
        return lastAppliedPower;
    }

    /** The encoder's speed right now, rpm, read without running the loop (the pre-match check). */
    public double measuredRpmNow() {
        try {
            return (motor.getVelocity() * 60.0) / config.ticksPerRev;
        } catch (Exception e) {
            return Double.NaN;
        }
    }

    /** True once the encoder check has found the wheel driven but silent or backwards (latched). */
    public boolean encoderFault() {
        return encoderFault;
    }

    /** True when the fault was a backwards reading (wired or set reversed), not a silent one. */
    public boolean encoderReversed() {
        return encoderReversed;
    }

    public boolean justReachedTarget() {
        if (justReachedTargetFlag) {
            justReachedTargetFlag = false;
            return true;
        }
        return false;
    }

    /** Call once per loop. Computes PIDF output and writes it to the motor. */
    public void update() {
        double dt = timer.seconds();
        if (dt <= 0) dt = 1e-3;

        double currentRpmNow = readRpm(dt);
        // Low-pass filter on measured RPM (alpha = 1 => no filtering).
        currentRpm = config.rpmFilterAlpha * currentRpmNow + (1.0 - config.rpmFilterAlpha) * currentRpm;

        timer.reset();

        double error = targetRpm - currentRpm;

        // Integral with clamping (anti-windup).
        integralSum += error * dt;
        if (integralSum > config.integralLimit) integralSum = config.integralLimit;
        if (integralSum < -config.integralLimit) integralSum = -config.integralLimit;

        // Derivative with low-pass on error change.
        double rawDeriv = (error - lastError) / dt;
        double deriv = config.derivativeAlpha * lastDerivativeEstimate + (1.0 - config.derivativeAlpha) * rawDeriv;
        lastDerivativeEstimate = deriv;

        // Feedforward: fraction of max speed at current battery voltage.
        double voltage = getBatteryVoltage();
        double maxTicksPerSec = (voltage / config.ffReferenceVoltage) * config.ffReferenceMaxTicksPerSec;
        if (maxTicksPerSec < 1e-3) maxTicksPerSec = 1e-3;
        double targetTicksPerSec = (targetRpm * config.ticksPerRev) / 60.0;
        double ff = (targetTicksPerSec / maxTicksPerSec) * config.kF;
        if (targetRpm > 0) ff += config.kS * (config.ffReferenceVoltage / Math.max(voltage, 1e-3));

        // Encoder check: driven hard forward, but the encoder says the wheel is not turning, or is
        // turning backwards.
        boolean pushedForward = shooterOn && lastAppliedPower >= ShooterConstants.SHOOTER_ENCODER_CHECK_POWER;
        if (!encoderFault && pushedForward && Math.abs(currentRpmNow) < ShooterConstants.SHOOTER_ENCODER_DEAD_RPM) {
            encoderSilentS += dt;
            if (encoderSilentS >= ShooterConstants.SHOOTER_ENCODER_DEAD_S) encoderFault = true;
        } else {
            encoderSilentS = 0.0;
        }
        if (!encoderFault && pushedForward && currentRpmNow <= -ShooterConstants.SHOOTER_ENCODER_DEAD_RPM) {
            encoderBackwardS += dt;
            if (encoderBackwardS >= ShooterConstants.SHOOTER_ENCODER_DEAD_S) {
                encoderFault = true;
                encoderReversed = true;
            }
        } else {
            encoderBackwardS = 0.0;
        }

        // PIDF output, clamped. With a dead encoder: the feedforward alone, no feedback to wind up.
        double out = encoderFault ? ff
                : ff + (config.kP * error) + (config.kI * integralSum) + (config.kD * deriv);
        out = Math.max(-1.0, Math.min(1.0, out));

        if (!shooterOn) out = 0.0;

        // Power smoothing (alpha = 1 => no smoothing).
        double smoothedOut = config.powerSmoothingAlpha * out + (1.0 - config.powerSmoothingAlpha) * lastAppliedPower;
        // Switched off: the smoothed tail ends at a real zero, not a residue under POWER_WRITE_EPSILON.
        if (out == 0.0 && Math.abs(smoothedOut) < POWER_WRITE_EPSILON) smoothedOut = 0.0;

        try {
            boolean changed = Double.isNaN(lastWrittenPower) || (smoothedOut == 0.0
                    ? lastWrittenPower != 0.0 : Math.abs(smoothedOut - lastWrittenPower) >= POWER_WRITE_EPSILON);
            if (changed) {
                motor.setPower(smoothedOut);
                lastWrittenPower = smoothedOut;
            }
        } catch (Exception e) {
            if (telemetry != null) telemetry.addData(config.label + ".power", "setPower failed: " + e.getMessage());
        }

        lastAppliedPower = smoothedOut;
        lastError = error;

        // At-target detection (rising edge sets justReachedTargetFlag).
        boolean atTargetNow = Math.abs(error) <= config.rpmTolerance;
        if (atTargetNow && !lastAtTarget) {
            justReachedTargetFlag = true;
        }
        lastAtTarget = atTargetNow;
    }

    private double getBatteryVoltage() {
        try {
            if (voltageSource != null) {
                double v = voltageSource.getAsDouble();
                if (v > 1e-3) return v;
            } else if (voltageSensor != null) {
                double v = voltageSensor.getVoltage();
                if (v > 1e-3) return v;
            }
        } catch (Exception ignored) {
        }
        return 11.0; // fallback
    }

    private double readRpm(double dtSeconds) {
        // Prefer built-in velocity; fall back to position diff.
        double ticksPerSecond;
        try {
            ticksPerSecond = motor.getVelocity(); // ticks/sec
            lastPos = motor.getCurrentPosition();
        } catch (Exception e) {
            int pos = motor.getCurrentPosition();
            int delta = pos - lastPos;
            lastPos = pos;
            double dt = (dtSeconds <= 0) ? 1e-3 : dtSeconds;
            ticksPerSecond = delta / dt;
        }
        return (ticksPerSecond * 60.0) / config.ticksPerRev;
    }
}
