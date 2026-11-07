package org.firstinspires.ftc.teamcode.hors.tuning.autotune;

import com.aaravdhawan25.pidautotuner.ftc.PIDMaster;
import com.aaravdhawan25.pidautotuner.ftc.dashboard.AutoTuneTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.hors.shooter.ShooterController;
import org.json.JSONArray;
import org.json.JSONObject;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * FtcAutoTune's velocity PIDF tuner (https://ftcautotune.dev/) on one shooter, with the result
 * converted into ShooterController.Config units, ready to paste into ShooterConstants.
 *
 * ROBOT ON BLOCKS OR IN A CLEAR AREA, NO POLLEN LOADED. The relay test drives the flywheel
 * bang-bang, full forward and full reverse, around the target.
 *
 *   1. Relay test (~10-30 s): the wheel oscillates around the target RPM; Ku and Tu are measured.
 *   2. Feedforward sweep (~5 s): open loop at each FEEDFORWARD_TEST_POWERS level; kF is fitted.
 *      The battery voltage is averaged over this phase, so the feedforward can be scaled to 12 V.
 *   3. Six Ziegler-Nichols candidates in FtcAutoTune units, then the "classic ZN" one (the library's
 *      velocity default) converted for ShooterController. Hold gamepad1.A to live-test it with
 *      FtcAutoTune's own controller (no voltage scaling, no RPM filter), not ShooterController.
 *
 * ShooterController filters RPM and smooths power (rpmFilterAlpha, powerSmoothingAlpha); the relay
 * test sees neither, so the pasted gains can ring a little more than the live test showed. Check
 * the hold in HORS Clutch Teleop (dpad down spins it up; the Pollen line shows the rpm) before trusting them.
 *
 * Settings: hors/tuning/autotune/TuningConfig. Motor, direction, ticks/rev and target RPM come from
 * hors/shooter/ShooterConstants through the subclass.
 */
abstract class ShooterAutoTuneOpMode extends LinearOpMode {

    /** Index of "PD (classic ZN)" in FtcAutoTune's candidate list, its velocity default. */
    static final int CLASSIC_ZN = 4;

    abstract String label();

    /** Field name in ShooterConstants, for the printed paste lines. */
    abstract String field();

    abstract String motorName();

    abstract DcMotorSimple.Direction direction();

    abstract ShooterController.Config config();

    private VoltageSensor battery;

    @Override
    public void runOpMode() {
        telemetry = new AutoTuneTelemetry(hardwareMap, telemetry);
        try {
            battery = hardwareMap.voltageSensor.iterator().next();
        } catch (Exception ignored) {
        }

        ShooterController.Config cfg = config();
        double targetTicksPerSec = AutoTuneGains.rpmToTicksPerSec(cfg.targetRpm, cfg.ticksPerRev);

        telemetry.addLine("=== FtcAutoTune: " + label() + " shooter ===");
        telemetry.addLine("Robot on blocks, NO POLLEN loaded.");
        telemetry.addData("Motor", "%s (%s)", motorName(), direction());
        telemetry.addData("Target", "%.0f RPM (%.1f ticks/s), from ShooterConstants",
                cfg.targetRpm, targetTicksPerSec);
        telemetry.addLine("TuningConfig is read at START: edit it on Pulse now.");
        telemetry.addLine("Edits after START need a restart of this OpMode.");
        telemetry.addLine("Press START. The relay test runs automatically.");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        // Built after START so TuningConfig edits made during INIT are the ones the test runs with.
        PIDMaster pid = new PIDMaster(
                hardwareMap, motorName(), direction() == DcMotorSimple.Direction.REVERSE, false,
                targetTicksPerSec,
                AutoTuneGains.rpmToTicksPerSec(TuningConfig.VELOCITY_HYSTERESIS_RPM, cfg.ticksPerRev),
                TuningConfig.RELAY_AMPLITUDE, TuningConfig.CYCLES_TO_COLLECT, TuningConfig.CYCLES_TO_IGNORE,
                TuningConfig.RELAY_TEST_TIMEOUT_S, TuningConfig.FEEDFORWARD_TEST_POWERS,
                TuningConfig.FEEDFORWARD_SETTLE_TIME_S, TuningConfig.TUNE_INTEGRAL_TERM,
                cfg.ticksPerRev);
        // PIDMaster's relay timeout is checked against getRuntime(), which counts from INIT.
        resetRuntime();

        double voltSum = 0;
        int voltCount = 0;
        while (opModeIsActive() && !pid.isTuningComplete()) {
            pid.tuningStep(getRuntime());
            if ("FEEDFORWARD_SWEEP".equals(pid.getSnapshot().optString("phase"))) {
                double v = volts();
                if (v > 0) {
                    voltSum += v;
                    voltCount++;
                }
            }
            for (String line : pid.getTelemetryLines()) telemetry.addLine(line);
            telemetry.update();
        }

        List<String> paste = new ArrayList<>();
        if (pid.isTuningSuccessful()) {
            paste = pasteLines(pid.getSnapshot(), cfg, voltCount > 0 ? voltSum / voltCount : Double.NaN);
            for (String l : paste) RobotLog.ii("ShooterAutoTune", l);
        }

        while (opModeIsActive()) {
            if (!pid.isTuningSuccessful()) {
                for (String line : pid.getTelemetryLines()) telemetry.addLine(line);
            } else {
                for (String line : pid.getResultTelemetryLines()) telemetry.addLine(line);
                telemetry.addLine();
                telemetry.addLine("=== Paste into ShooterConstants (also in the robot log, tag ShooterAutoTune) ===");
                for (String l : paste) telemetry.addLine(l);
                if (gamepad1.a) {
                    telemetry.addLine();
                    for (String line : pid.liveTestStep(getRuntime())) telemetry.addLine(line);
                } else {
                    pid.stopLiveTest();
                }
            }
            telemetry.update();
        }
        pid.stop();
    }

    private List<String> pasteLines(JSONObject snap, ShooterController.Config cfg, double vSweep) {
        List<String> out = new ArrayList<>();
        JSONArray candidates = snap.optJSONArray("candidates");
        double kF = snap.optDouble("kF", Double.NaN);
        if (candidates == null || candidates.length() <= CLASSIC_ZN || !(kF > 0) || !(vSweep > 0)) {
            out.add("// " + label() + ": no usable kF or battery reading from the sweep; nothing to paste.");
            return out;
        }
        JSONObject c = candidates.optJSONObject(CLASSIC_ZN);
        AutoTuneGains.Converted g = AutoTuneGains.convert(c.optDouble("kP"), c.optDouble("kI"),
                c.optDouble("kD"), kF, cfg.ticksPerRev, vSweep);
        out.add(String.format(Locale.ROOT,
                "// %s: CALIBRATED FROM ROBOT by FtcAutoTune v1.4.3, %s candidate, Ku %.6f Tu %.4f s, "
                        + "kF sweep at %.2f V; kS not measured",
                label(), c.optString("label"), snap.optDouble("ku"), snap.optDouble("tu"), vSweep));
        out.add(String.format(Locale.ROOT, "%s.ffReferenceVoltage = %.1f;", field(), AutoTuneGains.V_REF));
        out.add(String.format(Locale.ROOT, "%s.ffReferenceMaxTicksPerSec = %.1f;", field(), g.ffReferenceMaxTicksPerSec));
        out.add(String.format(Locale.ROOT, "%s.kF = 1.0;", field()));
        out.add(String.format(Locale.ROOT, "%s.kS = 0.0;", field()));
        out.add(String.format(Locale.ROOT, "%s.kP = %.6f;", field(), g.kP));
        out.add(String.format(Locale.ROOT, "%s.kI = %.6f;", field(), g.kI));
        out.add(String.format(Locale.ROOT, "%s.kD = %.6f;", field(), g.kD));
        return out;
    }

    private double volts() {
        try {
            if (battery != null) return battery.getVoltage();
        } catch (Exception ignored) {
        }
        return 0;
    }
}
