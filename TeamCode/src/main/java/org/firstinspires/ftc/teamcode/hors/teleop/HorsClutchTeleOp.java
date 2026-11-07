package org.firstinspires.ftc.teamcode.hors.teleop;

import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.pedropathing.math.Velocity;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Locale;

import org.firstinspires.ftc.teamcode.hors.HorsLocalizer;
import org.firstinspires.ftc.teamcode.hors.aim.AimController;
import org.firstinspires.ftc.teamcode.hors.aim.AimVelocityFilter;
import org.firstinspires.ftc.teamcode.hors.aim.Alliance;
import org.firstinspires.ftc.teamcode.hors.aim.FieldTargets;
import org.firstinspires.ftc.teamcode.hors.aim.PoseSanity;
import org.firstinspires.ftc.teamcode.hors.shooter.ClawKick;
import org.firstinspires.ftc.teamcode.hors.shooter.ShooterConstants;
import org.firstinspires.ftc.teamcode.hors.shooter.ShooterController;
import org.firstinspires.ftc.teamcode.subsystems.shooting.ClawController;
import org.firstinspires.ftc.teamcode.subsystems.shooting.GateController;

/**
 * HORS Clutch Teleop: a helper teleop with BioHORS's (team 33950) odometry auto-aim, outtake and
 * pollen flywheel control on this robot. The main Teleop is unchanged.
 *
 * Gates and the intake sequence are this robot's own (subsystems/shooting/GateController,
 * unchanged; its timing is MAINHORS26's). The claw runs MAINHORS26's kick (hors/shooter/ClawKick):
 * rest, kick for ClawController.closeMs, back to rest, so every Y kicks. The pollen claw rests at
 * POLLEN_CLAW_REST_POS (0.0) and kicks up to POLLEN_CLAW_KICK_POS (0.95), both set here, so the
 * main Teleop's ClawController (unchanged) is not affected. Only the pollen claw kicks; the nectar
 * claw is held at ClawController.openPos.
 *
 * Controls (gamepad1; gamepad2 mirrors the mechanism buttons):
 *   Left stick ............ drive (forward / strafe), the same mixing as Teleop
 *   Right stick X ......... turn (overridden while aiming)
 *   Left trigger (hold) ... AIM + POSITION HOLD (gamepad1): turn the robot to face our hive's cell on
 *                           this half of the field, and hold the spot where the trigger was pressed:
 *                           the left stick is ignored, the drive brakes, and if a defender pushes the
 *                           robot the Pinpoint drives it back (HOLD_*). Let go to drive again.
 *   Right bumper (hold) ... slow mode (0.45x). While aiming: nudge the aim RIGHT one step instead
 *   Left bumper ........... while aiming, nudge the aim LEFT one step. The nudge stays until an X reset.
 *   Right trigger ......... intake in
 *   A (hold) .............. OUTTAKE: both intake motors run out. Wins over right trigger.
 *   Y ..................... SHOOT SEQUENCE: once the pollen shooter is at speed (or after
 *                           SPINUP_TIMEOUT_S) run MAINHORS26's Y sequence: gates open and the
 *                           intake feeds for 1050 ms, the pollen claw kicks 400 ms before the end and returns
 *                           to rest 300 ms later, then the gates close. Y again during spin-up cancels.
 *   B ..................... toggle both gates open / closed
 *   Dpad down ............. nothing (the pollen shooter is always on)
 *
 * POLLEN SHOOTER ALWAYS ON: it starts spinning up at PLAY and runs at its target until the OpMode
 * stops. No button turns it off, so every Y fires as soon as the wheel is at speed.
 *
 *   Dpad left / right ..... pollen target RPM - / + (ShooterConstants.SHOOTER_TRIM_STEP_RPM), gamepad1 only
 *   X (hold) .............. RESET the pose: robot pushed into the alliance's corner beyond the loading
 *                           zone (RED: north-west), back on the side wall, side on the end wall, intake
 *                           facing the field; hold X for RESET_HOLD_S. The pad rumbles.
 *
 * Before PLAY: dpad left = RED, dpad right = BLUE.
 *
 * Aim uses the Pinpoint pose (hors/HorsLocalizer, BioHORS's field frame) and the hive's fixed field
 * coordinates; there is no camera. If the Pinpoint is not READY, the pose is not a number, or the
 * pose is impossible (off the field, or jumped), aiming falls back to manual turn until X resets it.
 *
 * The nectar shooter is OFF: it does not work on this robot, so this teleop never looks it up or
 * powers it. GateController still opens both gates during Y (it is unchanged).
 */
@TeleOp(name = "HORS Clutch Teleop", group = "Teleop")
public class HorsClutchTeleOp extends LinearOpMode {

    private static final double NORMAL_DRIVE_SCALE = 1.0;
    private static final double SLOW_DRIVE_SCALE = 0.45;
    private static final double TRIGGER_THRESHOLD = 0.1;
    private static final double RUMBLE_COOLDOWN_S = 1.2;

    /** Intake power in, and out (A). Teleop's INTAKE_POWER, and its opposite. */
    public static double INTAKE_POWER = 1.0;
    public static double OUTTAKE_POWER = -1.0;

    /**
     * Turn direction fix for this robot's drive. SET BY HAND (the team, 2026-11-07: turning was
     * reversed with Teleop's mixing as it is, so -1). It flips the right stick AND the aim, since
     * both feed the same turn.
     */
    public static double TURN_SIGN = -1.0;

    /**
     * Strafe direction fix. SET BY HAND (the team, 2026-11-07: the mecanum wheels are mounted the
     * wrong way round, so strafing was reversed; -1). Forward / backward and turning are not touched.
     * Set back to 1 if the wheels are swapped to the right corners.
     */
    public static double STRAFE_SIGN = -1.0;

    /**
     * The aim's turn command, on top of TURN_SIGN: positive turns the robot CLOCKWISE. With the
     * stick turning the right way, this should stay 1. Only if holding left trigger turns the robot
     * AWAY from the hive (and the stick turns correctly) set it to -1.
     */
    public static double AIM_YAW_SIGN = 1.0;

    /**
     * Where the pollen claw goes on a kick, servo position 0..1. SET BY HAND (the team, 2026-11-07:
     * "really high"; ClawController's kick is 0.62). Lower it if the claw hits
     * something or the servo buzzes at the top.
     */
    public static double POLLEN_CLAW_KICK_POS = 0.95;

    /**
     * Where the pollen claw rests between kicks, servo position 0..1. SET BY HAND (the team,
     * 2026-11-07: 0.0). Only HORS Clutch Teleop uses it; ClawController.closePos (0.3) is left
     * as it is for the main Teleop.
     */
    public static double POLLEN_CLAW_REST_POS = 0.0;

    // ===== Position hold (left trigger held) =====
    /** false: holding left trigger only stops and brakes, with no push back. */
    public static boolean POSITION_HOLD_ENABLED = true;
    /** Push-back power per inch off the held spot. PLACEHOLDER: raise it if a shove still moves the
     *  robot, lower it if the robot wobbles around the spot. */
    public static double HOLD_KP = 0.08;
    /** Most power the hold may use to push back, 0..1. PLACEHOLDER. */
    public static double HOLD_MAX_POWER = 0.6;
    /** Closer than this to the spot, inches: no push (brake only), so it does not jitter. SET BY HAND. */
    public static double HOLD_DEADBAND_IN = 0.5;
    /** Further than this from the spot, inches: stop pushing and just brake. A shove does not move a
     *  braked robot this far, so it means a bad pose or a wrong HOLD_SIGN. SET BY HAND. */
    public static double HOLD_GIVE_UP_IN = 18.0;
    /** NEEDS CHECKING on the robot: hold the trigger, push the robot a few inches. It should drive
     *  BACK. If it drives further away (and then gives up at HOLD_GIVE_UP_IN), set this to -1. */
    public static double HOLD_SIGN = 1.0;

    /** Longest wait for the pollen shooter to reach speed before Y shoots anyway, seconds. CARRIED OVER (BioHORS). */
    public static double SPINUP_TIMEOUT_S = 2.0;

    // Where the robot is assumed to start (RED; BLUE is the point mirror). CARRIED OVER (BioHORS's
    // park spot): by the west wall next to the loading zone, intake facing the field. Press X at the
    // reset spot before relying on aim.
    public static double DEFAULT_START_X_IN = -61.5;
    public static double DEFAULT_START_Y_IN = 49.0;
    public static double DEFAULT_START_HEADING_RAD = 0.0;

    // Where X resets the pose to (RED; BLUE is the point mirror): pushed into the north-west corner,
    // back flat on the west wall, left side flat on the north wall, intake facing the field.
    // CARRIED OVER from BioHORS (from FIRST's CAD and BioHORS's 7.8 in back / 17.84 in width).
    // SET FOR THIS ROBOT: x = -70.674 + (centre to back, in); y = 70.674 - (half the width, in).
    public static double RESET_X_IN = -62.87;
    public static double RESET_Y_IN = 61.75;
    public static double RESET_HEADING_RAD = 0.0;

    /** Aim velocity smoothing: 1 = raw, smaller = smoother but laggier. CARRIED OVER. */
    public static double AIM_VELOCITY_ALPHA = 0.5;
    private static final double MIN_VELOCITY_DT_S = 1e-4;
    private static final double MAX_VELOCITY_DT_S = 0.25;
    /** One bumper press while aiming turns the aim this much, radians. CARRIED OVER. */
    public static double AIM_NUDGE_STEP_RAD = Math.toRadians(1.0);
    /** The nudge never goes past this either way, radians. CARRIED OVER. */
    public static double AIM_NUDGE_MAX_RAD = Math.toRadians(10.0);
    /** How long X must be held to reset the pose, seconds. CARRIED OVER. */
    public static double RESET_HOLD_S = 0.5;

    private DcMotor frontLeft, frontRight, backLeft, backRight, intakeLeft, intakeRight;
    private DcMotorEx pollenMotor;
    private ShooterController pollen;
    private GateController gateController;
    private ClawKick pollenKick;
    private GoBildaPinpointDriver pinpoint;
    private Localizer localizer;
    private final AimController aim = new AimController();

    private Alliance alliance = Alliance.RED;
    private FieldTargets.Cell targetCell = FieldTargets.upCellAtKickoff(Alliance.RED);
    private PoseSanity poseSanity;
    private double aimNudgeRad = 0.0;
    private double pollenTrimRpm = 0.0;

    private boolean dpadLeftLast, dpadRightLast, bLast, xLast, yLast, lbLast, rbLast;

    private enum Shoot { IDLE, SPIN_UP, SEQUENCE }

    // Position hold (left trigger): the spot it holds, and its state.
    private boolean aimHeldLast = false;
    private boolean holdSet = false;
    private boolean holdGaveUp = false;
    private double holdX, holdY;
    private double holdErrorIn = Double.NaN;

    @Override
    public void runOpMode() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        // Drive: the same motors and directions as Teleop.
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        // BRAKE, as BioHORS's drive: at zero power the motors resist being turned, so the robot stops
        // where it is instead of coasting, and is harder to push.
        for (DcMotor m : new DcMotor[] {frontLeft, frontRight, backLeft, backRight}) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        intakeLeft = hardwareMap.get(DcMotor.class, "intakeLeft");
        intakeRight = hardwareMap.get(DcMotor.class, "intakeRight");
        intakeLeft.setDirection(DcMotor.Direction.FORWARD);
        intakeRight.setDirection(DcMotor.Direction.FORWARD);

        VoltageSensor battery = null;
        try {
            battery = hardwareMap.voltageSensor.iterator().next();
        } catch (Exception ignored) {
        }
        // Closed loop on the pollen motor's ENCODER: every update() reads getVelocity() (encoder
        // ticks/s), converts it to flywheel rpm and runs the PIDF on the rpm error. The mode is
        // RUN_WITHOUT_ENCODER only so the hub's own velocity PID does not fight ours; the encoder
        // is still read every loop. A dead or backwards encoder shows on the Pollen line.
        pollenMotor = hardwareMap.get(DcMotorEx.class, ShooterConstants.POLLEN_SHOOTER_NAME);
        pollenMotor.setDirection(ShooterConstants.POLLEN_SHOOTER_DIRECTION);
        pollen = new ShooterController(ShooterConstants.POLLEN_SHOOTER, pollenMotor, telemetry, battery);

        Servo pollenClaw = hardwareMap.get(Servo.class, "pollenClaw");
        Servo nectarClaw = hardwareMap.get(Servo.class, "nectarClaw");
        Servo pollenGate = hardwareMap.get(Servo.class, "pollenGate");
        Servo nectarGate = hardwareMap.get(Servo.class, "nectarGate");
        // This robot's claw positions (ClawController's init and trigger), MAINHORS26's kick logic.
        pollenKick = new ClawKick(pollenClaw, POLLEN_CLAW_REST_POS, POLLEN_CLAW_KICK_POS, ClawController.closeMs);
        nectarClaw.setPosition(ClawController.openPos);   // nectar is off: its claw stays at rest
        // GateController takes two flywheel references but never uses them: both null.
        gateController = new GateController(pollenGate, nectarGate, intakeLeft, intakeRight, null, null);
        gateController.setGateClosed(true);   // as MAINHORS26 starts: gates closed

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, HorsLocalizer.PINPOINT_NAME);
        localizer = HorsLocalizer.create(hardwareMap);
        localizer.setPose(defaultStartPose(alliance));
        AimVelocityFilter velocityFilter = new AimVelocityFilter(AIM_VELOCITY_ALPHA, HorsLocalizer.MAX_SPEED_IN_PER_S);
        poseSanity = new PoseSanity(HorsLocalizer.MAX_SPEED_IN_PER_S);

        // ----- INIT: pick the alliance -----
        while (!isStarted() && !isStopRequested()) {
            Alliance picked = null;
            if (gamepad1.dpad_left) picked = Alliance.RED;
            if (gamepad1.dpad_right) picked = Alliance.BLUE;
            if (picked != null && picked != alliance) {
                alliance = picked;
                targetCell = FieldTargets.upCellAtKickoff(alliance);
                localizer.setPose(defaultStartPose(alliance));
            }
            seedEdgeDetectors();
            localizer.update();
            telemetry.addData("Alliance", "%s  (dpad left = RED, dpad right = BLUE)", alliance);
            telemetry.addData("Pinpoint", pinpointReady() ? "READY" : "NOT READY");
            addPoseTelemetry();
            telemetry.addData("Pollen", "target %.0f rpm", pollen.getTargetRpm());
            telemetry.addLine("Press X at the reset corner after PLAY before trusting aim.");
            telemetry.update();
        }
        if (isStopRequested()) return;

        ElapsedTime loopTimer = new ElapsedTime();
        ElapsedTime sincePlay = new ElapsedTime();
        ElapsedTime xHold = new ElapsedTime();
        ElapsedTime shootTimer = new ElapsedTime();
        ElapsedTime sinceRumble = new ElapsedTime();
        boolean rumbledOnce = false;
        boolean xArmed = false;
        boolean pollenFaultLast = false;
        Shoot shoot = Shoot.IDLE;
        Pose lastPose = localizer.pose();

        while (opModeIsActive()) {
            long nowMs = System.currentTimeMillis();
            localizer.update();
            Pose pose = localizer.pose();
            double dt = loopTimer.seconds();
            loopTimer.reset();
            if (poseUsable(pose)) poseSanity.check(pose.x(), pose.y(), sincePlay.seconds());
            boolean poseOk = poseUsable(pose) && poseSanity.fault().isEmpty();

            // ----- Field velocity: the Pinpoint's own, else the pose change; smoothed -----
            double rawVx = Double.NaN;
            double rawVy = Double.NaN;
            if (poseOk) {
                Velocity pv = pinpointVelocity();
                if (pv != null && Double.isFinite(pv.vx) && Double.isFinite(pv.vy)) {
                    rawVx = pv.vx;
                    rawVy = pv.vy;
                } else if (lastPose != null && dt > MIN_VELOCITY_DT_S && dt < MAX_VELOCITY_DT_S
                        && Double.isFinite(lastPose.x()) && Double.isFinite(lastPose.y())) {
                    rawVx = (pose.x() - lastPose.x()) / dt;
                    rawVy = (pose.y() - lastPose.y()) / dt;
                }
            }
            double[] v = velocityFilter.update(rawVx, rawVy);
            double vx = v[0];
            double vy = v[1];
            lastPose = pose;

            // ----- Target: our hive's cell on the robot's half of the field -----
            if (poseOk) targetCell = FieldTargets.cellForHalf(pose.y(), targetCell);
            double[] cell = FieldTargets.cellAimPoint(alliance, targetCell);
            double[] aimPoint = poseOk
                    ? AimController.leadAimPoint(pose.x(), pose.y(), vx, vy, cell[0], cell[1],
                            AimController.BALL_GROUND_SPEED_IN_PER_S)
                    : cell;

            // ----- Aim nudge: bumpers while aiming -----
            boolean aimHeld = gamepad1.left_trigger > TRIGGER_THRESHOLD;
            boolean lbNow = gamepad1.left_bumper || gamepad2.left_bumper;
            boolean rbNow = gamepad1.right_bumper || gamepad2.right_bumper;
            if (aimHeld) {
                if (lbNow && !lbLast) aimNudgeRad = Math.min(AIM_NUDGE_MAX_RAD, aimNudgeRad + AIM_NUDGE_STEP_RAD);
                if (rbNow && !rbLast) aimNudgeRad = Math.max(-AIM_NUDGE_MAX_RAD, aimNudgeRad - AIM_NUDGE_STEP_RAD);
            }
            lbLast = lbNow;
            rbLast = rbNow;
            double headingErrorRad = poseOk
                    ? AimController.wrap(AimController.desiredHeading(pose.x(), pose.y(), aimPoint[0], aimPoint[1])
                            + aimNudgeRad - pose.heading())
                    : Double.NaN;

            // ----- Drive: Teleop's mixing; aim replaces the turn stick while left trigger is held -----
            boolean slowHeld = gamepad1.right_bumper && !aimHeld;
            double scale = slowHeld ? SLOW_DRIVE_SCALE : NORMAL_DRIVE_SCALE;
            double y = gamepad1.left_stick_y * scale;
            double x = gamepad1.left_stick_x * scale;
            double rx = gamepad1.right_stick_x * scale;
            boolean aiming = aimHeld && poseOk;

            // ----- Position hold while left trigger is held: the left stick is ignored, the drive
            // motors BRAKE, and the Pinpoint pushes the robot back to where the trigger was pressed,
            // so a defender cannot shove it off its spot. Turning stays with the aim. -----
            if (aimHeld && !aimHeldLast) {
                holdSet = poseOk;
                holdGaveUp = false;
                if (poseOk) {
                    holdX = pose.x();
                    holdY = pose.y();
                }
            }
            aimHeldLast = aimHeld;
            holdErrorIn = Double.NaN;
            if (aimHeld) {
                y = 0.0;   // stop: no driving while holding
                x = 0.0;
                if (POSITION_HOLD_ENABLED && holdSet && poseOk && !holdGaveUp) {
                    double ex = holdX - pose.x();
                    double ey = holdY - pose.y();
                    holdErrorIn = Math.hypot(ex, ey);
                    if (holdErrorIn > HOLD_GIVE_UP_IN) {
                        // Pushed (or driving) further than a shove could: something is wrong (pose or
                        // sign), so stop pushing and just brake until the trigger is let go.
                        holdGaveUp = true;
                    } else if (holdErrorIn > HOLD_DEADBAND_IN) {
                        // Field error into the robot's frame: forward along the heading, left 90 deg CCW.
                        double h = pose.heading();
                        double fwdErr = ex * Math.cos(h) + ey * Math.sin(h);
                        double leftErr = -ex * Math.sin(h) + ey * Math.cos(h);
                        double fwd = clamp(HOLD_KP * fwdErr, HOLD_MAX_POWER);
                        double right = clamp(-HOLD_KP * leftErr, HOLD_MAX_POWER);
                        // Teleop's mixing: negative y drives forward; positive x strafes right.
                        y = HOLD_SIGN * -fwd;
                        x = HOLD_SIGN * right;
                    }
                }
            }

            if (aiming) {
                // Clockwise-positive, like the right stick. Yaw has priority: translation gets only
                // the headroom the aim leaves.
                rx = AIM_YAW_SIGN * aim.yawCommand(pose.x(), pose.y(), pose.heading() - aimNudgeRad, vx, vy,
                        aimPoint[0], aimPoint[1]);
                double room = 1.0 - Math.abs(rx);
                double used = Math.abs(x) + Math.abs(y);
                if (used > room && used > 1e-9) {
                    x *= room / used;
                    y *= room / used;
                }
            }
            drive(y, x, rx);

            // ----- Y: shoot sequence. Spin up, then this robot's gate / intake / claw sequence -----
            boolean yNow = gamepad1.y || gamepad2.y;
            if (yNow && !yLast) {
                if (shoot == Shoot.IDLE && !gateController.isBusy()) {
                    shoot = Shoot.SPIN_UP;
                    shootTimer.reset();
                } else if (shoot == Shoot.SPIN_UP) {
                    shoot = Shoot.IDLE;   // cancel before the gates open
                }
            }
            yLast = yNow;
            if (shoot == Shoot.SPIN_UP && (pollen.isAtTarget() || shootTimer.seconds() >= SPINUP_TIMEOUT_S)) {
                gateController.startIntakeSequence(nowMs);
                shoot = Shoot.SEQUENCE;
            }
            // MAINHORS26: the gate sequence says when to kick (400 ms before its end).
            if (gateController.update(nowMs)) pollenKick.trigger(nowMs);
            pollenKick.update(nowMs);
            if (shoot == Shoot.SEQUENCE && !gateController.isBusy()) shoot = Shoot.IDLE;

            // ----- B: toggle the gates (ignored while the sequence runs) -----
            boolean bNow = gamepad1.b || gamepad2.b;
            if (bNow && !bLast && shoot == Shoot.IDLE) gateController.toggleGate();
            bLast = bNow;

            // ----- Pollen shooter: ALWAYS ON from PLAY to the end (the team, 2026-11-07). No button
            // turns it off; dpad down does nothing. -----
            boolean pollenOn = true;
            pollen.setShooterOn(pollenOn);

            boolean dpadLeftNow = gamepad1.dpad_left;
            if (dpadLeftNow && !dpadLeftLast) pollenTrimRpm -= ShooterConstants.SHOOTER_TRIM_STEP_RPM;
            dpadLeftLast = dpadLeftNow;
            boolean dpadRightNow = gamepad1.dpad_right;
            if (dpadRightNow && !dpadRightLast) pollenTrimRpm += ShooterConstants.SHOOTER_TRIM_STEP_RPM;
            dpadRightLast = dpadRightNow;
            pollenTrimRpm = Math.max(-ShooterConstants.POLLEN_SHOOTER.targetRpm, pollenTrimRpm);
            pollen.setTargetRpm(ShooterConstants.POLLEN_SHOOTER.targetRpm + pollenTrimRpm);
            pollen.update();

            if (pollenOn && pollen.justReachedTarget()
                    && (!rumbledOnce || sinceRumble.seconds() >= RUMBLE_COOLDOWN_S)) {
                gamepad1.rumble(220);
                gamepad2.rumble(220);
                sinceRumble.reset();
                rumbledOnce = true;
            }
            boolean pollenFault = pollen.encoderFault();
            if (pollenFault && !pollenFaultLast) {
                gamepad1.rumble(1.0, 0.0, 1000);
                gamepad2.rumble(1.0, 0.0, 1000);
            }
            pollenFaultLast = pollenFault;

            // ----- X held: reset the pose at the reset corner -----
            boolean xNow = gamepad1.x || gamepad2.x;
            if (xNow && !xLast) {
                xHold.reset();
                xArmed = true;
            }
            if (!xNow) xArmed = false;
            if (xArmed && xHold.seconds() >= RESET_HOLD_S) {
                xArmed = false;
                localizer.setPose(resetPose(alliance));
                lastPose = null;
                velocityFilter.reset();
                poseSanity.reset();
                aimNudgeRad = 0.0;
                holdSet = false;            // the held spot was in the old pose: forget it
                gamepad1.rumble(120);
            }
            xLast = xNow;

            // ----- Intake: right trigger in, A out (A wins). The gate sequence drives it while it runs. -----
            if (!gateController.isBusy()) {
                double intakePower = 0.0;
                if (gamepad1.a || gamepad2.a) {
                    intakePower = OUTTAKE_POWER;
                } else if (gamepad1.right_trigger > TRIGGER_THRESHOLD || gamepad2.right_trigger > TRIGGER_THRESHOLD) {
                    intakePower = INTAKE_POWER;
                }
                intakeLeft.setPower(intakePower);
                intakeRight.setPower(intakePower);
            }

            // ----- Telemetry -----
            String fire = !poseOk ? "AIM OFF: pose invalid"
                    : Math.abs(headingErrorRad) <= AimController.ON_TARGET_TOLERANCE_RAD ? "ON TARGET"
                    : String.format(Locale.ROOT, "aim off by %.1f deg", Math.toDegrees(headingErrorRad));
            if (shoot == Shoot.SPIN_UP) fire += " | Y: SPINNING UP";
            if (shoot == Shoot.SEQUENCE) fire += " | Y: SHOOTING";
            if (!gateController.isGateClosed()) fire += " | GATES OPEN";
            if (pollenFault) fire += " | POLLEN ENCODER FAULT";
            telemetry.addData("Fire", fire);
            telemetry.addData("Target", "%s %s cell", alliance, targetCell);
            if (aimHeld) {
                telemetry.addData("Hold", !POSITION_HOLD_ENABLED ? "brake only (POSITION_HOLD_ENABLED off)"
                        : !holdSet || !poseOk ? "brake only (no valid pose)"
                        : holdGaveUp ? "GAVE UP: too far off the spot, brake only (check HOLD_SIGN)"
                        : String.format(Locale.ROOT, "holding, %.1f in off the spot", holdErrorIn));
            }
            addPoseTelemetry();
            addShooterTelemetry(pollenOn);
            telemetry.update();
        }

        pollen.setShooterOn(false);
        pollen.update();
        drive(0, 0, 0);
        intakeLeft.setPower(0);
        intakeRight.setPower(0);
    }

    /**
     * Teleop's mecanum mixing, with the turn flipped by TURN_SIGN and the strafe by STRAFE_SIGN: y is the raw left stick y, and
     * rx positive (right stick right, or the aim turning right) turns the robot clockwise.
     */
    private void drive(double y, double x, double rx) {
        rx *= TURN_SIGN;
        x *= STRAFE_SIGN;
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        frontLeft.setPower((y + x + rx) / denominator);
        backLeft.setPower((y - x + rx) / denominator);
        frontRight.setPower((y - x - rx) / denominator);
        backRight.setPower((y + x - rx) / denominator);
    }

    private static double clamp(double v, double max) {
        return Math.max(-max, Math.min(max, v));
    }

    private void seedEdgeDetectors() {
        dpadLeftLast = gamepad1.dpad_left;
        dpadRightLast = gamepad1.dpad_right;
        bLast = gamepad1.b || gamepad2.b;
        xLast = gamepad1.x || gamepad2.x;
        yLast = gamepad1.y || gamepad2.y;
        lbLast = gamepad1.left_bumper || gamepad2.left_bumper;
        rbLast = gamepad1.right_bumper || gamepad2.right_bumper;
    }

    /** The pose X resets to: the RED value, point-mirrored for BLUE. */
    static Pose resetPose(Alliance alliance) {
        if (alliance == Alliance.RED) return new Pose(RESET_X_IN, RESET_Y_IN, RESET_HEADING_RAD);
        return new Pose(-RESET_X_IN, -RESET_Y_IN, AimController.wrap(RESET_HEADING_RAD + Math.PI));
    }

    /** The default start pose: the RED value, point-mirrored for BLUE. */
    static Pose defaultStartPose(Alliance alliance) {
        if (alliance == Alliance.RED) return new Pose(DEFAULT_START_X_IN, DEFAULT_START_Y_IN, DEFAULT_START_HEADING_RAD);
        return new Pose(-DEFAULT_START_X_IN, -DEFAULT_START_Y_IN, AimController.wrap(DEFAULT_START_HEADING_RAD + Math.PI));
    }

    private boolean pinpointReady() {
        try {
            return pinpoint != null && pinpoint.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY;
        } catch (Exception e) {
            return false;
        }
    }

    private boolean poseUsable(Pose pose) {
        return pinpointReady() && pose != null
                && Double.isFinite(pose.x()) && Double.isFinite(pose.y()) && Double.isFinite(pose.heading());
    }

    private Velocity pinpointVelocity() {
        try {
            return localizer.velocity();
        } catch (Exception e) {
            return null;
        }
    }

    private void addPoseTelemetry() {
        Pose p = localizer.pose();
        String why = !poseUsable(p) ? "(INVALID)"
                : poseSanity != null && !poseSanity.fault().isEmpty()
                        ? "(INVALID: " + poseSanity.fault() + ", hold X at the reset spot)" : "";
        String nudge = aimNudgeRad == 0.0 ? ""
                : String.format(Locale.ROOT, " nudge %+.0f deg", Math.toDegrees(aimNudgeRad));
        telemetry.addData("Pose", "x=%.1f y=%.1f h=%.1f deg%s %s",
                p.x(), p.y(), Math.toDegrees(p.heading()), nudge, why);
    }

    private void addShooterTelemetry(boolean on) {
        String state = pollen.encoderReversed() ? "ENCODER REVERSED? (reads backwards)"
                : pollen.encoderFault() ? "ENCODER? (no speed reading)"
                : !on ? (pollen.getCurrentRpm() > ShooterConstants.SHOOTER_COAST_SHOW_RPM ? "OFF, coasting" : "OFF")
                : pollen.isAtTarget() ? "OK" : "SPINNING UP";
        telemetry.addData("Pollen", "%.0f / %.0f rpm %s (trim %+.0f)",
                pollen.getCurrentRpm(), pollen.getTargetRpm(), state, pollenTrimRpm);
        // The raw encoder the loop runs on: if these stay at 0 while the wheel spins, the encoder
        // cable is unplugged or on the wrong port.
        telemetry.addData("Pollen encoder", "%d ticks, %.0f ticks/s, power %.2f",
                pollenMotor.getCurrentPosition(), pollenMotor.getVelocity(), pollen.getLastAppliedPower());
    }
}
