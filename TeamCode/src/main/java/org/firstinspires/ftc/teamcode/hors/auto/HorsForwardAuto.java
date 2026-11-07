package org.firstinspires.ftc.teamcode.hors.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Drives forward for DRIVE_TIME_S, then stops and stays stopped until the end of auto.
 *
 * DRIVE ONLY: it looks up the four drive motors and nothing else, so the shooters, intake, gates,
 * claws and every other mechanism are never commanded.
 *
 * Motor names and directions are the same as Teleop and HORS Clutch Teleop. There, pushing the
 * stick forward sends NEGATIVE power to all four motors, so forward here is negative power too.
 * If the robot drives backwards, set FORWARD_SIGN to 1.
 */
@Autonomous(name = "HORS Forward 1s", group = "Auto")
public class HorsForwardAuto extends LinearOpMode {

    /** How long to drive, seconds. SET BY HAND (the team, 2026-11-07). */
    public static double DRIVE_TIME_S = 1.0;

    /** Drive power while moving, 0..1. PLACEHOLDER: half power, so the distance is gentle and repeatable. */
    public static double DRIVE_POWER = 0.5;

    /** -1 = negative power is forward, as in Teleop's mixing. Set to 1 if the robot drives backwards. */
    public static double FORWARD_SIGN = -1.0;

    @Override
    public void runOpMode() {
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        DcMotor[] drive = {frontLeft, frontRight, backLeft, backRight};
        for (DcMotor m : drive) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);   // stop where it is, no coasting
            m.setPower(0.0);
        }

        telemetry.addData("Auto", "forward %.1f s at %.0f%% power, then stop. Drive only.",
                DRIVE_TIME_S, DRIVE_POWER * 100);
        telemetry.update();

        waitForStart();
        if (isStopRequested()) return;

        double power = FORWARD_SIGN * DRIVE_POWER;
        for (DcMotor m : drive) m.setPower(power);

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < DRIVE_TIME_S) {
            telemetry.addData("Auto", "driving forward: %.2f / %.1f s", timer.seconds(), DRIVE_TIME_S);
            telemetry.update();
        }

        for (DcMotor m : drive) m.setPower(0.0);

        while (opModeIsActive()) {
            telemetry.addData("Auto", "done, stopped");
            telemetry.update();
            idle();
        }
    }
}
