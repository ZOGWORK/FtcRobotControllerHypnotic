package org.firstinspires.ftc.teamcode.hors.aim;


/**
 * Turns the whole chassis to face a field point. There is no turret, so aiming is chassis yaw.
 *
 * Inputs are the robot's field pose (inches, heading radians CCW from +X) and a target point.
 * Output is a yaw command for DriveController, whose yaw is POSITIVE CLOCKWISE, so a positive
 * (CCW) heading error gives a negative yaw command.
 *
 * Pure math, no hardware: the teleop decides whether the pose is trustworthy before using it.
 */
public class AimController {

    // Which way the shooter fires, relative to the robot's front, radians CCW.
    // 0 = the shooter fires out of the robot's front. NEEDS MEASUREMENT: confirm on the robot.
    public static double SHOOTER_FACING_OFFSET_RAD = 0.0; // PLACEHOLDER

    public static double kP = 1.2;                  // PLACEHOLDER: yaw command per radian of error
    public static double MAX_YAW = 0.7;             // PLACEHOLDER: cap on the aim yaw command
    public static double MIN_YAW = 0.06;            // PLACEHOLDER: static-friction floor outside tolerance
    public static double ON_TARGET_TOLERANCE_RAD = Math.toRadians(2.0); // PLACEHOLDER
    // Chassis turn rate at full yaw command, rad/s. Turns the bearing-rate feedforward into a yaw
    // command. PLACEHOLDER: NEEDS MEASUREMENT (spin in place at full yaw, read the Pinpoint).
    public static double MAX_TURN_RATE_RAD_PER_S = 5.0;

    // Ball-flight lead (leadAimPoint). The ball keeps the robot's velocity in flight, so a moving
    // robot aims at a point shifted against its motion.
    // Average HORIZONTAL (ground) speed of a shot from leaving the robot to reaching the cell, in/s,
    // relative to the robot, at the shooter's target rpm: horizontal distance / flight time.
    // PLACEHOLDER: NEEDS MEASUREMENT (film a shot from a known distance, or the sim's ShotModel:
    // exitSpeed(rpm) x cos(hood angle)). 0 = UNSET: lead is exactly zero until it is measured.
    public static double BALL_GROUND_SPEED_IN_PER_S = 0.0;
    // Largest heading change the lead may make, radians, so a velocity spike can never swing the
    // aim wildly. SET BY HAND (the shoot-while-moving spec's +/-10 deg).
    public static double MAX_LEAD_RAD = Math.toRadians(10.0);
    /** Fixed-point iterations of the lead solve (the spec's 3). */
    static final int LEAD_ITERATIONS = 3;

    private double lastHeadingErrorRad = 0.0;

    /** Field bearing from the robot to the target, radians CCW from +X. */
    public static double bearingTo(double robotX, double robotY, double targetX, double targetY) {
        return Math.atan2(targetY - robotY, targetX - robotX);
    }

    /** Heading the robot must hold so the shooter faces the target. */
    public static double desiredHeading(double robotX, double robotY, double targetX, double targetY) {
        return bearingTo(robotX, robotY, targetX, targetY) - SHOOTER_FACING_OFFSET_RAD;
    }

    /** Wraps an angle to (-pi, pi]. */
    public static double wrap(double rad) {
        double a = rad % (2.0 * Math.PI);
        if (a <= -Math.PI) a += 2.0 * Math.PI;
        if (a > Math.PI) a -= 2.0 * Math.PI;
        return a;
    }

    /**
     * Rate the field bearing to the target turns at because the robot is moving, rad/s CCW.
     * vx, vy: robot field velocity, in/s.
     */
    public static double bearingRate(double robotX, double robotY, double vx, double vy,
                                     double targetX, double targetY) {
        double rx = targetX - robotX;
        double ry = targetY - robotY;
        double r2 = rx * rx + ry * ry;
        if (r2 < 1.0) return 0.0; // on top of the target: the bearing is meaningless
        return (ry * vx - rx * vy) / r2;
    }

    /**
     * The point to aim at so a ball fired from a moving robot reaches the target, {x, y} inches.
     *
     * The ball's ground velocity is the robot's velocity plus its own ground speed along the aim,
     * so aim at {@code target - robotVelocity * flightTime}, where
     * {@code flightTime = distance(robot, aimPoint) / ballGroundSpeed}. Solved by fixed-point
     * iteration from the true target, {@link #LEAD_ITERATIONS} times. The shift is then limited
     * to {@link #MAX_LEAD_RAD} of bearing either side of the true target.
     *
     * Fails soft: returns the true target (zero lead) when the ball speed is not a positive
     * number, the velocity is not finite (treated as zero), the robot is on top of the target, or
     * anything comes out non-finite.
     *
     * @param vx              robot field velocity x, in/s
     * @param vy              robot field velocity y, in/s
     * @param ballGroundSpeed the ball's horizontal speed relative to the robot, in/s
     */
    public static double[] leadAimPoint(double robotX, double robotY, double vx, double vy,
                                        double targetX, double targetY, double ballGroundSpeed) {
        double[] target = {targetX, targetY};
        if (!Double.isFinite(ballGroundSpeed) || ballGroundSpeed <= 0.0) return target;
        if (!Double.isFinite(vx) || !Double.isFinite(vy)) return target;
        if (!Double.isFinite(robotX) || !Double.isFinite(robotY)) return target;
        if (Math.hypot(targetX - robotX, targetY - robotY) < 1.0) return target;

        double ax = targetX;
        double ay = targetY;
        for (int i = 0; i < LEAD_ITERATIONS; i++) {
            double flightTime = Math.hypot(ax - robotX, ay - robotY) / ballGroundSpeed;
            ax = targetX - vx * flightTime;
            ay = targetY - vy * flightTime;
        }

        // Limit the shift to MAX_LEAD_RAD of bearing, keeping the shifted point's range.
        double trueBearing = bearingTo(robotX, robotY, targetX, targetY);
        double lead = wrap(bearingTo(robotX, robotY, ax, ay) - trueBearing);
        double maxLead = Double.isFinite(MAX_LEAD_RAD) ? Math.abs(MAX_LEAD_RAD) : 0.0;
        if (Math.abs(lead) > maxLead) {
            double range = Math.hypot(ax - robotX, ay - robotY);
            double b = trueBearing + Math.signum(lead) * maxLead;
            ax = robotX + range * Math.cos(b);
            ay = robotY + range * Math.sin(b);
        }
        if (!Double.isFinite(ax) || !Double.isFinite(ay)) return target;
        return new double[] {ax, ay};
    }

    /**
     * Heading change the lead makes: bearing to the aim point minus bearing to the true target,
     * radians CCW. 0 when either is unusable.
     */
    public static double leadAngle(double robotX, double robotY, double targetX, double targetY,
                                   double aimX, double aimY) {
        double a = wrap(bearingTo(robotX, robotY, aimX, aimY) - bearingTo(robotX, robotY, targetX, targetY));
        return Double.isFinite(a) ? a : 0.0;
    }

    /** Yaw command for a stationary robot. */
    public double yawCommand(double robotX, double robotY, double robotHeading,
                             double targetX, double targetY) {
        return yawCommand(robotX, robotY, robotHeading, 0.0, 0.0, targetX, targetY);
    }

    /**
     * Yaw command (DriveController units, positive clockwise) that turns the robot toward the
     * target: proportional on the heading error, plus a feedforward for how fast the bearing is
     * turning because the robot is driving (so aim keeps up while the driver strafes). Also
     * records the heading error for telemetry.
     *
     * @param vx robot field velocity x, in/s
     * @param vy robot field velocity y, in/s
     */
    public double yawCommand(double robotX, double robotY, double robotHeading, double vx, double vy,
                             double targetX, double targetY) {
        double error = wrap(desiredHeading(robotX, robotY, targetX, targetY) - robotHeading);
        lastHeadingErrorRad = error;
        double ff = bearingRate(robotX, robotY, vx, vy, targetX, targetY) / MAX_TURN_RATE_RAD_PER_S;

        double ccw;
        if (Math.abs(error) <= ON_TARGET_TOLERANCE_RAD) {
            ccw = ff;
        } else {
            double p = kP * error;
            p = Math.signum(p) * Math.max(MIN_YAW, Math.abs(p));
            ccw = p + ff;
        }
        ccw = Math.max(-MAX_YAW, Math.min(MAX_YAW, ccw));
        return -ccw; // CCW-positive -> DriveController's clockwise-positive yaw
    }

    /** Heading error from the last {@link #yawCommand} call, radians CCW. */
    public double getLastHeadingErrorRad() {
        return lastHeadingErrorRad;
    }

    public boolean isOnTarget() {
        return Math.abs(lastHeadingErrorRad) <= ON_TARGET_TOLERANCE_RAD;
    }
}
