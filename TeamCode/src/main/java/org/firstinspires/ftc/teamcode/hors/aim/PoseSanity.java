package org.firstinspires.ftc.teamcode.hors.aim;


/**
 * Catches a pose the robot cannot really have. The Pinpoint can report READY and still give a bad
 * pose (a pod knocked loose, a glitch, a wrong reset), and the teleop would aim and fire on it.
 *
 * A pose fails when it is more than OFF_FIELD_MARGIN_IN outside the field, or when it moved further
 * since the last pose it checked than the robot can drive in that time (top speed x elapsed x
 * JUMP_SPEED_FACTOR + JUMP_MARGIN_IN; elapsed counts any loops in between that it did not check, so
 * driving while the pose was unusable is not a jump). A
 * failure is latched: the pose counts as invalid until {@link #reset()} (the teleop's X reset).
 * Field frame, inches. Pure logic: the teleop passes the pose and a clock reading, seconds.
 */
public class PoseSanity {

    /** Half the field, inches: the walls are about 70.7 in from the centre (FIRST's CAD). APPROX. */
    public static double FIELD_HALF_IN = 70.7;
    /** How far past a wall a pose may be before it is impossible, inches. SET BY HAND: the robot's
     *  centre can never be past a wall, so this only absorbs odometry error. */
    public static double OFF_FIELD_MARGIN_IN = 6.0;
    /** Slack on the top-speed travel per loop. SET BY HAND. */
    public static double JUMP_SPEED_FACTOR = 1.5;
    /** Extra distance allowed per loop, inches. SET BY HAND. */
    public static double JUMP_MARGIN_IN = 3.0;

    private final double maxSpeedInPerS;
    private double lastX = Double.NaN;
    private double lastY = Double.NaN;
    private double lastS = Double.NaN;
    private String fault = "";

    /** @param maxSpeedInPerS the robot's top speed, in/s */
    public PoseSanity(double maxSpeedInPerS) {
        this.maxSpeedInPerS = maxSpeedInPerS;
    }

    /** With each usable pose and the time now, seconds. Returns true while the pose is believable. */
    public boolean check(double x, double y, double nowS) {
        if (!fault.isEmpty()) return false;
        if (!Double.isFinite(x) || !Double.isFinite(y)) return false; // not a number: not ours to latch
        double limit = FIELD_HALF_IN + OFF_FIELD_MARGIN_IN;
        if (Math.abs(x) > limit || Math.abs(y) > limit) {
            fault = "OFF FIELD";
        } else if (Double.isFinite(lastX) && nowS > lastS) {
            double moved = Math.hypot(x - lastX, y - lastY);
            if (moved > maxSpeedInPerS * (nowS - lastS) * JUMP_SPEED_FACTOR + JUMP_MARGIN_IN) fault = "POSE JUMPED";
        }
        lastX = x;
        lastY = y;
        lastS = nowS;
        return fault.isEmpty();
    }

    /** Trust the pose again (after the driver reset it); the next pose starts a fresh track. */
    public void reset() {
        fault = "";
        lastX = Double.NaN;
        lastY = Double.NaN;
        lastS = Double.NaN;
    }

    /** Why the pose is not believed ("" if it is). */
    public String fault() {
        return fault;
    }
}
