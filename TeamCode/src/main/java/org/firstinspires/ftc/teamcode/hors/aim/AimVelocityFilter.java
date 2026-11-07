package org.firstinspires.ftc.teamcode.hors.aim;

/**
 * Smooths the robot's field velocity for the aim (bearing-rate feedforward and ball-flight lead),
 * so per-loop jitter in the raw reading does not twitch the aim.
 *
 * Exponential smoothing: filtered += alpha * (raw - filtered). alpha = 1 passes the raw value
 * straight through; smaller alpha smooths harder (and lags more). The result's magnitude is then
 * clamped to the drivetrain's top speed, so a spike can never read as a physically impossible
 * velocity.
 *
 * Fails soft: a non-finite raw reading gives zero for that loop and resets the filter, so aim keeps
 * working on its P term with no feedforward and no lead. An alpha outside (0, 1] is treated as 1
 * (no smoothing); a top speed that is not a positive number gives zero velocity. Pure math.
 */
public final class AimVelocityFilter {

    private final double alpha;
    private final double maxSpeedInPerS;
    private double vx = 0.0;
    private double vy = 0.0;

    /**
     * @param alpha          smoothing factor in (0, 1]; 1 = no smoothing
     * @param maxSpeedInPerS drivetrain top speed, in/s; the output's magnitude never exceeds it
     */
    public AimVelocityFilter(double alpha, double maxSpeedInPerS) {
        this.alpha = Double.isFinite(alpha) && alpha > 0.0 && alpha <= 1.0 ? alpha : 1.0;
        this.maxSpeedInPerS = maxSpeedInPerS;
    }

    /**
     * Feeds one loop's raw field velocity (in/s) and returns the filtered velocity {vx, vy}, in/s.
     */
    public double[] update(double rawVx, double rawVy) {
        if (!Double.isFinite(rawVx) || !Double.isFinite(rawVy)
                || !Double.isFinite(maxSpeedInPerS) || maxSpeedInPerS <= 0.0) {
            reset();
            return new double[] {0.0, 0.0};
        }
        vx += alpha * (rawVx - vx);
        vy += alpha * (rawVy - vy);
        double speed = Math.hypot(vx, vy);
        if (speed > maxSpeedInPerS) {
            vx *= maxSpeedInPerS / speed;
            vy *= maxSpeedInPerS / speed;
        }
        return new double[] {vx, vy};
    }

    /** Forgets the history: the next update starts from zero. */
    public void reset() {
        vx = 0.0;
        vy = 0.0;
    }
}
