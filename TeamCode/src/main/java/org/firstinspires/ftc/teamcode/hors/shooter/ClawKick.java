package org.firstinspires.ftc.teamcode.hors.shooter;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * One claw kick, MAINHORS26's ClawController logic (sushilm20/MAINHORS26, subsystems/ClawController):
 * the servo rests at {@code restPos}; {@link #trigger} moves it to {@code kickPos}, and
 * {@code kickMs} later {@link #update} puts it back at rest, ready for the next Y.
 *
 * The positions are not MAINHORS26's: HORS Clutch Teleop passes this robot's own, from
 * subsystems/shooting/ClawController.
 */
public class ClawKick {

    private final Servo servo;
    private final double restPos;
    private final double kickPos;
    private final long kickMs;

    private boolean active = false;
    private long startMs = 0L;

    public ClawKick(Servo servo, double restPos, double kickPos, long kickMs) {
        this.servo = servo;
        this.restPos = restPos;
        this.kickPos = kickPos;
        this.kickMs = kickMs;
        servo.setPosition(restPos);
    }

    public void trigger(long nowMs) {
        servo.setPosition(kickPos);
        active = true;
        startMs = nowMs;
    }

    public void update(long nowMs) {
        if (active && nowMs >= startMs + kickMs) {
            servo.setPosition(restPos);
            active = false;
        }
    }

    public boolean isKicking() {
        return active;
    }
}
