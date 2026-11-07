package org.firstinspires.ftc.teamcode.hors.aim;

/**
 * Which alliance the robot is on. BIOBUZZ is point-symmetric about the field centre, so BLUE is
 * RED rotated half a turn: (x, y, h) -> (-x, -y, h + pi).
 */
public enum Alliance {
    RED,
    BLUE;

    public Alliance other() {
        return this == RED ? BLUE : RED;
    }
}
