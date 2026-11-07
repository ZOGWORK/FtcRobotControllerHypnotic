package org.firstinspires.ftc.teamcode.hors;

import com.pedropathing.localization.Localizer;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Pinpoint localizer for HORS Clutch Teleop's auto-aim: the same two-pod setup and field frame
 * as BioHORS (team 33950). It is kept apart from pedro/Constants, which is left unchanged.
 *
 * Field frame (BioHORS's): origin at the field centre, inches, +X toward the audience's right
 * (BLUE side), +Y away from the audience, heading radians CCW from +X.
 *
 * The Pinpoint and its pods sit in the same spot as on BioHORS (the team, 2026-11-07), so the
 * values below are BioHORS's, measured there with the Pedro Pinpoint Tuner on 2026-11-05.
 * Re-run the Pinpoint Tuner if a pod is moved.
 */
public final class HorsLocalizer {

    private HorsLocalizer() {
    }

    /** Hardware map name, the same one Teleop uses. */
    public static String PINPOINT_NAME = "driver";

    /** CARRIED OVER from BioHORS (same mounting; measured there 2026-11-05). */
    public static double X_POD_OFFSET_IN = -0.03132589454725972;
    /** CARRIED OVER from BioHORS (same mounting; measured there 2026-11-05). */
    public static double Y_POD_OFFSET_IN = -4.65472814604992;

    public static GoBildaPinpointDriver.GoBildaOdometryPods POD_TYPE =
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;             // CARRIED OVER
    public static GoBildaPinpointDriver.EncoderDirection X_POD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;                          // CARRIED OVER
    public static GoBildaPinpointDriver.EncoderDirection Y_POD_DIRECTION =
            GoBildaPinpointDriver.EncoderDirection.FORWARD;                          // CARRIED OVER

    /** Robot top speed, in/s, for the aim velocity clamp and the pose jump check. CARRIED OVER (BioHORS, from sim). */
    public static double MAX_SPEED_IN_PER_S = 60.0;

    /** Built from the fields above when called, so edits made before INIT apply. */
    public static Localizer create(HardwareMap hardwareMap) {
        PinpointConfig config = new PinpointConfig(c -> {
            c.name.set(PINPOINT_NAME);
            c.podType.set(POD_TYPE);
            c.xPodOffset.set(X_POD_OFFSET_IN);
            c.yPodOffset.set(Y_POD_OFFSET_IN);
            c.xPodDirection.set(X_POD_DIRECTION);
            c.yPodDirection.set(Y_POD_DIRECTION);
            c.globalDistanceUnit.set(DistanceUnit.INCH);
            c.offsetUnits.set(DistanceUnit.INCH);
        });
        return new PinpointLocalizer(hardwareMap, config);
    }
}
