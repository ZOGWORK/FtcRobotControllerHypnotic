package org.firstinspires.ftc.teamcode.hors.aim;

/**
 * Fixed BIOBUZZ field coordinates the aim system points at.
 *
 * Frame (same as Zenith's field file and the autos): origin at the field centre, inches,
 * +X toward the audience's right (BLUE side), +Y away from the audience, heading in radians
 * counter-clockwise from +X. RED is x < 0, BLUE is x > 0.
 *
 * Each alliance scores pollen into its OWN hive. A hive has a north and a south cell; one is up
 * and a tip swaps them. A launch only scores from the up cell's own floor quarter, so the aim
 * system picks the cell on the robot's side of the field: north of the hive's centre line (y > 0)
 * the north cell, south of it the south cell. The driver chooses by where they drive.
 */
public final class FieldTargets {

    private FieldTargets() {
    }

    /** Which of a hive's two cells. */
    public enum Cell {
        NORTH,
        SOUTH;

        public Cell other() {
            return this == NORTH ? SOUTH : NORTH;
        }
    }

    // SPEC (BIOBUZZ Competition Manual V1 §9.6, via @horizon36596/zenith-season-biobuzz
    // biobuzz.field.json): RED hive pivots at (-12.75, 0), BLUE at (+12.75, 0).
    public static final double HIVE_PIVOT_X_IN = 12.75; // SPEC

    // SPEC (FIRST BIOBUZZ field CAD, per Zenith's BIOBUZZ ledger): a cell's open outer face is
    // 18.528 in from the pivot along y. The aim point is the cell's middle, half of that.
    public static final double CELL_OUTER_FACE_FROM_PIVOT_IN = 18.528; // SPEC
    public static final double CELL_CENTRE_FROM_PIVOT_IN = CELL_OUTER_FACE_FROM_PIVOT_IN / 2.0;

    /**
     * How far past the centre line (y = 0) the robot must go before the aim switches cells, inches.
     * Stops the target flickering while the robot sits on the line. SET BY HAND.
     */
    public static final double HALF_SWITCH_HYSTERESIS_IN = 2.0;

    /**
     * The cell on the robot's half of the field: NORTH when y > 0, SOUTH when y < 0. Within
     * {@link #HALF_SWITCH_HYSTERESIS_IN} of the line it keeps {@code current}.
     */
    public static Cell cellForHalf(double robotY, Cell current) {
        if (robotY > HALF_SWITCH_HYSTERESIS_IN) return Cell.NORTH;
        if (robotY < -HALF_SWITCH_HYSTERESIS_IN) return Cell.SOUTH;
        return current;
    }

    /** The cell that is up at kickoff: SOUTH for RED, NORTH for BLUE (SPEC, manual Fig 10-2). */
    public static Cell upCellAtKickoff(Alliance alliance) {
        return alliance == Alliance.RED ? Cell.SOUTH : Cell.NORTH;
    }

    /** x of the alliance's own hive pivot, inches. */
    public static double hiveX(Alliance alliance) {
        return alliance == Alliance.RED ? -HIVE_PIVOT_X_IN : HIVE_PIVOT_X_IN;
    }

    /** Aim point of a cell of the alliance's own hive, {x, y} in inches. */
    public static double[] cellAimPoint(Alliance alliance, Cell cell) {
        double y = cell == Cell.NORTH ? CELL_CENTRE_FROM_PIVOT_IN : -CELL_CENTRE_FROM_PIVOT_IN;
        return new double[] {hiveX(alliance), y};
    }
}
