package org.firstinspires.ftc.teamcode.config;

/**
 * Red or blue. {@link TunableValues} holds the red-side numbers; blue is the same
 * setup mirrored across the field's X axis (Y and heading negated), with the driver
 * standing at the blue wall facing -Y.
 */
public enum Alliance {
    RED(1),
    BLUE(-1);

    private final int mirror;

    Alliance(int mirror) {
        this.mirror = mirror;
    }

    public double startXIn() {
        return TunableValues.START_POSE_X_IN;
    }

    public double startYIn() {
        return mirror * TunableValues.START_POSE_Y_IN;
    }

    public double startHeadingDeg() {
        return mirror * TunableValues.START_HEADING_DEG;
    }

    /** Field heading the driver's stick-up points along (red +Y = 90, blue -Y = -90). */
    public double stickZeroDeg() {
        return mirror * TunableValues.DRIVE_STICK_ZERO_DEG;
    }

    /** +1 when the driver's stick axes point along field +X/+Y, -1 when they point along -X/-Y. */
    public int driverFieldSign() {
        return mirror;
    }

    public int targetTagMin() {
        return this == RED ? TunableValues.VISION_RED_TAG_MIN : TunableValues.VISION_BLUE_TAG_MIN;
    }

    public int targetTagMax() {
        return this == RED ? TunableValues.VISION_RED_TAG_MAX : TunableValues.VISION_BLUE_TAG_MAX;
    }

    /** SDK BIOBUZZ tag library cluster name for this alliance's audience cell. */
    public String targetClusterName() {
        return this == RED ? "RED AUDIENCE" : "BLUE AUDIENCE";
    }
}
