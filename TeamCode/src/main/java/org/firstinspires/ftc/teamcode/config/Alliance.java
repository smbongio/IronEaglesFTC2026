package org.firstinspires.ftc.teamcode.config;

/**
 * Red or blue. {@link TunableValues} holds the red-side numbers. The BIOBUZZ field is
 * rotationally symmetric, so blue is the same setup rotated 180° about field center
 * (X and Y negated, heading + 180), with the driver standing at the blue wall facing -Y.
 */
public enum Alliance {
    RED(1),
    BLUE(-1);

    private final int sign;

    Alliance(int sign) {
        this.sign = sign;
    }

    /** Red field X to this alliance's field X. */
    public double x(double redX) {
        return sign * redX;
    }

    /** Red field Y to this alliance's field Y. */
    public double y(double redY) {
        return sign * redY;
    }

    /** Red field heading to this alliance's field heading, wrapped to (-180, 180]. */
    public double heading(double redHeadingDeg) {
        double h = this == RED ? redHeadingDeg : redHeadingDeg + 180.0;
        while (h > 180.0) {
            h -= 360.0;
        }
        while (h <= -180.0) {
            h += 360.0;
        }
        return h;
    }

    public double startXIn() {
        return x(TunableValues.START_POSE_X_IN);
    }

    public double startYIn() {
        return y(TunableValues.START_POSE_Y_IN);
    }

    public double startHeadingDeg() {
        return heading(TunableValues.START_HEADING_DEG);
    }

    /** Field heading the driver's stick-up points along (red +Y = 90, blue -Y = -90). */
    public double stickZeroDeg() {
        return sign * TunableValues.DRIVE_STICK_ZERO_DEG;
    }

    /** +1 when the driver's stick axes point along field +X/+Y, -1 when they point along -X/-Y. */
    public int driverFieldSign() {
        return sign;
    }

    /** SDK BIOBUZZ cluster name of the cell that starts up (the cell-up program's target). */
    public String upCellClusterName() {
        return this == RED ? "RED AUDIENCE" : "BLUE SCORING";
    }

    /** SDK BIOBUZZ cluster name of the cell that starts down (the cell-down program's target). */
    public String downCellClusterName() {
        return this == RED ? "RED SCORING" : "BLUE AUDIENCE";
    }
}
