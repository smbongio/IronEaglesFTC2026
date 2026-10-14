package org.firstinspires.ftc.teamcode.geometry;

/**
 * Robot-relative pose of the target cell center (inches and degrees), after CELL_CENTER_OFFSET_IN.
 * X is right of the camera, Y is forward, Z is up.
 */
public final class CellGeometry {
    public final int tagId;
    public final String source;
    public final double xIn;
    public final double yIn;
    public final double zIn;
    public final double yawDeg;
    public final double rangeIn;
    public final double bearingDeg;
    public final double elevationDeg;
    public final double tagHeightIn;
    public final boolean upright;

    public CellGeometry(int tagId, String source, double xIn, double yIn, double zIn, double yawDeg,
                        double rangeIn, double bearingDeg, double elevationDeg,
                        double tagHeightIn, boolean upright) {
        this.tagId = tagId;
        this.source = source;
        this.xIn = xIn;
        this.yIn = yIn;
        this.zIn = zIn;
        this.yawDeg = yawDeg;
        this.rangeIn = rangeIn;
        this.bearingDeg = bearingDeg;
        this.elevationDeg = elevationDeg;
        this.tagHeightIn = tagHeightIn;
        this.upright = upright;
    }
}
