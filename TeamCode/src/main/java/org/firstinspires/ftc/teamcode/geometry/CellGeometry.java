package org.firstinspires.ftc.teamcode.geometry;

/**
 * Robot-relative pose of the target cell opening (inches and degrees).
 * This is the BIOBUZZ cluster ftcPose: x is right of the camera, y is forward, z is up.
 */
public final class CellGeometry {
    public final String clusterName;
    public final int percentFound;
    public final double xIn;
    public final double yIn;
    public final double zIn;
    public final double yawDeg;
    public final double rangeIn;
    public final double bearingDeg;
    public final double elevationDeg;
    public final double rollDeg;
    public final boolean upright;

    public CellGeometry(String clusterName, int percentFound, double xIn, double yIn, double zIn, double yawDeg,
                        double rangeIn, double bearingDeg, double elevationDeg, double rollDeg, boolean upright) {
        this.clusterName = clusterName;
        this.percentFound = percentFound;
        this.xIn = xIn;
        this.yIn = yIn;
        this.zIn = zIn;
        this.yawDeg = yawDeg;
        this.rangeIn = rangeIn;
        this.bearingDeg = bearingDeg;
        this.elevationDeg = elevationDeg;
        this.rollDeg = rollDeg;
        this.upright = upright;
    }
}
