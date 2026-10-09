package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.interfaces.TargetGeometryInterface;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

/**
 * Concrete data model representing spatial geometry and position relative to an acquired AprilTag target.
 */
public class TargetGeometry implements TargetGeometryInterface {

    private final int tagId;
    private final double rangeInches;
    private final double bearingDegrees;
    private final double elevationDegrees;
    private final double yawDegrees;
    private final double xInches;
    private final double yInches;

    public TargetGeometry(int tagId, double rangeInches, double bearingDegrees,
                          double elevationDegrees, double yawDegrees,
                          double xInches, double yInches) {
        this.tagId            = tagId;
        this.rangeInches      = rangeInches;
        this.bearingDegrees   = bearingDegrees;
        this.elevationDegrees = elevationDegrees;
        this.yawDegrees       = yawDegrees;
        this.xInches          = xInches;
        this.yInches          = yInches;
    }

    /**
     * Constructs TargetGeometry directly from an FTC AprilTagDetection object.
     */
    public TargetGeometry(AprilTagDetection detection) {
        if (detection != null) {
            this.tagId            = (detection instanceof AprilTagSingleDetection) ? ((AprilTagSingleDetection) detection).id : -1;
            this.rangeInches      = (detection.ftcPose != null) ? detection.ftcPose.range : 0.0;
            this.bearingDegrees   = (detection.ftcPose != null) ? detection.ftcPose.bearing : 0.0;
            this.elevationDegrees = (detection.ftcPose != null) ? detection.ftcPose.elevation : 0.0;
            this.yawDegrees       = (detection.ftcPose != null) ? detection.ftcPose.yaw : 0.0;
            this.xInches          = (detection.ftcPose != null) ? detection.ftcPose.x : 0.0;
            this.yInches          = (detection.ftcPose != null) ? detection.ftcPose.y : 0.0;
        } else {
            this.tagId            = -1;
            this.rangeInches      = 0.0;
            this.bearingDegrees   = 0.0;
            this.elevationDegrees = 0.0;
            this.yawDegrees       = 0.0;
            this.xInches          = 0.0;
            this.yInches          = 0.0;
        }
    }

    @Override
    public int getTagId() {
        return tagId;
    }

    @Override
    public double getRangeInches() {
        return rangeInches;
    }

    @Override
    public double getBearingDegrees() {
        return bearingDegrees;
    }

    @Override
    public double getElevationDegrees() {
        return elevationDegrees;
    }

    @Override
    public double getYawDegrees() {
        return yawDegrees;
    }

    @Override
    public double getXInches() {
        return xInches;
    }

    @Override
    public double getYInches() {
        return yInches;
    }
}
