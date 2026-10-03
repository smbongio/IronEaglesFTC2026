package org.firstinspires.ftc.teamcode.interfaces;

/**
 * Interface representing spatial geometry and position relative to an acquired AprilTag target.
 */
public interface TargetGeometryInterface {

    /**
     * Unique AprilTag ID number.
     *
     * @return tag ID integer
     */
    int getTagId();

    /**
     * Direct 3D distance in inches from camera lens to the AprilTag.
     *
     * @return distance in inches
     */
    double getRangeInches();

    /**
     * Horizontal bearing angle in degrees from camera centerline to tag (negative = left, positive = right).
     *
     * @return bearing in degrees
     */
    double getBearingDegrees();

    /**
     * Vertical elevation angle in degrees from camera centerline to tag.
     *
     * @return elevation in degrees
     */
    double getElevationDegrees();

    /**
     * Horizontal orientation yaw angle in degrees of the tag face relative to camera face.
     *
     * @return yaw in degrees
     */
    double getYawDegrees();

    /**
     * Lateral X offset in inches (left/right relative to camera).
     *
     * @return X in inches
     */
    double getXInches();

    /**
     * Forward Y depth distance in inches (distance along camera optical axis).
     *
     * @return Y in inches
     */
    double getYInches();
}
