package org.firstinspires.ftc.teamcode.interfaces;

/**
 * Interface defining operational capabilities for a Mecanum drivetrain.
 */
public interface DrivetrainInterface {
    /**
     * Drives the robot given translational and rotational velocity inputs.
     *
     * @param x  Strafe input (-1.0 to 1.0, right is positive)
     * @param y  Forward input (-1.0 to 1.0, forward is positive)
     * @param rx Rotation input (-1.0 to 1.0, clockwise is positive)
     */
    void drive(double x, double y, double rx);

    /**
     * Automatically adjusts drivetrain position to align with an acquired AprilTag target.
     * Positions robot at optimal 24.0 inch shooting distance and 0 degree heading.
     *
     * @param target TargetGeometryInterface object containing current tag range, bearing, and yaw
     * @return true if drivetrain is fully aligned and in optimal shooting position
     */
    boolean alignToTarget(TargetGeometryInterface target);

    /**
     * Halts all drivetrain motor outputs.
     */
    void stop();
}
