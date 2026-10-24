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
     * Robot-centric drive after rotating stick inputs by IMU heading (degrees).
     * Heading 0 is robot-centric. Rotation (rx) is not field-rotated.
     */
    void driveFieldCentric(double x, double y, double rx, double headingDeg);

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

    /**
     * Latest gamepad command after deadzone: y (forward), x (strafe), rx (rotate).
     */
    String getStickStatus();

    /**
     * Power last requested of FL, FR, BL, BR.
     */
    String getRequestedPowerStatus();

    /**
     * Encoder ticks/second measured on FL, FR, BL, BR.
     */
    String getTicksPerSecStatus();

    /**
     * Writes gamepad vs requested power vs encoder ticks/s to RobotLog when due.
     */
    void logWheelDiagnostics();
}
