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
     * Halts all drivetrain motor outputs.
     */
    void stop();
}
