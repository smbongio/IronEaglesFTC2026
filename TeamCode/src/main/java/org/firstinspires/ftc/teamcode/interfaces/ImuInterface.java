package org.firstinspires.ftc.teamcode.interfaces;

/**
 * Control Hub IMU heading used for field-centric drive.
 */
public interface ImuInterface {
    boolean isPresent();

    void resetYaw();

    double getHeadingDegrees();
}
