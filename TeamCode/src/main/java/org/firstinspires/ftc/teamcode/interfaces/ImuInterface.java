package org.firstinspires.ftc.teamcode.interfaces;

/**
 * Heading source used for field-centric drive.
 */
public interface ImuInterface {
    boolean isPresent();

    void resetYaw();

    double getHeadingDegrees();
}
