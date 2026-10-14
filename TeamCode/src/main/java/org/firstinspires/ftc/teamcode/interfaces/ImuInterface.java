package org.firstinspires.ftc.teamcode.interfaces;

public interface ImuInterface {
    boolean isPresent();

    void resetYaw();

    double getHeadingDegrees();
}
