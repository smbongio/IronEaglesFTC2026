package org.firstinspires.ftc.teamcode.interfaces;

import org.firstinspires.ftc.teamcode.geometry.MovementPlan;

public interface DrivetrainInterface {
    boolean areDriveMotorsPresent();

    void driveFieldCentric(double x, double y, double rx, double headingDeg);

    void driveRobotCentric(double x, double y, double rx);

    void startMove(MovementPlan plan);

    boolean isMoveFinished();

    void stop();
}
