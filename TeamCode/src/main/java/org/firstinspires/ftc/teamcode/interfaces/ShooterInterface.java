package org.firstinspires.ftc.teamcode.interfaces;

public interface ShooterInterface {
    void start();

    void runForward();

    void runReverse();

    void stop();

    void update();

    boolean isRunning();

    boolean isReversing();

    boolean isStoppedWarning();

    boolean isAtTargetRpm();

    double getCommandedTicksPerSec();

    double getActualTicksPerSec();

    double getCommandedRpm();

    double getActualRpm();
}
