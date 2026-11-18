package org.firstinspires.ftc.teamcode.interfaces;

/**
 * Interface for controlling the Shooter mechanism.
 */
public interface ShooterInterface extends MechanismInterface {
    /**
     * Runs the shooter at {@code speed} times the target ticks/second (+1 forward, -1 reverse).
     */
    void runShooter(double speed);

    /**
     * Power the hub is applying to the shooter motor this loop.
     */
    double getAppliedPower();

    /** Ticks/second the shooter is being told to hold (0 when stopped). */
    double getTargetTps();

    /** Ticks/second the shooter's encoder is reading. */
    double getMeasuredTps();
}
