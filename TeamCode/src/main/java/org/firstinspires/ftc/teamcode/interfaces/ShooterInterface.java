package org.firstinspires.ftc.teamcode.interfaces;

/**
 * Interface for controlling the Shooter mechanism.
 */
public interface ShooterInterface extends MechanismInterface {
    /**
     * Runs the shooter at {@code speed} times the target ticks/second (+1 forward, -1 reverse).
     */
    void runShooter(double speed);

    /** Runs the speed loop: call once per loop while the OpMode runs. */
    void update();

    /**
     * Power applied to the shooter motor (and copied to the follower) this loop.
     */
    double getAppliedPower();

    /** Ticks/second the shooter is being told to hold (0 when stopped). */
    double getTargetTps();

    /** Ticks/second the shooter's encoder is reading. */
    double getMeasuredTps();
}
