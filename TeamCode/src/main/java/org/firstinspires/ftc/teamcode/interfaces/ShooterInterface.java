package org.firstinspires.ftc.teamcode.interfaces;

/**
 * Interface for controlling the Shooter mechanism.
 */
public interface ShooterInterface extends MechanismInterface {
    /**
     * Actuates the shooter at the specified speed.
     *
     * @param speed Target operational speed multiplier
     */
    void runShooter(double speed);

    /**
     * Power the hub is applying to the shooter motor this loop.
     */
    double getAppliedPower();
}
