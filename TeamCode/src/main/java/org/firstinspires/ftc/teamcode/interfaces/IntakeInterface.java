package org.firstinspires.ftc.teamcode.interfaces;

/**
 * Interface for controlling the Intake mechanism.
 */
public interface IntakeInterface extends MechanismInterface {
    /**
     * Actuates the intake at the specified speed.
     *
     * @param speed Target operational speed multiplier
     */
    void runIntake(double speed);
}
