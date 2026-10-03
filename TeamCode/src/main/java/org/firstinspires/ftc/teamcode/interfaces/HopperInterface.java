package org.firstinspires.ftc.teamcode.interfaces;

/**
 * Interface for controlling the Hopper mechanism.
 */
public interface HopperInterface extends MechanismInterface {
    /**
     * Actuates the hopper at the specified speed.
     *
     * @param speed Target operational speed multiplier
     */
    void runHopper(double speed);
}
