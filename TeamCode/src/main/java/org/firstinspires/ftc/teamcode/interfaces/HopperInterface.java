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

    /**
     * Drives the hopper motor back to the upright 180-degree position (144 encoder ticks) using RUN_TO_POSITION.
     */
    void returnToUpright();

    /**
     * Checks if the hopper is currently in the upright 180-degree position.
     *
     * @return true if hopper position is upright
     */
    boolean isUpright();
}
