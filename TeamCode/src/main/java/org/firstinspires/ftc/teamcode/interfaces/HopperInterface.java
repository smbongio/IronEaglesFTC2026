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
     * Steps the hopper forward by 180 degrees (144 encoder ticks for REV Core Hex Motor).
     * If called continuously while holding a button, it steps 180 degrees iteratively.
     */
    void stepForward180();

    /**
     * Steps the hopper forward by 360 degrees (one full turn) then stops.
     * Call every loop until {@link #isStepping()} is false.
     */
    void stepForward360();

    /**
     * True while a 180 or 360 encoder step is in progress.
     */
    boolean isStepping();

    /**
     * Drives the hopper motor back to the nearest upright 180-degree position using RUN_TO_POSITION.
     */
    void returnToUpright();

    /**
     * Checks if the hopper is currently in the upright 180-degree position.
     *
     * @return true if hopper position is upright
     */
    boolean isUpright();
}
