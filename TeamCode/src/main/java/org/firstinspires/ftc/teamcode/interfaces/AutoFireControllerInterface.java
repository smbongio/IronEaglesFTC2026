package org.firstinspires.ftc.teamcode.interfaces;

/**
 * Interface for orchestrating the automated targeting, positioning, and firing sequence.
 * Connects Vision, Drivetrain, Shooter, and Hopper interfaces to execute the firing sequence:
 *   Step 1: Retrieve AprilTag geometry from VisionSubsystemInterface.
 *   Step 2: Auto-position and orient robot via DrivetrainInterface to target position.
 *   Step 3: Feed game elements into spinning flywheel via HopperInterface.
 */
public interface AutoFireControllerInterface {

    /**
     * States for the automated firing sequence.
     */
    enum FiringState {
        IDLE,       // Waiting for aim-assist trigger
        ALIGNING,   // Executing alignment trajectory or auto-steering to target
        FEEDING     // Target aligned; running hopper to feed elements into flywheel
    }

    /**
     * Updates and advances the non-blocking auto-fire state machine.
     * Must be called continuously in the TeleOp loop.
     *
     * @param isFireTriggerHeld true if driver is holding the fire button
     * @param isDriverSteering  true if driver is touching joysticks (manual override)
     */
    void update(boolean isFireTriggerHeld, boolean isDriverSteering);

    /**
     * Returns the current state of the automated firing sequence.
     *
     * @return FiringState
     */
    FiringState getCurrentState();

    /**
     * Immediately halts all firing motors, resets sequence timers, and returns to IDLE state.
     */
    void abort();
}
