package org.firstinspires.ftc.teamcode.interfaces;

/**
 * Interface for orchestrating the automated targeting, positioning, and firing sequence.
 * Connects Vision, Drivetrain, Shooter, and Hopper interfaces to execute the 3-step firing sequence:
 *   Step 1: Retrieve AprilTag geometry (range, bearing, yaw) from VisionSubsystemInterface.
 *   Step 2: Auto-position and orient robot via DrivetrainInterface to optimal shooting distance & angle.
 *   Step 3: Spin up ShooterInterface flywheel and feed balls via HopperInterface.
 */
public interface AutoFireControllerInterface {

    /**
     * High-level states for the automated firing sequence.
     */
    enum FiringState {
        IDLE,           // Waiting for fire button trigger
        ALIGNING,       // Step 1 & 2: Querying AprilTag geometry & driving to optimal 24" & 0° position
        SPINNING_UP,    // Step 3a: Spinning up shooter flywheel motor to 100% RPM
        FEEDING,        // Step 3b: Actuating hopper feeder motor to push balls into flywheel
        COMPLETE,       // All 4 balls fired; sequence complete
        ABORTED         // Interrupted by driver trigger release or manual joystick steering
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
