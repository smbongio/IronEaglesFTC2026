package org.firstinspires.ftc.teamcode.interfaces;

/**
 * Interface for orchestrating the automated targeting, positioning, and firing sequence.
 */
public interface AutoFireControllerInterface {

    /**
     * High-level states for the automated firing sequence.
     */
    enum FiringState {
        IDLE,           // Waiting for fire button trigger
        ALIGNING,       // Drivetrain auto-positioning to target distance & angle
        SPINNING_UP,    // Shooter motor flywheel spinning up to full RPM
        FEEDING,        // Hopper/intake feeding balls into shooter
        COMPLETE,       // Firing sequence finished
        ABORTED         // Interrupted by driver release or manual override
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
