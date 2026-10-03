package org.firstinspires.ftc.teamcode.interfaces;

/**
 * Interface for controlling robot visual signals (LED indicators / REV Blinkin).
 * Used to signal target acquisition status to drivers and field observers.
 */
public interface LedIndicatorInterface {

    /**
     * Sets the LED to signal that no valid alliance target is currently acquired.
     */
    void showSearching();

    /**
     * Sets the LED to signal that a valid alliance target cell AprilTag is locked and in range.
     *
     * @param isBlueAlliance true if on Blue Alliance (flashes Blue), false if Red (flashes Red)
     */
    void showTargetLocked(boolean isBlueAlliance);

    /**
     * Sets the LED to signal that the automated firing sequence is actively executing.
     */
    void showFiring();

    /**
     * Turns off all LED signals.
     */
    void turnOff();
}
