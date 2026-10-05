package org.firstinspires.ftc.teamcode.interfaces;

/**
 * Interface for controlling robot visual signals (LED indicators / REV Blinkin).
 * Used to signal target acquisition status and shooting state to drivers and field observers.
 */
public interface LedIndicatorInterface {

    /**
     * LED indicator state enumeration.
     */
    enum LedState {
        OFF,         // No LED light (not shooting and no target in range)
        IN_RANGE,    // Quickly blinking LED (target in range and ready to shoot)
        SHOOTING     // Solid LED (actively shooting)
    }

    /**
     * Sets the active LED indicator state.
     *
     * @param state          Target LED display mode (OFF, IN_RANGE, or SHOOTING)
     * @param isBlueAlliance true if on Blue Alliance, false if Red
     */
    void setLedState(LedState state, boolean isBlueAlliance);

    /**
     * Sets the active LED indicator state.
     *
     * @param state Target LED display mode (OFF, IN_RANGE, or SHOOTING)
     */
    void setLedState(LedState state);

    /**
     * Turns off all LED signals.
     */
    void turnOff();
}
