package org.firstinspires.ftc.teamcode.interfaces;

/**
 * Generic interface for a single-motor momentary mechanism.
 */
public interface MechanismInterface {
    /**
     * Sets the mechanism motor power.
     *
     * @param power Power level between -1.0 and 1.0
     */
    void setPower(double power);

    /**
     * Halts mechanism motor output.
     */
    void stop();

    /**
     * Checks if mechanism is currently active.
     *
     * @return true if motor power is non-zero
     */
    boolean isRunning();
}
