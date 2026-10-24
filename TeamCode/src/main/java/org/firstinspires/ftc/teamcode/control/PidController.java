package org.firstinspires.ftc.teamcode.control;

/**
 * Discrete PID. Output is not added to the setpoint; the caller applies feedforward.
 */
public class PidController {
    private double kP;
    private double kI;
    private double kD;
    private double iLimit;
    private double outLimit;

    private double integral;
    private double lastError;
    private boolean hasLast;

    public void setGains(double kP, double kI, double kD, double iLimit, double outLimit) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.iLimit = Math.abs(iLimit);
        this.outLimit = Math.abs(outLimit);
    }

    public double calculate(double setpoint, double measurement, double dtSec) {
        double error = setpoint - measurement;
        if (dtSec > 0.0) {
            integral += error * dtSec;
        }
        integral = clamp(integral, -iLimit, iLimit);

        double derivative = 0.0;
        if (hasLast && dtSec > 1.0e-4) {
            derivative = (error - lastError) / dtSec;
        }
        lastError = error;
        hasLast = true;

        return clamp(kP * error + kI * integral + kD * derivative, -outLimit, outLimit);
    }

    public void reset() {
        integral = 0.0;
        lastError = 0.0;
        hasLast = false;
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
