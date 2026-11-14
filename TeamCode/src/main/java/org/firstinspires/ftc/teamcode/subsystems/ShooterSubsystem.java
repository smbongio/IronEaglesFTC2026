package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.config.TunableValues;
import org.firstinspires.ftc.teamcode.interfaces.ShooterInterface;

/**
 * Shooter is open-loop power only. Its encoder jack (EH port 3) is the rear
 * odometry pod, so the hub must never run velocity control on this motor.
 */
public class ShooterSubsystem implements ShooterInterface {

    private final DcMotor motor;
    private String lastCommand = "init";
    private double lastFractionOfTarget = 0.0;

    public ShooterSubsystem(DcMotor motor) {
        this.motor = motor;
        if (this.motor != null) {
            this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    @Override
    public void runShooter(double speed) {
        lastCommand = "run";
        lastFractionOfTarget = speed;
        setPower(speed);
    }

    @Override
    public void setPower(double power) {
        if (motor != null) {
            motor.setPower(power * TunableValues.SHOOTER_POWER * TunableValues.SHOOTER_DIR);
        }
    }

    @Override
    public void stop() {
        lastCommand = "stop";
        lastFractionOfTarget = 0.0;
        if (motor != null) {
            motor.setPower(0);
        }
    }

    @Override
    public boolean isRunning() {
        return motor != null && Math.abs(motor.getPower()) > 0.01;
    }

    @Override
    public double getAppliedPower() {
        return motor == null ? 0.0 : motor.getPower();
    }

    @Override
    public String getCommandStatus() {
        if (motor == null) {
            return lastCommand + " motor=NULL";
        }
        return String.format("%s cmdPwr=%.2f measPwr=%.2f",
                lastCommand,
                lastFractionOfTarget * TunableValues.SHOOTER_POWER,
                getAppliedPower());
    }
}
