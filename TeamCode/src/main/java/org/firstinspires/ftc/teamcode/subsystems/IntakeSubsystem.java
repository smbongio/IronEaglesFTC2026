package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.config.TunableValues;
import org.firstinspires.ftc.teamcode.interfaces.IntakeInterface;

/**
 * Concrete implementation of the Intake mechanism subsystem.
 */
public class IntakeSubsystem implements IntakeInterface {

    private final DcMotor motor;
    private final DcMotorEx motorEx;
    private String lastCommand = "init";
    // 0.0 = stopped, 1.0 = full INTAKE_TARGET_TICKS_PER_SEC. TeleOp currently always passes 1.0.
    private double lastFractionOfTarget = 0.0;

    public IntakeSubsystem(DcMotor motor) {
        this.motor = motor;
        this.motorEx = motor instanceof DcMotorEx ? (DcMotorEx) motor : null;
        if (this.motorEx != null) {
            this.motorEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    @Override
    public void runIntake(double speed) {
        lastCommand = "run";
        lastFractionOfTarget = speed;
        setPower(speed);
    }

    @Override
    public void setPower(double power) {
        if (motorEx != null) {
            motorEx.setVelocity(
                    TunableValues.INTAKE_TARGET_TICKS_PER_SEC * power * TunableValues.INTAKE_DIR);
        } else if (motor != null) {
            motor.setPower(power * TunableValues.INTAKE_DIR);
        }
    }

    @Override
    public void stop() {
        lastCommand = "stop";
        lastFractionOfTarget = 0.0;
        if (motorEx != null) {
            motorEx.setVelocity(0);
        }
        if (motor != null) {
            motor.setPower(0);
        }
    }

    @Override
    public boolean isRunning() {
        if (motorEx != null) {
            return Math.abs(motorEx.getVelocity()) > 1.0;
        }
        return motor != null && Math.abs(motor.getPower()) > 0.01;
    }

    @Override
    public double getTicksPerSec() {
        return motorEx == null ? 0.0 : motorEx.getVelocity();
    }

    @Override
    public String getCommandStatus() {
        if (motor == null) {
            return lastCommand + " motor=NULL";
        }
        return String.format("%s cmdTicks=%.0f measTicks=%.0f pwr=%.2f",
                lastCommand,
                TunableValues.INTAKE_TARGET_TICKS_PER_SEC * lastFractionOfTarget,
                getTicksPerSec(),
                motor.getPower());
    }
}
