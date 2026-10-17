package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.config.TunableValues;
import org.firstinspires.ftc.teamcode.interfaces.IntakeInterface;

/**
 * Concrete implementation of the Intake mechanism subsystem.
 */
public class IntakeSubsystem implements IntakeInterface {

    private final DcMotor motor;

    public IntakeSubsystem(DcMotor motor) {
        this.motor = motor;
    }

    @Override
    public void runIntake(double speed) {
        setPower(speed);
    }

    @Override
    public void setPower(double power) {
        if (motor != null) {
            motor.setPower(power * TunableValues.INTAKE_DIR);
        }
    }

    @Override
    public void stop() {
        if (motor != null) {
            motor.setPower(0);
        }
    }

    @Override
    public boolean isRunning() {
        return motor != null && Math.abs(motor.getPower()) > 0.01;
    }
}
