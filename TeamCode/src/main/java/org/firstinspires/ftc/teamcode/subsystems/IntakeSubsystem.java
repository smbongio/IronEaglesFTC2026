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

    public IntakeSubsystem(DcMotor motor) {
        this.motor = motor;
        this.motorEx = motor instanceof DcMotorEx ? (DcMotorEx) motor : null;
        if (this.motorEx != null) {
            this.motorEx.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    @Override
    public void runIntake(double speed) {
        setPower(speed);
    }

    @Override
    public void setPower(double power) {
        if (motorEx != null) {
            motorEx.setVelocity(targetTicksPerSec(power));
        } else if (motor != null) {
            motor.setPower(power * TunableValues.INTAKE_DIR);
        }
    }

    @Override
    public void stop() {
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

    private double ticksPerRev() {
        if (motorEx != null) {
            double fromType = motorEx.getMotorType().getTicksPerRev();
            if (fromType > 1.0) {
                return fromType;
            }
        }
        return TunableValues.INTAKE_TICKS_PER_REV;
    }

    private double targetTicksPerSec(double speedFraction) {
        return (TunableValues.INTAKE_TARGET_RPM / 60.0) * ticksPerRev()
                * speedFraction * TunableValues.INTAKE_DIR;
    }
}
