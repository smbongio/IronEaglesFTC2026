package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.IronEaglesTeleOp2026;
import org.firstinspires.ftc.teamcode.interfaces.ShooterInterface;

/**
 * Concrete implementation of the Shooter mechanism subsystem.
 */
public class ShooterSubsystem implements ShooterInterface {

    private final DcMotor motor;

    public ShooterSubsystem(DcMotor motor) {
        this.motor = motor;
    }

    @Override
    public void runShooter(double speed) {
        setPower(speed);
    }

    @Override
    public void setPower(double power) {
        if (motor != null) {
            motor.setPower(power * IronEaglesTeleOp2026.SHOOTER_DIR);
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
