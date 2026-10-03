package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.IronEaglesTeleOp2026;
import org.firstinspires.ftc.teamcode.interfaces.HopperInterface;

/**
 * Concrete implementation of the Hopper mechanism subsystem.
 */
public class HopperSubsystem implements HopperInterface {

    private final DcMotor motor;

    public HopperSubsystem(DcMotor motor) {
        this.motor = motor;
    }

    @Override
    public void runHopper(double speed) {
        setPower(speed);
    }

    @Override
    public void setPower(double power) {
        if (motor != null) {
            motor.setPower(power * IronEaglesTeleOp2026.HOPPER_DIR);
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
