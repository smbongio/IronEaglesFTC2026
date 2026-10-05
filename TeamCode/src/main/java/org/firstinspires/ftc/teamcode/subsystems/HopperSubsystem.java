package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.IronEaglesTeleOp2026;
import org.firstinspires.ftc.teamcode.interfaces.HopperInterface;

/**
 * Concrete implementation of the Hopper mechanism subsystem (REV Core Hex Motor).
 */
public class HopperSubsystem implements HopperInterface {

    // REV Core Hex Motor has 288 ticks per revolution (360 degrees).
    // 180 degrees upright position corresponds to 144 ticks.
    public static final int UPRIGHT_POSITION_TICKS = 144;
    public static final double RETURN_UPRIGHT_POWER  = 0.5;

    private final DcMotor motor;

    public HopperSubsystem(DcMotor motor) {
        this.motor = motor;
    }

    @Override
    public void runHopper(double speed) {
        if (motor != null) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            setPower(speed);
        }
    }

    @Override
    public void setPower(double power) {
        if (motor != null) {
            motor.setPower(power * IronEaglesTeleOp2026.HOPPER_DIR);
        }
    }

    @Override
    public void returnToUpright() {
        if (motor != null) {
            motor.setTargetPosition(UPRIGHT_POSITION_TICKS);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(RETURN_UPRIGHT_POWER);
        }
    }

    @Override
    public boolean isUpright() {
        if (motor == null) return true;
        return Math.abs(motor.getCurrentPosition() - UPRIGHT_POSITION_TICKS) <= 10;
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
