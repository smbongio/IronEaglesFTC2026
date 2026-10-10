package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.IronEaglesTeleOp2026;
import org.firstinspires.ftc.teamcode.interfaces.HopperInterface;

/**
 * Concrete implementation of the Hopper mechanism subsystem (REV Core Hex Motor).
 */
public class HopperSubsystem implements HopperInterface {

    // REV Core Hex Motor has 288 ticks per revolution (360 degrees).
    // 180 degrees corresponds to 144 ticks.
    public static final int TICKS_PER_180_DEGREES = 144;
    public static final double STEP_POWER           = 0.8;

    private final DcMotor motor;
    private int currentTargetTicks = 0;

    public HopperSubsystem(DcMotor motor) {
        this.motor = motor;
        if (motor == null) {
            RobotLog.e("[HOPPER SUBSYSTEM ERROR] Hopper DcMotor is NULL! Check hardware config name 'hopper'.");
        }
    }

    @Override
    public void stepForward180() {
        if (motor == null) {
            RobotLog.e("[HOPPER ERROR] stepForward180 called but hopper DcMotor is NULL!");
            return;
        }

        int currentPos = motor.getCurrentPosition();
        boolean busy = motor.isBusy();

        // If motor is not busy or close to current target, command next 180-degree step (+144 ticks)
        if (!busy || Math.abs(currentPos - currentTargetTicks) <= 15) {
            currentTargetTicks += TICKS_PER_180_DEGREES;
            RobotLog.i("[HOPPER STEP 180] Command -> CurrentPos: %d | New TargetTicks: %d", currentPos, currentTargetTicks);

            motor.setTargetPosition(currentTargetTicks);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(STEP_POWER * IronEaglesTeleOp2026.HOPPER_DIR);
        } else {
            RobotLog.i("[HOPPER STEP IN-PROGRESS] CurrentPos: %d | TargetTicks: %d | Busy: %b", currentPos, currentTargetTicks, busy);
        }
    }

    @Override
    public void returnToUpright() {
        if (motor == null) return;

        int currentPos = motor.getCurrentPosition();
        int remainder = Math.abs(currentPos % TICKS_PER_180_DEGREES);

        if (remainder > 15) {
            currentTargetTicks = ((currentPos / TICKS_PER_180_DEGREES) + 1) * TICKS_PER_180_DEGREES;
        } else {
            currentTargetTicks = (currentPos / TICKS_PER_180_DEGREES) * TICKS_PER_180_DEGREES;
        }

        RobotLog.i("[HOPPER RETURN UPRIGHT] CurrentPos: %d -> Returning to TargetTicks: %d", currentPos, currentTargetTicks);
        motor.setTargetPosition(currentTargetTicks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(STEP_POWER * IronEaglesTeleOp2026.HOPPER_DIR);
    }

    @Override
    public boolean isUpright() {
        if (motor == null) return true;
        int currentPos = motor.getCurrentPosition();
        return Math.abs(currentPos % TICKS_PER_180_DEGREES) <= 15;
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
    public void stop() {
        if (motor != null) {
            if (motor.getMode() != DcMotor.RunMode.RUN_TO_POSITION) {
                motor.setPower(0);
            }
        }
    }

    @Override
    public boolean isRunning() {
        return motor != null && (motor.isBusy() || Math.abs(motor.getPower()) > 0.01);
    }
}
