package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.IronEaglesTeleOp2026;
import org.firstinspires.ftc.teamcode.interfaces.HopperInterface;

/**
 * Concrete implementation of the Hopper mechanism subsystem (REV Core Hex Motor).
 * Handles exact 180-degree step indexing (144 encoder ticks) and position holding.
 */
public class HopperSubsystem implements HopperInterface {

    // REV Core Hex Motor has 288 ticks per revolution (360 degrees).
    // 180 degrees corresponds to 144 ticks.
    public static final int TICKS_PER_180_DEGREES = 144;
    public static final double STEP_POWER           = 0.6;

    private final DcMotor motor;
    private int targetTicks = 0;
    private boolean isStepping = false;

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

        // If not currently stepping, initiate a single 180-degree step (+144 ticks in HOPPER_DIR)
        if (!isStepping) {
            targetTicks = currentPos + (TICKS_PER_180_DEGREES * IronEaglesTeleOp2026.HOPPER_DIR);
            isStepping = true;

            RobotLog.i("[HOPPER STEP START] StartPos: %d -> TargetPos: %d", currentPos, targetTicks);
            motor.setTargetPosition(targetTicks);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(Math.abs(STEP_POWER));
        } else {
            // Check if active 180-degree step is complete
            if (!motor.isBusy() || Math.abs(currentPos - targetTicks) <= 5) {
                RobotLog.i("[HOPPER STEP COMPLETE] Reached TargetPos: %d (CurrentPos: %d). Stopping motor.", targetTicks, currentPos);
                motor.setPower(0);
                isStepping = false; // Reset flag so next 180-degree step can trigger if button remains held
            }
        }
    }

    @Override
    public void returnToUpright() {
        if (motor == null) return;

        int currentPos = motor.getCurrentPosition();

        // If currently executing a step, allow current 180-degree step to complete before stopping
        if (isStepping) {
            if (!motor.isBusy() || Math.abs(currentPos - targetTicks) <= 5) {
                RobotLog.i("[HOPPER STEP FINISHED ON RELEASE] Reached TargetPos: %d", targetTicks);
                motor.setPower(0);
                isStepping = false;
            }
            return;
        }

        // Align target position to nearest 180-degree increment (144 ticks)
        int nearestTarget = (int) (Math.round((double) currentPos / TICKS_PER_180_DEGREES) * TICKS_PER_180_DEGREES);
        if (Math.abs(currentPos - nearestTarget) > 5) {
            RobotLog.i("[HOPPER RETURN UPRIGHT] CurrentPos: %d -> Returning to Nearest TargetTicks: %d", currentPos, nearestTarget);
            motor.setTargetPosition(nearestTarget);
            motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motor.setPower(Math.abs(STEP_POWER));
        } else {
            motor.setPower(0);
        }
    }

    @Override
    public boolean isUpright() {
        if (motor == null) return true;
        int currentPos = motor.getCurrentPosition();
        return Math.abs(currentPos % TICKS_PER_180_DEGREES) <= 10;
    }

    @Override
    public void runHopper(double speed) {
        if (motor != null) {
            isStepping = false;
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
            if (motor.getMode() == DcMotor.RunMode.RUN_TO_POSITION) {
                int currentPos = motor.getCurrentPosition();
                if (!motor.isBusy() || Math.abs(currentPos - targetTicks) <= 5) {
                    motor.setPower(0);
                    isStepping = false;
                }
            } else {
                motor.setPower(0);
                isStepping = false;
            }
        }
    }

    @Override
    public boolean isRunning() {
        return motor != null && (motor.isBusy() || Math.abs(motor.getPower()) > 0.01);
    }
}
