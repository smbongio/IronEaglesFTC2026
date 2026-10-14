package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.interfaces.HopperInterface;
import org.firstinspires.ftc.teamcode.util.RobotLogger;

public class HopperSubsystem implements HopperInterface {
    private static final int TICKS_PER_180 = 144;
    private static final double STEP_POWER = 0.6;
    private static final int DONE_TICKS = 5;

    private final RobotLogger log = new RobotLogger("HOPPER");
    private final DcMotor motor;
    private int targetTicks;
    private boolean stepping;

    public HopperSubsystem(DcMotor motor) {
        this.motor = motor;
        if (motor == null) {
            log.w("Hopper motor missing.");
            return;
        }
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        log.i("Hopper ready. ticksPer180=%d", TICKS_PER_180);
    }

    @Override
    public void stepForward180() {
        startStep(TICKS_PER_180);
    }

    @Override
    public void stepReverse180() {
        startStep(-TICKS_PER_180);
    }

    @Override
    public void update() {
        if (motor == null || !stepping) {
            return;
        }
        int pos = motor.getCurrentPosition();
        if (!motor.isBusy() || Math.abs(pos - targetTicks) <= DONE_TICKS) {
            motor.setPower(0);
            stepping = false;
            log.i("Step complete at %d (target %d)", pos, targetTicks);
        }
    }

    @Override
    public boolean isStepping() {
        return stepping;
    }

    @Override
    public void stop() {
        if (motor == null) {
            stepping = false;
            return;
        }
        if (stepping) {
            log.i("Hopper force-stopped at %d (was targeting %d)", motor.getCurrentPosition(), targetTicks);
        }
        stepping = false;
        motor.setPower(0);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void startStep(int delta) {
        if (motor == null) {
            return;
        }
        if (stepping) {
            update();
            return;
        }
        targetTicks = motor.getCurrentPosition() + delta;
        stepping = true;
        motor.setTargetPosition(targetTicks);
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motor.setPower(STEP_POWER);
        log.i("Start step %+d -> target %d", delta, targetTicks);
    }
}
