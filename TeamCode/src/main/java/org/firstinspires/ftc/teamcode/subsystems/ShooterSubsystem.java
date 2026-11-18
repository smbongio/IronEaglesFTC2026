package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.config.TunableValues;
import org.firstinspires.ftc.teamcode.interfaces.ShooterInterface;

/**
 * Shooter on ehPort2. The hub holds {@link TunableValues#SHOOTER_TARGET_TPS} using the
 * motor's own encoder (ehPort2 encoder jack) and its built-in velocity PIDF.
 */
public class ShooterSubsystem implements ShooterInterface {

    private final DcMotorEx motor;
    private String lastCommand = "init";
    private double commandedTps = 0.0;
    private boolean velocityMode;

    public ShooterSubsystem(DcMotor shooterMotor) {
        if (shooterMotor instanceof DcMotorEx) {
            motor = (DcMotorEx) shooterMotor;
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else {
            motor = null;
            if (shooterMotor != null) {
                RobotLog.e("[SHOOTER] Motor does not support velocity control; shooter disabled.");
            }
        }
    }

    /** {@code speed} is +1 forward or -1 reverse, scaled by the target ticks/second. */
    @Override
    public void runShooter(double speed) {
        lastCommand = "run";
        setVelocity(speed * TunableValues.SHOOTER_TARGET_TPS);
    }

    /** Same as {@link #runShooter}: a fraction of the target ticks/second. */
    @Override
    public void setPower(double power) {
        runShooter(power);
    }

    /** Lets the flywheel coast down instead of having the velocity loop brake it to 0. */
    @Override
    public void stop() {
        lastCommand = "stop";
        commandedTps = 0.0;
        if (motor != null && velocityMode) {
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setPower(0.0);
            velocityMode = false;
        }
    }

    @Override
    public boolean isRunning() {
        return motor != null && Math.abs(commandedTps) > 0.0;
    }

    @Override
    public double getAppliedPower() {
        return motor == null ? 0.0 : motor.getPower();
    }

    @Override
    public double getTargetTps() {
        return commandedTps;
    }

    @Override
    public double getMeasuredTps() {
        return motor == null ? 0.0 : motor.getVelocity();
    }

    @Override
    public String getCommandStatus() {
        if (motor == null) {
            return lastCommand + " motor=NULL";
        }
        return String.format("%s target=%.0f measured=%.0f ticks/s power=%.2f",
                lastCommand, commandedTps, getMeasuredTps(), getAppliedPower());
    }

    private void setVelocity(double tps) {
        commandedTps = tps;
        if (motor == null) {
            return;
        }
        if (!velocityMode) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            velocityMode = true;
        }
        motor.setVelocity(tps * TunableValues.SHOOTER_DIR);
    }
}
