package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.config.TunableValues;
import org.firstinspires.ftc.teamcode.interfaces.ShooterInterface;

/**
 * Concrete implementation of the Shooter mechanism subsystem.
 */
public class ShooterSubsystem implements ShooterInterface {

    private final DcMotor shooter;
    private final DcMotor shooterFollow;

    public ShooterSubsystem(DcMotor shooter) {
        this(shooter, null);
    }

    public ShooterSubsystem(DcMotor shooter, DcMotor shooterFollow) {
        this.shooter = shooter;
        this.shooterFollow = shooterFollow;
        enableVelocity(this.shooter);
        enableVelocity(this.shooterFollow);
    }

    @Override
    public void runShooter(double speed) {
        setPower(speed);
    }

    @Override
    public void setPower(double power) {
        command(shooter, TunableValues.SHOOTER_DIR, power);
        command(shooterFollow, TunableValues.SHOOTER_FOLLOW_DIR, power);
    }

    @Override
    public void stop() {
        stopMotor(shooter);
        stopMotor(shooterFollow);
    }

    @Override
    public boolean isRunning() {
        return isMotorRunning(shooter) || isMotorRunning(shooterFollow);
    }

    private void enableVelocity(DcMotor motor) {
        if (motor instanceof DcMotorEx) {
            ((DcMotorEx) motor).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private void command(DcMotor motor, int dir, double speedFraction) {
        if (motor == null) {
            return;
        }
        if (motor instanceof DcMotorEx) {
            DcMotorEx motorEx = (DcMotorEx) motor;
            motorEx.setVelocity(targetTicksPerSec(motorEx, dir, speedFraction));
        } else {
            motor.setPower(speedFraction * dir);
        }
    }

    private void stopMotor(DcMotor motor) {
        if (motor == null) {
            return;
        }
        if (motor instanceof DcMotorEx) {
            ((DcMotorEx) motor).setVelocity(0);
        }
        motor.setPower(0);
    }

    private boolean isMotorRunning(DcMotor motor) {
        if (motor instanceof DcMotorEx) {
            return Math.abs(((DcMotorEx) motor).getVelocity()) > 1.0;
        }
        return motor != null && Math.abs(motor.getPower()) > 0.01;
    }

    private double targetTicksPerSec(DcMotorEx motor, int dir, double speedFraction) {
        double ticksPerRev = motor.getMotorType().getTicksPerRev();
        if (ticksPerRev <= 1.0) {
            ticksPerRev = TunableValues.SHOOTER_TICKS_PER_REV;
        }
        return (TunableValues.SHOOTER_TARGET_RPM / 60.0) * ticksPerRev * speedFraction * dir;
    }
}
