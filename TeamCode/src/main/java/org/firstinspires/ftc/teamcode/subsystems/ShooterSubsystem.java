package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
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
    }

    @Override
    public void runShooter(double speed) {
        setPower(speed);
    }

    @Override
    public void setPower(double power) {
        if (shooter != null) {
            shooter.setPower(power * TunableValues.SHOOTER_DIR);
        }
        if (shooterFollow != null) {
            shooterFollow.setPower(power * TunableValues.SHOOTER_FOLLOW_DIR);
        }
    }

    @Override
    public void stop() {
        if (shooter != null) {
            shooter.setPower(0);
        }
        if (shooterFollow != null) {
            shooterFollow.setPower(0);
        }
    }

    @Override
    public boolean isRunning() {
        return (shooter != null && Math.abs(shooter.getPower()) > 0.01)
                || (shooterFollow != null && Math.abs(shooterFollow.getPower()) > 0.01);
    }
}
