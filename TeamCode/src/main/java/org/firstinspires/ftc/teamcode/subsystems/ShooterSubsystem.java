package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.config.TunableValues;
import org.firstinspires.ftc.teamcode.interfaces.ShooterInterface;

/**
 * Both shooter motors use hub velocity (ticks/s).
 */
public class ShooterSubsystem implements ShooterInterface {

    private final DcMotor shooter;
    private final DcMotor shooterFollow;
    private String lastCommand = "init";
    private double lastFractionOfTarget = 0.0;
    private double appliedP = Double.NaN;
    private double appliedI = Double.NaN;
    private double appliedD = Double.NaN;
    private double appliedF = Double.NaN;

    public ShooterSubsystem(DcMotor shooter) {
        this(shooter, null);
    }

    public ShooterSubsystem(DcMotor shooter, DcMotor shooterFollow) {
        this.shooter = shooter;
        this.shooterFollow = shooterFollow;
        enableVelocity(this.shooter);
        enableVelocity(this.shooterFollow);
        applyTunablePidf();
    }

    @Override
    public void runShooter(double speed) {
        lastCommand = "run";
        lastFractionOfTarget = speed;
        setPower(speed);
    }

    @Override
    public void setPower(double power) {
        applyTunablePidf();
        command(shooter, TunableValues.SHOOTER_DIR, power,
                TunableValues.SHOOTER_TARGET_TICKS_PER_SEC);
        command(shooterFollow, TunableValues.SHOOTER_FOLLOW_DIR, power,
                TunableValues.SHOOTER_FOLLOW_TARGET_TICKS_PER_SEC);
    }

    @Override
    public void stop() {
        lastCommand = "stop";
        lastFractionOfTarget = 0.0;
        stopMotor(shooter);
        stopMotor(shooterFollow);
    }

    @Override
    public boolean isRunning() {
        return isMotorRunning(shooter) || isMotorRunning(shooterFollow);
    }

    @Override
    public double getTicksPerSec() {
        return measuredTicksPerSec(shooter);
    }

    @Override
    public double getFollowTicksPerSec() {
        return measuredTicksPerSec(shooterFollow);
    }

    @Override
    public PIDFCoefficients getPidf() {
        return velocityPidf(shooter);
    }

    @Override
    public PIDFCoefficients getFollowPidf() {
        return velocityPidf(shooterFollow);
    }

    @Override
    public String getCommandStatus() {
        return String.format("%s leaderCmd=%.0f followCmd=%.0f %s %s",
                lastCommand,
                TunableValues.SHOOTER_TARGET_TICKS_PER_SEC * lastFractionOfTarget,
                TunableValues.SHOOTER_FOLLOW_TARGET_TICKS_PER_SEC * lastFractionOfTarget,
                motorStatus(shooter, "leader"),
                motorStatus(shooterFollow, "follow"));
    }

    private void enableVelocity(DcMotor motor) {
        if (motor instanceof DcMotorEx) {
            ((DcMotorEx) motor).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    private void command(DcMotor motor, int direction, double fractionOfTarget, double targetTicksPerSec) {
        if (motor == null) {
            return;
        }
        if (motor instanceof DcMotorEx) {
            double ticksPerSec = targetTicksPerSec * fractionOfTarget * direction;
            ((DcMotorEx) motor).setVelocity(ticksPerSec);
        } else {
            motor.setPower(fractionOfTarget * direction);
        }
    }

    private void applyTunablePidf() {
        double p = TunableValues.SHOOTER_P;
        double i = TunableValues.SHOOTER_I;
        double d = TunableValues.SHOOTER_D;
        double f = TunableValues.SHOOTER_F;
        if (appliedP == p && appliedI == i && appliedD == d && appliedF == f) {
            return;
        }
        setVelocityPidf(shooter, p, i, d, f);
        setVelocityPidf(shooterFollow, p, i, d, f);
        appliedP = p;
        appliedI = i;
        appliedD = d;
        appliedF = f;
    }

    private static void setVelocityPidf(DcMotor motor, double p, double i, double d, double f) {
        if (motor instanceof DcMotorEx) {
            ((DcMotorEx) motor).setVelocityPIDFCoefficients(p, i, d, f);
        }
    }

    private static String motorStatus(DcMotor motor, String name) {
        if (motor == null) {
            return name + "=NULL";
        }
        double ticksPerSec = motor instanceof DcMotorEx ? ((DcMotorEx) motor).getVelocity() : 0.0;
        return String.format("%sTicks=%.0f %sPwr=%.2f", name, ticksPerSec, name, motor.getPower());
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
        if (motor == null) {
            return false;
        }
        if (motor instanceof DcMotorEx && Math.abs(((DcMotorEx) motor).getVelocity()) > 1.0) {
            return true;
        }
        return Math.abs(motor.getPower()) > 0.01;
    }

    private static double measuredTicksPerSec(DcMotor motor) {
        if (!(motor instanceof DcMotorEx)) {
            return 0.0;
        }
        return ((DcMotorEx) motor).getVelocity();
    }

    private static PIDFCoefficients velocityPidf(DcMotor motor) {
        if (!(motor instanceof DcMotorEx)) {
            return new PIDFCoefficients(0, 0, 0, 0);
        }
        return ((DcMotorEx) motor).getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
