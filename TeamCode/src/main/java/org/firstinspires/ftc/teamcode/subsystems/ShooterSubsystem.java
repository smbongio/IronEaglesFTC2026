package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.config.TunableValues;
import org.firstinspires.ftc.teamcode.interfaces.ShooterInterface;
import org.firstinspires.ftc.teamcode.util.RobotLogger;

public class ShooterSubsystem implements ShooterInterface {
    private final RobotLogger log = new RobotLogger("SHOOTER");
    private final DcMotorEx motor;

    public ShooterSubsystem(DcMotor motor) {
        this.motor = motor instanceof DcMotorEx ? (DcMotorEx) motor : null;
        if (this.motor == null) {
            log.w("Shooter motor missing.");
            return;
        }
        this.motor.setDirection(DcMotor.Direction.REVERSE);
        log.i("Shooter motor found. type ticks/rev=%.1f", this.motor.getMotorType().getTicksPerRev());
    }

    @Override
    public void start() {
        if (motor == null) {
            return;
        }
        double maxTicks = motor.getMotorType().getAchieveableMaxTicksPerSecond();
        if (maxTicks > 1) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            double velocity = maxTicks * TunableValues.SHOOTER_TARGET_RPM_FRACTION;
            motor.setVelocity(velocity);
            log.i("Shooter started at 75%% of max: %.0f ticks/s", velocity);
        } else {
            motor.setPower(TunableValues.SHOOTER_TARGET_RPM_FRACTION);
            log.w("Shooter motor type has no max ticks/s. Using power %.2f", TunableValues.SHOOTER_TARGET_RPM_FRACTION);
        }
    }
}
