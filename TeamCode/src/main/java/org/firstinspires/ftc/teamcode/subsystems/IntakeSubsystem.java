package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.config.TunableValues;
import org.firstinspires.ftc.teamcode.interfaces.IntakeInterface;
import org.firstinspires.ftc.teamcode.util.RobotLogger;

public class IntakeSubsystem implements IntakeInterface {
    private final RobotLogger log = new RobotLogger("INTAKE");
    private final DcMotorEx motor;
    private final boolean useVelocity;
    private final double inwardTicksPerSec;
    private final double outwardTicksPerSec;
    private String lastCommand = "init";

    public IntakeSubsystem(DcMotor motor) {
        this.motor = motor instanceof DcMotorEx ? (DcMotorEx) motor : null;
        boolean velocity = false;
        double inward = 0;
        double outward = 0;
        if (this.motor == null) {
            log.w("Intake motor missing.");
        } else {
            double ticksPerRev = this.motor.getMotorType().getTicksPerRev();
            double maxTicks = this.motor.getMotorType().getAchieveableMaxTicksPerSecond();
            velocity = ticksPerRev > 1 && maxTicks > 1;
            if (velocity) {
                this.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                inward = maxTicks * TunableValues.INTAKE_TARGET_RPM_FRACTION;
                outward = maxTicks * TunableValues.INTAKE_REVERSE_RPM_FRACTION;
                log.i("Intake using velocity. inward=%.0f outward=%.0f ticks/s", inward, outward);
            } else {
                log.w("Intake encoder/motor type missing. Using 100%% power.");
            }
        }
        this.useVelocity = velocity;
        this.inwardTicksPerSec = inward;
        this.outwardTicksPerSec = outward;
    }

    @Override
    public boolean isUsingVelocity() {
        return useVelocity;
    }

    @Override
    public void runInward() {
        apply(1.0, inwardTicksPerSec, "inward");
    }

    @Override
    public void runOutward() {
        apply(-1.0, -outwardTicksPerSec, "outward");
    }

    @Override
    public void stop() {
        if (motor == null) {
            return;
        }
        if (useVelocity) {
            motor.setVelocity(0);
        }
        motor.setPower(0);
        if (!"stop".equals(lastCommand)) {
            lastCommand = "stop";
            log.i("Intake stop");
        }
    }

    private void apply(double power, double velocity, String direction) {
        if (motor == null) {
            return;
        }
        if (useVelocity) {
            motor.setVelocity(velocity);
        } else {
            motor.setPower(power);
        }
        if (!direction.equals(lastCommand)) {
            lastCommand = direction;
            log.i("Intake %s velocityMode=%b", direction, useVelocity);
        }
    }
}
