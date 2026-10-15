package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.config.TunableValues;
import org.firstinspires.ftc.teamcode.interfaces.ShooterInterface;
import org.firstinspires.ftc.teamcode.util.RobotLogger;

public class ShooterSubsystem implements ShooterInterface {
    private static final double STOPPED_FRACTION = 0.15;

    private final RobotLogger log = new RobotLogger("SHOOTER");
    private final DcMotorEx motor;
    private final boolean useVelocity;
    private final double forwardTicksPerSec;
    private final double reverseTicksPerSec;
    private boolean reverse;
    private boolean warnedStopped;

    public ShooterSubsystem(DcMotor motor) {
        this.motor = motor instanceof DcMotorEx ? (DcMotorEx) motor : null;
        boolean velocity = false;
        double forward = 0;
        double reverseTicks = 0;
        if (this.motor == null) {
            log.w("Shooter motor missing.");
        } else {
            this.motor.setDirection(DcMotor.Direction.REVERSE);
            double ticksPerRev = this.motor.getMotorType().getTicksPerRev();
            double maxTicks = this.motor.getMotorType().getAchieveableMaxTicksPerSecond();
            velocity = ticksPerRev > 1;
            if (velocity) {
                this.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                forward = TunableValues.SHOOTER_TARGET_RPM / 60.0 * ticksPerRev;
                reverseTicks = forward;
                log.i("Shooter using velocity. target=%.0f rpm forward=%.0f ticks/s",
                        TunableValues.SHOOTER_TARGET_RPM, forward);
            } else {
                log.w("Shooter motor type has no ticks/rev. Falling back to power from %.0f rpm (maxTicks=%.0f)",
                        TunableValues.SHOOTER_TARGET_RPM, maxTicks);
            }
        }
        this.useVelocity = velocity;
        this.forwardTicksPerSec = forward;
        this.reverseTicksPerSec = reverseTicks;
    }

    @Override
    public void start() {
        runForward();
        apply();
        log.i("Shooter started forward");
    }

    @Override
    public void runForward() {
        if (reverse) {
            log.i("Shooter forward");
        }
        reverse = false;
    }

    @Override
    public void runReverse() {
        if (!reverse) {
            log.i("Shooter reverse");
        }
        reverse = true;
    }

    @Override
    public void update() {
        apply();
        if (!useVelocity || motor == null) {
            return;
        }
        double commanded = getCommandedTicksPerSec();
        double actual = getActualTicksPerSec();
        boolean stopped = Math.abs(commanded) > 100 && Math.abs(actual) < Math.abs(commanded) * STOPPED_FRACTION;
        if (stopped && !warnedStopped) {
            warnedStopped = true;
            log.w("SHOOTER STOPPED commanded=%.0f actual=%.0f. Press GP2 A to respool; not auto-restarting.",
                    commanded, actual);
        } else if (!stopped && warnedStopped) {
            warnedStopped = false;
            log.i("Shooter speed recovered commanded=%.0f actual=%.0f", commanded, actual);
        }
    }

    @Override
    public boolean isReversing() {
        return reverse;
    }

    @Override
    public boolean isStoppedWarning() {
        return warnedStopped;
    }

    @Override
    public double getCommandedTicksPerSec() {
        if (!useVelocity) {
            return 0;
        }
        return reverse ? -reverseTicksPerSec : forwardTicksPerSec;
    }

    @Override
    public double getActualTicksPerSec() {
        return motor == null ? 0 : motor.getVelocity();
    }

    private double fallbackPower() {
        if (motor == null) {
            return 0;
        }
        double ticksPerRev = motor.getMotorType().getTicksPerRev();
        double maxTicks = motor.getMotorType().getAchieveableMaxTicksPerSecond();
        if (ticksPerRev > 1 && maxTicks > 1) {
            double maxRpm = maxTicks / ticksPerRev * 60.0;
            if (maxRpm > 1) {
                return Math.max(0.0, Math.min(1.0, TunableValues.SHOOTER_TARGET_RPM / maxRpm));
            }
        }
        return 1.0;
    }

    private void apply() {
        if (motor == null) {
            return;
        }
        if (useVelocity) {
            motor.setVelocity(getCommandedTicksPerSec());
        } else {
            double power = fallbackPower();
            motor.setPower(reverse ? -power : power);
        }
    }
}
