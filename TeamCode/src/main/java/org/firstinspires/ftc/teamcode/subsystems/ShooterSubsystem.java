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
    private final double ticksPerRev;
    private final double forwardTicksPerSec;
    private final double reverseTicksPerSec;
    private boolean running;
    private boolean reverse;
    private boolean warnedStopped;

    public ShooterSubsystem(DcMotor motor) {
        this.motor = motor instanceof DcMotorEx ? (DcMotorEx) motor : null;
        boolean velocity = false;
        double tpr = 0;
        double forward = 0;
        double reverseTicks = 0;
        if (this.motor == null) {
            log.w("Shooter motor missing.");
        } else {
            this.motor.setDirection(DcMotor.Direction.REVERSE);
            tpr = this.motor.getMotorType().getTicksPerRev();
            double maxTicks = this.motor.getMotorType().getAchieveableMaxTicksPerSecond();
            velocity = tpr > 1;
            if (velocity) {
                this.motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                forward = TunableValues.SHOOTER_TARGET_RPM / 60.0 * tpr;
                reverseTicks = forward;
                log.i("Shooter using velocity. target=%.0f rpm forward=%.0f ticks/s",
                        TunableValues.SHOOTER_TARGET_RPM, forward);
            } else {
                log.w("Shooter motor type has no ticks/rev. Falling back to power from %.0f rpm (maxTicks=%.0f)",
                        TunableValues.SHOOTER_TARGET_RPM, maxTicks);
            }
        }
        this.useVelocity = velocity;
        this.ticksPerRev = tpr;
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
        if (!running || reverse) {
            log.i("Shooter forward");
        }
        running = true;
        reverse = false;
    }

    @Override
    public void runReverse() {
        if (!running || !reverse) {
            log.i("Shooter reverse");
        }
        running = true;
        reverse = true;
    }

    @Override
    public void stop() {
        if (running) {
            log.i("Shooter stop");
        }
        running = false;
        reverse = false;
        warnedStopped = false;
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
            log.w("SHOOTER STOPPED commanded=%.0f actual=%.0f. Hold GP2 A to run; not auto-restarting.",
                    commanded, actual);
        } else if (!stopped && warnedStopped) {
            warnedStopped = false;
            log.i("Shooter speed recovered commanded=%.0f actual=%.0f", commanded, actual);
        }
    }

    @Override
    public boolean isRunning() {
        return running;
    }

    @Override
    public boolean isReversing() {
        return running && reverse;
    }

    @Override
    public boolean isStoppedWarning() {
        return warnedStopped;
    }

    @Override
    public boolean isAtTargetRpm() {
        if (!running) {
            return false;
        }
        if (!useVelocity) {
            return true;
        }
        double commanded = Math.abs(getCommandedTicksPerSec());
        if (commanded < 1) {
            return false;
        }
        return Math.abs(getActualTicksPerSec()) >= commanded * TunableValues.SHOOTER_READY_RPM_FRACTION;
    }

    @Override
    public double getCommandedTicksPerSec() {
        if (!useVelocity || !running) {
            return 0;
        }
        return reverse ? -reverseTicksPerSec : forwardTicksPerSec;
    }

    @Override
    public double getActualTicksPerSec() {
        return motor == null ? 0 : motor.getVelocity();
    }

    @Override
    public double getCommandedRpm() {
        return ticksToRpm(getCommandedTicksPerSec());
    }

    @Override
    public double getActualRpm() {
        return ticksToRpm(getActualTicksPerSec());
    }

    private double ticksToRpm(double ticksPerSec) {
        if (ticksPerRev <= 1) {
            return 0;
        }
        return ticksPerSec / ticksPerRev * 60.0;
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
        if (!running) {
            if (useVelocity) {
                motor.setVelocity(0);
            }
            motor.setPower(0);
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
