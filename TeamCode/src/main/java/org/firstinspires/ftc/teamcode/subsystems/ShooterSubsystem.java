package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.config.TunableValues;
import org.firstinspires.ftc.teamcode.interfaces.ShooterInterface;

/**
 * Two-motor shooter. ehPort2 is the leader: our speed loop sets its power from its own
 * encoder. Every {@link #update()} the follower on ehPort3 gets the leader's power
 * (times {@link TunableValues#SHOOTER_FOLLOW_DIR}).
 *
 * The loop runs here, not in the hub, because in hub velocity mode getPower() only reports
 * the target as a fraction, not the power actually applied, so the follower couldn't match.
 * ehPort3's encoder is the rear odometry pod: never reset it or change its direction.
 */
public class ShooterSubsystem implements ShooterInterface {

    private final DcMotorEx leader;
    private final DcMotor follower;
    private String lastCommand = "init";
    private double commandedTps = 0.0;

    public ShooterSubsystem(DcMotor leaderMotor, DcMotor followerMotor) {
        if (leaderMotor instanceof DcMotorEx) {
            leader = (DcMotorEx) leaderMotor;
            leader.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leader.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        } else {
            leader = null;
            if (leaderMotor != null) {
                RobotLog.e("[SHOOTER] ehPort2 cannot read velocity; shooter disabled.");
            }
        }
        follower = followerMotor;
        if (follower == null) {
            RobotLog.w("[SHOOTER] Follower ehPort3 missing; shooting with ehPort2 only.");
        }
    }

    /** {@code speed} is +1 forward or -1 reverse, scaled by the target ticks/second. */
    @Override
    public void runShooter(double speed) {
        lastCommand = "run";
        commandedTps = speed * TunableValues.SHOOTER_TARGET_TPS;
    }

    /** Same as {@link #runShooter}: a fraction of the target ticks/second. */
    @Override
    public void setPower(double power) {
        runShooter(power);
    }

    /** Zero power on both motors; FLOAT lets the flywheel coast down. */
    @Override
    public void stop() {
        lastCommand = "stop";
        commandedTps = 0.0;
    }

    @Override
    public void update() {
        if (leader == null) {
            return;
        }
        double power = 0.0;
        if (commandedTps != 0.0) {
            double error = commandedTps - getMeasuredTps();
            power = Range.clip(TunableValues.SHOOTER_KF * commandedTps + TunableValues.SHOOTER_KP * error,
                    -1.0, 1.0);
        }
        leader.setPower(power * TunableValues.SHOOTER_DIR);
        if (follower != null) {
            follower.setPower(leader.getPower() * TunableValues.SHOOTER_FOLLOW_DIR);
        }
    }

    @Override
    public boolean isRunning() {
        return leader != null && commandedTps != 0.0;
    }

    @Override
    public double getAppliedPower() {
        return leader == null ? 0.0 : leader.getPower();
    }

    @Override
    public double getTargetTps() {
        return commandedTps;
    }

    @Override
    public double getMeasuredTps() {
        return leader == null ? 0.0 : leader.getVelocity() * TunableValues.SHOOTER_DIR;
    }

    @Override
    public String getCommandStatus() {
        if (leader == null) {
            return lastCommand + " motor=NULL";
        }
        return String.format("%s target=%.0f measured=%.0f ticks/s power=%.2f",
                lastCommand, commandedTps, getMeasuredTps(), getAppliedPower());
    }
}
