package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.config.TunableValues;

import java.util.ArrayList;
import java.util.List;

/**
 * Three dead wheels, no IMU. FTCLib {@link HolonomicOdometry} turns the encoder
 * distances into field X, Y, and heading.
 *
 * {@code leftPod}  front-to-back wheel on the left side  (EH encoder 0, intake port).
 * {@code rightPod} front-to-back wheel on the right side (EH encoder 2).
 * {@code backPod}  side-to-side wheel at the back        (CH encoder 1, backRight port).
 *
 * Field frame: +X audience, +Y blue, heading 0 faces +X, counter-clockwise positive.
 */
public class DeadWheelOdometry {
    private static final double LOG_PERIOD_SEC = 0.10;
    private static final int MAX_TICK_STEP = 8000;

    private final DcMotorEx leftPod;
    private final DcMotorEx rightPod;
    private final DcMotorEx backPod;
    private final HolonomicOdometry odometry;
    private final ElapsedTime logTimer = new ElapsedTime();
    private final ElapsedTime velocityTimer = new ElapsedTime();

    private int leftTicks;
    private int rightTicks;
    private int backTicks;
    private int lastLeftTicks;
    private int lastRightTicks;
    private int lastBackTicks;
    private int leftBaseTicks;
    private int rightBaseTicks;
    private int backBaseTicks;
    private String leftError;
    private String rightError;
    private String backError;

    private double xMm;
    private double yMm;
    private double headingRad;
    private double velXMmS;
    private double velYMmS;
    private double velHeadingRadS;
    private boolean poseValid;

    private boolean wasActive;
    private double lastLoggedXMm;
    private double lastLoggedYMm;
    private double lastLoggedHeadingRad;

    public DeadWheelOdometry(DcMotor leftPodMotor, DcMotor rightPodMotor, DcMotor backPodMotor) {
        leftPod = asEncoder(leftPodMotor, "left pod");
        rightPod = asEncoder(rightPodMotor, "right pod");
        backPod = asEncoder(backPodMotor, "back pod");
        if (leftPod == null) {
            leftError = "left pod missing";
        }
        if (rightPod == null) {
            rightError = "right pod missing";
        }
        if (backPod == null) {
            backError = "back pod missing";
        }

        Pose2d start = new Pose2d(
                TunableValues.START_POSE_X_IN * 25.4,
                TunableValues.START_POSE_Y_IN * 25.4,
                new Rotation2d(Math.toRadians(TunableValues.START_HEADING_DEG)));
        odometry = new HolonomicOdometry(start,
                TunableValues.ODO_TRACK_WIDTH_MM, TunableValues.ODO_BACK_POD_X_MM);
        readEncoders();
        setPose(start);
        RobotLog.i("[ODO] initialized. startPose x=%.1f y=%.1f in heading=%.1f  trackWidth=%.1f in backPodX=%.1f in  left=%s right=%s back=%s",
                TunableValues.START_POSE_X_IN,
                TunableValues.START_POSE_Y_IN,
                TunableValues.START_HEADING_DEG,
                TunableValues.ODO_TRACK_WIDTH_MM / 25.4,
                TunableValues.ODO_BACK_POD_X_MM / 25.4,
                leftPod != null ? "EH0" : "MISSING",
                rightPod != null ? "EH2" : "MISSING",
                backPod != null ? "CH1" : "MISSING");
    }

    public void update() {
        readEncoders();
        if (!isPresent()) {
            poseValid = false;
            return;
        }
        int dLeft = leftTicks - lastLeftTicks;
        int dRight = rightTicks - lastRightTicks;
        int dBack = backTicks - lastBackTicks;
        lastLeftTicks = leftTicks;
        lastRightTicks = rightTicks;
        lastBackTicks = backTicks;
        if (Math.abs(dLeft) > MAX_TICK_STEP || Math.abs(dRight) > MAX_TICK_STEP
                || Math.abs(dBack) > MAX_TICK_STEP) {
            RobotLog.w("[ODO] skipped encoder step left=%d right=%d back=%d ticks", dLeft, dRight, dBack);
            leftBaseTicks += dLeft;
            rightBaseTicks += dRight;
            backBaseTicks += dBack;
            zeroVelocity();
            return;
        }

        double perMm = TunableValues.ODO_TICKS_PER_MM;
        double leftMm = (leftTicks - leftBaseTicks) / perMm;
        double rightMm = (rightTicks - rightBaseTicks) / perMm;
        double backMm = (backTicks - backBaseTicks) / perMm;
        // FTCLib computes heading as (left - right) / trackWidth, which is clockwise-positive.
        // Passing right first makes it counter-clockwise-positive like the field frame.
        odometry.update(rightMm, leftMm, backMm);

        Pose2d pose = odometry.getPose();
        double newX = pose.getX();
        double newY = pose.getY();
        double newHeading = pose.getHeading();
        double dt = velocityTimer.seconds();
        velocityTimer.reset();
        if (dt > 0.001) {
            velXMmS = (newX - xMm) / dt;
            velYMmS = (newY - yMm) / dt;
            velHeadingRadS = wrapRadians(newHeading - headingRad) / dt;
        }
        xMm = newX;
        yMm = newY;
        headingRad = newHeading;
        poseValid = true;
    }

    public List<String> getCurrentHardwareErrors() {
        List<String> errors = new ArrayList<>();
        if (leftError != null) {
            errors.add(leftError);
        }
        if (rightError != null) {
            errors.add(rightError);
        }
        if (backError != null) {
            errors.add(backError);
        }
        return errors;
    }

    public boolean isPresent() {
        return leftPod != null && rightPod != null && backPod != null;
    }

    /** Keeps X/Y and puts heading back to {@link TunableValues#START_HEADING_DEG} (Back button). */
    public void resetHeading() {
        if (!isPresent()) {
            return;
        }
        setPose(new Pose2d(xMm, yMm, new Rotation2d(Math.toRadians(TunableValues.START_HEADING_DEG))));
        RobotLog.i("[ODO] Heading restored to start %.1f deg (Back button).",
                TunableValues.START_HEADING_DEG);
    }

    public double getHeadingDegrees() {
        return Math.toDegrees(headingRad);
    }

    /**
     * Heading for field-centric drive. Stick-up is field +Y (red-wall view),
     * not robot-forward, so this is heading minus {@link TunableValues#DRIVE_STICK_ZERO_DEG}.
     */
    public double getDriveHeadingDegrees() {
        return wrapDegrees(getHeadingDegrees() - TunableValues.START_HEADING_DEG
                - TunableValues.DRIVE_STICK_ZERO_DEG);
    }

    public double getPosXMm() {
        return xMm;
    }

    public double getPosYMm() {
        return yMm;
    }

    public int getLeftTicks() {
        return leftTicks;
    }

    public int getRightTicks() {
        return rightTicks;
    }

    public int getBackTicks() {
        return backTicks;
    }

    public boolean hasLocalizerVelocity() {
        return poseValid;
    }

    public double getVelXMmS() {
        return velXMmS;
    }

    public double getVelYMmS() {
        return velYMmS;
    }

    public double getVelHeadingRadS() {
        return velHeadingRadS;
    }

    public void logOdometryDiagnostics() {
        if (!isPresent()) {
            return;
        }
        boolean active = Math.abs(xMm - lastLoggedXMm) > 0.5
                || Math.abs(yMm - lastLoggedYMm) > 0.5
                || Math.abs(wrapRadians(headingRad - lastLoggedHeadingRad)) > Math.toRadians(0.1);
        if (!active && !wasActive) {
            return;
        }
        if (active == wasActive && logTimer.seconds() < LOG_PERIOD_SEC) {
            return;
        }
        RobotLog.i("[ODO] PODS L=%d R=%d B=%d  POSE x=%.1f y=%.1f in heading=%.1f  vx=%.0f vy=%.0f mm/s  w=%.2f rad/s",
                leftTicks, rightTicks, backTicks, xMm / 25.4, yMm / 25.4, getHeadingDegrees(),
                velXMmS, velYMmS, velHeadingRadS);
        logTimer.reset();
        wasActive = active;
        lastLoggedXMm = xMm;
        lastLoggedYMm = yMm;
        lastLoggedHeadingRad = headingRad;
    }

    private void setPose(Pose2d pose) {
        leftBaseTicks = leftTicks;
        rightBaseTicks = rightTicks;
        backBaseTicks = backTicks;
        lastLeftTicks = leftTicks;
        lastRightTicks = rightTicks;
        lastBackTicks = backTicks;
        odometry.updatePose(pose);
        xMm = pose.getX();
        yMm = pose.getY();
        headingRad = pose.getHeading();
        zeroVelocity();
        velocityTimer.reset();
    }

    private void zeroVelocity() {
        velXMmS = 0.0;
        velYMmS = 0.0;
        velHeadingRadS = 0.0;
    }

    private void readEncoders() {
        if (leftPod != null) {
            try {
                leftTicks = leftPod.getCurrentPosition() * TunableValues.ODO_LEFT_DIR;
                leftError = null;
            } catch (RuntimeException e) {
                leftError = "left pod disconnected";
            }
        }
        if (rightPod != null) {
            try {
                rightTicks = rightPod.getCurrentPosition() * TunableValues.ODO_RIGHT_DIR;
                rightError = null;
            } catch (RuntimeException e) {
                rightError = "right pod disconnected";
            }
        }
        if (backPod != null) {
            try {
                backTicks = backPod.getCurrentPosition() * TunableValues.ODO_BACK_DIR;
                backError = null;
            } catch (RuntimeException e) {
                backError = "back pod disconnected";
            }
        }
    }

    private static DcMotorEx asEncoder(DcMotor motor, String podName) {
        if (!(motor instanceof DcMotorEx)) {
            RobotLog.w("[ODO] %s is not in the hardware map.", podName);
            return null;
        }
        return (DcMotorEx) motor;
    }

    private static double wrapRadians(double heading) {
        while (heading > Math.PI) {
            heading -= 2.0 * Math.PI;
        }
        while (heading <= -Math.PI) {
            heading += 2.0 * Math.PI;
        }
        return heading;
    }

    private static double wrapDegrees(double heading) {
        while (heading > 180.0) {
            heading -= 360.0;
        }
        while (heading <= -180.0) {
            heading += 360.0;
        }
        return heading;
    }
}
