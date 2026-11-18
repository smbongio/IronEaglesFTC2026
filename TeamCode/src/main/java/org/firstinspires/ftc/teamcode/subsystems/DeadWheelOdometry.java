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
 * {@code leftPod}  front-to-back wheel on the left side  (ehPort1 encoder, no motor).
 * {@code rightPod} front-to-back wheel on the right side (ehPort0 encoder, intake motor's port).
 * {@code rearPod}  side-to-side X wheel at the rear      (ehPort3 encoder, no motor).
 *
 * Field frame: +X audience, +Y blue, heading 0 faces +X, counter-clockwise positive.
 */
public class DeadWheelOdometry {
    private static final double LOG_PERIOD_SEC = 0.10;
    private static final int MAX_TICK_STEP = 8000;

    private final DcMotorEx leftPod;
    private final DcMotorEx rightPod;
    private final DcMotorEx rearPod;
    private final HolonomicOdometry odometry;
    private final double[][] alignmentCorrection;
    private final ElapsedTime logTimer = new ElapsedTime();
    private final ElapsedTime velocityTimer = new ElapsedTime();

    private int leftTicks;
    private int rightTicks;
    private int rearTicks;
    private int lastLeftTicks;
    private int lastRightTicks;
    private int lastRearTicks;
    private int leftBaseTicks;
    private int rightBaseTicks;
    private int rearBaseTicks;
    private String leftError;
    private String rightError;
    private String rearError;

    private double xMm;
    private double yMm;
    private double headingRad;
    private double totalTurnRad;
    private double velXMmS;
    private double velYMmS;
    private double velHeadingRadS;
    private boolean poseValid;

    private boolean wasActive;
    private double lastLoggedXMm;
    private double lastLoggedYMm;
    private double lastLoggedHeadingRad;

    private double startHeadingDeg;

    public DeadWheelOdometry(DcMotor leftPodMotor, DcMotor rightPodMotor, DcMotor rearPodMotor,
                             double startXIn, double startYIn, double startHeadingDeg) {
        leftPod = asEncoder(leftPodMotor, "left pod");
        rightPod = asEncoder(rightPodMotor, "right pod");
        rearPod = asEncoder(rearPodMotor, "rear pod");
        if (leftPod == null) {
            leftError = "left pod missing";
        }
        if (rightPod == null) {
            rightError = "right pod missing";
        }
        if (rearPod == null) {
            rearError = "rear pod missing";
        }

        this.startHeadingDeg = startHeadingDeg;
        Pose2d start = poseFromInches(startXIn, startYIn, startHeadingDeg);
        odometry = new HolonomicOdometry(start,
                TunableValues.ODO_TRACK_WIDTH_MM, TunableValues.ODO_REAR_POD_X_MM);
        alignmentCorrection = buildAlignmentCorrection();
        readEncoders();
        setPose(start);
        RobotLog.i("[ODO] initialized. start FIELD X=%.1f Y=%.1f in heading=%.1f  trackWidth=%.2f in rearPodX=%.2f in  ticksPerMm=%.3f  LEFT=%s RIGHT=%s REAR=%s",
                startXIn,
                startYIn,
                startHeadingDeg,
                TunableValues.ODO_TRACK_WIDTH_MM / 25.4,
                TunableValues.ODO_REAR_POD_X_MM / 25.4,
                TunableValues.ODO_TICKS_PER_MM,
                leftPod != null ? "ehPort1" : "MISSING",
                rightPod != null ? "ehPort0" : "MISSING",
                rearPod != null ? "ehPort3" : "MISSING");
        RobotLog.i("[ODO] alignment LEFT per sideways=%.4f RIGHT per sideways=%.4f REAR per forward=%.4f",
                TunableValues.ODO_LEFT_PER_SIDEWAYS,
                TunableValues.ODO_RIGHT_PER_SIDEWAYS,
                TunableValues.ODO_REAR_PER_FORWARD);
    }

    /**
     * Maps measured pod distances to what perfectly aligned pods would read.
     * Robot motion (forward f, sideways s at the pod midpoint, turn t) drives the pods as
     * LEFT = f + kL*s - (T/2)*t, RIGHT = f + kR*s + (T/2)*t, REAR = s + xR*t + kB*f.
     * Ideal pods have kL = kR = kB = 0, so the correction is ideal * inverse(measured).
     */
    private static double[][] buildAlignmentCorrection() {
        double halfTrack = TunableValues.ODO_TRACK_WIDTH_MM / 2.0;
        double rearX = TunableValues.ODO_REAR_POD_X_MM;
        double[][] measured = {
                {1.0, TunableValues.ODO_LEFT_PER_SIDEWAYS, -halfTrack},
                {1.0, TunableValues.ODO_RIGHT_PER_SIDEWAYS, halfTrack},
                {TunableValues.ODO_REAR_PER_FORWARD, 1.0, rearX},
        };
        double[][] ideal = {
                {1.0, 0.0, -halfTrack},
                {1.0, 0.0, halfTrack},
                {0.0, 1.0, rearX},
        };
        return multiply(ideal, invert(measured));
    }

    public void update() {
        readEncoders();
        if (!isPresent()) {
            poseValid = false;
            return;
        }
        int dLeft = leftTicks - lastLeftTicks;
        int dRight = rightTicks - lastRightTicks;
        int dRear = rearTicks - lastRearTicks;
        lastLeftTicks = leftTicks;
        lastRightTicks = rightTicks;
        lastRearTicks = rearTicks;
        if (Math.abs(dLeft) > MAX_TICK_STEP || Math.abs(dRight) > MAX_TICK_STEP
                || Math.abs(dRear) > MAX_TICK_STEP) {
            RobotLog.w("[ODO] skipped encoder step LEFT=%d RIGHT=%d REAR=%d ticks", dLeft, dRight, dRear);
            leftBaseTicks += dLeft;
            rightBaseTicks += dRight;
            rearBaseTicks += dRear;
            zeroVelocity();
            return;
        }

        double perMm = TunableValues.ODO_TICKS_PER_MM;
        double rawLeftMm = (leftTicks - leftBaseTicks) / perMm;
        double rawRightMm = (rightTicks - rightBaseTicks) / perMm;
        double rawRearMm = (rearTicks - rearBaseTicks) / perMm;
        double[][] c = alignmentCorrection;
        double leftMm = c[0][0] * rawLeftMm + c[0][1] * rawRightMm + c[0][2] * rawRearMm;
        double rightMm = c[1][0] * rawLeftMm + c[1][1] * rawRightMm + c[1][2] * rawRearMm;
        double rearMm = c[2][0] * rawLeftMm + c[2][1] * rawRightMm + c[2][2] * rawRearMm;
        // FTCLib computes heading as (left - right) / trackWidth, which is clockwise-positive.
        // Passing right first makes it counter-clockwise-positive like the field frame.
        odometry.update(rightMm, leftMm, rearMm);

        Pose2d pose = odometry.getPose();
        double newX = pose.getX();
        double newY = pose.getY();
        double newHeading = pose.getHeading();
        double dHeading = wrapRadians(newHeading - headingRad);
        totalTurnRad += dHeading;
        double dt = velocityTimer.seconds();
        velocityTimer.reset();
        if (dt > 0.001) {
            velXMmS = (newX - xMm) / dt;
            velYMmS = (newY - yMm) / dt;
            velHeadingRadS = dHeading / dt;
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
        if (rearError != null) {
            errors.add(rearError);
        }
        return errors;
    }

    public boolean isPresent() {
        return leftPod != null && rightPod != null && rearPod != null;
    }

    /**
     * Moves the pose to a new start (e.g. the alliance start or Autonomous' final pose).
     * The Back button then restores this heading.
     */
    public void setPoseInches(double xIn, double yIn, double headingDeg) {
        startHeadingDeg = headingDeg;
        readEncoders();
        setPose(poseFromInches(xIn, yIn, headingDeg));
        RobotLog.i("[ODO] Pose set to FIELD X=%.1f Y=%.1f in heading=%.1f", xIn, yIn, headingDeg);
    }

    /** Keeps field X/Y and puts heading back to the start heading (Back button). */
    public void resetHeading() {
        if (!isPresent()) {
            return;
        }
        setPose(new Pose2d(xMm, yMm, new Rotation2d(Math.toRadians(startHeadingDeg))));
        RobotLog.i("[ODO] Heading restored to start %.1f deg (Back button).", startHeadingDeg);
    }

    public double getHeadingDegrees() {
        return Math.toDegrees(headingRad);
    }

    /** Signed degrees turned since init, not wrapped (10 CCW spins = +3600). Back button does not add to it. */
    public double getTotalTurnDegrees() {
        return Math.toDegrees(totalTurnRad);
    }

    /**
     * Heading for field-centric drive: field heading minus the field direction the
     * driver's stick-up points along (red +Y = 90, blue -Y = -90).
     */
    public double getDriveHeadingDegrees(double stickZeroDeg) {
        return wrapDegrees(getHeadingDegrees() - stickZeroDeg);
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

    public int getRearTicks() {
        return rearTicks;
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
        RobotLog.i("[ODO] PODS LEFT=%d RIGHT=%d REAR=%d  FIELD X=%.1f Y=%.1f in heading=%.1f totalTurn=%.1f  vX=%.0f vY=%.0f mm/s  w=%.2f rad/s",
                leftTicks, rightTicks, rearTicks, xMm / 25.4, yMm / 25.4, getHeadingDegrees(),
                getTotalTurnDegrees(), velXMmS, velYMmS, velHeadingRadS);
        logTimer.reset();
        wasActive = active;
        lastLoggedXMm = xMm;
        lastLoggedYMm = yMm;
        lastLoggedHeadingRad = headingRad;
    }

    private void setPose(Pose2d pose) {
        leftBaseTicks = leftTicks;
        rightBaseTicks = rightTicks;
        rearBaseTicks = rearTicks;
        lastLeftTicks = leftTicks;
        lastRightTicks = rightTicks;
        lastRearTicks = rearTicks;
        odometry.updatePose(pose);
        xMm = pose.getX();
        yMm = pose.getY();
        headingRad = pose.getHeading();
        zeroVelocity();
        velocityTimer.reset();
    }

    private static Pose2d poseFromInches(double xIn, double yIn, double headingDeg) {
        return new Pose2d(xIn * 25.4, yIn * 25.4, new Rotation2d(Math.toRadians(headingDeg)));
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
        if (rearPod != null) {
            try {
                rearTicks = rearPod.getCurrentPosition() * TunableValues.ODO_REAR_DIR;
                rearError = null;
            } catch (RuntimeException e) {
                rearError = "rear pod disconnected";
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

    private static double[][] invert(double[][] m) {
        double a = m[0][0], b = m[0][1], c = m[0][2];
        double d = m[1][0], e = m[1][1], f = m[1][2];
        double g = m[2][0], h = m[2][1], i = m[2][2];
        double det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
        return new double[][] {
                {(e * i - f * h) / det, (c * h - b * i) / det, (b * f - c * e) / det},
                {(f * g - d * i) / det, (a * i - c * g) / det, (c * d - a * f) / det},
                {(d * h - e * g) / det, (b * g - a * h) / det, (a * e - b * d) / det},
        };
    }

    private static double[][] multiply(double[][] x, double[][] y) {
        double[][] out = new double[3][3];
        for (int r = 0; r < 3; r++) {
            for (int col = 0; col < 3; col++) {
                out[r][col] = x[r][0] * y[0][col] + x[r][1] * y[1][col] + x[r][2] * y[2][col];
            }
        }
        return out;
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
