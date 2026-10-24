package org.firstinspires.ftc.teamcode.control;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.config.TunableValues;

/**
 * Cancels motion on axes the driver is not commanding. Does not lock heading
 * or change speed on an axis the stick is holding.
 *
 * OctoQuad field frame at heading 0: +X forward, +Y right, heading CCW.
 * Stick frame: +y forward, +x right, +rx clockwise.
 */
public class DriveVelocityAssist {
    private static final double MIN_DT_SEC = 0.005;
    private static final double MAX_DT_SEC = 0.10;
    private static final double STICK_DEADZONE = 0.05;

    private final PidController pidX = new PidController();
    private final PidController pidY = new PidController();
    private final PidController pidRx = new PidController();
    private final ElapsedTime timer = new ElapsedTime();
    private final ElapsedTime logTimer = new ElapsedTime();
    private boolean wasAssisting;

    private double cmdX;
    private double cmdY;
    private double cmdRx;
    private double measX;
    private double measY;
    private double measRx;
    private double corrX;
    private double corrY;
    private double corrRx;
    private double outX;
    private double outY;
    private double outRx;

    public void reset() {
        pidX.reset();
        pidY.reset();
        pidRx.reset();
        timer.reset();
        corrX = 0.0;
        corrY = 0.0;
        corrRx = 0.0;
        outX = 0.0;
        outY = 0.0;
        outRx = 0.0;
        cmdX = 0.0;
        cmdY = 0.0;
        cmdRx = 0.0;
        measX = 0.0;
        measY = 0.0;
        measRx = 0.0;
    }

    public void apply(double x, double y, double rx, double velXMmS, double velYMmS, double velHeadingRadS) {
        cmdX = x;
        cmdY = y;
        cmdRx = rx;

        double maxVel = Math.max(TunableValues.DRIVE_ASSIST_MAX_VEL_MM_S, 1.0);
        double maxOmega = Math.max(TunableValues.DRIVE_ASSIST_MAX_OMEGA_RAD_S, 0.01);
        measX = velYMmS / maxVel;
        measY = velXMmS / maxVel;
        measRx = -velHeadingRadS / maxOmega;

        if (!TunableValues.DRIVE_ASSIST_ENABLED
                || (Math.abs(x) <= STICK_DEADZONE
                && Math.abs(y) <= STICK_DEADZONE
                && Math.abs(rx) <= STICK_DEADZONE)) {
            pidX.reset();
            pidY.reset();
            pidRx.reset();
            timer.reset();
            corrX = 0.0;
            corrY = 0.0;
            corrRx = 0.0;
            outX = x;
            outY = y;
            outRx = rx;
            logIfDue(false);
            return;
        }

        double dt = timer.seconds();
        timer.reset();
        if (dt < MIN_DT_SEC) {
            dt = MIN_DT_SEC;
        } else if (dt > MAX_DT_SEC) {
            dt = MAX_DT_SEC;
        }

        pidX.setGains(
                TunableValues.DRIVE_ASSIST_TRANSLATION_P,
                TunableValues.DRIVE_ASSIST_TRANSLATION_I,
                TunableValues.DRIVE_ASSIST_TRANSLATION_D,
                TunableValues.DRIVE_ASSIST_I_LIMIT,
                TunableValues.DRIVE_ASSIST_MAX_CORRECTION);
        pidY.setGains(
                TunableValues.DRIVE_ASSIST_TRANSLATION_P,
                TunableValues.DRIVE_ASSIST_TRANSLATION_I,
                TunableValues.DRIVE_ASSIST_TRANSLATION_D,
                TunableValues.DRIVE_ASSIST_I_LIMIT,
                TunableValues.DRIVE_ASSIST_MAX_CORRECTION);
        pidRx.setGains(
                TunableValues.DRIVE_ASSIST_ROTATION_P,
                TunableValues.DRIVE_ASSIST_ROTATION_I,
                TunableValues.DRIVE_ASSIST_ROTATION_D,
                TunableValues.DRIVE_ASSIST_I_LIMIT,
                TunableValues.DRIVE_ASSIST_MAX_CORRECTION);

        corrX = correctionFor(pidX, x, measX, dt);
        corrY = correctionFor(pidY, y, measY, dt);
        corrRx = correctionFor(pidRx, rx, measRx, dt);
        outX = x + corrX;
        outY = y + corrY;
        outRx = rx + corrRx;
        logIfDue(true);
    }

    public double getX() {
        return outX;
    }

    public double getY() {
        return outY;
    }

    public double getRx() {
        return outRx;
    }

    public String getStatus() {
        return String.format(
                "cmd y=%.2f x=%.2f rx=%.2f  meas y=%.2f x=%.2f rx=%.2f  corr y=%.2f x=%.2f rx=%.2f  out y=%.2f x=%.2f rx=%.2f",
                cmdY, cmdX, cmdRx, measY, measX, measRx, corrY, corrX, corrRx, outY, outX, outRx);
    }

    private static double correctionFor(PidController pid, double command, double measured, double dt) {
        if (Math.abs(command) > STICK_DEADZONE) {
            pid.reset();
            return 0.0;
        }
        return pid.calculate(0.0, measured, dt);
    }

    private void logIfDue(boolean assisting) {
        if (!assisting && !wasAssisting) {
            return;
        }
        if (assisting == wasAssisting && logTimer.seconds() < 0.10) {
            return;
        }
        RobotLog.i("[ASSIST] %s", getStatus());
        logTimer.reset();
        wasAssisting = assisting;
    }
}
