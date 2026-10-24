package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.config.TunableValues;
import org.firstinspires.ftc.teamcode.interfaces.DrivetrainInterface;

/**
 * Concrete implementation of the Mecanum Drivetrain subsystem.
 */
public class MecanumDrivetrain implements DrivetrainInterface {

    private static final double LOG_PERIOD_SEC = 0.10;

    private final DcMotor fl, fr, bl, br;
    private final ElapsedTime logTimer = new ElapsedTime();

    private double stickX;
    private double stickY;
    private double stickRx;
    private double headingDeg;
    private double flPower;
    private double frPower;
    private double blPower;
    private double brPower;
    private boolean wasActive;

    public MecanumDrivetrain(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br) {
        this.fl = fl;
        this.fr = fr;
        this.bl = bl;
        this.br = br;
    }

    @Override
    public void drive(double x, double y, double rx) {
        stickX = x;
        stickY = y;
        stickRx = rx;
        applyMix(x, y, rx);
    }

    @Override
    public void driveFieldCentric(double x, double y, double rx, double headingDeg) {
        stickX = x;
        stickY = y;
        stickRx = rx;
        this.headingDeg = headingDeg;
        double headingRad = Math.toRadians(headingDeg);
        double rotX = x * Math.cos(-headingRad) - y * Math.sin(-headingRad);
        double rotY = x * Math.sin(-headingRad) + y * Math.cos(-headingRad);
        applyMix(rotX, rotY, rx);
    }

    @Override
    public void stop() {
        stickX = 0.0;
        stickY = 0.0;
        stickRx = 0.0;
        applyPowers(0.0, 0.0, 0.0, 0.0);
    }

    @Override
    public String getStickStatus() {
        return String.format("y=%.2f x=%.2f rx=%.2f heading=%.1f", stickY, stickX, stickRx, headingDeg);
    }

    @Override
    public String getRequestedPowerStatus() {
        return String.format("FL %.2f  FR %.2f  BL %.2f  BR %.2f", flPower, frPower, blPower, brPower);
    }

    @Override
    public String getTicksPerSecStatus() {
        return String.format("FL %.0f  FR %.0f  BL %.0f  BR %.0f",
                ticksPerSec(fl), ticksPerSec(fr), ticksPerSec(bl), ticksPerSec(br));
    }

    @Override
    public void logWheelDiagnostics() {
        boolean active = isActive();
        if (!active && !wasActive) {
            return;
        }
        if (active == wasActive && logTimer.seconds() < LOG_PERIOD_SEC) {
            return;
        }
        RobotLog.i("[WHEELS] %s  pwr %s  tps %s",
                getStickStatus(), getRequestedPowerStatus(), getTicksPerSecStatus());
        logTimer.reset();
        wasActive = active;
    }

    private void applyMix(double x, double y, double rx) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        applyPowers(
                ((y + x + rx) / denominator) * TunableValues.FRONT_LEFT_DIR,
                ((y - x - rx) / denominator) * TunableValues.FRONT_RIGHT_DIR,
                ((y - x + rx) / denominator) * TunableValues.BACK_LEFT_DIR,
                ((y + x - rx) / denominator) * TunableValues.BACK_RIGHT_DIR);
    }

    private void applyPowers(double flP, double frP, double blP, double brP) {
        flPower = flP;
        frPower = frP;
        blPower = blP;
        brPower = brP;
        if (fl != null) fl.setPower(flP);
        if (fr != null) fr.setPower(frP);
        if (bl != null) bl.setPower(blP);
        if (br != null) br.setPower(brP);
    }

    private boolean isActive() {
        return Math.abs(stickX) > 0.01
                || Math.abs(stickY) > 0.01
                || Math.abs(stickRx) > 0.01
                || Math.abs(flPower) > 0.01
                || Math.abs(frPower) > 0.01
                || Math.abs(blPower) > 0.01
                || Math.abs(brPower) > 0.01
                || Math.abs(ticksPerSec(fl)) > 1.0
                || Math.abs(ticksPerSec(fr)) > 1.0
                || Math.abs(ticksPerSec(bl)) > 1.0
                || Math.abs(ticksPerSec(br)) > 1.0;
    }

    private static double ticksPerSec(DcMotor motor) {
        if (!(motor instanceof DcMotorEx)) {
            return 0.0;
        }
        return ((DcMotorEx) motor).getVelocity();
    }
}
