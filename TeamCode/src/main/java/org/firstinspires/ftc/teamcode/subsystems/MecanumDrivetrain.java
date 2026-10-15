package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.config.TunableValues;
import org.firstinspires.ftc.teamcode.geometry.MovementPlan;
import org.firstinspires.ftc.teamcode.interfaces.DrivetrainInterface;
import org.firstinspires.ftc.teamcode.interfaces.ImuInterface;
import org.firstinspires.ftc.teamcode.util.RobotLogger;

public class MecanumDrivetrain implements DrivetrainInterface {
    private static final int FL_DIR = 1;
    private static final int FR_DIR = -1;
    private static final int BL_DIR = 1;
    private static final int BR_DIR = -1;
    private static final double HEADING_TOLERANCE_DEG = 3.0;

    private final RobotLogger log = new RobotLogger("DRIVE");
    private final DcMotorEx fl;
    private final DcMotorEx fr;
    private final DcMotorEx bl;
    private final DcMotorEx br;
    private final ImuInterface imu;

    private boolean moving;
    private boolean translating;
    private int[] startTicks = new int[4];
    private int[] targetDelta = new int[4];
    private double targetHeadingDeg;

    public MecanumDrivetrain(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br, ImuInterface imu) {
        this.fl = asEx(fl);
        this.fr = asEx(fr);
        this.bl = asEx(bl);
        this.br = asEx(br);
        this.imu = imu;
        configure(this.fl);
        configure(this.fr);
        configure(this.bl);
        configure(this.br);
        log.i("Drive motors present=%b", areDriveMotorsPresent());
    }

    private static DcMotorEx asEx(DcMotor motor) {
        return motor instanceof DcMotorEx ? (DcMotorEx) motor : null;
    }

    private void configure(DcMotorEx motor) {
        if (motor == null) {
            return;
        }
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public boolean areDriveMotorsPresent() {
        return fl != null && fr != null && bl != null && br != null;
    }

    @Override
    public void driveFieldCentric(double x, double y, double rx, double headingDeg) {
        double headingRad = Math.toRadians(headingDeg);
        double rotX = x * Math.cos(-headingRad) - y * Math.sin(-headingRad);
        double rotY = x * Math.sin(-headingRad) + y * Math.cos(-headingRad);
        driveRobotCentric(rotX, rotY, rx);
    }

    @Override
    public void driveRobotCentric(double x, double y, double rx) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        setPowers(
                ((y + x + rx) / denominator) * FL_DIR,
                ((y - x - rx) / denominator) * FR_DIR,
                ((y - x + rx) / denominator) * BL_DIR,
                ((y + x - rx) / denominator) * BR_DIR);
    }

    @Override
    public void startMove(MovementPlan plan) {
        if (!areDriveMotorsPresent() || plan == null) {
            log.w("startMove ignored. motors=%b plan=%s", areDriveMotorsPresent(), plan);
            moving = false;
            return;
        }
        startTicks = currentTicks();
        double ticksPerInch = ticksPerInch();
        double flIn = plan.driveInches + plan.strafeInches;
        double frIn = plan.driveInches - plan.strafeInches;
        double blIn = plan.driveInches - plan.strafeInches;
        double brIn = plan.driveInches + plan.strafeInches;
        targetDelta = new int[] {
                (int) Math.round(flIn * ticksPerInch * FL_DIR),
                (int) Math.round(frIn * ticksPerInch * FR_DIR),
                (int) Math.round(blIn * ticksPerInch * BL_DIR),
                (int) Math.round(brIn * ticksPerInch * BR_DIR)
        };
        targetHeadingDeg = wrap(imu.getHeadingDegrees() + plan.turnDegrees);
        translating = Math.abs(plan.driveInches) > 0.5 || Math.abs(plan.strafeInches) > 0.5;
        moving = true;
        log.i("startMove %s ticksPerInch=%.1f targetHeading=%.1f translating=%b",
                plan, ticksPerInch, targetHeadingDeg, translating);
    }

    @Override
    public boolean isMoveFinished() {
        if (!moving) {
            return true;
        }
        if (translating) {
            if (!translationDone()) {
                driveTowardTargets();
                return false;
            }
            log.i("Translation finished. Starting turn to %.1f", targetHeadingDeg);
            translating = false;
        }
        double error = wrap(targetHeadingDeg - imu.getHeadingDegrees());
        if (Math.abs(error) <= HEADING_TOLERANCE_DEG) {
            stop();
            moving = false;
            log.i("Move finished. heading=%.1f", imu.getHeadingDegrees());
            return true;
        }
        // IMU CCW+, drive rx clockwise+
        double turn = -Math.copySign(TunableValues.AUTO_FIRE_TURN_POWER, error);
        driveRobotCentric(0, 0, turn);
        return false;
    }

    @Override
    public void stop() {
        moving = false;
        translating = false;
        setPowers(0, 0, 0, 0);
    }

    private void driveTowardTargets() {
        double[] errors = new double[4];
        int[] now = currentTicks();
        double max = 1;
        for (int i = 0; i < 4; i++) {
            errors[i] = targetDelta[i] - (now[i] - startTicks[i]);
            max = Math.max(max, Math.abs(errors[i]));
        }
        double scale = TunableValues.AUTO_FIRE_DRIVE_POWER / max;
        setPowers(clamp(errors[0] * scale), clamp(errors[1] * scale),
                clamp(errors[2] * scale), clamp(errors[3] * scale));
    }

    private boolean translationDone() {
        int[] now = currentTicks();
        for (int i = 0; i < 4; i++) {
            if (Math.abs(targetDelta[i] - (now[i] - startTicks[i])) > ticksPerInch() * 0.75) {
                return false;
            }
        }
        return true;
    }

    private int[] currentTicks() {
        return new int[] {
                fl.getCurrentPosition(), fr.getCurrentPosition(),
                bl.getCurrentPosition(), br.getCurrentPosition()
        };
    }

    private double ticksPerInch() {
        double ticksPerRev = fl.getMotorType().getTicksPerRev();
        if (ticksPerRev <= 1) {
            ticksPerRev = 537.7; // common goBILDA 312 RPM; logged if we had to guess
            log.w("Motor type ticksPerRev missing; using %.1f", ticksPerRev);
        }
        return ticksPerRev / (Math.PI * TunableValues.WHEEL_DIAMETER_IN);
    }

    private void setPowers(double flP, double frP, double blP, double brP) {
        if (fl != null) fl.setPower(flP);
        if (fr != null) fr.setPower(frP);
        if (bl != null) bl.setPower(blP);
        if (br != null) br.setPower(brP);
    }

    private static double clamp(double v) {
        return Math.max(-1.0, Math.min(1.0, v));
    }

    private static double wrap(double deg) {
        while (deg > 180) deg -= 360;
        while (deg <= -180) deg += 360;
        return deg;
    }
}
