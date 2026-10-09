package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.IronEaglesTeleOp2026;
import org.firstinspires.ftc.teamcode.interfaces.DrivetrainInterface;
import org.firstinspires.ftc.teamcode.interfaces.TargetGeometryInterface;

/**
 * Concrete implementation of the Mecanum Drivetrain subsystem.
 */
public class MecanumDrivetrain implements DrivetrainInterface {

    public static int FRONT_LEFT_DIR  =  1;
    public static int FRONT_RIGHT_DIR = -1;
    public static int BACK_LEFT_DIR   =  1;
    public static int BACK_RIGHT_DIR  = -1;

    private final DcMotor fl, fr, bl, br;

    public MecanumDrivetrain(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br) {
        this.fl = fl;
        this.fr = fr;
        this.bl = bl;
        this.br = br;
    }

    @Override
    public void drive(double x, double y, double rx) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
        double flP = ((y + x + rx) / denominator) * FRONT_LEFT_DIR;
        double blP = ((y - x + rx) / denominator) * BACK_LEFT_DIR;
        double frP = ((y - x - rx) / denominator) * FRONT_RIGHT_DIR;
        double brP = ((y + x - rx) / denominator) * BACK_RIGHT_DIR;

        if (fl != null) fl.setPower(flP);
        if (fr != null) fr.setPower(frP);
        if (bl != null) bl.setPower(blP);
        if (br != null) br.setPower(brP);
    }

    @Override
    public boolean alignToTarget(TargetGeometryInterface target) {
        if (target == null) {
            stop();
            return false;
        }

        double rangeError   = target.getRangeInches() - 24.0; // Target distance 24 inches
        double bearingError = target.getBearingDegrees();    // Target angle 0 degrees

        // Proportional Gain Constants (Kp)
        double Kp_forward = 0.03;
        double Kp_turn    = 0.02;

        double forwardPower = rangeError * Kp_forward;
        double turnPower    = bearingError * Kp_turn;

        drive(0, forwardPower, turnPower);

        // Returns true when aligned within tolerance (+/- 1.0 inch and +/- 2.0 degrees)
        return Math.abs(rangeError) < 1.0 && Math.abs(bearingError) < 2.0;
    }

    @Override
    public void stop() {
        if (fl != null) fl.setPower(0);
        if (fr != null) fr.setPower(0);
        if (bl != null) bl.setPower(0);
        if (br != null) br.setPower(0);
    }
}
