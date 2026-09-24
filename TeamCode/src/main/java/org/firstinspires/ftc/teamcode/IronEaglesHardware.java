package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.interfaces.DrivetrainInterface;
import org.firstinspires.ftc.teamcode.interfaces.HopperInterface;
import org.firstinspires.ftc.teamcode.interfaces.IntakeInterface;
import org.firstinspires.ftc.teamcode.interfaces.RobotHardwareInterface;
import org.firstinspires.ftc.teamcode.interfaces.ShooterInterface;

import java.util.ArrayList;
import java.util.List;

/**
 * Concrete hardware implementation mapping Control Hub and Expansion Hub peripherals.
 */
public class IronEaglesHardware implements RobotHardwareInterface {

    // Control Hub Motors
    public DcMotor frontRight;
    public DcMotor backRight;
    public DcMotor backLeft;
    public DcMotor frontLeft;
    public WebcamName camera;

    // Expansion Hub Motors
    public DcMotor intake;
    public DcMotor hopper;
    public DcMotor shooter;

    public final List<String> missingDevices = new ArrayList<>();

    private DrivetrainSubsystem drivetrainSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private HopperSubsystem hopperSubsystem;
    private ShooterSubsystem shooterSubsystem;

    private <T> T getHardwareSafely(HardwareMap hardwareMap, Class<T> clazz, String name) {
        try {
            T device = hardwareMap.tryGet(clazz, name);
            if (device == null) {
                missingDevices.add(name);
            }
            return device;
        } catch (Exception e) {
            missingDevices.add(name);
            return null;
        }
    }

    @Override
    public void init(HardwareMap hardwareMap) {
        missingDevices.clear();

        // Enable AUTO Bulk Caching Mode for all REV Hubs to optimize cycle speed
        try {
            List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
            for (LynxModule hub : allHubs) {
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }
        } catch (Exception ignored) {
        }

        // Control Hub Motors
        frontRight = getHardwareSafely(hardwareMap, DcMotor.class, "frontRight");
        backRight  = getHardwareSafely(hardwareMap, DcMotor.class, "backRight");
        backLeft   = getHardwareSafely(hardwareMap, DcMotor.class, "backLeft");
        frontLeft  = getHardwareSafely(hardwareMap, DcMotor.class, "frontLeft");

        // Expansion Hub Motors
        intake  = getHardwareSafely(hardwareMap, DcMotor.class, "intake");
        hopper  = getHardwareSafely(hardwareMap, DcMotor.class, "hopper");
        shooter = getHardwareSafely(hardwareMap, DcMotor.class, "shooter");

        // Webcam
        camera = getHardwareSafely(hardwareMap, WebcamName.class, "camera");

        // Configure brake behavior on drivetrain motors if present
        if (frontLeft != null)  frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (frontRight != null) frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (backLeft != null)   backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (backRight != null)  backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Instantiate Subsystems
        drivetrainSubsystem = new DrivetrainSubsystem(frontLeft, frontRight, backLeft, backRight);
        intakeSubsystem     = new IntakeSubsystem(intake);
        hopperSubsystem     = new HopperSubsystem(hopper);
        shooterSubsystem    = new ShooterSubsystem(shooter);
    }

    @Override
    public DrivetrainInterface getDrivetrain() {
        return drivetrainSubsystem;
    }

    @Override
    public IntakeInterface getIntake() {
        return intakeSubsystem;
    }

    @Override
    public HopperInterface getHopper() {
        return hopperSubsystem;
    }

    @Override
    public ShooterInterface getShooter() {
        return shooterSubsystem;
    }

    // Inner Subsystem implementations
    public static class DrivetrainSubsystem implements DrivetrainInterface {
        private final DcMotor fl, fr, bl, br;

        public DrivetrainSubsystem(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br) {
            this.fl = fl;
            this.fr = fr;
            this.bl = bl;
            this.br = br;
        }

        @Override
        public void drive(double x, double y, double rx) {
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
            double flP = ((y + x + rx) / denominator) * IronEaglesTeleOp2026.FRONT_LEFT_DIR;
            double blP = ((y - x + rx) / denominator) * IronEaglesTeleOp2026.BACK_LEFT_DIR;
            double frP = ((y - x - rx) / denominator) * IronEaglesTeleOp2026.FRONT_RIGHT_DIR;
            double brP = ((y + x - rx) / denominator) * IronEaglesTeleOp2026.BACK_RIGHT_DIR;

            if (fl != null) fl.setPower(flP);
            if (fr != null) fr.setPower(frP);
            if (bl != null) bl.setPower(blP);
            if (br != null) br.setPower(brP);
        }

        @Override
        public void stop() {
            if (fl != null) fl.setPower(0);
            if (fr != null) fr.setPower(0);
            if (bl != null) bl.setPower(0);
            if (br != null) br.setPower(0);
        }
    }

    public static class IntakeSubsystem implements IntakeInterface {
        private final DcMotor motor;

        public IntakeSubsystem(DcMotor motor) {
            this.motor = motor;
        }

        @Override
        public void runIntake(double speed) {
            setPower(speed);
        }

        @Override
        public void setPower(double power) {
            if (motor != null) {
                motor.setPower(power * IronEaglesTeleOp2026.INTAKE_DIR);
            }
        }

        @Override
        public void stop() {
            if (motor != null) {
                motor.setPower(0);
            }
        }

        @Override
        public boolean isRunning() {
            return motor != null && Math.abs(motor.getPower()) > 0.01;
        }
    }

    public static class HopperSubsystem implements HopperInterface {
        private final DcMotor motor;

        public HopperSubsystem(DcMotor motor) {
            this.motor = motor;
        }

        @Override
        public void runHopper(double speed) {
            setPower(speed);
        }

        @Override
        public void setPower(double power) {
            if (motor != null) {
                motor.setPower(power * IronEaglesTeleOp2026.HOPPER_DIR);
            }
        }

        @Override
        public void stop() {
            if (motor != null) {
                motor.setPower(0);
            }
        }

        @Override
        public boolean isRunning() {
            return motor != null && Math.abs(motor.getPower()) > 0.01;
        }
    }

    public static class ShooterSubsystem implements ShooterInterface {
        private final DcMotor motor;

        public ShooterSubsystem(DcMotor motor) {
            this.motor = motor;
        }

        @Override
        public void runShooter(double speed) {
            setPower(speed);
        }

        @Override
        public void setPower(double power) {
            if (motor != null) {
                motor.setPower(power * IronEaglesTeleOp2026.SHOOTER_DIR);
            }
        }

        @Override
        public void stop() {
            if (motor != null) {
                motor.setPower(0);
            }
        }

        @Override
        public boolean isRunning() {
            return motor != null && Math.abs(motor.getPower()) > 0.01;
        }
    }
}
