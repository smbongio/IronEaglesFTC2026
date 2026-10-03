package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

/**
 * Concrete hardware implementation mapping Control Hub and Expansion Hub peripherals.
 */
public class IronEaglesHardware {

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

    public void logDiagnostics(HardwareMap hardwareMap) {
        RobotLog.i("=================== HARDWARE DIAGNOSTICS ===================");
        RobotLog.i("Total DcMotors found in active HardwareMap: " + hardwareMap.dcMotor.size());
        for (Map.Entry<String, DcMotor> entry : hardwareMap.dcMotor.entrySet()) {
            RobotLog.i(" -> Configured DcMotor: '" + entry.getKey() + "'");
        }

        try {
            List<WebcamName> webcams = hardwareMap.getAll(WebcamName.class);
            RobotLog.i("Total Webcams found in active HardwareMap: " + webcams.size());
        } catch (Exception e) {
            RobotLog.i("No webcams found in active HardwareMap.");
        }
        RobotLog.i("============================================================");
    }

    public void init(HardwareMap hardwareMap) {
        missingDevices.clear();
        logDiagnostics(hardwareMap);

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

    public DrivetrainSubsystem getDrivetrain() {
        return drivetrainSubsystem;
    }

    public IntakeSubsystem getIntake() {
        return intakeSubsystem;
    }

    public HopperSubsystem getHopper() {
        return hopperSubsystem;
    }

    public ShooterSubsystem getShooter() {
        return shooterSubsystem;
    }

    // Subsystem implementations
    public static class DrivetrainSubsystem {
        private final DcMotor fl, fr, bl, br;

        public DrivetrainSubsystem(DcMotor fl, DcMotor fr, DcMotor bl, DcMotor br) {
            this.fl = fl;
            this.fr = fr;
            this.bl = bl;
            this.br = br;
        }

        /**
         * Calculates Mecanum wheel powers and applies them to the 4 drive motors.
         *
         * @param x  Strafe input (-1.0 to 1.0, right is positive, left is negative)
         * @param y  Forward input (-1.0 to 1.0, forward is positive, backward is negative)
         * @param rx Rotation input (-1.0 to 1.0, clockwise is positive, counter-clockwise is negative)
         *
         * EXAMPLES & MATH DERIVATIONS:
         * -----------------------------------------------------------------------------------------
         * Case 1: Full Forward Drive (y = 1.0, x = 0.0, rx = 0.0)
         *   denominator = max(|1.0| + |0.0| + |0.0|, 1.0) = 1.0
         *   flP = (( 1.0 + 0.0 + 0.0) / 1.0) * FRONT_LEFT_DIR  (1) = +1.0 (Spin Forward 100%)
         *   blP = (( 1.0 - 0.0 + 0.0) / 1.0) * BACK_LEFT_DIR   (1) = +1.0 (Spin Forward 100%)
         *   frP = (( 1.0 - 0.0 - 0.0) / 1.0) * FRONT_RIGHT_DIR (1) = +1.0 (Spin Forward 100%)
         *   brP = (( 1.0 + 0.0 - 0.0) / 1.0) * BACK_RIGHT_DIR  (1) = +1.0 (Spin Forward 100%)
         *   Result: All 4 wheels spin forward in unison -> Chassis moves straight forward.
         *
         * Case 2: Full Strafe Right (y = 0.0, x = 1.0, rx = 0.0)
         *   denominator = max(|0.0| + |1.0| + |0.0|, 1.0) = 1.0
         *   flP = (( 0.0 + 1.0 + 0.0) / 1.0) * FRONT_LEFT_DIR  (1) = +1.0 (Spin Forward)
         *   blP = (( 0.0 - 1.0 + 0.0) / 1.0) * BACK_LEFT_DIR   (1) = -1.0 (Spin Backward)
         *   frP = (( 0.0 - 1.0 - 0.0) / 1.0) * FRONT_RIGHT_DIR (1) = -1.0 (Spin Backward)
         *   brP = (( 0.0 + 1.0 - 0.0) / 1.0) * BACK_RIGHT_DIR  (1) = +1.0 (Spin Forward)
         *   Result: 45° roller force vectors push the chassis directly to the right.
         *
         * Case 3: Clockwise Turn / Rotate Right (y = 0.0, x = 0.0, rx = 1.0)
         *   denominator = max(|0.0| + |0.0| + |1.0|, 1.0) = 1.0
         *   flP = (( 0.0 + 0.0 + 1.0) / 1.0) * FRONT_LEFT_DIR  (1) = +1.0 (Spin Forward)
         *   blP = (( 0.0 - 0.0 + 1.0) / 1.0) * BACK_LEFT_DIR   (1) = +1.0 (Spin Forward)
         *   frP = (( 0.0 - 0.0 - 1.0) / 1.0) * FRONT_RIGHT_DIR (1) = -1.0 (Spin Backward)
         *   brP = (( 0.0 + 0.0 - 1.0) / 1.0) * BACK_RIGHT_DIR  (1) = -1.0 (Spin Backward)
         *   Result: Left side drives forward, right side drives backward -> Chassis spins clockwise.
         * -----------------------------------------------------------------------------------------
         */
        public void drive(double x, double y, double rx) {
            // Step 1: Calculate the maximum potential power sum across all 3 movement vectors.
            // If the driver pushes joysticks diagonally while turning (e.g. y=1, x=1, rx=1),
            // the sum would equal 3.0. Since motor power can only range from -1.0 to 1.0,
            // we divide by 'denominator' to scale all 4 wheel powers down proportionally,
            // preventing power clipping and preserving the exact driving direction.
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);

            // Step 2: Apply Mecanum Kinematics Equations for each wheel.
            // Mecanum 45-degree rollers create diagonal force vectors:
            // - Forward/Backward (y) moves all 4 wheels in the same direction (+).
            // - Strafing Right (x) spins FrontLeft & BackRight forward (+), FrontRight & BackLeft backward (-).
            // - Turning Right (rx) spins Left wheels forward (+), Right wheels backward (-).
            // Each result is normalized by 'denominator' and multiplied by its direction constant (1 or -1).
            double flP = ((y + x + rx) / denominator) * IronEaglesTeleOp2026.FRONT_LEFT_DIR;
            double blP = ((y - x + rx) / denominator) * IronEaglesTeleOp2026.BACK_LEFT_DIR;
            double frP = ((y - x - rx) / denominator) * IronEaglesTeleOp2026.FRONT_RIGHT_DIR;
            double brP = ((y + x - rx) / denominator) * IronEaglesTeleOp2026.BACK_RIGHT_DIR;

            // Step 3: Write calculated power values (-1.0 to 1.0) to physical motors if present.
            if (fl != null) fl.setPower(flP);
            if (fr != null) fr.setPower(frP);
            if (bl != null) bl.setPower(blP);
            if (br != null) br.setPower(brP);
        }

        public void stop() {
            if (fl != null) fl.setPower(0);
            if (fr != null) fr.setPower(0);
            if (bl != null) bl.setPower(0);
            if (br != null) br.setPower(0);
        }
    }

    public static class IntakeSubsystem {
        private final DcMotor motor;

        public IntakeSubsystem(DcMotor motor) {
            this.motor = motor;
        }

        public void runIntake(double speed) {
            setPower(speed);
        }

        public void setPower(double power) {
            if (motor != null) {
                motor.setPower(power * IronEaglesTeleOp2026.INTAKE_DIR);
            }
        }

        public void stop() {
            if (motor != null) {
                motor.setPower(0);
            }
        }

        public boolean isRunning() {
            return motor != null && Math.abs(motor.getPower()) > 0.01;
        }
    }

    public static class HopperSubsystem {
        private final DcMotor motor;

        public HopperSubsystem(DcMotor motor) {
            this.motor = motor;
        }

        public void runHopper(double speed) {
            setPower(speed);
        }

        public void setPower(double power) {
            if (motor != null) {
                motor.setPower(power * IronEaglesTeleOp2026.HOPPER_DIR);
            }
        }

        public void stop() {
            if (motor != null) {
                motor.setPower(0);
            }
        }

        public boolean isRunning() {
            return motor != null && Math.abs(motor.getPower()) > 0.01;
        }
    }

    public static class ShooterSubsystem {
        private final DcMotor motor;

        public ShooterSubsystem(DcMotor motor) {
            this.motor = motor;
        }

        public void runShooter(double speed) {
            setPower(speed);
        }

        public void setPower(double power) {
            if (motor != null) {
                motor.setPower(power * IronEaglesTeleOp2026.SHOOTER_DIR);
            }
        }

        public void stop() {
            if (motor != null) {
                motor.setPower(0);
            }
        }

        public boolean isRunning() {
            return motor != null && Math.abs(motor.getPower()) > 0.01;
        }
    }
}
