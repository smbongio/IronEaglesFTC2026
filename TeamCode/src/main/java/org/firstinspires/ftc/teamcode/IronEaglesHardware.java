package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.interfaces.DrivetrainInterface;
import org.firstinspires.ftc.teamcode.interfaces.HopperInterface;
import org.firstinspires.ftc.teamcode.interfaces.IntakeInterface;
import org.firstinspires.ftc.teamcode.interfaces.RobotHardwareInterface;
import org.firstinspires.ftc.teamcode.interfaces.ShooterInterface;
import org.firstinspires.ftc.teamcode.subsystems.BlinkinLedSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HopperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

/**
 * Concrete hardware implementation mapping Control Hub and Expansion Hub peripherals.
 */
public class IronEaglesHardware implements RobotHardwareInterface {

    // Config names are the hub ports (see IronEaglesConfig.xml); fields name the use.
    public DcMotor frontRight;  // chPort0
    public DcMotor backRight;   // chPort1
    public DcMotor backLeft;    // chPort2
    public DcMotor frontLeft;   // chPort3
    public RevBlinkinLedDriver blinkin;  // chServo0
    public WebcamName camera;   // camera (USB)

    public DcMotor intake;      // ehPort0
    public DcMotor shooter;     // ehPort2
    public DcMotor shooterFollow;  // ehPort3. Same device object as rearPod.
    // REV Smart Servo, continuous mode.
    public CRServo hopper;      // ehServo0

    // Dead-wheel encoders. A pod on a motor port shares that port's one encoder jack.
    public DcMotor rightPod;    // ehPort0 encoder (front-to-back). Same device object as intake.
    public DcMotor leftPod;     // ehPort1 encoder (front-to-back). No motor on this port.
    public DcMotor rearPod;     // ehPort3 encoder, the X wheel (side-to-side). Same device as shooterFollow.

    public final List<String> missingDevices = new ArrayList<>();

    private MecanumDrivetrain drivetrainSubsystem;
    private IntakeSubsystem intakeSubsystem;
    private HopperSubsystem hopperSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private BlinkinLedSubsystem ledSubsystem;

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
            RobotLog.i("Total Servos found in active HardwareMap: " + hardwareMap.servo.size());
            for (Map.Entry<String, Servo> entry : hardwareMap.servo.entrySet()) {
                RobotLog.i(" -> Configured Servo: '" + entry.getKey() + "'");
            }
        } catch (Exception ignored) {}

        try {
            RobotLog.i("Total CRServos found in active HardwareMap: " + hardwareMap.crservo.size());
            for (Map.Entry<String, CRServo> entry : hardwareMap.crservo.entrySet()) {
                RobotLog.i(" -> Configured CRServo: '" + entry.getKey() + "'");
            }
        } catch (Exception ignored) {}

        try {
            List<WebcamName> webcams = hardwareMap.getAll(WebcamName.class);
            RobotLog.i("Total Webcams found in active HardwareMap: " + webcams.size());
        } catch (Exception e) {
            RobotLog.i("No webcams found in active HardwareMap.");
        }

        try {
            List<RevBlinkinLedDriver> blinkins = hardwareMap.getAll(RevBlinkinLedDriver.class);
            RobotLog.i("Total Blinkin LED Drivers found in active HardwareMap: " + blinkins.size());
        } catch (Exception e) {
            RobotLog.i("No Blinkin LED Drivers found in active HardwareMap.");
        }
        RobotLog.i("============================================================");
    }

    @Override
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

        // Control Hub
        frontRight = getHardwareSafely(hardwareMap, DcMotor.class, "chPort0");
        backRight  = getHardwareSafely(hardwareMap, DcMotor.class, "chPort1");
        backLeft   = getHardwareSafely(hardwareMap, DcMotor.class, "chPort2");
        frontLeft  = getHardwareSafely(hardwareMap, DcMotor.class, "chPort3");
        blinkin    = getHardwareSafely(hardwareMap, RevBlinkinLedDriver.class, "chServo0");

        // Expansion Hub
        intake   = getHardwareSafely(hardwareMap, DcMotor.class, "ehPort0");
        leftPod  = getHardwareSafely(hardwareMap, DcMotor.class, "ehPort1");
        shooter  = getHardwareSafely(hardwareMap, DcMotor.class, "ehPort2");
        shooterFollow = getHardwareSafely(hardwareMap, DcMotor.class, "ehPort3");
        hopper   = getHardwareSafely(hardwareMap, CRServo.class, "ehServo0");
        rightPod = intake;
        rearPod  = shooterFollow;

        // USB
        camera = getHardwareSafely(hardwareMap, WebcamName.class, "camera");

        // Configure brake behavior and direction on drivetrain motors if present
        if (frontLeft != null)  frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (frontRight != null) {
            frontRight.setDirection(DcMotor.Direction.REVERSE); // Front Right wheel is mounted reversed relative to other wheels
            frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        if (backLeft != null)   backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        if (backRight != null)  backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Configure intake and shooter motor directions (Reversed by default)
        if (intake != null) {
            intake.setDirection(DcMotor.Direction.REVERSE);
            // Power only. ehPort0's encoder jack is the right odometry pod, not an intake encoder.
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        if (shooter != null) {
            shooter.setDirection(DcMotor.Direction.REVERSE);
            shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        if (shooterFollow != null) {
            // Direction stays FORWARD: it also sets the rear pod's encoder sign. See SHOOTER_FOLLOW_DIR.
            shooterFollow.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            shooterFollow.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        // Instantiate Subsystems from dedicated subsystem classes
        drivetrainSubsystem = new MecanumDrivetrain(frontLeft, frontRight, backLeft, backRight);
        intakeSubsystem     = new IntakeSubsystem(intake);
        hopperSubsystem     = new HopperSubsystem(hopper);
        shooterSubsystem    = new ShooterSubsystem(shooter, shooterFollow);
        ledSubsystem        = new BlinkinLedSubsystem(blinkin);
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

    public BlinkinLedSubsystem getLed() {
        return ledSubsystem;
    }
}
