package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.interfaces.DrivetrainInterface;
import org.firstinspires.ftc.teamcode.interfaces.HopperInterface;
import org.firstinspires.ftc.teamcode.interfaces.IntakeInterface;
import org.firstinspires.ftc.teamcode.interfaces.RobotHardwareInterface;
import org.firstinspires.ftc.teamcode.interfaces.ShooterInterface;
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

    private MecanumDrivetrain drivetrainSubsystem;
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

        // Configure hopper motor (REV Core Hex Motor) encoder reset and brake behavior
        if (hopper != null) {
            hopper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            hopper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            hopper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        // Instantiate Subsystems from dedicated subsystem classes
        drivetrainSubsystem = new MecanumDrivetrain(frontLeft, frontRight, backLeft, backRight);
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
}
