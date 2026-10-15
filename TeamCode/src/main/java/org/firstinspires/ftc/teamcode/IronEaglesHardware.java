package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.RobotLogger;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

/**
 * Maps Control Hub and Expansion Hub devices. Subsystems are constructed by TeleOp.
 */
public class IronEaglesHardware {
    public DcMotor frontRight;
    public DcMotor backRight;
    public DcMotor backLeft;
    public DcMotor frontLeft;
    public WebcamName camera;
    public DcMotor intake;
    public DcMotor hopper;
    public DcMotor shooter;
    public final List<String> missingDevices = new ArrayList<>();

    private final RobotLogger log = new RobotLogger("HARDWARE");

    private <T> T getHardwareSafely(HardwareMap hardwareMap, Class<T> clazz, String name) {
        try {
            T device = hardwareMap.tryGet(clazz, name);
            if (device == null) {
                missingDevices.add(name);
                log.w("Missing device '%s'", name);
            }
            return device;
        } catch (Exception e) {
            missingDevices.add(name);
            log.w("Missing device '%s': %s", name, e.getMessage());
            return null;
        }
    }

    public void logDiagnostics(HardwareMap hardwareMap) {
        log.i("=================== HARDWARE DIAGNOSTICS ===================");
        log.i("Total DcMotors found in active HardwareMap: %d", hardwareMap.dcMotor.size());
        for (Map.Entry<String, DcMotor> entry : hardwareMap.dcMotor.entrySet()) {
            log.i(" -> Configured DcMotor: '%s'", entry.getKey());
        }
        try {
            log.i("Total Servos found in active HardwareMap: %d", hardwareMap.servo.size());
            for (Map.Entry<String, Servo> entry : hardwareMap.servo.entrySet()) {
                log.i(" -> Configured Servo: '%s'", entry.getKey());
            }
        } catch (Exception ignored) {
        }
        try {
            log.i("Total CRServos found in active HardwareMap: %d", hardwareMap.crservo.size());
            for (Map.Entry<String, CRServo> entry : hardwareMap.crservo.entrySet()) {
                log.i(" -> Configured CRServo: '%s'", entry.getKey());
            }
        } catch (Exception ignored) {
        }
        try {
            List<WebcamName> webcams = hardwareMap.getAll(WebcamName.class);
            log.i("Total Webcams found in active HardwareMap: %d", webcams.size());
        } catch (Exception e) {
            log.i("No webcams found in active HardwareMap.");
        }
        log.i("============================================================");
    }

    public void init(HardwareMap hardwareMap) {
        missingDevices.clear();
        logDiagnostics(hardwareMap);

        try {
            List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
            for (LynxModule hub : allHubs) {
                hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            }
            log.i("Bulk caching AUTO on %d hub(s)", hardwareMap.getAll(LynxModule.class).size());
        } catch (Exception e) {
            log.w("Bulk caching setup failed: %s", e.getMessage());
        }

        frontRight = getHardwareSafely(hardwareMap, DcMotor.class, "frontRight");
        backRight = getHardwareSafely(hardwareMap, DcMotor.class, "backRight");
        backLeft = getHardwareSafely(hardwareMap, DcMotor.class, "backLeft");
        frontLeft = getHardwareSafely(hardwareMap, DcMotor.class, "frontLeft");
        intake = getHardwareSafely(hardwareMap, DcMotor.class, "intake");
        hopper = getHardwareSafely(hardwareMap, DcMotor.class, "hopper");
        shooter = getHardwareSafely(hardwareMap, DcMotor.class, "shooter");
        camera = getHardwareSafely(hardwareMap, WebcamName.class, "camera");

        if (frontRight != null) {
            frontRight.setDirection(DcMotor.Direction.REVERSE);
        }
        if (backLeft != null) {
            backLeft.setDirection(DcMotor.Direction.REVERSE);
        }
        if (intake != null) {
            intake.setDirection(DcMotor.Direction.REVERSE);
        }
        if (shooter != null) {
            shooter.setDirection(DcMotor.Direction.REVERSE);
        }

        logDriveEncoderCheck("frontLeft", frontLeft);
        logDriveEncoderCheck("frontRight", frontRight);
        logDriveEncoderCheck("backLeft", backLeft);
        logDriveEncoderCheck("backRight", backRight);
        if (frontLeft == null || frontRight == null || backLeft == null || backRight == null) {
            log.w("Drive motor(s) missing. Auto fire will be disabled.");
        } else {
            log.i("Drive motors present. Encoder cables assumed connected; unplugged encoders cannot be proven at rest.");
        }
    }

    private void logDriveEncoderCheck(String name, DcMotor motor) {
        if (motor == null) {
            log.w("Drive encoder check %s: motor missing", name);
            return;
        }
        double ticksPerRev = motor.getMotorType().getTicksPerRev();
        int ticks = motor.getCurrentPosition();
        if (ticksPerRev <= 1) {
            log.w("Drive encoder check %s: motor type has no ticks/rev. Auto fire will be disabled.", name);
        } else {
            log.i("Drive encoder check %s: ticks=%d ticks/rev=%.1f type=%s",
                    name, ticks, ticksPerRev, motor.getMotorType().getName());
        }
    }
}
