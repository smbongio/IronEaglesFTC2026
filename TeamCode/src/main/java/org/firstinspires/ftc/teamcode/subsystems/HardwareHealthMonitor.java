package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.GlobalWarningSource;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.IronEaglesHardware;

import java.util.ArrayList;
import java.util.List;

/**
 * Polls hubs, motors, OctoQuad, and camera. Driver Station shows a global
 * warning while anything is missing or not responding.
 */
public class HardwareHealthMonitor implements GlobalWarningSource {
    private static final String[] MOTOR_NAMES = {
            "frontLeft", "frontRight", "backLeft", "backRight",
            "intake", "hopper", "shooter", "shooterFollow"
    };

    private final HardwareMap hardwareMap;
    private final IronEaglesHardware robot;
    private final OctoQuadOdometry octoquad;
    private final AprilTagVision vision;
    private final List<String> errors = new ArrayList<>();
    private String lastLogged = "";
    private volatile boolean suppressed;

    public HardwareHealthMonitor(HardwareMap hardwareMap, IronEaglesHardware robot,
                                 OctoQuadOdometry octoquad, AprilTagVision vision) {
        this.hardwareMap = hardwareMap;
        this.robot = robot;
        this.octoquad = octoquad;
        this.vision = vision;
        RobotLog.registerGlobalWarningSource(this);
    }

    public void update() {
        errors.clear();
        for (String name : robot.missingDevices) {
            errors.add(name + " missing");
        }
        checkHubs();
        checkMotors();
        addIfPresent(octoquad.getDisconnectError());
        addIfPresent(vision.getDisconnectError());
        String line = getErrorLine();
        if (!line.equals(lastLogged)) {
            if (line.isEmpty()) {
                RobotLog.i("[HARDWARE] All devices responding.");
            } else {
                RobotLog.e("[HARDWARE] ERROR: %s", line);
            }
            lastLogged = line;
        }
    }

    public boolean hasErrors() {
        return !errors.isEmpty();
    }

    public String getErrorLine() {
        if (errors.isEmpty()) {
            return "";
        }
        return String.join(", ", errors);
    }

    public void close() {
        RobotLog.unregisterGlobalWarningSource(this);
    }

    @Override
    public String getGlobalWarning() {
        if (suppressed) {
            return null;
        }
        String line = getErrorLine();
        return line.isEmpty() ? null : "ERROR: " + line;
    }

    @Override
    public boolean shouldTriggerWarningSound() {
        return hasErrors();
    }

    @Override
    public void suppressGlobalWarning(boolean suppress) {
        suppressed = suppress;
    }

    @Override
    public void setGlobalWarning(String warning) {
    }

    @Override
    public void clearGlobalWarning() {
        errors.clear();
    }

    private void checkHubs() {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            String name = hub.getDeviceName();
            if (hub.isNotResponding()) {
                errors.add(name + " disconnected");
            }
        }
    }

    private void checkMotors() {
        for (String name : MOTOR_NAMES) {
            DcMotor motor = motorByName(name);
            if (motor == null) {
                continue;
            }
            try {
                motor.getCurrentPosition();
            } catch (RuntimeException e) {
                errors.add(name + " disconnected");
            }
        }
    }

    private DcMotor motorByName(String name) {
        switch (name) {
            case "frontLeft":
                return robot.frontLeft;
            case "frontRight":
                return robot.frontRight;
            case "backLeft":
                return robot.backLeft;
            case "backRight":
                return robot.backRight;
            case "intake":
                return robot.intake;
            case "hopper":
                return robot.hopper;
            case "shooter":
                return robot.shooter;
            case "shooterFollow":
                return robot.shooterFollow;
            default:
                return null;
        }
    }

    private void addIfPresent(String error) {
        if (error != null && !error.isEmpty()) {
            errors.add(error);
        }
    }
}
