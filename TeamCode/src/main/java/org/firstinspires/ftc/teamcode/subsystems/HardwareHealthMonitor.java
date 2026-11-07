package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.IronEaglesHardware;

import java.util.ArrayList;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;

/**
 * Polls hubs, motors, OctoQuad, and camera. Errors are shown only on the
 * TeleOp ERROR telemetry line, never as a Driver Station global warning.
 * Once an error is seen it stays on the line and in the log even if it clears.
 */
public class HardwareHealthMonitor {
    private static final String[] MOTOR_NAMES = {
            "frontLeft", "frontRight", "backLeft", "backRight",
            "intake", "hopper", "shooter", "shooterFollow"
    };

    private final HardwareMap hardwareMap;
    private final IronEaglesHardware robot;
    private final OctoQuadOdometry octoquad;
    private final AprilTagVision vision;
    private final HubPowerMonitor power;
    private final Set<String> stickyErrors = new LinkedHashSet<>();

    public HardwareHealthMonitor(HardwareMap hardwareMap, IronEaglesHardware robot,
                                 OctoQuadOdometry octoquad, AprilTagVision vision) {
        this.hardwareMap = hardwareMap;
        this.robot = robot;
        this.octoquad = octoquad;
        this.vision = vision;
        this.power = new HubPowerMonitor(hardwareMap, octoquad);
    }

    public void update() {
        List<String> current = new ArrayList<>();
        for (String name : robot.missingDevices) {
            current.add(name + " missing");
        }
        checkHubs(current);
        checkMotors(current);
        power.update();
        current.addAll(power.getCurrentErrors());
        current.addAll(octoquad.getCurrentHardwareErrors());
        addIfPresent(current, vision.getDisconnectError());
        for (String error : current) {
            if (stickyErrors.add(error)) {
                RobotLog.e("[HARDWARE] ERROR: %s", error);
                RobotLog.e("[HARDWARE] SEEN: %s", String.join(" | ", stickyErrors));
            }
        }
    }

    public String getErrorLine() {
        if (stickyErrors.isEmpty()) {
            return "";
        }
        return String.join(" | ", stickyErrors);
    }

    public String getPowerLine() {
        return power.getTelemetryLine();
    }

    public void close() {
        power.logSummary();
        vision.close();
    }

    private void checkHubs(List<String> next) {
        for (LynxModule hub : hardwareMap.getAll(LynxModule.class)) {
            if (hub.isNotResponding()) {
                next.add(hub.getDeviceName() + " disconnected");
            }
        }
    }

    private void checkMotors(List<String> next) {
        for (String name : MOTOR_NAMES) {
            DcMotor motor = motorByName(name);
            if (motor == null) {
                continue;
            }
            try {
                motor.getCurrentPosition();
            } catch (RuntimeException e) {
                next.add(name + " disconnected");
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

    private static void addIfPresent(List<String> next, String error) {
        if (error != null && !error.isEmpty()) {
            next.add(error);
        }
    }
}
