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
 * Polls hubs, motors, dead-wheel encoders, and camera. Errors are shown
 * only on the TeleOp ERROR telemetry line, never as a Driver Station global warning.
 * Once an error is seen it stays on the line and in the log even if it clears.
 */
public class HardwareHealthMonitor {
    private static final String[] MOTOR_NAMES = {
            "chPort0", "chPort1", "chPort2", "chPort3",
            "ehPort0", "ehPort1", "ehPort2", "ehPort3"
    };

    private final HardwareMap hardwareMap;
    private final IronEaglesHardware robot;
    private final DeadWheelOdometry odometry;
    private final AprilTagVision vision;
    private final HubPowerMonitor power;
    private final Set<String> stickyErrors = new LinkedHashSet<>();

    public HardwareHealthMonitor(HardwareMap hardwareMap, IronEaglesHardware robot,
                                 DeadWheelOdometry odometry, AprilTagVision vision) {
        this.hardwareMap = hardwareMap;
        this.robot = robot;
        this.odometry = odometry;
        this.vision = vision;
        this.power = new HubPowerMonitor(hardwareMap);
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
        current.addAll(odometry.getCurrentHardwareErrors());
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

    public double getBatteryVolts() {
        return power.getControlHubVin();
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
                next.add(name + " (" + useOf(name) + ") disconnected");
            }
        }
    }

    private DcMotor motorByName(String name) {
        switch (name) {
            case "chPort0":
                return robot.frontRight;
            case "chPort1":
                return robot.backRight;
            case "chPort2":
                return robot.backLeft;
            case "chPort3":
                return robot.frontLeft;
            case "ehPort0":
                return robot.intake;
            case "ehPort1":
                return robot.leftPod;
            case "ehPort2":
                return robot.shooter;
            case "ehPort3":
                return robot.rearPod;
            default:
                return null;
        }
    }

    private static String useOf(String name) {
        switch (name) {
            case "chPort0":
                return "front right";
            case "chPort1":
                return "back right";
            case "chPort2":
                return "back left";
            case "chPort3":
                return "front left";
            case "ehPort0":
                return "intake / right pod";
            case "ehPort1":
                return "left pod";
            case "ehPort2":
                return "shooter";
            case "ehPort3":
                return "shooter follower / rear pod";
            default:
                return "?";
        }
    }

    private static void addIfPresent(List<String> next, String error) {
        if (error != null && !error.isEmpty()) {
            next.add(error);
        }
    }
}
