package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.config.TunableValues;
import org.firstinspires.ftc.teamcode.geometry.CellGeometry;
import org.firstinspires.ftc.teamcode.interfaces.AllianceInterface;
import org.firstinspires.ftc.teamcode.interfaces.AutoFireControllerInterface;
import org.firstinspires.ftc.teamcode.interfaces.LedInterface;
import org.firstinspires.ftc.teamcode.subsystems.AllianceSelector;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagVision;
import org.firstinspires.ftc.teamcode.subsystems.AutoFireAvailability;
import org.firstinspires.ftc.teamcode.subsystems.AutoFireController;
import org.firstinspires.ftc.teamcode.subsystems.ControlHubImu;
import org.firstinspires.ftc.teamcode.subsystems.HopperSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.NoOpLed;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShootingGeometry;
import org.firstinspires.ftc.teamcode.subsystems.TargetCellTracker;
import org.firstinspires.ftc.teamcode.util.RobotLogger;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;

@TeleOp(name = "IronEaglesTeleOp2026", group = "TeleOp")
public class IronEaglesTeleOp2026 extends LinearOpMode {
    private final RobotLogger log = new RobotLogger("TELEOP");
    private final ElapsedTime visionLogTimer = new ElapsedTime();
    private String lastVisionLogKey = "";
    private boolean wasGp2RightTrigger;
    private int suppressCancelMask;

    private AllianceSelector alliance;
    private ControlHubImu imu;
    private AprilTagVision vision;
    private TargetCellTracker tracker;
    private ShootingGeometry shooting;
    private MecanumDrivetrain drive;
    private HopperSubsystem hopper;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private NoOpLed led;
    private AutoFireAvailability availability;
    private AutoFireController autoFire;
    private IronEaglesHardware robot;

    @Override
    public void runOpMode() {
        log.i("OpMode initializing IronEaglesTeleOp2026");
        logTunables();

        robot = new IronEaglesHardware();
        robot.init(hardwareMap);

        alliance = new AllianceSelector();
        imu = new ControlHubImu(hardwareMap);
        vision = new AprilTagVision(hardwareMap, robot.camera);
        tracker = new TargetCellTracker();
        shooting = new ShootingGeometry();
        drive = new MecanumDrivetrain(robot.frontLeft, robot.frontRight, robot.backLeft, robot.backRight, imu);
        hopper = new HopperSubsystem(robot.hopper);
        intake = new IntakeSubsystem(robot.intake);
        shooter = new ShooterSubsystem(robot.shooter);
        led = new NoOpLed();
        availability = new AutoFireAvailability(alliance, imu, vision, drive, tracker, shooting);
        autoFire = new AutoFireController(availability, tracker, shooting, drive, hopper, shooter);

        while (opModeInInit()) {
            chooseAllianceIfPressed();
            writeTelemetry("INIT");
            telemetry.update();
        }

        if (alliance.isSet()) {
            alliance.lock();
            tracker.setAlliance(alliance.getColor());
            log.i("Play. Alliance locked %s", alliance.getColor());
        } else {
            log.w("Play with alliance NOT SET. Auto fire disabled until X=Blue or B=Red.");
        }
        if (TunableValues.SHOOTER_RUN_ENTIRE_MATCH) {
            shooter.start();
            log.i("Match started. shooter always-on, heading=%.1f camera=%s",
                    imu.getHeadingDegrees(), vision.getCameraStateName());
        } else {
            shooter.stop();
            log.i("Match started. shooter manual (hold GP2 A), heading=%.1f camera=%s",
                    imu.getHeadingDegrees(), vision.getCameraStateName());
        }
        visionLogTimer.reset();

        while (opModeIsActive()) {
            chooseAllianceIfPressed();
            if (gamepad1.backWasPressed()) {
                imu.resetYaw();
            }

            List<AprilTagDetection> detections = vision.sample();
            boolean driverDriving = stick(gamepad1.left_stick_x) || stick(gamepad1.left_stick_y)
                    || stick(gamepad1.right_stick_x);
            if (!autoFireFreezesTracker()) {
                tracker.update(detections, !driverDriving);
            }

            boolean available = availability.isAutoFireAvailable();
            led.setState(available ? LedInterface.State.AUTO_FIRE_AVAILABLE : LedInterface.State.OFF);

            boolean gp2Rt = gamepad2.right_trigger > TunableValues.DEADZONE;
            if (autoFire.isActive() && isCancelInput()) {
                log.i("Manual input cancelled auto fire from %s", autoFire.getState());
                autoFire.abort();
                suppressCancelMask = 0;
            } else if (!autoFire.isActive() && gp2Rt && !wasGp2RightTrigger) {
                autoFire.start();
                if (autoFire.isActive()) {
                    suppressCancelMask = buttonMask();
                }
            }
            wasGp2RightTrigger = gp2Rt;

            if (autoFire.isActive()) {
                autoFire.update();
            } else {
                applyManualControls();
                applyShooter();
            }

            writeTelemetry("RUN");
            logVision(detections);
            telemetry.update();
        }
        log.i("OpMode stopped");
    }

    private void chooseAllianceIfPressed() {
        if (alliance.isLocked()) {
            return;
        }
        if (gamepad1.xWasPressed()) {
            alliance.choose(AllianceInterface.Color.BLUE);
            tracker.setAlliance(AllianceInterface.Color.BLUE);
            if (!opModeInInit()) {
                alliance.lock();
            }
        } else if (gamepad1.bWasPressed()) {
            alliance.choose(AllianceInterface.Color.RED);
            tracker.setAlliance(AllianceInterface.Color.RED);
            if (!opModeInInit()) {
                alliance.lock();
            }
        }
    }

    private void applyManualControls() {
        double y = deadzone(-gamepad1.left_stick_y);
        double x = deadzone(gamepad1.left_stick_x);
        double rx = deadzone(gamepad1.right_stick_x);
        double heading = imu.isPresent() ? imu.getHeadingDegrees() : 0.0;
        drive.driveFieldCentric(x, y, rx, heading);

        if (gamepad2.left_trigger > TunableValues.DEADZONE) {
            intake.runInward();
        } else if (gamepad2.left_bumper) {
            intake.runOutward();
        } else {
            intake.stop();
        }

        if (gamepad2.x) {
            hopper.stepForward180();
        } else if (gamepad2.y) {
            hopper.stepReverse180();
        }
        hopper.update();
    }

    private boolean autoFireFreezesTracker() {
        AutoFireControllerInterface.State state = autoFire.getState();
        return state == AutoFireControllerInterface.State.MOVING
                || state == AutoFireControllerInterface.State.SPOOLING
                || state == AutoFireControllerInterface.State.FEEDING;
    }

    private void applyShooter() {
        if (gamepad2.right_bumper) {
            shooter.runReverse();
        } else if (TunableValues.SHOOTER_RUN_ENTIRE_MATCH) {
            if (gamepad2.aWasPressed()) {
                log.i("Operator respool");
            }
            shooter.runForward();
        } else if (gamepad2.a) {
            shooter.runForward();
        } else {
            shooter.stop();
        }
        shooter.update();
    }

    private boolean isCancelInput() {
        if (stick(gamepad1.left_stick_x) || stick(gamepad1.left_stick_y) || stick(gamepad1.right_stick_x)
                || stick(gamepad1.right_stick_y) || stick(gamepad2.left_stick_x) || stick(gamepad2.left_stick_y)
                || stick(gamepad2.right_stick_x) || stick(gamepad2.right_stick_y)) {
            return true;
        }
        int held = buttonMask();
        suppressCancelMask &= held;
        return (held & ~suppressCancelMask) != 0;
    }

    private int buttonMask() {
        int mask = 0;
        if (gamepad1.left_trigger > TunableValues.DEADZONE) mask |= 1;
        if (gamepad1.right_trigger > TunableValues.DEADZONE) mask |= 1 << 1;
        if (gamepad2.left_trigger > TunableValues.DEADZONE) mask |= 1 << 2;
        if (gamepad1.a) mask |= 1 << 3;
        if (gamepad1.b) mask |= 1 << 4;
        if (gamepad1.x) mask |= 1 << 5;
        if (gamepad1.y) mask |= 1 << 6;
        if (gamepad1.left_bumper) mask |= 1 << 7;
        if (gamepad1.right_bumper) mask |= 1 << 8;
        if (gamepad1.dpad_up) mask |= 1 << 9;
        if (gamepad1.dpad_down) mask |= 1 << 10;
        if (gamepad1.dpad_left) mask |= 1 << 11;
        if (gamepad1.dpad_right) mask |= 1 << 12;
        if (gamepad1.back) mask |= 1 << 13;
        if (gamepad1.start) mask |= 1 << 14;
        if (gamepad1.left_stick_button) mask |= 1 << 15;
        if (gamepad1.right_stick_button) mask |= 1 << 16;
        if (gamepad2.a) mask |= 1 << 17;
        if (gamepad2.b) mask |= 1 << 18;
        if (gamepad2.x) mask |= 1 << 19;
        if (gamepad2.y) mask |= 1 << 20;
        if (gamepad2.left_bumper) mask |= 1 << 21;
        if (gamepad2.right_bumper) mask |= 1 << 22;
        if (gamepad2.dpad_up) mask |= 1 << 23;
        if (gamepad2.dpad_down) mask |= 1 << 24;
        if (gamepad2.dpad_left) mask |= 1 << 25;
        if (gamepad2.dpad_right) mask |= 1 << 26;
        if (gamepad2.back) mask |= 1 << 27;
        if (gamepad2.start) mask |= 1 << 28;
        if (gamepad2.left_stick_button) mask |= 1 << 29;
        if (gamepad2.right_stick_button) mask |= 1 << 30;
        return mask;
    }

    private static boolean stick(double value) {
        return Math.abs(value) > TunableValues.DEADZONE;
    }

    private static double deadzone(double value) {
        return stick(value) ? value : 0.0;
    }

    private void writeTelemetry(String phase) {
        String allianceName = alliance.isSet() ? alliance.getColor().name() : "NOT SET";
        String reason = availability.unavailableReason();
        boolean available = reason == null;
        CellGeometry geometry = tracker.getLastKnownGood();

        telemetry.addData("Phase", phase);
        telemetry.addData("Alliance", allianceName + (alliance.isLocked() ? " (locked)" : ""));
        telemetry.addData("TARGET_CELL_IN_VIEW", tracker.isTargetCellInView());
        telemetry.addData("AUTO_FIRE_AVAILABLE", available);
        telemetry.addData("AUTO_FIRE reason", available ? "ok" : reason);
        telemetry.addData("LED", led.getState());
        telemetry.addData("Auto fire", autoFire.getState());
        telemetry.addData("IMU", imu.isPresent() ? String.format("%.1f deg", imu.getHeadingDegrees()) : "MISSING");
        telemetry.addData("Camera", "%s  %.1f fps", vision.getCameraStateName(), vision.getFps());
        telemetry.addData("Detection gate", "%d/%d", tracker.getGateHits(), tracker.getGateWindow());
        telemetry.addData("Drive auto-fire ready", drive.areDriveMotorsPresent());
        telemetry.addData("Intake", intake.isUsingVelocity() ? "velocity RPM" : "100% power fallback");
        telemetry.addData("Hopper stepping", hopper.isStepping());
        String shooterMode;
        if (shooter.isReversing()) {
            shooterMode = "REVERSE";
        } else if (shooter.isRunning()) {
            shooterMode = "forward";
        } else {
            shooterMode = "stopped";
        }
        telemetry.addData("Shooter", shooterMode);
        telemetry.addData("Shooter RPM", "cmd %.0f  actual %.0f  ready=%b",
                shooter.getCommandedRpm(), shooter.getActualRpm(), shooter.isAtTargetRpm());
        if (shooter.isStoppedWarning()) {
            telemetry.addData("WARNING shooter", "STOPPED. Hold GP2 A to run.");
        }
        if (geometry != null) {
            telemetry.addData("Last good cluster", "%s  %d%%", geometry.clusterName, geometry.percentFound);
            telemetry.addData("Last good x/y/z (in)", "%.1f / %.1f / %.1f", geometry.xIn, geometry.yIn, geometry.zIn);
            telemetry.addData("Last good range/yaw/bearing", "%.1f in / %.1f / %.1f",
                    geometry.rangeIn, geometry.yawDeg, geometry.bearingDeg);
            telemetry.addData("Last good roll/upright", "%.1f / %b", geometry.rollDeg, geometry.upright);
            telemetry.addData("In cone", shooting.isInsideCone(geometry));
        } else {
            telemetry.addData("Last good geometry", "--");
        }
        if (!robot.missingDevices.isEmpty()) {
            telemetry.addData("Missing devices", String.join(", ", robot.missingDevices));
        }
        if (!drive.areDriveMotorsPresent()) {
            telemetry.addData("WARNING", "Drive motors/encoders missing. Auto fire disabled.");
        }
        if (!intake.isUsingVelocity()) {
            telemetry.addData("WARNING intake", "Encoder/motor type missing. Intake at 100% power.");
        }
        if (!alliance.isSet()) {
            telemetry.addData("WARNING alliance", "Press GP1 X=Blue or B=Red");
        }
        if (robot.frontLeft != null && robot.frontLeft.getMotorType().getTicksPerRev() <= 1) {
            telemetry.addData("WARNING motors", "Drive/intake/shooter types are Unspecified. Set them on the DS.");
        }
    }

    private void logVision(List<AprilTagDetection> detections) {
        StringBuilder details = new StringBuilder();
        for (int i = 0; i < detections.size(); i++) {
            AprilTagDetection detection = detections.get(i);
            if (i > 0) {
                details.append(" | ");
            }
            if (detection instanceof AprilTagSingleDetection) {
                AprilTagSingleDetection tag = (AprilTagSingleDetection) detection;
                details.append(String.format("single id=%d pose=%b", tag.id, tag.ftcPose != null));
            } else if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
                String name = cluster.metadata == null ? "?" : cluster.metadata.name;
                details.append(String.format("cluster %s pct=%d pose=%b",
                        name, cluster.percentClusterFound, cluster.ftcPose != null));
            } else {
                details.append(detection == null ? "null" : detection.getClass().getSimpleName());
            }
        }
        String key = vision.getCameraStateName() + "|" + detections.size() + "|" + tracker.isGatePassing()
                + "|" + tracker.getGateHits() + "|" + details;
        if (key.equals(lastVisionLogKey) && visionLogTimer.seconds() < 2.0) {
            return;
        }
        lastVisionLogKey = key;
        visionLogTimer.reset();
        log.i("detected=%b inView=%b gate=%b %d/%d state=%s fps=%.1f count=%d %s",
                !detections.isEmpty(), tracker.isTargetCellInView(), tracker.isGatePassing(),
                tracker.getGateHits(), tracker.getGateWindow(),
                vision.getCameraStateName(), vision.getFps(), detections.size(), details);
    }

    private void logTunables() {
        log.i("Tunables shooterAlwaysOn=%b shooterRpm=%.0f readyFrac=%.2f intakeRpm=%.0f shootDist=%.1f maxStart=%.1f wheel=%.1f",
                TunableValues.SHOOTER_RUN_ENTIRE_MATCH, TunableValues.SHOOTER_TARGET_RPM,
                TunableValues.SHOOTER_READY_RPM_FRACTION, TunableValues.INTAKE_TARGET_RPM,
                TunableValues.SHOOTING_DISTANCE_IN, TunableValues.AUTO_FIRE_MAX_START_RANGE_IN,
                TunableValues.WHEEL_DIAMETER_IN);
        log.i("Tunables imu logo=%s usb=%s gate=%d/%d deadzone=%.2f",
                TunableValues.IMU_LOGO_DIRECTION, TunableValues.IMU_USB_DIRECTION,
                TunableValues.TAG_DETECTION_MIN_HITS, TunableValues.TAG_DETECTION_WINDOW,
                TunableValues.DEADZONE);
    }
}
