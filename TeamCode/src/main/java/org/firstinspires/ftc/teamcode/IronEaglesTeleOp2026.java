package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.config.TunableValues;
import org.firstinspires.ftc.teamcode.geometry.CellGeometry;
import org.firstinspires.ftc.teamcode.interfaces.AllianceInterface;
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
        autoFire = new AutoFireController(availability, tracker, shooting, drive, hopper);

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
        shooter.start();
        visionLogTimer.reset();
        log.i("Match started. shooter started, heading=%.1f camera=%s",
                imu.getHeadingDegrees(), vision.getCameraStateName());

        while (opModeIsActive()) {
            chooseAllianceIfPressed();
            if (gamepad1.backWasPressed()) {
                imu.resetYaw();
            }

            List<AprilTagDetection> detections = vision.sample();
            tracker.update(detections);

            boolean available = availability.isAutoFireAvailable();
            led.setState(available ? LedInterface.State.AUTO_FIRE_AVAILABLE : LedInterface.State.OFF);

            boolean gp2Rt = gamepad2.right_trigger > TunableValues.DEADZONE;
            if (autoFire.isActive() && hasManualInput()) {
                log.i("Manual input cancelled auto fire from %s", autoFire.getState());
                autoFire.abort();
            } else if (!autoFire.isActive() && gp2Rt && !wasGp2RightTrigger) {
                autoFire.start();
            }
            wasGp2RightTrigger = gp2Rt;

            if (autoFire.isActive()) {
                autoFire.update();
            } else {
                applyManualControls();
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

    private boolean hasManualInput() {
        return stick(gamepad1.left_stick_x) || stick(gamepad1.left_stick_y) || stick(gamepad1.right_stick_x)
                || stick(gamepad1.right_stick_y) || stick(gamepad2.left_stick_x) || stick(gamepad2.left_stick_y)
                || stick(gamepad2.right_stick_x) || stick(gamepad2.right_stick_y)
                || gamepad1.left_trigger > TunableValues.DEADZONE
                || gamepad1.right_trigger > TunableValues.DEADZONE
                || gamepad2.left_trigger > TunableValues.DEADZONE
                || gamepad1.a || gamepad1.b || gamepad1.x || gamepad1.y
                || gamepad1.left_bumper || gamepad1.right_bumper
                || gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_left || gamepad1.dpad_right
                || gamepad1.back || gamepad1.start || gamepad1.left_stick_button || gamepad1.right_stick_button
                || gamepad2.a || gamepad2.b || gamepad2.x || gamepad2.y
                || gamepad2.left_bumper || gamepad2.right_bumper
                || gamepad2.dpad_up || gamepad2.dpad_down || gamepad2.dpad_left || gamepad2.dpad_right
                || gamepad2.back || gamepad2.start || gamepad2.left_stick_button || gamepad2.right_stick_button;
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
        if (geometry != null) {
            telemetry.addData("Last good source", geometry.source);
            telemetry.addData("Last good x/y/z (in)", "%.1f / %.1f / %.1f", geometry.xIn, geometry.yIn, geometry.zIn);
            telemetry.addData("Last good range/yaw/bearing", "%.1f in / %.1f / %.1f",
                    geometry.rangeIn, geometry.yawDeg, geometry.bearingDeg);
            telemetry.addData("Last good height/upright", "%.1f in / %b", geometry.tagHeightIn, geometry.upright);
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
        log.i("Tunables shooterFrac=%.2f intakeFrac=%.2f shootDist=%.1f camH=%.1f camTilt=%.1f wheel=%.1f cellUp=%.1f",
                TunableValues.SHOOTER_TARGET_RPM_FRACTION, TunableValues.INTAKE_TARGET_RPM_FRACTION,
                TunableValues.SHOOTING_DISTANCE_IN, TunableValues.CAMERA_HEIGHT_IN,
                TunableValues.CAMERA_TILT_DEG, TunableValues.WHEEL_DIAMETER_IN,
                TunableValues.CELL_UP_MIN_HEIGHT_IN);
        log.i("Tunables imu logo=%s usb=%s gate=%d/%d deadzone=%.2f",
                TunableValues.IMU_LOGO_DIRECTION, TunableValues.IMU_USB_DIRECTION,
                TunableValues.TAG_DETECTION_MIN_HITS, TunableValues.TAG_DETECTION_WINDOW,
                TunableValues.DEADZONE);
    }
}
