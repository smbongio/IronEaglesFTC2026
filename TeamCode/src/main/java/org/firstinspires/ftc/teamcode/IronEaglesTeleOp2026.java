package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.config.TunableValues;
import org.firstinspires.ftc.teamcode.control.DriveVelocityAssist;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagVision;
import org.firstinspires.ftc.teamcode.subsystems.HardwareHealthMonitor;
import org.firstinspires.ftc.teamcode.subsystems.OctoQuadOdometry;
import org.firstinspires.ftc.teamcode.subsystems.TargetCellGate;

@TeleOp(name = "IronEaglesTeleOp2026", group = "TeleOp")
public class IronEaglesTeleOp2026 extends LinearOpMode {

    private final IronEaglesHardware robot = new IronEaglesHardware();
    private final DriveVelocityAssist driveAssist = new DriveVelocityAssist();
    private final TargetCellGate targetCell = new TargetCellGate();
    private OctoQuadOdometry imu;
    private AprilTagVision vision;
    private HardwareHealthMonitor hardwareHealth;

    private boolean wasFlywheelRunning = false;
    private boolean wasManualDriving = false;
    private boolean wasIntakeRunning = false;
    private boolean wasHopperRunning = false;
    private boolean wasGp2DpadUp = false;
    private boolean wasGp2DpadDown = false;
    private boolean wasGp2DpadLeft = false;
    private boolean wasGp2DpadRight = false;

    @Override
    public void runOpMode() {
        RobotLog.i("OpMode Initializing: IronEaglesTeleOp2026");

        robot.init(hardwareMap);
        imu = new OctoQuadOdometry(hardwareMap);
        vision = new AprilTagVision(hardwareMap, robot.camera);
        vision.sample();
        hardwareHealth = new HardwareHealthMonitor(hardwareMap, robot, imu, vision);
        hardwareHealth.update();
        telemetry.setMsTransmissionInterval(100);

        if (robot.missingDevices.isEmpty()) {
            telemetry.addData("STATUS", "Initialized - All devices found!");
            RobotLog.i("Hardware Map Initialization Complete: All devices found.");
        } else {
            telemetry.addData("WARNING", "Initialized with missing config devices!");
            telemetry.addData("MISSING", String.join(", ", robot.missingDevices));
            RobotLog.w("Hardware Map Initialization Warning: Missing devices -> %s",
                    String.join(", ", robot.missingDevices));
        }
        telemetry.addData("X", poseXIn());
        telemetry.addData("Y", poseYIn());
        telemetry.addData("HEADING/YAW", imu.isPresent()
                ? String.format("%s  heading=%.1f", imu.getHeadingSource(), imu.getHeadingDegrees())
                : "MISSING - drive is robot-centric");
        addOdometryBlockTelemetry();
        telemetry.addData("POWER", hardwareHealth.getPowerLine());
        telemetry.addData("TARGET", vision.isPresent() ? "waiting" : "FALSE");
        addErrorLine();
        telemetry.update();

        RobotLog.i("[CONTROLS]\n%s", TunableValues.GAMEPAD_CONTROLS);
        waitForStart();
        RobotLog.i("OpMode Started: Match timer running. heading=%.1f", imu.getHeadingDegrees());

        try {
            while (opModeIsActive()) {
                imu.update();
                targetCell.update(vision.sample());
                hardwareHealth.update();

                if (gamepad1.backWasPressed()) {
                    imu.resetYaw();
                }

                double rawY  = -gamepad1.left_stick_y;
                double rawX  =  gamepad1.left_stick_x;
                double rawRx =  gamepad1.right_stick_x;
                double y  = squareStick(Math.abs(rawY)  > 0.05 ? rawY  : 0.0);
                double x  = squareStick(Math.abs(rawX)  > 0.05 ? rawX  : 0.0);
                double rx = squareStick(Math.abs(rawRx) > 0.05 ? rawRx : 0.0);

                updateSpeedTargets();
                driveFieldCentric(x, y, rx);
                updateIntake();
                updateShooter();
                updateHopper();

                robot.getLed().update(targetCell.isTargetVisible());
                robot.getDrivetrain().logWheelDiagnostics();
                imu.logOdometryDiagnostics();
                writeTelemetry();
            }
        } finally {
            robot.getLed().close();
            hardwareHealth.close();
        }
    }

    private void driveFieldCentric(double x, double y, double rx) {
        boolean isManualDriving = (x != 0 || y != 0 || rx != 0);
        if (isManualDriving && !wasManualDriving) {
            RobotLog.i("[MANUAL DRIVETRAIN] Joysticks engaged -> Y: %.2f | X: %.2f | Rot: %.2f heading: %.1f",
                    y, x, rx, imu.getHeadingDegrees());
            wasManualDriving = true;
        } else if (!isManualDriving && wasManualDriving) {
            RobotLog.i("[MANUAL DRIVETRAIN] Joysticks released -> Drivetrain stopped.");
            wasManualDriving = false;
        }

        double heading = imu.isPresent() ? imu.getDriveHeadingDegrees() : 0.0;
        if (imu.hasLocalizerVelocity()) {
            driveAssist.apply(x, y, rx, imu.getVelXMmS(), imu.getVelYMmS(), imu.getVelHeadingRadS());
            x = driveAssist.getX();
            y = driveAssist.getY();
            rx = driveAssist.getRx();
        } else {
            driveAssist.reset();
        }
        robot.getDrivetrain().driveFieldCentric(x, y, rx, heading);
    }

    private void updateIntake() {
        if (gamepad2.left_trigger > 0.2) {
            logIntakeStart("forward");
            robot.getIntake().runIntake(1.0);
        } else if (gamepad2.left_bumper) {
            logIntakeStart("reverse");
            robot.getIntake().runIntake(-1.0);
        } else {
            if (wasIntakeRunning) {
                RobotLog.i("[MANUAL INTAKE] Released -> Intake stopped.");
                wasIntakeRunning = false;
            }
            robot.getIntake().stop();
        }
    }

    private void logIntakeStart(String direction) {
        if (!wasIntakeRunning) {
            RobotLog.i("[MANUAL INTAKE] %s at %.0f ticks/s.",
                    direction, TunableValues.INTAKE_TARGET_TICKS_PER_SEC);
            wasIntakeRunning = true;
        }
    }

    private void updateShooter() {
        if (gamepad2.right_trigger > 0.2) {
            logShooterStart("forward");
            robot.getShooter().runShooter(1.0);
        } else if (gamepad2.right_bumper) {
            logShooterStart("reverse");
            robot.getShooter().runShooter(-1.0);
        } else {
            if (wasFlywheelRunning) {
                RobotLog.i("[FLYWHEEL] Released -> Shooter stopped.");
                wasFlywheelRunning = false;
            }
            robot.getShooter().stop();
        }
    }

    private void logShooterStart(String direction) {
        if (!wasFlywheelRunning) {
            RobotLog.i("[FLYWHEEL] Manual %s at %.0f ticks/s.",
                    direction, TunableValues.SHOOTER_TARGET_TICKS_PER_SEC);
            wasFlywheelRunning = true;
        }
    }

    private void updateHopper() {
        if (gamepad2.x) {
            logHopperStart("forward");
            robot.getHopper().runHopper(TunableValues.HOPPER_MANUAL_POWER);
        } else if (gamepad2.y) {
            logHopperStart("reverse");
            robot.getHopper().runHopper(-TunableValues.HOPPER_MANUAL_POWER);
        } else {
            if (wasHopperRunning) {
                RobotLog.i("[HOPPER] Released -> Hopper stopped.");
                wasHopperRunning = false;
            }
            robot.getHopper().stop();
        }
    }

    private void logHopperStart(String direction) {
        if (!wasHopperRunning) {
            RobotLog.i("[HOPPER] Manual %s.", direction);
            wasHopperRunning = true;
        }
    }

    private boolean motorAtTarget(double measuredTicksPerSec, double targetTicksPerSec) {
        double target = Math.abs(targetTicksPerSec);
        double band = target * TunableValues.SHOOTER_AT_SPEED_TOLERANCE;
        return Math.abs(Math.abs(measuredTicksPerSec) - target) <= band;
    }

    private boolean bothShootersAtTarget() {
        boolean leaderOk = motorAtTarget(
                robot.getShooter().getTicksPerSec(), TunableValues.SHOOTER_TARGET_TICKS_PER_SEC);
        if (robot.shooterFollow == null) {
            return leaderOk;
        }
        boolean followOk = motorAtTarget(
                robot.getShooter().getFollowTicksPerSec(), TunableValues.SHOOTER_TARGET_TICKS_PER_SEC);
        return leaderOk && followOk;
    }

    private void updateSpeedTargets() {
        if (gamepad2.dpad_up && !wasGp2DpadUp) {
            bumpShooterTarget(TunableValues.SHOOTER_TICKS_PER_SEC_STEP);
        } else if (gamepad2.dpad_down && !wasGp2DpadDown) {
            bumpShooterTarget(-TunableValues.SHOOTER_TICKS_PER_SEC_STEP);
        }
        if (gamepad2.dpad_right && !wasGp2DpadRight) {
            bumpIntakeTarget(TunableValues.INTAKE_TICKS_PER_SEC_STEP);
        } else if (gamepad2.dpad_left && !wasGp2DpadLeft) {
            bumpIntakeTarget(-TunableValues.INTAKE_TICKS_PER_SEC_STEP);
        }
        wasGp2DpadUp = gamepad2.dpad_up;
        wasGp2DpadDown = gamepad2.dpad_down;
        wasGp2DpadLeft = gamepad2.dpad_left;
        wasGp2DpadRight = gamepad2.dpad_right;
    }

    private static void bumpShooterTarget(double deltaTicksPerSec) {
        TunableValues.SHOOTER_TARGET_TICKS_PER_SEC =
                Math.max(0.0, TunableValues.SHOOTER_TARGET_TICKS_PER_SEC + deltaTicksPerSec);
        RobotLog.i("[SHOOTER TARGET] %.0f ticks/s", TunableValues.SHOOTER_TARGET_TICKS_PER_SEC);
    }

    private static void bumpIntakeTarget(double deltaTicksPerSec) {
        TunableValues.INTAKE_TARGET_TICKS_PER_SEC =
                Math.max(0.0, TunableValues.INTAKE_TARGET_TICKS_PER_SEC + deltaTicksPerSec);
        RobotLog.i("[INTAKE TARGET] %.0f ticks/s", TunableValues.INTAKE_TARGET_TICKS_PER_SEC);
    }

    private String autoStatus() {
        boolean aligning = gamepad1.a;
        boolean firing = gamepad2.a;
        if (aligning && firing) {
            return "AUTO-ALIGNING AUTO-FIRING";
        }
        if (aligning) {
            return "AUTO-ALIGNING";
        }
        if (firing) {
            return "AUTO-FIRING";
        }
        return " ";
    }

    private void writeTelemetry() {
        telemetry.addData("SHOOTER TARGET", "%.0f ticks/s  (GP2 Up/Down)",
                TunableValues.SHOOTER_TARGET_TICKS_PER_SEC);
        telemetry.addData("INTAKE TARGET", "%.0f ticks/s  (GP2 Left/Right)",
                TunableValues.INTAKE_TARGET_TICKS_PER_SEC);
        telemetry.addData("SHOOTER TICKS/S", "leader %.0f  follow %.0f",
                robot.getShooter().getTicksPerSec(), robot.getShooter().getFollowTicksPerSec());
        telemetry.addData("SHOOTERS AT SPEED", bothShootersAtTarget()
                ? "YES" : String.format("NO  (need +/- %.0f%%)",
                TunableValues.SHOOTER_AT_SPEED_TOLERANCE * 100.0));
        telemetry.addData("TARGET", targetCell.getStatus());
        telemetry.addData("GATE", targetCell.getGateStatus());
        telemetry.addData("LED", robot.getLed().getStatus());
        telemetry.addData("WHEELS T/S", robot.getDrivetrain().getTicksPerSecStatus());
        telemetry.addData("X", poseXIn());
        telemetry.addData("Y", poseYIn());
        telemetry.addData("HEADING/YAW", imu.isPresent()
                ? String.format("%s  %.1f deg  (Back=start)", imu.getHeadingSource(), imu.getHeadingDegrees())
                : "MISSING");
        addOdometryBlockTelemetry();
        telemetry.addData("POWER", hardwareHealth.getPowerLine());
        telemetry.addData("AUTO", autoStatus());
        addErrorLine();
        telemetry.update();
    }

    private String poseXIn() {
        return imu.isPresent()
                ? String.format("%.1f in", imu.getPosXMm() / 25.4)
                : "MISSING";
    }

    private String poseYIn() {
        return imu.isPresent()
                ? String.format("%.1f in", imu.getPosYMm() / 25.4)
                : "MISSING";
    }

    private void addOdometryBlockTelemetry() {
        if (!imu.isPresent()) {
            telemetry.addData("ENC POS", "MISSING");
            telemetry.addData("ENC VEL", "MISSING");
            telemetry.addData("LOC STATUS", "MISSING");
            telemetry.addData("LOC XY", "MISSING");
            telemetry.addData("LOC VEL", "MISSING");
            telemetry.addData("OCTO GYRO", "MISSING");
            telemetry.addData("HUB GYRO", "MISSING");
            return;
        }
        telemetry.addData("ENC POS", imu.getEncoderPositionLine());
        telemetry.addData("ENC VEL", imu.getEncoderVelocityLine());
        telemetry.addData("LOC STATUS", imu.getLocalizerStatusLine());
        telemetry.addData("LOC XY", imu.getLocalizerPoseLine());
        telemetry.addData("LOC VEL", imu.getLocalizerVelocityLine());
        telemetry.addData("OCTO GYRO", imu.getOctoGyroLine());
        telemetry.addData("HUB GYRO", imu.getHubGyroLine());
    }

    private void addErrorLine() {
        String line = hardwareHealth.getErrorLine();
        telemetry.addData("ERROR", line.isEmpty() ? " " : line);
    }

    private static double squareStick(double value) {
        if (!TunableValues.DRIVE_STICK_SQUARED) {
            return value;
        }
        return value * Math.abs(value);
    }
}
