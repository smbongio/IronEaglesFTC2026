package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.config.TunableValues;
import org.firstinspires.ftc.teamcode.control.DriveVelocityAssist;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagVision;
import org.firstinspires.ftc.teamcode.subsystems.HardwareHealthMonitor;
import org.firstinspires.ftc.teamcode.subsystems.DeadWheelOdometry;
import org.firstinspires.ftc.teamcode.subsystems.TargetCellGate;

@TeleOp(name = "IronEaglesTeleOp2026", group = "TeleOp")
public class IronEaglesTeleOp2026 extends LinearOpMode {

    private final IronEaglesHardware robot = new IronEaglesHardware();
    private final DriveVelocityAssist driveAssist = new DriveVelocityAssist();
    private final TargetCellGate targetCell = new TargetCellGate();
    private DeadWheelOdometry odometry;
    private AprilTagVision vision;
    private HardwareHealthMonitor hardwareHealth;

    private boolean wasFlywheelRunning = false;
    private boolean wasManualDriving = false;
    private boolean wasIntakeRunning = false;
    private boolean wasHopperRunning = false;
    private boolean wasGp2DpadUp = false;
    private boolean wasGp2DpadDown = false;

    @Override
    public void runOpMode() {
        RobotLog.i("OpMode Initializing: IronEaglesTeleOp2026");

        robot.init(hardwareMap);
        odometry = new DeadWheelOdometry(robot.leftPod, robot.rightPod, robot.backPod);
        vision = new AprilTagVision(hardwareMap, robot.camera);
        vision.sample();
        hardwareHealth = new HardwareHealthMonitor(hardwareMap, robot, odometry, vision);
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
        writeTelemetry();

        RobotLog.i("[CONTROLS]\n%s", TunableValues.GAMEPAD_CONTROLS);
        waitForStart();
        RobotLog.i("OpMode Started: Match timer running. heading=%.1f", odometry.getHeadingDegrees());

        try {
            while (opModeIsActive()) {
                odometry.update();
                targetCell.update(vision.sample());
                hardwareHealth.update();

                if (gamepad1.backWasPressed()) {
                    odometry.resetHeading();
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
                odometry.logOdometryDiagnostics();
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
                    y, x, rx, odometry.getHeadingDegrees());
            wasManualDriving = true;
        } else if (!isManualDriving && wasManualDriving) {
            RobotLog.i("[MANUAL DRIVETRAIN] Joysticks released -> Drivetrain stopped.");
            wasManualDriving = false;
        }

        double heading = odometry.isPresent() ? odometry.getDriveHeadingDegrees() : 0.0;
        if (odometry.hasLocalizerVelocity()) {
            driveAssist.apply(x, y, rx,
                    odometry.getVelXMmS(), odometry.getVelYMmS(), odometry.getVelHeadingRadS());
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
            RobotLog.i("[MANUAL INTAKE] %s at %.0f%% power.",
                    direction, TunableValues.INTAKE_POWER * 100.0);
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

    private void updateSpeedTargets() {
        if (gamepad2.dpad_up && !wasGp2DpadUp) {
            bumpShooterTarget(TunableValues.SHOOTER_TICKS_PER_SEC_STEP);
        } else if (gamepad2.dpad_down && !wasGp2DpadDown) {
            bumpShooterTarget(-TunableValues.SHOOTER_TICKS_PER_SEC_STEP);
        }
        wasGp2DpadUp = gamepad2.dpad_up;
        wasGp2DpadDown = gamepad2.dpad_down;
    }

    private static void bumpShooterTarget(double deltaTicksPerSec) {
        TunableValues.SHOOTER_TARGET_TICKS_PER_SEC =
                Math.max(0.0, TunableValues.SHOOTER_TARGET_TICKS_PER_SEC + deltaTicksPerSec);
        RobotLog.i("[SHOOTER TARGET] %.0f ticks/s", TunableValues.SHOOTER_TARGET_TICKS_PER_SEC);
    }

    private void writeTelemetry() {
        telemetry.addData("X", poseInches(true));
        telemetry.addData("Y", poseInches(false));
        telemetry.addData("LEFT POD TICKS", podTicks(odometry.getLeftTicks()));
        telemetry.addData("RIGHT POD TICKS", podTicks(odometry.getRightTicks()));
        telemetry.addData("BACK POD TICKS", podTicks(odometry.getBackTicks()));
        telemetry.addData("HEADING DEGREES", headingLine());
        telemetry.addData("SHOOTER TARGET TICKS/SEC", "%.0f",
                TunableValues.SHOOTER_TARGET_TICKS_PER_SEC);
        telemetry.addData("SHOOTER TICKS/SEC", "%.0f",
                robot.getShooter().getTicksPerSec());
        telemetry.addData("INTAKE POWER", "%.0f%%",
                Math.abs(robot.getIntake().getAppliedPower()) * 100.0);
        telemetry.addData("ERRORS", errorLine());
        telemetry.update();
    }

    private String poseInches(boolean x) {
        if (!odometry.isPresent()) {
            return "MISSING";
        }
        double mm = x ? odometry.getPosXMm() : odometry.getPosYMm();
        return String.format("%.1f", mm / 25.4);
    }

    private String podTicks(int ticks) {
        if (!odometry.isPresent()) {
            return "MISSING";
        }
        return Integer.toString(ticks);
    }

    private String headingLine() {
        if (!odometry.isPresent()) {
            return "MISSING";
        }
        return String.format("%.1f", odometry.getHeadingDegrees());
    }

    private String errorLine() {
        String line = hardwareHealth.getErrorLine();
        return line.isEmpty() ? " " : line;
    }

    private static double squareStick(double value) {
        if (!TunableValues.DRIVE_STICK_SQUARED) {
            return value;
        }
        return value * Math.abs(value);
    }
}
