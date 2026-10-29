package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
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
    private boolean wasHopperXPressed = false;
    private boolean wasHopperWaitingForSpeed = false;

    private enum TelemetryPage { ALL, INTAKE, HOPPER, SHOOTER }
    private TelemetryPage telemetryPage = TelemetryPage.ALL;
    private boolean wasDpadUp = false;
    private boolean wasDpadDown = false;
    private boolean wasDpadLeft = false;
    private boolean wasDpadRight = false;
    private boolean wasGp2DpadUp = false;
    private boolean wasGp2DpadDown = false;

    @Override
    public void runOpMode() {
        RobotLog.i("OpMode Initializing: IronEaglesTeleOp2026");

        robot.init(hardwareMap);
        imu = new OctoQuadOdometry(hardwareMap);
        vision = new AprilTagVision(hardwareMap, robot.camera);
        hardwareHealth = new HardwareHealthMonitor(hardwareMap, robot, imu, vision);
        hardwareHealth.update();
        if (hardwareHealth.hasErrors()) {
            telemetry.addData("ERROR", hardwareHealth.getErrorLine());
        }

        if (robot.missingDevices.isEmpty()) {
            telemetry.addData("Status", "Initialized - All devices found!");
            RobotLog.i("Hardware Map Initialization Complete: All devices found.");
        } else {
            telemetry.addData("WARNING", "Initialized with missing config devices!");
            telemetry.addData("Missing", String.join(", ", robot.missingDevices));
            RobotLog.w("Hardware Map Initialization Warning: Missing devices -> %s",
                    String.join(", ", robot.missingDevices));
        }
        telemetry.addData("Heading", imu.isPresent()
                ? String.format("%s  heading=%.1f", imu.getHeadingSource(), imu.getHeadingDegrees())
                : "MISSING - drive is robot-centric");
        telemetry.addData("Target", vision.isPresent() ? "waiting" : "FALSE");
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

                updateShooterTargetFromGamepad2();
                driveFieldCentric(x, y, rx);
                updateIntake();
                updateShooter();
                updateHopper();

                robot.getLed().update(targetCell.isTargetVisible());
                robot.getDrivetrain().logWheelDiagnostics();
                imu.logOdometryDiagnostics();
                updateTelemetryPage();
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

        double heading = imu.isPresent() ? imu.getHeadingDegrees() : 0.0;
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
        if (gamepad1.left_trigger > 0.2) {
            if (!wasIntakeRunning) {
                RobotLog.i("[MANUAL INTAKE] Left trigger squeezed -> Running intake at %.0f ticks/s.",
                        TunableValues.INTAKE_TARGET_TICKS_PER_SEC);
                wasIntakeRunning = true;
            }
            robot.getIntake().runIntake(1.0);
        } else {
            if (wasIntakeRunning) {
                RobotLog.i("[MANUAL INTAKE] Left trigger released -> Intake motor stopped.");
                wasIntakeRunning = false;
            }
            robot.getIntake().stop();
        }
    }

    private void updateShooter() {
        if (gamepad1.right_trigger > 0.2) {
            if (!wasFlywheelRunning) {
                RobotLog.i("[FLYWHEEL] Manual trigger squeezed: Running flywheel at %.0f ticks/s.",
                        TunableValues.SHOOTER_TARGET_TICKS_PER_SEC);
                wasFlywheelRunning = true;
            }
            robot.getShooter().runShooter(1.0);
        } else {
            if (wasFlywheelRunning) {
                RobotLog.i("[FLYWHEEL] Flywheel motor stopped.");
                wasFlywheelRunning = false;
            }
            robot.getShooter().stop();
        }
    }

    private void updateHopper() {
        if (gamepad1.x) {
            if (!wasHopperXPressed) {
                RobotLog.i("[GAMEPAD] X Button PRESSED");
                wasHopperXPressed = true;
            }
            if (robot.getHopper().isStepping() || bothShootersAtTarget()) {
                if (wasHopperWaitingForSpeed) {
                    RobotLog.i("[HOPPER] Shooters at target +/- %.0f%%. Stepping 180 deg.",
                            TunableValues.SHOOTER_AT_SPEED_TOLERANCE * 100.0);
                    wasHopperWaitingForSpeed = false;
                }
                robot.getHopper().stepForward180();
            } else {
                if (!wasHopperWaitingForSpeed) {
                    RobotLog.i("[HOPPER] Waiting for shooters at target +/- %.0f%% (leader %.0f  follow %.0f  target %.0f).",
                            TunableValues.SHOOTER_AT_SPEED_TOLERANCE * 100.0,
                            robot.getShooter().getTicksPerSec(),
                            robot.getShooter().getFollowTicksPerSec(),
                            TunableValues.SHOOTER_TARGET_TICKS_PER_SEC);
                    wasHopperWaitingForSpeed = true;
                }
            }
        } else {
            if (wasHopperXPressed) {
                RobotLog.i("[GAMEPAD] X Button RELEASED");
                wasHopperXPressed = false;
                wasHopperWaitingForSpeed = false;
            }
            if (!robot.getHopper().isUpright()) {
                robot.getHopper().returnToUpright();
            } else {
                robot.getHopper().stop();
            }
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

    private void updateShooterTargetFromGamepad2() {
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

    private void updateTelemetryPage() {
        TelemetryPage selected = telemetryPage;
        if (gamepad1.dpad_up && !wasDpadUp) {
            selected = TelemetryPage.ALL;
        } else if (gamepad1.dpad_left && !wasDpadLeft) {
            selected = TelemetryPage.INTAKE;
        } else if (gamepad1.dpad_down && !wasDpadDown) {
            selected = TelemetryPage.HOPPER;
        } else if (gamepad1.dpad_right && !wasDpadRight) {
            selected = TelemetryPage.SHOOTER;
        }
        if (selected != telemetryPage) {
            telemetryPage = selected;
            RobotLog.i("[TELEMETRY PAGE] %s", telemetryPage);
        }
        wasDpadUp = gamepad1.dpad_up;
        wasDpadDown = gamepad1.dpad_down;
        wasDpadLeft = gamepad1.dpad_left;
        wasDpadRight = gamepad1.dpad_right;
    }

    private void writeTelemetry() {
        if (hardwareHealth.hasErrors()) {
            telemetry.addData("ERROR", hardwareHealth.getErrorLine());
        }
        telemetry.addData("Page", "%s  (Up=ALL L=INTAKE D=HOPPER R=SHOOTER)", telemetryPage);
        telemetry.addData("GP1 Pressed", pressedButtons(gamepad1));
        telemetry.addData("Heading", imu.isPresent()
                ? String.format("%s  %.1f deg  (Back=zero)", imu.getHeadingSource(), imu.getHeadingDegrees())
                : "MISSING");
        telemetry.addData("Target", targetCell.getStatus());
        telemetry.addData("LED", robot.getLed().getStatus());
        telemetry.addData("Pods", imu.getPodTickStatus());
        if (!robot.missingDevices.isEmpty()) {
            telemetry.addData("Missing Devices", String.join(", ", robot.missingDevices));
        }

        switch (telemetryPage) {
            case INTAKE:
                telemetry.addData("Intake cmd", robot.getIntake().getCommandStatus());
                telemetry.addData("Intake ticks/s", "%.0f (target %.0f)",
                        robot.getIntake().getTicksPerSec(), TunableValues.INTAKE_TARGET_TICKS_PER_SEC);
                break;
            case HOPPER:
                telemetry.addData("Hopper cmd", robot.getHopper().getCommandStatus());
                telemetry.addData("Hopper Upright", robot.getHopper().isUpright() ? "YES" : "NO");
                telemetry.addData("Shooters at speed", bothShootersAtTarget()
                        ? "YES" : String.format("NO  (need +/- %.0f%%)",
                        TunableValues.SHOOTER_AT_SPEED_TOLERANCE * 100.0));
                break;
            case SHOOTER:
                addShooterMotorTelemetry(
                        "shooter",
                        TunableValues.SHOOTER_TARGET_TICKS_PER_SEC,
                        robot.getShooter().getTicksPerSec(),
                        robot.getShooter().getPidf());
                addShooterMotorTelemetry(
                        "shooterFollow",
                        TunableValues.SHOOTER_TARGET_TICKS_PER_SEC,
                        robot.getShooter().getFollowTicksPerSec(),
                        robot.getShooter().getFollowPidf());
                break;
            case ALL:
            default:
                telemetry.addData("Target gate", targetCell.getGateStatus());
                telemetry.addData("Drive", TunableValues.DRIVE_ASSIST_ENABLED
                        ? "field-centric + velocity PID"
                        : "field-centric");
                telemetry.addData("Pose", imu.getPoseStatus());
                telemetry.addData("Odo vel", imu.getVelocityStatus());
                telemetry.addData("Assist", driveAssist.getStatus());
                telemetry.addData("Drive cmd", robot.getDrivetrain().getStickStatus());
                telemetry.addData("Wheels pwr", robot.getDrivetrain().getRequestedPowerStatus());
                telemetry.addData("Wheels t/s", robot.getDrivetrain().getTicksPerSecStatus());
                telemetry.addData("Intake ticks/s", "%.0f", robot.getIntake().getTicksPerSec());
                telemetry.addData("Shooter ticks/s", "leader %.0f  follow %.0f",
                        robot.getShooter().getTicksPerSec(), robot.getShooter().getFollowTicksPerSec());
                telemetry.addData("Hopper", "running=%s pwr=%s",
                        robot.getHopper().isRunning() ? "YES" : "NO",
                        motorPower(robot.hopper));
                break;
        }
        telemetry.update();
    }

    private void addShooterMotorTelemetry(String name, double targetTicksPerSec,
                                          double currentTicksPerSec, PIDFCoefficients pidf) {
        telemetry.addLine(name + ":");
        telemetry.addLine(String.format("target ticks/s: %.0f", targetTicksPerSec));
        telemetry.addLine(String.format("current ticks/s: %.0f", currentTicksPerSec));
        telemetry.addLine(String.format("P: %.2f, I: %.2f, D: %.2f, F: %.2f",
                pidf.p, pidf.i, pidf.d, pidf.f));
    }

    private static String motorPower(DcMotor motor) {
        return motor == null ? "n/a" : String.format("%.2f", motor.getPower());
    }

    private static String pressedButtons(Gamepad gamepad) {
        StringBuilder pressed = new StringBuilder();
        appendPressed(pressed, gamepad.dpad_up, "D-Up");
        appendPressed(pressed, gamepad.dpad_down, "D-Down");
        appendPressed(pressed, gamepad.dpad_left, "D-Left");
        appendPressed(pressed, gamepad.dpad_right, "D-Right");
        appendPressed(pressed, gamepad.a, "A");
        appendPressed(pressed, gamepad.b, "B");
        appendPressed(pressed, gamepad.x, "X");
        appendPressed(pressed, gamepad.y, "Y");
        appendPressed(pressed, gamepad.left_bumper, "LB");
        appendPressed(pressed, gamepad.right_bumper, "RB");
        appendPressed(pressed, gamepad.left_trigger > 0.2, "LT");
        appendPressed(pressed, gamepad.right_trigger > 0.2, "RT");
        appendPressed(pressed, gamepad.left_stick_button, "L3");
        appendPressed(pressed, gamepad.right_stick_button, "R3");
        appendPressed(pressed, gamepad.start, "Start");
        appendPressed(pressed, gamepad.back, "Back");
        return pressed.length() == 0 ? "none" : pressed.toString();
    }

    private static void appendPressed(StringBuilder pressed, boolean isPressed, String name) {
        if (!isPressed) {
            return;
        }
        if (pressed.length() > 0) {
            pressed.append(' ');
        }
        pressed.append(name);
    }

    private static double squareStick(double value) {
        if (!TunableValues.DRIVE_STICK_SQUARED) {
            return value;
        }
        return value * Math.abs(value);
    }
}
