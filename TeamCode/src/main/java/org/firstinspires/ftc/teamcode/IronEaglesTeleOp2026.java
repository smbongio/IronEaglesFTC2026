package org.firstinspires.ftc.teamcode;

// Import FTC linear OpMode base class for sequential program execution
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// Import FTC TeleOp annotation to register this program on Driver Station menu
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
// Import FTC RobotLog utility for persistent logging
import com.qualcomm.robotcore.util.RobotLog;

// Road Runner Imports
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import org.firstinspires.ftc.teamcode.config.TunableValues;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

// Import high-level auto-fire controller interface
import org.firstinspires.ftc.teamcode.interfaces.AutoFireControllerInterface;
// Import target spatial geometry interface
import org.firstinspires.ftc.teamcode.interfaces.TargetGeometryInterface;
// Import high-level LED indicator interface
import org.firstinspires.ftc.teamcode.interfaces.LedIndicatorInterface;
// Import high-level vision subsystem interface
import org.firstinspires.ftc.teamcode.interfaces.VisionSubsystemInterface;
// Import concrete subsystem implementations
import org.firstinspires.ftc.teamcode.interfaces.ImuInterface;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagVisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.BlinkinLedSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ControlHubImu;

// Register OpMode named IronEaglesTeleOp2026 under TeleOp group
@TeleOp(name = "IronEaglesTeleOp2026", group = "TeleOp")
public class IronEaglesTeleOp2026 extends LinearOpMode {

    // Set default team alliance flag to blue team
    public static boolean IS_BLUE_ALLIANCE = true;

    // Create instance of main robot hardware manager
    private final IronEaglesHardware robot = new IronEaglesHardware();
    private ImuInterface imu;

    // Declare reference for vision subsystem interface
    private VisionSubsystemInterface vision;
    // Declare reference for LED indicator interface
    private LedIndicatorInterface led;
    // Current firing state for automated shooting sequence
    private AutoFireControllerInterface.FiringState firingState = AutoFireControllerInterface.FiringState.IDLE;

    // State tracking for the button edge-detector
    private boolean wasInAimAssistMode = false;
    // State tracking for flywheel logging
    private boolean wasFlywheelSpoolingAimAssist = false;
    private boolean wasFlywheelRunningManual = false;
    // State tracking for LED indicator logging
    private LedIndicatorInterface.LedState currentLedState = null;
    // State tracking for manual drivetrain logging
    private boolean wasManualDriving = false;
    // State tracking for manual intake logging
    private boolean wasIntakeRunning = false;
    // State tracking for manual hopper button logging
    private boolean wasHopperXPressed = false;

    private enum TelemetryPage { ALL, INTAKE, HOPPER, SHOOTER }
    private TelemetryPage telemetryPage = TelemetryPage.ALL;
    private boolean wasDpadUp = false;
    private boolean wasDpadDown = false;
    private boolean wasDpadLeft = false;
    private boolean wasDpadRight = false;
    private boolean wasGp2DpadUp = false;
    private boolean wasGp2DpadDown = false;
    private boolean hopperFedThisSpool = false;
    private boolean hopper360InProgress = false;

    // Helper method to log firing state transitions
    private void setFiringState(AutoFireControllerInterface.FiringState newState) {
        if (this.firingState != newState) {
            RobotLog.i("==================================================");
            RobotLog.i("[AIM-ASSIST STATE TRANSITION] %s -> %s", this.firingState, newState);
            RobotLog.i("==================================================");
            this.firingState = newState;
        }
    }

    // Main execution entry point called when driver selects OpMode
    @Override
    public void runOpMode() {
        RobotLog.i("OpMode Initializing: IronEaglesTeleOp2026");

        // Initialize all hardware motors and camera on REV hubs
        robot.init(hardwareMap);
        imu = new ControlHubImu(hardwareMap);

        // Instantiate concrete Vision and LED Subsystems
        vision = new AprilTagVisionSubsystem(hardwareMap, robot.camera);
        led    = new BlinkinLedSubsystem(hardwareMap, "led");

        // Initialize Road Runner Drivetrain (This takes ownership of the 4 wheel motors)
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Check if vision subsystem interface is assigned
        if (vision != null) {
            // Set active alliance team color in vision subsystem for target detection
            vision.setAlliance(IS_BLUE_ALLIANCE ? VisionSubsystemInterface.Alliance.BLUE : VisionSubsystemInterface.Alliance.RED);
            RobotLog.i("Vision Subsystem Alliance set to: %s", IS_BLUE_ALLIANCE ? "BLUE" : "RED");
        } else {
            RobotLog.w("Vision Subsystem interface is NULL upon initialization.");
        }

        // Check if all configured hardware devices were successfully found
        if (robot.missingDevices.isEmpty()) { 
            telemetry.addData("Status", "Initialized - All devices found!");
            RobotLog.i("Hardware Map Initialization Complete: All devices found.");
        } else { 
            telemetry.addData("WARNING", "Initialized with missing config devices!");
            telemetry.addData("Missing", String.join(", ", robot.missingDevices));
            RobotLog.w("Hardware Map Initialization Warning: Missing devices -> %s", String.join(", ", robot.missingDevices));
        }
        telemetry.update();

        // Pause execution until driver presses play button on Driver Hub
        waitForStart();
        RobotLog.i("OpMode Started: Match timer running. heading=%.1f", imu.getHeadingDegrees());

        // Loop continuously while match is active until stop is pressed
        while (opModeIsActive()) {
            
            // =========================================================================
            // CRITICAL: Update Road Runner Odometry (Must run every loop!)
            // =========================================================================
            drive.update();

            if (gamepad1.backWasPressed()) {
                imu.resetYaw();
            }

            // Read joysticks
            double rawY  = -gamepad1.left_stick_y;
            double rawX  =  gamepad1.left_stick_x;
            double rawRx =  gamepad1.right_stick_x;

            // Apply 5 percent deadzone filters
            double y  = Math.abs(rawY)  > 0.05 ? rawY  : 0.0;
            double x  = Math.abs(rawX)  > 0.05 ? rawX  : 0.0;
            double rx = Math.abs(rawRx) > 0.05 ? rawRx : 0.0;

            boolean isManualShootTriggerHeld = gamepad1.right_trigger > 0.2;
            boolean isInAimAssistMode = gamepad1.right_bumper;
            boolean shooterCommanded = isInAimAssistMode || isManualShootTriggerHeld;
            updateShooterTargetFromGamepad2();
            boolean isInRange = (vision != null && vision.isInShootingRange());
            boolean isShooting = robot.getShooter().isRunning();

            // Mode Transition Logging
            if (isInAimAssistMode && !wasInAimAssistMode) {
                RobotLog.i(">>> ENTERING AIM-ASSIST MODE (Right Bumper Pressed) <<<");
            } else if (!isInAimAssistMode && wasInAimAssistMode) {
                RobotLog.i("<<< EXITING AIM-ASSIST MODE -> RETURNING TO MANUAL MODE >>>");
            }

            // Process LED indicator visual signals
            if (led != null) {
                LedIndicatorInterface.LedState targetLedState;
                if (isShooting) { 
                    targetLedState = LedIndicatorInterface.LedState.SHOOTING;
                } else if (isInRange) { 
                    targetLedState = LedIndicatorInterface.LedState.IN_RANGE;
                } else { 
                    targetLedState = LedIndicatorInterface.LedState.OFF;
                }

                if (currentLedState != targetLedState) {
                    RobotLog.i("[LED INDICATOR] Visual Signal State: %s -> %s", currentLedState, targetLedState);
                    currentLedState = targetLedState;
                    led.setLedState(targetLedState);
                }
            }

            // =========================================================================
            // SUBSYSTEM 1: FLYWHEEL (Decoupled for instant spool-up)
            // =========================================================================
            if (isInAimAssistMode) {
                if (!wasFlywheelSpoolingAimAssist) {
                    RobotLog.i("[FLYWHEEL] Aim-Assist active: Spooling flywheel to %.0f ticks/s.",
                            TunableValues.SHOOTER_TARGET_TICKS_PER_SEC);
                    wasFlywheelSpoolingAimAssist = true;
                    wasFlywheelRunningManual = false;
                }
                robot.getShooter().runShooter(1.0);
            } else if (isManualShootTriggerHeld) {
                if (!wasFlywheelRunningManual) {
                    RobotLog.i("[FLYWHEEL] Manual trigger squeezed: Running flywheel at %.0f ticks/s.",
                            TunableValues.SHOOTER_TARGET_TICKS_PER_SEC);
                    wasFlywheelRunningManual = true;
                    wasFlywheelSpoolingAimAssist = false;
                }
                robot.getShooter().runShooter(1.0);
            } else {
                if (wasFlywheelSpoolingAimAssist || wasFlywheelRunningManual) {
                    RobotLog.i("[FLYWHEEL] Flywheel motor stopped.");
                    wasFlywheelSpoolingAimAssist = false;
                    wasFlywheelRunningManual = false;
                }
                robot.getShooter().stop();
                hopperFedThisSpool = false;
                hopper360InProgress = false;
            }

            // =========================================================================
            // SUBSYSTEM 2: AIM-ASSIST STATE MACHINE & DRIVETRAIN
            // =========================================================================
            if (isInAimAssistMode) {
                
                // 1. Edge Detector: The exact moment the button is pressed
                if (!wasInAimAssistMode) {
                    RobotLog.i("[AIM-ASSIST] Initializing Aim-Assist: Querying Vision Subsystem for target geometry...");
                    TargetGeometryInterface target = (vision != null) ? vision.getBestTargetGeometry() : null;
                    
                    if (target != null) {
                        // TAKE THE SNAPSHOT
                        Pose2d currentPose = drive.getPoseEstimate();
                        
                        RobotLog.i("[AIM-ASSIST] Target Acquired! Tag ID: %d | Range: %.2f in | Bearing: %.2f deg | Pose: (X: %.2f, Y: %.2f)",
                                target.getTagId(), target.getRangeInches(), target.getBearingDegrees(), currentPose.getX(), currentPose.getY());

                        Trajectory alignShot = drive.trajectoryBuilder(currentPose)
                                .lineToLinearHeading(new Pose2d(
                                        currentPose.getX() + target.getXInches(), 
                                        currentPose.getY() + target.getYInches(), 
                                        Math.toRadians(target.getBearingDegrees())
                                ))
                                .build();

                        // Fire the asynchronous movement
                        RobotLog.i("[AIM-ASSIST] Dispatched Road Runner alignment trajectory asynchronously.");
                        drive.followTrajectoryAsync(alignShot);
                        setFiringState(AutoFireControllerInterface.FiringState.ALIGNING);
                    } else {
                        // Edge case: Target not found, stay idle
                        RobotLog.w("[AIM-ASSIST WARNING] Target query returned NULL (No AprilTag visible in camera frame).");
                        setFiringState(AutoFireControllerInterface.FiringState.IDLE);
                    }
                }

                // 2. Continuous Alignment Check
                if (firingState == AutoFireControllerInterface.FiringState.ALIGNING) {
                    RobotLog.i("[AIM-ASSIST ALIGNING] Executing alignment trajectory... drive.isBusy() = %b", drive.isBusy());
                    // drive.isBusy() asks Road Runner if the trajectory is still executing
                    if (!drive.isBusy()) {
                        // The drivetrain has reached the exact snapshot coordinates
                        RobotLog.i("[AIM-ASSIST ALIGNED] Trajectory execution complete! Robot in position. Transitioning to FEEDING.");
                        setFiringState(AutoFireControllerInterface.FiringState.FEEDING);
                    }
                }

                // Feeding waits until both shooters are at target ticks/s (see hopper 360 below).

            } 
            // =========================================================================
            // MODE 2: ALL MANUAL MODE 
            // =========================================================================
            else {
                if (firingState != AutoFireControllerInterface.FiringState.IDLE) {
                    RobotLog.i("[MANUAL MODE] Exited Aim-Assist: Resetting state machine from %s back to IDLE.", firingState);
                    setFiringState(AutoFireControllerInterface.FiringState.IDLE);
                }

                // 1. Manual field-centric mecanum (IMU heading; Back zeros yaw)
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
                robot.getDrivetrain().driveFieldCentric(x, y, rx, heading);

                // 2. Manual Intake Control (Left Trigger)
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

                // 3. Manual Hopper Control (X Button — skipped while auto 360 feed is running)
                if (!hopper360InProgress) {
                    if (gamepad1.x) {
                        if (!wasHopperXPressed) {
                            RobotLog.i("[GAMEPAD] X Button PRESSED -> Triggering Hopper 180-deg Step");
                            wasHopperXPressed = true;
                        }
                        robot.getHopper().stepForward180();
                    } else {
                        if (wasHopperXPressed) {
                            RobotLog.i("[GAMEPAD] X Button RELEASED");
                            wasHopperXPressed = false;
                        }
                        if (!robot.getHopper().isUpright()) {
                            robot.getHopper().returnToUpright();
                        } else {
                            robot.getHopper().stop();
                        }
                    }
                }
            }

            if (shooterCommanded) {
                updateHopperFeedWhenAtSpeed();
            }

            // Update edge detector
            wasInAimAssistMode = isInAimAssistMode;
            updateTelemetryPage();
            writeTelemetry();
        }
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

    private boolean motorAtTarget(double measuredTicksPerSec, double targetTicksPerSec) {
        double target = Math.abs(targetTicksPerSec);
        double band = Math.max(TunableValues.SHOOTER_TICKS_PER_SEC_STEP, target * 0.10);
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

    private void updateHopperFeedWhenAtSpeed() {
        if (hopperFedThisSpool) {
            return;
        }
        if (!hopper360InProgress && !bothShootersAtTarget()) {
            return;
        }
        boolean wasStepping = robot.getHopper().isStepping();
        robot.getHopper().stepForward360();
        hopper360InProgress = robot.getHopper().isStepping();
        if (wasStepping && !hopper360InProgress) {
            hopperFedThisSpool = true;
            RobotLog.i("[HOPPER] 360-degree feed complete.");
        }
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
        telemetry.addData("Page", "%s  (Up=ALL L=INTAKE D=HOPPER R=SHOOTER)", telemetryPage);
        telemetry.addData("GP1 Pressed", pressedButtons(gamepad1));
        telemetry.addData("IMU", imu.isPresent()
                ? String.format("%.1f deg  (Back=zero)", imu.getHeadingDegrees())
                : "MISSING");
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
                telemetry.addData("Fire State", firingState);
                telemetry.addData("Hopper cmd", robot.getHopper().getCommandStatus());
                telemetry.addData("Hopper Upright", robot.getHopper().isUpright() ? "YES" : "NO");
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
                telemetry.addData("Drive", "field-centric");
                telemetry.addData("Fire State", firingState);
                telemetry.addData("Intake ticks/s", "%.0f", robot.getIntake().getTicksPerSec());
                telemetry.addData("Shooter ticks/s", "leader %.0f  follow %.0f",
                        robot.getShooter().getTicksPerSec(), robot.getShooter().getFollowTicksPerSec());
                telemetry.addData("Hopper", "running=%s pwr=%s",
                        robot.getHopper().isRunning() ? "YES" : "NO",
                        motorPower(robot.hopper));
                telemetry.addData("Wheels", "FL %s  FR %s  BL %s  BR %s",
                        motorPower(robot.frontLeft), motorPower(robot.frontRight),
                        motorPower(robot.backLeft), motorPower(robot.backRight));
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
}
