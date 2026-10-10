package org.firstinspires.ftc.teamcode;

// Import FTC linear OpMode base class for sequential program execution
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// Import FTC TeleOp annotation to register this program on Driver Station menu
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// Import FTC RobotLog utility for persistent logging
import com.qualcomm.robotcore.util.RobotLog;

// Road Runner Imports
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
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
import org.firstinspires.ftc.teamcode.subsystems.AprilTagVisionSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.BlinkinLedSubsystem;

// Register OpMode named IronEaglesTeleOp2026 under TeleOp group
@TeleOp(name = "IronEaglesTeleOp2026", group = "TeleOp")
public class IronEaglesTeleOp2026 extends LinearOpMode {

    // Set wheel motor spin direction multipliers
    public static int FRONT_LEFT_DIR  =  1;
    public static int FRONT_RIGHT_DIR = -1;
    public static int BACK_LEFT_DIR   =  1;
    public static int BACK_RIGHT_DIR  = -1;
    // Set motor directions for intake, hopper, and shooter (Intake & Shooter directions reversed)
    public static int INTAKE_DIR      =  1;
    public static int HOPPER_DIR      =  1;
    public static int SHOOTER_DIR     =  1;

    // Set default team alliance flag to blue team
    public static boolean IS_BLUE_ALLIANCE = true;

    // Create instance of main robot hardware manager
    private final IronEaglesHardware robot = new IronEaglesHardware();

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
        RobotLog.i("OpMode Started: Match timer running.");

        // Loop continuously while match is active until stop is pressed
        while (opModeIsActive()) {
            
            // =========================================================================
            // CRITICAL: Update Road Runner Odometry (Must run every loop!)
            // =========================================================================
            drive.update();

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
                    RobotLog.i("[FLYWHEEL] Aim-Assist active: Spooling up flywheel motor to 100%% power.");
                    wasFlywheelSpoolingAimAssist = true;
                    wasFlywheelRunningManual = false;
                }
                robot.getShooter().runShooter(1.0);
            } else if (isManualShootTriggerHeld) {
                if (!wasFlywheelRunningManual) {
                    RobotLog.i("[FLYWHEEL] Manual trigger squeezed (%.2f): Running flywheel motor.", gamepad1.right_trigger);
                    wasFlywheelRunningManual = true;
                    wasFlywheelSpoolingAimAssist = false;
                }
                robot.getShooter().runShooter(gamepad1.right_trigger);
            } else {
                if (wasFlywheelSpoolingAimAssist || wasFlywheelRunningManual) {
                    RobotLog.i("[FLYWHEEL] Flywheel motor stopped.");
                    wasFlywheelSpoolingAimAssist = false;
                    wasFlywheelRunningManual = false;
                }
                robot.getShooter().stop();
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

                // 3. Feeding logic
                if (firingState == AutoFireControllerInterface.FiringState.FEEDING) {
                    RobotLog.i("[AIM-ASSIST FEEDING] Actuating hopper motor at 100%% power to feed elements into flywheel.");
                    // Flywheel is already running from Subsystem 1, just start the hopper
                    robot.getHopper().runHopper(1.0);
                }

            } 
            // =========================================================================
            // MODE 2: ALL MANUAL MODE 
            // =========================================================================
            else {
                if (firingState != AutoFireControllerInterface.FiringState.IDLE) {
                    RobotLog.i("[MANUAL MODE] Exited Aim-Assist: Resetting state machine from %s back to IDLE.", firingState);
                    setFiringState(AutoFireControllerInterface.FiringState.IDLE);
                }

                // 1. Manual Mecanum Drivetrain Control via Road Runner
                boolean isManualDriving = (x != 0 || y != 0 || rx != 0);
                if (isManualDriving && !wasManualDriving) {
                    RobotLog.i("[MANUAL DRIVETRAIN] Joysticks engaged -> Y: %.2f | X: %.2f | Rot: %.2f", y, x, rx);
                    wasManualDriving = true;
                } else if (!isManualDriving && wasManualDriving) {
                    RobotLog.i("[MANUAL DRIVETRAIN] Joysticks released -> Drivetrain stopped.");
                    wasManualDriving = false;
                }

                drive.setWeightedDrivePower(
                        new Pose2d(y, x, rx) // Road Runner maps Y to forward, X to strafe
                );

                // 2. Manual Intake Control (Left Trigger)
                if (gamepad1.left_trigger > 0.2) { 
                    if (!wasIntakeRunning) {
                        RobotLog.i("[MANUAL INTAKE] Left trigger squeezed (%.2f) -> Running intake motor.", gamepad1.left_trigger);
                        wasIntakeRunning = true;
                    }
                    robot.getIntake().runIntake(gamepad1.left_trigger);
                } else { 
                    if (wasIntakeRunning) {
                        RobotLog.i("[MANUAL INTAKE] Left trigger released -> Intake motor stopped.");
                        wasIntakeRunning = false;
                    }
                    robot.getIntake().stop();
                }

                // 3. Manual Hopper Control (X Button — Steps 180° iteratively & returns upright)
                if (gamepad1.x) { 
                    robot.getHopper().stepForward180();
                } else { 
                    if (!robot.getHopper().isUpright()) { 
                        RobotLog.i("[MANUAL HOPPER] Returning hopper to 180 deg upright position...");
                        robot.getHopper().returnToUpright();
                    } else { 
                        robot.getHopper().stop();
                    }
                }
            }

            // Update edge detector
            wasInAimAssistMode = isInAimAssistMode;

            // =========================================================================
            // TELEMETRY UPDATES
            // =========================================================================
            telemetry.addData("Status", "Running");
            telemetry.addData("Ready to Shoot", isInRange ? "YES" : "NO");
            telemetry.addData("Fire State", firingState);
            telemetry.addData("GP1 Joysticks", "LY: %.2f | LX: %.2f | RX: %.2f", rawY, rawX, rawRx);
            telemetry.addData("GP1 Triggers", "L2: %.2f | R2: %.2f", gamepad1.left_trigger, gamepad1.right_trigger);
            telemetry.addData("GP1 Buttons", "X: %b | RB: %b | A: %b | B: %b | Y: %b", gamepad1.x, gamepad1.right_bumper, gamepad1.a, gamepad1.b, gamepad1.y);

            if (!robot.missingDevices.isEmpty()) {
                telemetry.addData("Missing Devices", String.join(", ", robot.missingDevices));
            }
            telemetry.update();
        }
    }
}
