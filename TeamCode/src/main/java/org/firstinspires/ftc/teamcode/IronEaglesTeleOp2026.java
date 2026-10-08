package org.firstinspires.ftc.teamcode;

// Import FTC linear OpMode base class for sequential program execution
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// Import FTC TeleOp annotation to register this program on Driver Station menu
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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

// Register OpMode named IronEaglesTeleOp2026 under TeleOp group
@TeleOp(name = "IronEaglesTeleOp2026", group = "TeleOp")
public class IronEaglesTeleOp2026 extends LinearOpMode {

    // Set motor directions (Assuming SampleMecanumDrive handles wheel inversions now)
    public static int INTAKE_DIR      =  -1;
    public static int HOPPER_DIR      =  1;
    public static int SHOOTER_DIR     =  -1;

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
    private boolean wasInAutoShootMode = false;

    // Main execution entry point called when driver selects OpMode
    @Override
    public void runOpMode() {
        // Initialize all hardware motors and camera on REV hubs
        robot.init(hardwareMap);

        // Initialize Road Runner Drivetrain (This takes ownership of the 4 wheel motors)
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Check if vision subsystem interface is assigned
        if (vision != null) {
            // Set active alliance team color in vision subsystem for target detection
            vision.setAlliance(IS_BLUE_ALLIANCE ? VisionSubsystemInterface.Alliance.BLUE : VisionSubsystemInterface.Alliance.RED);
        }

        // Check if all configured hardware devices were successfully found
        if (robot.missingDevices.isEmpty()) {
            telemetry.addData("Status", "Initialized - All devices found!");
        } else {
            telemetry.addData("WARNING", "Initialized with missing config devices!");
            telemetry.addData("Missing", String.join(", ", robot.missingDevices));
        }
        telemetry.update();

        // Pause execution until driver presses play button on Driver Hub
        waitForStart();

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
            boolean isInAutoShootMode = gamepad1.right_bumper;
            boolean isInRange = (vision != null && vision.isInShootingRange());
            boolean isShooting = robot.getShooter().isRunning();

            // Process LED indicator visual signals
            if (led != null) {
                if (isShooting) {
                    led.setLedState(LedIndicatorInterface.LedState.SHOOTING);
                } else if (isInRange) {
                    led.setLedState(LedIndicatorInterface.LedState.IN_RANGE);
                } else {
                    led.setLedState(LedIndicatorInterface.LedState.OFF);
                }
            }

            // =========================================================================
            // SUBSYSTEM 1: FLYWHEEL (Decoupled for instant spool-up)
            // =========================================================================
            if (isInAutoShootMode) {
                robot.getShooter().runShooter(1.0);
            } else if (isManualShootTriggerHeld) {
                robot.getShooter().runShooter(gamepad1.right_trigger);
            } else {
                robot.getShooter().stop();
            }

            // =========================================================================
            // SUBSYSTEM 2: AUTO-SHOOT STATE MACHINE & DRIVETRAIN
            // =========================================================================
            if (isInAutoShootMode) {

                // 1. Edge Detector: The exact moment the button is pressed
                if (!wasInAutoShootMode) {
                    TargetGeometryInterface target = (vision != null) ? vision.getBestTargetGeometry() : null;

                    if (target != null) {
                        // TAKE THE SNAPSHOT
                        Pose2d currentPose = drive.getPoseEstimate();

                        // NOTE: You will need to replace getOffsetX() and getOffsetY() with
                        // the actual method names from your TargetGeometryInterface.
                        Trajectory alignShot = drive.trajectoryBuilder(currentPose)
                                .lineToLinearHeading(new Pose2d(
                                        currentPose.getX() + target.getOffsetX(),
                                        currentPose.getY() + target.getOffsetY(),
                                        Math.toRadians(target.getTargetHeading())
                                ))
                                .build();

                        // Fire the asynchronous movement
                        drive.followTrajectoryAsync(alignShot);
                        firingState = AutoFireControllerInterface.FiringState.ALIGNING;
                    } else {
                        // Edge case: Target not found, stay idle
                        firingState = AutoFireControllerInterface.FiringState.IDLE;
                    }
                }

                // 2. Continuous Alignment Check
                if (firingState == AutoFireControllerInterface.FiringState.ALIGNING) {
                    // drive.isBusy() asks Road Runner if the trajectory is still executing
                    if (!drive.isBusy()) {
                        // The drivetrain has reached the exact snapshot coordinates
                        firingState = AutoFireControllerInterface.FiringState.FEEDING;
                    }
                }

                // 3. Feeding logic
                if (firingState == AutoFireControllerInterface.FiringState.FEEDING) {
                    // Flywheel is already running from Subsystem 1, just start the hopper
                    robot.getHopper().runHopper(1.0);
                }

            }
            // =========================================================================
            // MODE 2: ALL MANUAL MODE
            // =========================================================================
            else {
                firingState = AutoFireControllerInterface.FiringState.IDLE;

                // 1. Manual Mecanum Drivetrain Control via Road Runner
                // This replaces robot.getDrivetrain().drive(x, y, rx) so odometry tracks accurately
                drive.setWeightedDrivePower(
                        new Pose2d(y, x, rx) // Road Runner maps Y to forward, X to strafe
                );

                // 2. Manual Intake Control (X button)
                if (gamepad1.x) {
                    robot.getIntake().runIntake(1.0);
                } else {
                    robot.getIntake().stop();
                }

                // 3. Manual Hopper Control (Left Trigger)
                if (gamepad1.left_trigger > 0.2) {
                    robot.getHopper().runHopper(gamepad1.left_trigger);
                } else {
                    if (!robot.getHopper().isUpright()) {
                        robot.getHopper().returnToUpright();
                    } else {
                        robot.getHopper().stop();
                    }
                }
            }

            // Update edge detector
            wasInAutoShootMode = isInAutoShootMode;

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
