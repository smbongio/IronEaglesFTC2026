package org.firstinspires.ftc.teamcode;

// Import FTC linear OpMode base class for sequential program execution
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// Import FTC TeleOp annotation to register this program on Driver Station menu
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

    // Set front left motor spin direction multiplier to normal
    public static int FRONT_LEFT_DIR  =  1;
    // Set front right motor spin direction multiplier to inverted
    public static int FRONT_RIGHT_DIR = -1;
    // Set back left motor spin direction multiplier to normal
    public static int BACK_LEFT_DIR   =  1;
    // Set back right motor spin direction multiplier to inverted
    public static int BACK_RIGHT_DIR  = 1;
    // Set intake motor spin direction multiplier to normal
    public static int INTAKE_DIR      =  1;
    // Set hopper motor spin direction multiplier to normal
    public static int HOPPER_DIR      =  1;
    // Set shooter motor spin direction multiplier to normal
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

    // Main execution entry point called when driver selects OpMode
    @Override
    public void runOpMode() {
        // Initialize all hardware motors and camera on REV hubs
        robot.init(hardwareMap);

        // Check if vision subsystem interface is assigned
        if (vision != null) {
            // Set active alliance team color in vision subsystem for target detection
            vision.setAlliance(IS_BLUE_ALLIANCE ? VisionSubsystemInterface.Alliance.BLUE : VisionSubsystemInterface.Alliance.RED);
        }

        // Check if all configured hardware devices were successfully found
        if (robot.missingDevices.isEmpty()) { // If no devices are missing
            // Display initialization success status message on Driver Hub
            telemetry.addData("Status", "Initialized - All devices found!");
        } else { // Otherwise display warning listing missing device names
            // Display warning status message for missing hardware
            telemetry.addData("WARNING", "Initialized with missing config devices!");
            // Display comma-separated list of missing device names
            telemetry.addData("Missing", String.join(", ", robot.missingDevices));
        }
        // Send initialization status message to Driver Hub screen
        telemetry.update();

        // Pause execution until driver presses play button on Driver Hub
        waitForStart();

        // Loop continuously while match is active until stop is pressed
        while (opModeIsActive()) {
            // Read left joystick Y axis and invert for forward drive
            double rawY  = -gamepad1.left_stick_y;
            // Read left joystick X axis for strafe drive
            double rawX  =  gamepad1.left_stick_x;
            // Read right joystick X axis for turn drive
            double rawRx =  gamepad1.right_stick_x;

            // Apply 5 percent deadzone filter to forward drive input
            double y  = Math.abs(rawY)  > 0.05 ? rawY  : 0.0;
            // Apply 5 percent deadzone filter to strafe drive input
            double x  = Math.abs(rawX)  > 0.05 ? rawX  : 0.0;
            // Apply 5 percent deadzone filter to turn drive input
            double rx = Math.abs(rawRx) > 0.05 ? rawRx : 0.0;

            // Check if right trigger is squeezed past 20 percent threshold for manual shooter override
            boolean isManualShootTriggerHeld = gamepad1.right_trigger > 0.2;

            // Check if right bumper is pressed to trigger automated firing sequence
            boolean isAutoShootBumperHeld = gamepad1.right_bumper;

            // Query vision subsystem to check if target is acquired and in valid shooting range
            boolean isInRange = (vision != null && vision.isInShootingRange());

            // Check if shooter motor is actively running
            boolean isShooting = robot.getShooter().isRunning();

            // Process LED indicator visual signals based on current system state
            if (led != null) {
                if (isShooting) { // Solid LED when actively shooting
                    led.setLedState(LedIndicatorInterface.LedState.SHOOTING);
                } else if (isInRange) { // Quickly blinking LED when target is in range and ready to shoot
                    led.setLedState(LedIndicatorInterface.LedState.IN_RANGE);
                } else { // No LED light when not shooting and no target in range
                    led.setLedState(LedIndicatorInterface.LedState.OFF);
                }
            }

            // =========================================================================
            // MODE 1: AUTO-SHOOT MODE (Triggered when Right Bumper is held down)
            // =========================================================================
            if (isAutoShootBumperHeld) {
                // Spin up shooter flywheel motor immediately
                robot.getShooter().runShooter(1.0);

                // If state is IDLE, start sequence by updating state to ALIGNING
                if (firingState == AutoFireControllerInterface.FiringState.IDLE) {
                    firingState = AutoFireControllerInterface.FiringState.ALIGNING;
                }

                // Check and execute alignment with the goal
                if (firingState == AutoFireControllerInterface.FiringState.ALIGNING) {
                    TargetGeometryInterface target = (vision != null) ? vision.getBestTargetGeometry() : null;
                    boolean aligned;

                    if (target != null) { // Begin or check alignment with target
                        aligned = robot.getDrivetrain().alignToTarget(target);
                    } else { // Fallback if no target visible
                        aligned = true;
                        robot.getDrivetrain().stop();
                    }

                    // Are we aligned? Change state to FEEDING (SHOOT) and stop drivetrain
                    if (aligned) {
                        robot.getDrivetrain().stop();
                        firingState = AutoFireControllerInterface.FiringState.FEEDING;
                    }
                }

                // Once aligned, run the hopper and the shooter to feed balls through
                if (firingState == AutoFireControllerInterface.FiringState.FEEDING) {
                    robot.getShooter().runShooter(1.0);
                    robot.getHopper().runHopper(1.0);
                }
            }
            // =========================================================================
            // MODE 2: ALL MANUAL MODE (Executed when Right Bumper is NOT held down)
            // =========================================================================
            else {
                // Reset auto-shoot state machine to IDLE
                firingState = AutoFireControllerInterface.FiringState.IDLE;

                // 1. Manual Mecanum Drivetrain Control
                robot.getDrivetrain().drive(x, y, rx);

                // 2. Manual Intake Control (X button)
                if (gamepad1.x) { // If X button is pressed
                    // Run intake motor at 100 percent speed
                    robot.getIntake().runIntake(1.0);
                } else { // Otherwise stop intake motor immediately
                    // Stop intake motor
                    robot.getIntake().stop();
                }

                // 3. Manual Hopper Control (Left Trigger)
                if (gamepad1.left_trigger > 0.2) { // If left trigger is squeezed
                    // Run hopper motor at speed proportional to trigger press
                    robot.getHopper().runHopper(gamepad1.left_trigger);
                } else { // Otherwise stop hopper motor immediately
                    // Stop hopper motor
                    robot.getHopper().stop();
                }

                // 4. Manual Shooter Control (Right Trigger)
                if (isManualShootTriggerHeld) { // If right trigger is squeezed
                    // Run manual shooter motor at speed proportional to trigger press
                    robot.getShooter().runShooter(gamepad1.right_trigger);
                } else { // Otherwise stop shooter motor immediately
                    // Stop shooter motor
                    robot.getShooter().stop();
                }
            }

            // Display operational status on Driver Hub telemetry
            telemetry.addData("Status", "Running");
            // Display target acquisition and shooting range readiness status on Driver Hub
            telemetry.addData("Ready to Shoot", isInRange ? "YES" : "NO");
            // Display current auto-fire state machine status on Driver Hub
            telemetry.addData("Fire State", firingState);
            // Display raw joystick values for driver testing
            telemetry.addData("GP1 Joysticks", "LY: %.2f | LX: %.2f | RX: %.2f", rawY, rawX, rawRx);
            // Display raw trigger values for driver testing
            telemetry.addData("GP1 Triggers", "L2: %.2f | R2: %.2f", gamepad1.left_trigger, gamepad1.right_trigger);
            // Display raw button states for driver testing
            telemetry.addData("GP1 Buttons", "X: %b | RB: %b | A: %b | B: %b | Y: %b", gamepad1.x, gamepad1.right_bumper, gamepad1.a, gamepad1.b, gamepad1.y);

            // Check if any hardware devices are missing from configuration
            if (!robot.missingDevices.isEmpty()) {
                // List missing hardware device names on screen
                telemetry.addData("Missing Devices", String.join(", ", robot.missingDevices));
            }
            // Update Driver Hub telemetry display for current loop cycle
            telemetry.update();
        }
    }
}
