package org.firstinspires.ftc.teamcode;

// Import FTC linear OpMode base class for sequential program execution
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// Import FTC TeleOp annotation to register this program on Driver Station menu
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// Import high-level auto-fire controller interface
import org.firstinspires.ftc.teamcode.interfaces.AutoFireControllerInterface;
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
    public static int BACK_RIGHT_DIR  = -1;
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
    // Declare reference for automated firing controller interface
    private AutoFireControllerInterface autoFireController;

    // Main execution entry point called when driver selects OpMode
    @Override
    public void runOpMode() {
        // Initialize all hardware motors and camera on REV hubs
        robot.init(hardwareMap);

        // If vision subsystem exists, set active alliance color
        if (vision != null) {
            vision.setAlliance(IS_BLUE_ALLIANCE ? VisionSubsystemInterface.Alliance.BLUE : VisionSubsystemInterface.Alliance.RED);
        }

        // If all hardware devices were found, report ready status
        if (robot.missingDevices.isEmpty()) {
            telemetry.addData("Status", "Initialized - All devices found!");
        } else { // Otherwise display warning listing missing device names
            telemetry.addData("WARNING", "Initialized with missing config devices!");
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

            // Calculate and send power values to 4 drive motors
            robot.getDrivetrain().drive(x, y, rx);

            // If X button is pressed, run intake at 100 percent speed
            if (gamepad1.x) {
                robot.getIntake().runIntake(1.0);
            } else { // Otherwise stop intake motor immediately
                robot.getIntake().stop();
            }

            // If left trigger is squeezed past 20 percent threshold
            if (gamepad1.left_trigger > 0.2) { // Run hopper motor at speed proportional to trigger press
                robot.getHopper().runHopper(gamepad1.left_trigger);
            } else { // Otherwise stop hopper motor immediately
                robot.getHopper().stop();
            }

            // If right trigger is squeezed past 20 percent threshold
            if (gamepad1.right_trigger > 0.2) { // Run shooter motor at speed proportional to trigger press
                robot.getShooter().runShooter(gamepad1.right_trigger);
            } else { // Otherwise stop shooter motor immediately
                robot.getShooter().stop();
            }

            // Display operational status on Driver Hub telemetry
            telemetry.addData("Status", "Running");
            // Display raw joystick values for driver testing
            telemetry.addData("GP1 Joysticks", "LY: %.2f | LX: %.2f | RX: %.2f", rawY, rawX, rawRx);
            // Display raw trigger values for driver testing
            telemetry.addData("GP1 Triggers", "L2: %.2f | R2: %.2f", gamepad1.left_trigger, gamepad1.right_trigger);
            // Display raw button states for driver testing
            telemetry.addData("GP1 Buttons", "X: %b | A: %b | B: %b | Y: %b", gamepad1.x, gamepad1.a, gamepad1.b, gamepad1.y);

            // If any hardware devices are missing, list them on screen
            if (!robot.missingDevices.isEmpty()) {
                telemetry.addData("Missing Devices", String.join(", ", robot.missingDevices));
            }
            // Update Driver Hub telemetry display for current loop cycle
            telemetry.update();
        }
    }
}
