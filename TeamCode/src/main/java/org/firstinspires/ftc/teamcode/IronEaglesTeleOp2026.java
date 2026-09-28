package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "IronEaglesTeleOp2026", group = "TeleOp")
public class IronEaglesTeleOp2026 extends LinearOpMode {

    // Direction Multiplier Constants (1 or -1)
    // Right-side motors are set to -1 because they are physically mounted opposite the left side
    public static int FRONT_LEFT_DIR  = 1;
    public static int FRONT_RIGHT_DIR = -1;
    public static int BACK_LEFT_DIR   = 1;
    public static int BACK_RIGHT_DIR  = -1;
    public static int INTAKE_DIR      = 1;
    public static int HOPPER_DIR      = 1;
    public static int SHOOTER_DIR     =  1;

    private final IronEaglesHardware robot = new IronEaglesHardware();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        if (robot.missingDevices.isEmpty()) {
            telemetry.addData("Status", "Initialized - All devices found!");
        } else {
            telemetry.addData("WARNING", "Initialized with missing config devices!");
            telemetry.addData("Missing", String.join(", ", robot.missingDevices));
        }
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Drivetrain Kinematics with 5% deadzone filter to prevent stick drift
            // Inverted rawRx to fix rotation direction
            double rawY  = gamepad1.left_stick_y;
            double rawX  = gamepad1.left_stick_x;
            double rawRx = gamepad1.right_stick_x;

            double y  = Math.abs(rawY)  > 0.05 ? rawY  : 0.0;
            double x  = Math.abs(rawX)  > 0.05 ? rawX  : 0.0;
            double rx = Math.abs(rawRx) > 0.05 ? rawRx : 0.0;

            robot.getDrivetrain().drive(x, y, rx);

            // Intake (X button momentary)
            if (gamepad1.x) {
                robot.getIntake().runIntake(1.0);
            } else {
                robot.getIntake().stop();
            }

            // Hopper (Left Trigger / L2 threshold > 0.2)
            if (gamepad1.left_trigger > 0.2) {
                robot.getHopper().runHopper(gamepad1.left_trigger);
            } else {
                robot.getHopper().stop();
            }

            // Shooter (Right Trigger / R2 threshold > 0.2)
            if (gamepad1.right_trigger > 0.2) {
                robot.getShooter().runShooter(gamepad1.right_trigger);
            } else {
                robot.getShooter().stop();
            }

            // Telemetry Output for Live Testing
            telemetry.addData("Status", "Running");
            telemetry.addData("GP1 Joysticks", "LY: %.2f | LX: %.2f | RX: %.2f", rawY, rawX, rawRx);
            telemetry.addData("GP1 Triggers", "L2: %.2f | R2: %.2f", gamepad1.left_trigger, gamepad1.right_trigger);
            telemetry.addData("GP1 Buttons", "X: %b | A: %b | B: %b | Y: %b", gamepad1.x, gamepad1.a, gamepad1.b, gamepad1.y);

            if (!robot.missingDevices.isEmpty()) {
                telemetry.addData("Missing Devices", String.join(", ", robot.missingDevices));
            }
            telemetry.update();
        }
    }
}
