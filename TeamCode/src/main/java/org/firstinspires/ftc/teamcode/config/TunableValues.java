package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

/**
 * Field-tunable values for mechanical-debugging. Flip a motor direction here
 * instead of hunting through OpMode and subsystem classes.
 *
 * Each *_DIR is +1 or -1. It is multiplied into setPower() (and hopper step ticks).
 *
 * Intake/shooter speed is encoder ticks per second (what setVelocity() takes).
 * No RPM conversion.
 *
 * Gamepad map is {@link #GAMEPAD_CONTROLS}.
 */
public final class TunableValues {
    private TunableValues() {}

    public static final String GAMEPAD_CONTROLS =
            "Gamepad 1:\n"
                    + "  Left stick Y     drive forward / back (field-centric)\n"
                    + "  Left stick X     strafe left / right (field-centric)\n"
                    + "  Right stick X    rotate\n"
                    + "  Left trigger     intake at INTAKE_TARGET_TICKS_PER_SEC (on/off; ignored below 0.2)\n"
                    + "  Right trigger    shooter at SHOOTER_TARGET_TICKS_PER_SEC (on/off; ignored below 0.2)\n"
                    + "  X                hold: hopper 180-deg step after shooters are within SHOOTER_AT_SPEED_TOLERANCE; release returns upright\n"
                    + "  D-pad up         telemetry page: ALL (summary)\n"
                    + "  D-pad left       telemetry page: INTAKE\n"
                    + "  D-pad down       telemetry page: HOPPER\n"
                    + "  D-pad right      telemetry page: SHOOTER\n"
                    + "  Back             reset OctoQuad heading (field-centric forward)\n"
                    + "Gamepad 2:\n"
                    + "  D-pad up         both shooters target ticks/s += SHOOTER_TICKS_PER_SEC_STEP\n"
                    + "  D-pad down       both shooters target ticks/s -= SHOOTER_TICKS_PER_SEC_STEP\n"
                    + "Unused: A, B, Y, bumpers, Start, stick clicks, right stick Y.";

    // Hub on the robot's right side, upright: logo face toward the right wall,
    // USB-C ports toward the floor.
    public static RevHubOrientationOnRobot.LogoFacingDirection IMU_LOGO_DIRECTION =
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;
    public static RevHubOrientationOnRobot.UsbFacingDirection IMU_USB_DIRECTION =
            RevHubOrientationOnRobot.UsbFacingDirection.DOWN;

    public static int FRONT_LEFT_DIR = 1;
    public static int FRONT_RIGHT_DIR = -1;
    public static int BACK_LEFT_DIR = 1;
    public static int BACK_RIGHT_DIR = -1;

    public static int INTAKE_DIR = 1;
    public static int HOPPER_DIR = -1;
    public static int SHOOTER_DIR = 1;
    public static int SHOOTER_FOLLOW_DIR = -1;

    // Hub velocity command (encoder ticks per second).
    public static double INTAKE_TARGET_TICKS_PER_SEC = 500.0;
    public static double SHOOTER_TARGET_TICKS_PER_SEC = 1500.0;
    public static double SHOOTER_TICKS_PER_SEC_STEP = 25.0;
    // Hopper X waits until both shooters are within this fraction of target ticks/s.
    public static double SHOOTER_AT_SPEED_TOLERANCE = 0.05;

    // Shared velocity PIDF for shooter and shooterFollow. Hub firmware defaults.
    public static double SHOOTER_P = 10.0;
    public static double SHOOTER_I = 2.0;
    public static double SHOOTER_D = 1.0;
    public static double SHOOTER_F = 0.0;

    // Control Hub I2C bus the OctoQuad is plugged into.
    // Bus 0 is the built-in IMU (no external jack). First physical I2C jack is bus 1.
    // Must match IronEaglesConfig.xml <OctoQuadFTC bus="...">.
    public static int OCTOQUAD_I2C_BUS = 1;

    // OctoQuad MK2 dead wheels. Port 0 = forward (X), port 1 = strafe (Y).
    // *_DIR is +1 or -1. They do not change field-centric heading.
    public static int OCTOQUAD_FORWARD_PORT = 0;
    public static int OCTOQUAD_STRAFE_PORT = 1;
    public static int OCTOQUAD_FORWARD_DIR = 1;
    public static int OCTOQUAD_STRAFE_DIR = 1;

    // Placeholders so the onboard localizer can start. Tune later for x/y pose.
    public static float OCTOQUAD_FORWARD_TICKS_PER_MM = 12.34f;
    public static float OCTOQUAD_STRAFE_TICKS_PER_MM = 12.34f;
    public static float OCTOQUAD_TCP_OFFSET_X_MM = 123.4f;
    public static float OCTOQUAD_TCP_OFFSET_Y_MM = 123.4f;
    public static float OCTOQUAD_IMU_SCALAR = 1.0f;

    // Cancels OctoQuad motion on axes the driver is not holding. Held axes pass through.
    public static boolean DRIVE_ASSIST_ENABLED = true;
    public static double DRIVE_ASSIST_MAX_VEL_MM_S = 2000.0;
    public static double DRIVE_ASSIST_MAX_OMEGA_RAD_S = 3.0;
    public static double DRIVE_ASSIST_TRANSLATION_P = 0.25;
    public static double DRIVE_ASSIST_TRANSLATION_I = 0.0;
    public static double DRIVE_ASSIST_TRANSLATION_D = 0.0;
    public static double DRIVE_ASSIST_ROTATION_P = 0.25;
    public static double DRIVE_ASSIST_ROTATION_I = 0.0;
    public static double DRIVE_ASSIST_ROTATION_D = 0.0;
    public static double DRIVE_ASSIST_I_LIMIT = 0.25;
    public static double DRIVE_ASSIST_MAX_CORRECTION = 0.25;
}
