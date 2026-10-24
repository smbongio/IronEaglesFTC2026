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
 * Gamepad 1:
 *   Left stick Y     drive forward / back
 *   Left stick X     strafe left / right
 *   Right stick X    rotate
 *   Left trigger     intake at INTAKE_TARGET_TICKS_PER_SEC (on/off; ignored below 0.2)
 *   Right trigger    shooter at SHOOTER_TARGET_TICKS_PER_SEC (on/off; ignored below 0.2)
 *   Right bumper     hold: aim-assist (spool shooter, align, then hopper)
 *   X                hold: hopper 180-degree step; release returns upright
 *   D-pad up         telemetry page: ALL (summary)
 *   D-pad left       telemetry page: INTAKE
 *   D-pad down       telemetry page: HOPPER
 *   D-pad right      telemetry page: SHOOTER
 *   Back             reset OctoQuad heading (field-centric forward)
 *
 * Gamepad 2:
 *   D-pad up         both shooters target ticks/s += 25
 *   D-pad down       both shooters target ticks/s -= 25
 *
 * Unused: A, B, Y, left bumper, right stick Y, d-pad left/right.
 */
public final class TunableValues {
    private TunableValues() {}

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

    // Shared velocity PIDF for shooter and shooterFollow. Hub firmware defaults.
    public static double SHOOTER_P = 10.0;
    public static double SHOOTER_I = 2.0;
    public static double SHOOTER_D = 1.0;
    public static double SHOOTER_F = 0.0;

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
}
