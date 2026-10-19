package org.firstinspires.ftc.teamcode.config;

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
 *
 * Gamepad 2:
 *   D-pad up         both shooters target ticks/s += 25
 *   D-pad down       both shooters target ticks/s -= 25
 *
 * Unused: A, B, Y, left bumper, right stick Y, d-pad left/right.
 */
public final class TunableValues {
    private TunableValues() {}

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
    public static double SHOOTER_FOLLOW_TARGET_TICKS_PER_SEC = 1500.0;
    public static double SHOOTER_TICKS_PER_SEC_STEP = 25.0;

    // Shared velocity PIDF for shooter and shooterFollow. Hub firmware defaults.
    public static double SHOOTER_P = 10.0;
    public static double SHOOTER_I = 0.5;
    public static double SHOOTER_D = 1.0;
    public static double SHOOTER_F = 0.0;
}
