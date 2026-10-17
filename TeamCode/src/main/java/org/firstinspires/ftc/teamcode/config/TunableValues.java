package org.firstinspires.ftc.teamcode.config;

/**
 * Field-tunable values for mechanical-debugging. Flip a motor direction here
 * instead of hunting through OpMode and subsystem classes.
 *
 * Each *_DIR is +1 or -1. It is multiplied into setPower() (and hopper step ticks).
 *
 * Gamepad 1 controls (IronEaglesTeleOp2026). Gamepad 2 is unused.
 *
 *   Left stick Y     drive forward / back
 *   Left stick X     strafe left / right
 *   Right stick X    rotate
 *   Left trigger     intake at INTAKE_TARGET_RPM (on/off; ignored below 0.2)
 *   Right trigger    shooter at SHOOTER_TARGET_RPM (on/off; ignored below 0.2)
 *   Right bumper     hold: aim-assist (spool shooter, align, then hopper)
 *   X                hold: hopper 180-degree step; release returns upright
 *   D-pad up         telemetry page: ALL (summary)
 *   D-pad left       telemetry page: INTAKE
 *   D-pad down       telemetry page: HOPPER
 *   D-pad right      telemetry page: SHOOTER
 *
 * Unused: A, B, Y, left bumper, right stick Y, gamepad 2.
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

    public static double INTAKE_TARGET_RPM = 500.0;
    public static double SHOOTER_TARGET_RPM = 2000.0;
    // Encoder ticks per output revolution. Hardware config is UnspecifiedMotor, so set these
    // to match the motor (REV HD Hex 20:1 = 560, 40:1 = 1120, Core Hex = 288).
    public static double INTAKE_TICKS_PER_REV = 560.0;
    public static double SHOOTER_TICKS_PER_REV = 560.0;
}
