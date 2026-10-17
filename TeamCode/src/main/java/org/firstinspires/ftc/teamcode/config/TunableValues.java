package org.firstinspires.ftc.teamcode.config;

/**
 * Field-tunable values for mechanical-debugging. Flip a motor direction here
 * instead of hunting through OpMode and subsystem classes.
 *
 * Each *_DIR is +1 or -1. It is multiplied into setPower() (and hopper step ticks).
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
    public static int SHOOTER_FOLLOW_DIR = 1;
}
