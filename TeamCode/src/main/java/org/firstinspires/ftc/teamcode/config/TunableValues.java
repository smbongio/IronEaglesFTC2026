package org.firstinspires.ftc.teamcode.config;

/**
 * Field-tunable values. Flip a motor direction here instead of hunting through
 * OpMode and subsystem classes.
 *
 * Each *_DIR is +1 or -1. It is multiplied into setPower() (and hopper step ticks).
 *
 * Intake is open-loop motor power. Shooter speed is encoder ticks per second
 * (what setVelocity() takes). No RPM conversion.
 *
 * Gamepad map is {@link #GAMEPAD_CONTROLS}.
 */
public final class TunableValues {
    private TunableValues() {}

    public static final String GAMEPAD_CONTROLS =
            "Gamepad 1 (Driver):\n"
                    + "  Left stick       field-centric X/Y\n"
                    + "  Right stick X    yaw\n"
                    + "  A                hold: auto-align placeholder\n"
                    + "  Back             restore start heading (+X)\n"
                    + "Gamepad 2 (Operator):\n"
                    + "  RT               hold: shooter forward\n"
                    + "  RB               hold: shooter reverse\n"
                    + "  D-pad up         tap: shooter target += 25 ticks/s\n"
                    + "  D-pad down       tap: shooter target -= 25 ticks/s\n"
                    + "  A                hold: auto-fire placeholder\n"
                    + "  LT               hold: intake forward\n"
                    + "  LB               hold: intake reverse\n"
                    + "  X                hold: hopper forward\n"
                    + "  Y                hold: hopper reverse";

    public static int FRONT_LEFT_DIR = 1;
    public static int FRONT_RIGHT_DIR = -1;
    public static int BACK_LEFT_DIR = 1;
    public static int BACK_RIGHT_DIR = -1;

    public static int INTAKE_DIR = 1;
    public static int HOPPER_DIR = -1;
    public static int SHOOTER_DIR = 1;
    public static int SHOOTER_FOLLOW_DIR = -1;

    // Intake has no encoder. Fraction of full motor power while LT/LB is held.
    public static double INTAKE_POWER = 0.25;

    // Hub velocity command (encoder ticks per second).
    public static double SHOOTER_TARGET_TICKS_PER_SEC = 1000.0;
    public static double SHOOTER_TICKS_PER_SEC_STEP = 25.0;
    public static double SHOOTER_AT_SPEED_TOLERANCE = 0.05;
    public static double HOPPER_MANUAL_POWER = 0.6;

    // Velocity PIDF for the shooter. Hub firmware defaults.
    public static double SHOOTER_P = 10.0;
    public static double SHOOTER_I = 2.0;
    public static double SHOOTER_D = 1.0;
    public static double SHOOTER_F = 0.0;

    // Three dead wheels, no IMU (FTCLib HolonomicOdometry).
    // leftPod = EH encoder 0, rightPod = EH encoder 2 (both front-to-back).
    // backPod = CH encoder 1 (side-to-side, rear).
    // Set each *_DIR so pushing the robot forward makes left and right ticks go up,
    // and pushing it left makes back ticks go up.
    // EH0 reads through the intake motor, which is REVERSE, and needed -1 on the 11/12 lap.
    public static int ODO_LEFT_DIR = -1;
    public static int ODO_RIGHT_DIR = 1;
    public static int ODO_BACK_DIR = 1;

    // REV Through Bore (quadrature 8192 ticks/rev) on 4.0 in diameter pods.
    // Theoretical: 8192 / (pi * 4 * 25.4) mm. Push a known distance on carpet to refine.
    public static double ODO_TICKS_PER_MM = 25.665;

    // Left-to-right distance between the two front-to-back wheels' contact points.
    // Placeholder until measured. Tune: spin in place 10 turns, then scale this by
    // (logged degrees / 3600).
    public static double ODO_TRACK_WIDTH_MM = 14.0 * 25.4;
    // Back pod distance in front of robot center (negative = behind center).
    // If an in-place spin makes X/Y drift, this sign or size is wrong.
    public static double ODO_BACK_POD_X_MM = -7.5 * 25.4;

    // Official FTC field frame (BIOBUZZ square, red alliance audience-left):
    // origin at field center, walls at ±72 in. Stand at the Red Wall looking in:
    // +Y toward blue, +X to the right (toward the audience).
    // 18x18 robot, front bumper on the audience wall (the +X wall / cell),
    // near bumper 52 in from the red wall, front faces +X.
    // Start center: X = 72-9 = +63 in, Y = 52+9-72 = -11 in, heading 0°.
    public static double FIELD_SIZE_IN = 144.0;
    public static double ROBOT_SIZE_IN = 18.0;
    public static double START_NEAR_BUMPER_FROM_RED_IN = 52.0;
    public static double START_POSE_X_IN =
            FIELD_SIZE_IN / 2.0 - ROBOT_SIZE_IN / 2.0;
    public static double START_POSE_Y_IN =
            START_NEAR_BUMPER_FROM_RED_IN + ROBOT_SIZE_IN / 2.0 - FIELD_SIZE_IN / 2.0;
    public static double START_HEADING_DEG = 0.0;
    // Stick-up is field +Y (toward blue, away from a red-side driver). Odometry heading
    // 0 is field +X (audience), so field-centric drive uses heading minus this.
    public static double DRIVE_STICK_ZERO_DEG = 90.0;

    // Sign-preserving square on analog sticks after deadzone. Full stick is still full power.
    public static boolean DRIVE_STICK_SQUARED = true;

    // Cancels dead-wheel motion on axes the driver is not holding. Held axes pass through.
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

    // Red audience hive (tags 34-37). TRUE when SUCCESSES of the last SAMPLES frames saw it.
    // Example: SAMPLE_RATE=10 and SUCCESS_RATE=3 means 3 of 10. Default is 5 of 10 (50%).
    public static int VISION_TARGET_TAG_MIN = 34;
    public static int VISION_TARGET_TAG_MAX = 37;
    public static int VISION_SAMPLE_RATE = 10;
    public static int VISION_SUCCESS_RATE = 5;

    // Blinkin green on/off cycles per second while the target cell gate is TRUE.
    public static double LED_FLASH_HZ = 5.0;

    // Hub power logging. Events fire when voltage/current steps past these
    // amounts, or when the rail is below the low threshold.
    public static double POWER_VIN_LOW_V = 11.0;
    public static double POWER_VIN_STEP_V = 0.50;
    public static double POWER_V5_LOW_V = 4.75;
    public static double POWER_V5_STEP_V = 0.15;
    public static double POWER_CURRENT_STEP_A = 2.0;
    // FTC legal pack has a 20A inline fuse. Flag before it blows.
    public static double POWER_PACK_HIGH_A = 18.0;
}
