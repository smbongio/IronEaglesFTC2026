package org.firstinspires.ftc.teamcode.config;

/**
 * Field-tunable values. Flip a motor direction here instead of hunting through
 * OpMode and subsystem classes.
 *
 * Each *_DIR is +1 or -1. It is multiplied into setPower().
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
                    + "  D-pad up         tap: shooter power += 5%\n"
                    + "  D-pad down       tap: shooter power -= 5%\n"
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
    // Hopper is now a continuous-mode Smart Servo. -1 was set for the old Core Hex motor;
    // flip it if X runs the hopper backwards.
    public static int HOPPER_DIR = -1;
    public static int SHOOTER_DIR = 1;

    // Intake has no encoder. Fraction of full motor power while LT/LB is held.
    public static double INTAKE_POWER = 0.25;

    // Shooter has no encoder (its jack is the right odometry pod). Fraction of full
    // motor power while RT/RB is held. Starting point; tune on the field.
    public static double SHOOTER_POWER = 0.35;
    public static double SHOOTER_POWER_STEP = 0.05;
    public static double HOPPER_MANUAL_POWER = 0.6;

    // Three dead wheels, no IMU (FTCLib HolonomicOdometry).
    // LEFT = EH encoder 0, RIGHT = EH encoder 3 (both front-to-back).
    // REAR = EH encoder 1 (side-to-side).
    // Set each *_DIR so pushing the robot forward makes LEFT and RIGHT ticks go up,
    // and pushing it left makes REAR ticks go up.
    // EH0 and EH3 read through the intake and shooter motors, which are both REVERSE.
    public static int ODO_LEFT_DIR = -1;
    public static int ODO_RIGHT_DIR = 1;
    public static int ODO_REAR_DIR = 1;

    // REV Through Bore (quadrature 8192 ticks/rev) on 4.0 in diameter pods.
    // Theoretical: 8192 / (pi * 4 * 25.4) mm. Push a known distance on carpet to refine.
    public static double ODO_TICKS_PER_MM = 25.665;

    // Left-to-right distance between the two front-to-back wheels' contact points (measured).
    // Fine-tune: spin in place 10 turns, then scale this by (logged degrees / 3600).
    public static double ODO_TRACK_WIDTH_MM = 8.25 * 25.4;
    // Rear pod distance in front of the midpoint between the left and right pods
    // (negative = behind). Measured 8.75 in behind.
    // If an in-place spin makes field X/Y drift, this sign or size is wrong.
    public static double ODO_REAR_POD_X_MM = -8.75 * 25.4;

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
