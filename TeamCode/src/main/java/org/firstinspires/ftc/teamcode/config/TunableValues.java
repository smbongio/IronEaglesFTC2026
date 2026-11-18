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
            "Gamepad 1 during INIT:\n"
                    + "  B                alliance RED\n"
                    + "  X                alliance BLUE\n"
                    + "  Y                use start pose instead of Auto's final pose\n"
                    + "Gamepad 1 (Driver):\n"
                    + "  Left stick       field-centric X/Y\n"
                    + "  Right stick X    yaw\n"
                    + "  A                hold: auto-align placeholder\n"
                    + "  Back             restore start heading (faces +X)\n"
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
    public static double INTAKE_POWER = 0.50;

    // Shooter runs open-loop (no encoder plugged into ehPort2). Fraction of full
    // motor power while RT/RB is held. Starting point; tune on the field.
    public static double SHOOTER_POWER = 0.35;
    public static double SHOOTER_POWER_STEP = 0.05;
    public static double HOPPER_MANUAL_POWER = 0.6;

    // Three dead wheels, no IMU (FTCLib HolonomicOdometry).
    // LEFT = ehPort1 encoder, RIGHT = ehPort0 encoder (both front-to-back).
    // REAR = ehPort3 encoder, the X wheel (side-to-side).
    // Set each *_DIR so pushing the robot forward makes LEFT and RIGHT ticks go up,
    // and pushing it left makes REAR ticks go up.
    // ehPort0 reads through the intake motor, which is REVERSE (that flips its sign).
    // Hand push 11/14: LEFT counted down going forward, so it is -1. RIGHT and REAR confirmed.
    // REAR was +1 while it read through the REVERSE shooter on port 3. The shooter moved to
    // ehPort2 on 11/16, so ehPort3 is no longer reversed: -1 keeps the same sign. Re-check by hand.
    public static int ODO_LEFT_DIR = -1;
    public static int ODO_RIGHT_DIR = -1;
    public static int ODO_REAR_DIR = -1;

    // REV Through Bore (quadrature 8192 ticks/rev) on 3.5 in diameter pod wheels (confirmed 11/14).
    // 8192 / (pi * 3.5 * 25.4) = 29.33. Push a known distance on carpet to refine.
    public static double ODO_TICKS_PER_MM = 8192.0 / (Math.PI * 3.5 * 25.4);

    // Left-to-right distance between the two front-to-back wheels' contact points.
    // Measured 8.25 in; 8.205 makes the 11/14 10-turn spin read 3600 with the alignment
    // correction below. Re-tune after changing the alignment values:
    // spin in place 10 turns, then scale this by (logged degrees / 3600).
    public static double ODO_TRACK_WIDTH_MM = 8.205 * 25.4;
    // Rear pod distance in front of the midpoint between the left and right pods
    // (negative = behind). Re-measured 11/14 with the robot on the floor: 9.0 in behind.
    // Measure this physically: a spin test can't, because the rear pod reads its distance from
    // the drivetrain's rotation center, which need not be the pod midpoint.
    public static double ODO_REAR_POD_X_MM = -9.0 * 25.4;

    // Pod misalignment: how much each pod falsely reads per unit of travel it should ignore.
    // LEFT/RIGHT: drag the robot sideways along a tape line without turning it, then
    //   set = (change in LEFT or RIGHT POD TICKS) / (change in REAR POD TICKS).
    // REAR: drag it straight forward the same way, then
    //   set = (change in REAR POD TICKS) / (average change in LEFT and RIGHT POD TICKS).
    // 11/14 10 ft sideways drag: LEFT +2136, RIGHT -531, REAR -88599 ticks.
    public static double ODO_LEFT_PER_SIDEWAYS = -0.0241;
    public static double ODO_RIGHT_PER_SIDEWAYS = 0.0060;
    public static double ODO_REAR_PER_FORWARD = 0.0;

    // Official FTC field frame (BIOBUZZ square, red alliance audience-left):
    // origin at field center, walls at ±72 in. Stand at the Red Wall looking in:
    // +Y toward blue, +X to the right (toward the audience).
    // 18x18 robot, front bumper on the audience wall (the +X wall / cell),
    // near bumper 52 in from the red wall, front faces +X.
    // Start center: X = 72-9 = +63 in, Y = 52+9-72 = -11 in, heading 0°.
    // These are the RED start. Blue mirrors them across the X axis (see Alliance): (63, +11, 0°).
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
    // Red-side value; a blue driver at the blue wall uses -90 (see Alliance).
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

    // Our alliance's audience hive: red tags 34-37, blue tags 38-41 (SDK BIOBUZZ library).
    // TRUE when SUCCESSES of the last SAMPLES frames saw it.
    // Example: SAMPLE_RATE=10 and SUCCESS_RATE=3 means 3 of 10. Default is 5 of 10 (50%).
    public static int VISION_RED_TAG_MIN = 34;
    public static int VISION_RED_TAG_MAX = 37;
    public static int VISION_BLUE_TAG_MIN = 38;
    public static int VISION_BLUE_TAG_MAX = 41;
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
