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
                    + "  D-pad right      tap: intake target += 25 ticks/s\n"
                    + "  D-pad left       tap: intake target -= 25 ticks/s\n"
                    + "  A                hold: auto-fire placeholder\n"
                    + "  LT               hold: intake forward\n"
                    + "  LB               hold: intake reverse\n"
                    + "  X                hold: hopper forward\n"
                    + "  Y                hold: hopper reverse";

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
    public static double INTAKE_TICKS_PER_SEC_STEP = 25.0;
    public static double SHOOTER_TARGET_TICKS_PER_SEC = 1000.0;
    public static double SHOOTER_TICKS_PER_SEC_STEP = 25.0;
    public static double SHOOTER_AT_SPEED_TOLERANCE = 0.05;
    public static double HOPPER_MANUAL_POWER = 0.6;

    // Shared velocity PIDF for shooter and shooterFollow. Hub firmware defaults.
    public static double SHOOTER_P = 10.0;
    public static double SHOOTER_I = 2.0;
    public static double SHOOTER_D = 1.0;
    public static double SHOOTER_F = 0.0;

    // Control Hub I2C bus the OctoQuad is plugged into (first external jack is I2C 1).
    // Must match IronEaglesConfig.xml <OctoQuadFTC bus="..."> on Control Hub.
    public static int OCTOQUAD_I2C_BUS = 1;

    // true: heading and X/Y come from the OctoQuad IMU/localizer.
    // false: heading and X/Y come from Control Hub IMU + dead-wheel ticks.
    public static boolean USE_OCTOQUAD_IMU = true;

    // OctoQuad MK2 dead wheels (encoder jacks on the OctoQuad, not hub motor ports).
    // X wheel = robot forward/back, port 6. At heading 0 this is field +X.
    // Y wheel = robot left/right strafe, port 7. At heading 0 this is field +Y.
    // *_DIR is +1 or -1. They do not change field-centric heading.
    public static int OCTOQUAD_FORWARD_PORT = 6;
    public static int OCTOQUAD_STRAFE_PORT = 7;
    public static int OCTOQUAD_FORWARD_DIR = 1;
    public static int OCTOQUAD_STRAFE_DIR = 1;

    // REV Through Bore (quadrature 8192 ticks/rev) on 4.0 in diameter pods.
    // Theoretical: 8192 / (pi * 4 * 25.4) mm. Push a known distance on carpet to refine.
    public static float OCTOQUAD_FORWARD_TICKS_PER_MM = 25.665f;
    public static float OCTOQUAD_STRAFE_TICKS_PER_MM = 25.665f;

    // Chassis inches, origin at center, +X right, +Y forward, bumpers at ±9:
    // OctoQuad board (−3, 1, 11), X/forward pod (4, 1), Y/strafe pod (0, −7.5).
    // TCP is pod-axis intersection (4, −7.5). Offset is TCP → center in the
    // OctoQuad localizer frame (+X forward, +Y left): 7.5 in forward, 4 in left.
    // If an in-place spin makes pose orbit, negate OCTOQUAD_TCP_OFFSET_Y_MM.
    public static float OCTOQUAD_TCP_OFFSET_X_MM = 7.5f * 25.4f;
    public static float OCTOQUAD_TCP_OFFSET_Y_MM = 4.0f * 25.4f;
    // Board height above the field. MK2 localizer has no Z TCP; this is docs only.
    public static float OCTOQUAD_HEIGHT_Z_IN = 11.0f;
    public static float OCTOQUAD_IMU_SCALAR = 1.0f;
    // Board remounted flat (logs: IMU axis=Z). OctoQuad onboard XY is 90° CW from
    // the FTC field. Rotate readings CCW so Driver Hub X/Y are +X audience, +Y blue.
    public static double OCTOQUAD_POSE_ROTATION_DEG = 90.0;
    // -1 if driving toward the +X wall makes X decrease.
    public static int OCTOQUAD_POSE_X_DIR = -1;
    // -1 if driving toward +Y (blue) makes Y decrease.
    public static int OCTOQUAD_POSE_Y_DIR = -1;

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
    // Stick-up is field +Y (toward blue, away from a red-side driver). IMU heading
    // 0 is field +X (audience), so field-centric drive uses heading minus this.
    public static double DRIVE_STICK_ZERO_DEG = 90.0;

    // Sign-preserving square on analog sticks after deadzone. Full stick is still full power.
    public static boolean DRIVE_STICK_SQUARED = true;

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
