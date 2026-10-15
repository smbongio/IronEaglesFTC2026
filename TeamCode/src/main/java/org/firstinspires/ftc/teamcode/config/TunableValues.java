package org.firstinspires.ftc.teamcode.config;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

/**
 * Starting values from TELEOP_TECHNICAL_REQUIREMENTS.md. Change these after field testing.
 */
public final class TunableValues {
    private TunableValues() {}

    public static RevHubOrientationOnRobot.LogoFacingDirection IMU_LOGO_DIRECTION =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public static RevHubOrientationOnRobot.UsbFacingDirection IMU_USB_DIRECTION =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    public static boolean SHOOTER_RUN_ENTIRE_MATCH = false;
    public static double SHOOTER_TARGET_RPM = 2000.0;
    public static double INTAKE_TARGET_RPM = 500.0;
    public static double INTAKE_REVERSE_RPM = 500.0;

    public static double SHOOTING_DISTANCE_IN = 24.0;
    public static double AUTO_FIRE_MAX_START_RANGE_IN = 60.0;
    public static double DISTANCE_TOLERANCE_IN = 2.0;
    public static double AIM_OFFSET_DEG = 0.0;
    public static double ALIGNMENT_TOLERANCE_DEG = 5.0;
    public static double AUTO_FIRE_DRIVE_POWER = 0.4;
    public static double AUTO_FIRE_TURN_POWER = 0.3;
    public static double WHEEL_DIAMETER_IN = 3.0;
    public static int TAG_DETECTION_MIN_HITS = 3;
    public static int TAG_DETECTION_WINDOW = 5;
    public static double DEADZONE = 0.05;
}
