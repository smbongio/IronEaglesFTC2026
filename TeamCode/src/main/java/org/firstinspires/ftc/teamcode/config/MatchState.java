package org.firstinspires.ftc.teamcode.config;

/**
 * Hands the alliance and final pose from Autonomous to TeleOp. Static fields survive
 * between OpModes until the Robot Controller app restarts.
 */
public final class MatchState {
    public static Alliance alliance;

    public static boolean hasAutoPose;
    public static double autoXIn;
    public static double autoYIn;
    public static double autoHeadingDeg;

    private MatchState() {
    }

    public static void saveAutoPose(double xIn, double yIn, double headingDeg) {
        autoXIn = xIn;
        autoYIn = yIn;
        autoHeadingDeg = headingDeg;
        hasAutoPose = true;
    }

    public static void clearAutoPose() {
        hasAutoPose = false;
    }
}
