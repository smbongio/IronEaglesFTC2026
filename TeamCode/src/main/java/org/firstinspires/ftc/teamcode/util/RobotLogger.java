package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.RobotLog;

/**
 * Verbose robot logging for bring-up. Prefixes every line so we can grep a subsystem.
 */
public final class RobotLogger {
    private final String tag;

    public RobotLogger(String tag) {
        this.tag = tag;
    }

    public void i(String format, Object... args) {
        RobotLog.i("[" + tag + "] " + String.format(format, args));
    }

    public void w(String format, Object... args) {
        RobotLog.w("[" + tag + "] " + String.format(format, args));
    }

    public void e(String format, Object... args) {
        RobotLog.e("[" + tag + "] " + String.format(format, args));
    }
}
