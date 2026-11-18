package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.config.Alliance;

/**
 * REV Blinkin on Control Hub servo 0. Solid alliance color for the whole match
 * (off until one is chosen).
 */
public class BlinkinLedSubsystem {
    private final RevBlinkinLedDriver blinkin;
    private BlinkinPattern lastPattern;

    public BlinkinLedSubsystem(RevBlinkinLedDriver blinkin) {
        this.blinkin = blinkin;
        if (blinkin == null) {
            RobotLog.w("[LED] Blinkin is NULL. Check hardware config name 'chServo0'.");
            return;
        }
        setPattern(BlinkinPattern.BLACK);
    }

    public boolean isPresent() {
        return blinkin != null;
    }

    /** Solid red or blue from now on; {@code null} turns it off. */
    public void setAlliance(Alliance alliance) {
        if (alliance == null) {
            setPattern(BlinkinPattern.BLACK);
        } else {
            setPattern(alliance == Alliance.RED ? BlinkinPattern.RED : BlinkinPattern.BLUE);
        }
    }

    public String getStatus() {
        if (blinkin == null) {
            return "MISSING";
        }
        if (lastPattern == null || lastPattern == BlinkinPattern.BLACK) {
            return "OFF";
        }
        return lastPattern.name();
    }

    public void close() {
        setPattern(BlinkinPattern.BLACK);
    }

    private void setPattern(BlinkinPattern pattern) {
        if (blinkin == null || pattern == lastPattern) {
            return;
        }
        try {
            blinkin.setPattern(pattern);
            lastPattern = pattern;
        } catch (RuntimeException e) {
            RobotLog.e("[LED] setPattern failed: %s", e.getMessage());
        }
    }
}
