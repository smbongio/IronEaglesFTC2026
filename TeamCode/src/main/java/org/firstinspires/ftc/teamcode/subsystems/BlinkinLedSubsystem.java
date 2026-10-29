package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.config.TunableValues;

/**
 * REV Blinkin on Control Hub servo 0. Flashes green while the target cell is visible.
 */
public class BlinkinLedSubsystem {
    private final RevBlinkinLedDriver blinkin;
    private final ElapsedTime flashTimer = new ElapsedTime();
    private boolean flashOn;
    private boolean wasVisible;
    private BlinkinPattern lastPattern;

    public BlinkinLedSubsystem(RevBlinkinLedDriver blinkin) {
        this.blinkin = blinkin;
        if (blinkin == null) {
            RobotLog.w("[LED] Blinkin is NULL. Check hardware config name 'blinkin'.");
            return;
        }
        setPattern(BlinkinPattern.BLACK);
    }

    public boolean isPresent() {
        return blinkin != null;
    }

    /**
     * 5 Hz green flash while {@code targetVisible}; off otherwise.
     */
    public void update(boolean targetVisible) {
        if (blinkin == null) {
            return;
        }
        if (!targetVisible) {
            wasVisible = false;
            flashOn = false;
            flashTimer.reset();
            setPattern(BlinkinPattern.BLACK);
            return;
        }
        if (!wasVisible) {
            wasVisible = true;
            flashOn = true;
            flashTimer.reset();
        }
        double halfPeriodMs = 500.0 / Math.max(0.1, TunableValues.LED_FLASH_HZ);
        if (flashTimer.milliseconds() >= halfPeriodMs) {
            flashOn = !flashOn;
            flashTimer.reset();
        }
        setPattern(flashOn ? BlinkinPattern.GREEN : BlinkinPattern.BLACK);
    }

    public String getStatus() {
        if (blinkin == null) {
            return "MISSING";
        }
        if (lastPattern == BlinkinPattern.GREEN) {
            return "GREEN";
        }
        return "OFF";
    }

    public void close() {
        flashOn = false;
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
