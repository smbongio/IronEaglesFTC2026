package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.interfaces.LedIndicatorInterface;

/**
 * Concrete implementation of the LED Indicator subsystem wrapping a REV Blinkin LED Driver.
 */
public class BlinkinLedSubsystem implements LedIndicatorInterface {

    private RevBlinkinLedDriver blinkinLedDriver;

    public BlinkinLedSubsystem(HardwareMap hardwareMap, String deviceName) {
        try {
            blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, deviceName);
            setLedState(LedState.OFF);
            RobotLog.i("BlinkinLedSubsystem initialized successfully on port: %s", deviceName);
        } catch (Exception e) {
            RobotLog.w("BlinkinLedSubsystem initialization failed for device '%s': %s", deviceName, e.getMessage());
            blinkinLedDriver = null;
        }
    }

    @Override
    public void setLedState(LedState state) {
        if (blinkinLedDriver == null) return;

        switch (state) {
            case SHOOTING: // Solid Green when actively shooting
                blinkinLedDriver.setPattern(BlinkinPattern.GREEN);
                break;

            case IN_RANGE: // Flashing / Beats per Minute pattern when in shooting range
                blinkinLedDriver.setPattern(BlinkinPattern.CP1_2_BEATS_PER_MINUTE);
                break;

            case OFF: // Turned off
            default:
                blinkinLedDriver.setPattern(BlinkinPattern.BLACK);
                break;
        }
    }

    @Override
    public void turnOff() {
        if (blinkinLedDriver != null) {
            blinkinLedDriver.setPattern(BlinkinPattern.BLACK);
        }
    }
}
