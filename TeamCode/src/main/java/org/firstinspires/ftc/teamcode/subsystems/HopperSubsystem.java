package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.config.TunableValues;
import org.firstinspires.ftc.teamcode.interfaces.HopperInterface;

/**
 * Hopper on a REV Smart Servo in continuous mode (Expansion Hub servo port 0).
 * Open-loop speed only. A servo port has no encoder, so there are no exact 180-degree steps.
 */
public class HopperSubsystem implements HopperInterface {

    private final CRServo servo;
    private String lastCommand = "init";

    public HopperSubsystem(CRServo servo) {
        this.servo = servo;
        if (servo == null) {
            RobotLog.e("[HOPPER SUBSYSTEM ERROR] Hopper CRServo is NULL! Check hardware config name 'ehServo0'.");
        }
    }

    @Override
    public void runHopper(double speed) {
        lastCommand = "run";
        setPower(speed);
    }

    @Override
    public void setPower(double power) {
        if (servo != null) {
            servo.setPower(power * TunableValues.HOPPER_DIR);
        }
    }

    @Override
    public void stop() {
        lastCommand = "stop";
        if (servo != null) {
            servo.setPower(0);
        }
    }

    @Override
    public boolean isRunning() {
        return servo != null && Math.abs(servo.getPower()) > 0.01;
    }

    @Override
    public String getCommandStatus() {
        if (servo == null) {
            return lastCommand + " servo=NULL";
        }
        return String.format("%s pwr=%.2f", lastCommand, servo.getPower());
    }
}
