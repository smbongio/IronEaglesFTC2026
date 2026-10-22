package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.config.TunableValues;
import org.firstinspires.ftc.teamcode.interfaces.ImuInterface;

/**
 * Control Hub IMU named "imu". Missing IMU falls back to heading 0 (robot-centric).
 */
public class ControlHubImu implements ImuInterface {
    private final IMU imu;

    public ControlHubImu(HardwareMap hardwareMap) {
        imu = hardwareMap.tryGet(IMU.class, "imu");
        if (imu == null) {
            RobotLog.w("[IMU] 'imu' not found. Driving will be robot-centric.");
            return;
        }
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                TunableValues.IMU_LOGO_DIRECTION, TunableValues.IMU_USB_DIRECTION);
        imu.initialize(new IMU.Parameters(orientation));
        imu.resetYaw();
        RobotLog.i("[IMU] initialized. logo=%s usb=%s. Heading zeroed.",
                TunableValues.IMU_LOGO_DIRECTION, TunableValues.IMU_USB_DIRECTION);
    }

    @Override
    public boolean isPresent() {
        return imu != null;
    }

    @Override
    public void resetYaw() {
        if (imu != null) {
            imu.resetYaw();
            RobotLog.i("[IMU] Heading zeroed (Back button).");
        }
    }

    @Override
    public double getHeadingDegrees() {
        if (imu == null) {
            return 0.0;
        }
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
}
