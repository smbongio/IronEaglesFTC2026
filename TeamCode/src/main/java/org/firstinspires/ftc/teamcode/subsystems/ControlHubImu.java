package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.config.TunableValues;
import org.firstinspires.ftc.teamcode.interfaces.ImuInterface;
import org.firstinspires.ftc.teamcode.util.RobotLogger;

public class ControlHubImu implements ImuInterface {
    private final RobotLogger log = new RobotLogger("IMU");
    private final IMU imu;

    public ControlHubImu(HardwareMap hardwareMap) {
        imu = hardwareMap.tryGet(IMU.class, "imu");
        if (imu == null) {
            log.w("IMU 'imu' not found. Auto fire disabled. Driving will be robot-centric.");
            return;
        }
        // Logo upright on a hub mounted on the right side of the robot. USB assumed forward until measured.
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                TunableValues.IMU_LOGO_DIRECTION, TunableValues.IMU_USB_DIRECTION);
        imu.initialize(new IMU.Parameters(orientation));
        imu.resetYaw();
        log.i("IMU initialized. logo=%s usb=%s (usb assumed until measured). Heading zeroed.",
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
            log.i("Heading zeroed (Back button or init).");
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
