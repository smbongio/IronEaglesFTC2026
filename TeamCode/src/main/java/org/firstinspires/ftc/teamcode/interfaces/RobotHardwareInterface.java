package org.firstinspires.ftc.teamcode.interfaces;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Contract for robot hardware initialization and component retrieval.
 */
public interface RobotHardwareInterface {
    /**
     * Initializes hardware devices mapped on Control Hub and Expansion Hub.
     *
     * @param hardwareMap HardwareMap instance provided by FTC LinearOpMode
     */
    void init(HardwareMap hardwareMap);

    DrivetrainInterface getDrivetrain();
    IntakeInterface getIntake();
    HopperInterface getHopper();
    ShooterInterface getShooter();
}
