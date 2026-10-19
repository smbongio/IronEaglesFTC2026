package org.firstinspires.ftc.teamcode.interfaces;

import com.qualcomm.robotcore.hardware.PIDFCoefficients;

/**
 * Interface for controlling the Shooter mechanism.
 */
public interface ShooterInterface extends MechanismInterface {
    /**
     * Actuates the shooter at the specified speed.
     *
     * @param speed Target operational speed multiplier
     */
    void runShooter(double speed);

    /**
     * Measured leader shooter encoder speed, in ticks per second.
     */
    double getTicksPerSec();

    /**
     * Measured follow shooter encoder speed, in ticks per second.
     */
    double getFollowTicksPerSec();

    /**
     * Hub velocity PIDF currently in use on the leader shooter motor.
     */
    PIDFCoefficients getPidf();

    /**
     * Hub velocity PIDF currently in use on the follow shooter motor.
     */
    PIDFCoefficients getFollowPidf();
}
