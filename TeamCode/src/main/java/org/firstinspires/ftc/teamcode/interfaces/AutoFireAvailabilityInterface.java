package org.firstinspires.ftc.teamcode.interfaces;

/**
 * Single place that computes AUTO_FIRE_AVAILABLE from alliance, IMU, drive encoders, gate, and cone.
 */
public interface AutoFireAvailabilityInterface {
    boolean isAutoFireAvailable();

    String unavailableReason();
}
