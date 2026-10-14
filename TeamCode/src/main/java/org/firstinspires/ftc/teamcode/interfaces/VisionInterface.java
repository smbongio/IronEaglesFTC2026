package org.firstinspires.ftc.teamcode.interfaces;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public interface VisionInterface {
    boolean isPresent();

    String getCameraStateName();

    double getFps();

    List<AprilTagDetection> sample();
}
