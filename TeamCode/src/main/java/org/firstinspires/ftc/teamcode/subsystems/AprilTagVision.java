package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionPortal.CameraState;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

/**
 * Webcam VisionPortal + AprilTag processor. Sample only; no aiming.
 */
public class AprilTagVision {
    private AprilTagProcessor processor;
    private VisionPortal portal;
    private String lastState = "";
    private boolean sawStreaming;

    public AprilTagVision(HardwareMap hardwareMap, WebcamName webcam) {
        if (webcam == null) {
            RobotLog.w("[VISION] Webcam is null. Vision disabled.");
            return;
        }
        try {
            processor = new AprilTagProcessor.Builder().build();
            portal = new VisionPortal.Builder()
                    .setCamera(webcam)
                    .addProcessor(processor)
                    .build();
            RobotLog.i("[VISION] Portal opened on %s", webcam.getDeviceName());
        } catch (Exception e) {
            RobotLog.e("[VISION] Init failed: %s", e.getMessage());
        }
    }

    public boolean isPresent() {
        return portal != null && processor != null;
    }

    public String getCameraStateName() {
        if (portal == null) {
            return "NONE";
        }
        return portal.getCameraState().name();
    }

    public List<AprilTagDetection> sample() {
        String state = getCameraStateName();
        if (state.equals(CameraState.STREAMING.name())) {
            sawStreaming = true;
        }
        if (!state.equals(lastState)) {
            RobotLog.i("[VISION] Camera state %s -> %s", lastState, state);
            lastState = state;
        }
        if (processor == null) {
            return new ArrayList<>();
        }
        return processor.getDetections();
    }

    public String getDisconnectError() {
        if (portal == null) {
            return null;
        }
        CameraState state = portal.getCameraState();
        if (state == CameraState.ERROR) {
            return "camera error";
        }
        if (sawStreaming && (state == CameraState.CLOSED || state == CameraState.STOPPING_STREAM)) {
            return "camera disconnected";
        }
        return null;
    }
}
