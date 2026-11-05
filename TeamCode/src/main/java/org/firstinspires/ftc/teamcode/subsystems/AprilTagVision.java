package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.GlobalWarningSource;
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
    private final WebcamName webcam;
    private AprilTagProcessor processor;
    private VisionPortal portal;
    private String lastState = "";
    private boolean sawStreaming;
    private boolean failed;
    private String stickyDisconnectError;
    private boolean webcamWarningSuppressed;

    public AprilTagVision(HardwareMap hardwareMap, WebcamName webcam) {
        this.webcam = webcam;
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
            fail("camera error");
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
        if (failed) {
            return new ArrayList<>();
        }
        String state = getCameraStateName();
        if (state.equals(CameraState.STREAMING.name())) {
            sawStreaming = true;
        }
        if (!state.equals(lastState)) {
            RobotLog.i("[VISION] Camera state %s -> %s", lastState, state);
            lastState = state;
        }
        if (state.equals(CameraState.ERROR.name())) {
            fail("camera error");
            return new ArrayList<>();
        }
        if (sawStreaming && (state.equals(CameraState.CAMERA_DEVICE_CLOSED.name())
                || state.equals(CameraState.CLOSING_CAMERA_DEVICE.name())
                || state.equals(CameraState.STOPPING_STREAM.name()))) {
            fail("camera disconnected");
            return new ArrayList<>();
        }
        if (processor == null) {
            return new ArrayList<>();
        }
        return processor.getDetections();
    }

    public String getDisconnectError() {
        return stickyDisconnectError;
    }

    public void close() {
        closePortal();
        if (webcamWarningSuppressed && webcam instanceof GlobalWarningSource) {
            ((GlobalWarningSource) webcam).suppressGlobalWarning(false);
            webcamWarningSuppressed = false;
        }
    }

    private void fail(String message) {
        if (failed) {
            return;
        }
        failed = true;
        stickyDisconnectError = message;
        RobotLog.e("[VISION] Camera failed: %s", message);
        closePortal();
        suppressWebcamWarning();
    }

    private void closePortal() {
        if (portal == null) {
            return;
        }
        try {
            portal.close();
        } catch (RuntimeException e) {
            RobotLog.e("[VISION] Portal close failed: %s", e.getMessage());
        }
        portal = null;
        processor = null;
    }

    private void suppressWebcamWarning() {
        if (webcamWarningSuppressed || !(webcam instanceof GlobalWarningSource)) {
            return;
        }
        ((GlobalWarningSource) webcam).suppressGlobalWarning(true);
        webcamWarningSuppressed = true;
    }
}
