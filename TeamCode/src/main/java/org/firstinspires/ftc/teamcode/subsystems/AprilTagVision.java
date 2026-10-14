package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.interfaces.VisionInterface;
import org.firstinspires.ftc.teamcode.util.RobotLogger;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class AprilTagVision implements VisionInterface {
    private final RobotLogger log = new RobotLogger("VISION");
    private AprilTagProcessor processor;
    private VisionPortal portal;
    private String lastState = "";

    public AprilTagVision(HardwareMap hardwareMap, WebcamName webcam) {
        if (webcam == null) {
            log.w("Webcam is null. Vision disabled.");
            return;
        }
        try {
            processor = new AprilTagProcessor.Builder().build();
            portal = new VisionPortal.Builder()
                    .setCamera(webcam)
                    .addProcessor(processor)
                    .build();
            log.i("Vision portal opened on %s", webcam.getDeviceName());
        } catch (Exception e) {
            log.e("Vision init failed: %s", e.getMessage());
        }
    }

    @Override
    public boolean isPresent() {
        return portal != null && processor != null;
    }

    @Override
    public String getCameraStateName() {
        if (portal == null) {
            return "NONE";
        }
        return portal.getCameraState().name();
    }

    @Override
    public double getFps() {
        return portal == null ? 0.0 : portal.getFps();
    }

    @Override
    public List<AprilTagDetection> sample() {
        String state = getCameraStateName();
        if (!state.equals(lastState)) {
            log.i("Camera state %s -> %s  fps=%.1f", lastState, state, getFps());
            lastState = state;
        }
        if (processor == null) {
            return new ArrayList<>();
        }
        return processor.getDetections();
    }
}
