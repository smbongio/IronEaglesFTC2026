package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.interfaces.TargetGeometryInterface;
import org.firstinspires.ftc.teamcode.interfaces.VisionSubsystemInterface;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Set;

/**
 * Concrete implementation of the Vision Subsystem using FTC VisionPortal & AprilTagProcessor.
 * Detects AprilTags, filters for alliance targets, and calculates spatial geometry for aiming.
 */
public class AprilTagVisionSubsystem implements VisionSubsystemInterface {

    // Valid shooting distance thresholds in inches
    public static double MIN_SHOOTING_RANGE_INCHES = 12.0;
    public static double MAX_SHOOTING_RANGE_INCHES = 48.0;
    public static double MAX_SHOOTING_BEARING_DEG  = 15.0;

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    private Alliance activeAlliance = Alliance.BLUE;
    private Set<Integer> activeTargetTagIds = new HashSet<>();

    public AprilTagVisionSubsystem(HardwareMap hardwareMap, WebcamName webcamName) {
        if (webcamName != null) {
            try {
                aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();
                visionPortal = new VisionPortal.Builder()
                        .setCamera(webcamName)
                        .addProcessor(aprilTagProcessor)
                        .build();
                RobotLog.i("AprilTagVisionSubsystem initialized successfully with webcam: %s", webcamName.getDeviceName());
            } catch (Exception e) {
                RobotLog.e("AprilTagVisionSubsystem initialization failed: " + e.getMessage(), e);
            }
        } else {
            RobotLog.w("AprilTagVisionSubsystem: WebcamName is null. Vision streaming disabled.");
        }
    }

    @Override
    public void setAlliance(Alliance alliance) {
        this.activeAlliance = alliance;
        RobotLog.i("AprilTagVisionSubsystem alliance set to: %s", alliance);
    }

    @Override
    public void setActiveTargetTagIds(Set<Integer> activeTargetTagIds) {
        this.activeTargetTagIds = (activeTargetTagIds != null) ? activeTargetTagIds : new HashSet<>();
    }

    @Override
    public List<AprilTagDetection> getRawDetections() {
        if (aprilTagProcessor != null) {
            return aprilTagProcessor.getDetections();
        }
        return new ArrayList<>();
    }

    @Override
    public List<TargetGeometryInterface> getAllTargetGeometries() {
        List<TargetGeometryInterface> geometries = new ArrayList<>();
        List<AprilTagDetection> detections = getRawDetections();
        for (AprilTagDetection detection : detections) {
            if (detection != null && detection.ftcPose != null) {
                geometries.add(new TargetGeometry(detection));
            }
        }
        return geometries;
    }

    @Override
    public List<TargetGeometryInterface> getAllianceTargetGeometries() {
        List<TargetGeometryInterface> geometries = new ArrayList<>();
        List<AprilTagDetection> detections = getRawDetections();
        for (AprilTagDetection detection : detections) {
            if (detection != null && detection.ftcPose != null) {
                int id = (detection instanceof AprilTagSingleDetection) ? ((AprilTagSingleDetection) detection).id : -1;
                if (activeTargetTagIds.isEmpty() || activeTargetTagIds.contains(id)) {
                    geometries.add(new TargetGeometry(detection));
                }
            }
        }
        return geometries;
    }

    @Override
    public boolean isTargetAcquired() {
        return !getAllianceTargetGeometries().isEmpty();
    }

    @Override
    public boolean isInShootingRange() {
        TargetGeometryInterface bestTarget = getBestTargetGeometry();
        if (bestTarget == null) return false;

        double range = bestTarget.getRangeInches();
        double absBearing = Math.abs(bestTarget.getBearingDegrees());

        return (range >= MIN_SHOOTING_RANGE_INCHES && range <= MAX_SHOOTING_RANGE_INCHES && absBearing <= MAX_SHOOTING_BEARING_DEG);
    }

    @Override
    public TargetGeometryInterface getBestTargetGeometry() {
        List<TargetGeometryInterface> geometries = getAllianceTargetGeometries();
        if (geometries.isEmpty()) return null;

        TargetGeometryInterface bestTarget = null;
        double minRange = Double.MAX_VALUE;

        for (TargetGeometryInterface geom : geometries) {
            if (geom.getRangeInches() < minRange) {
                minRange = geom.getRangeInches();
                bestTarget = geom;
            }
        }
        return bestTarget;
    }

    @Override
    public void stopStreaming() {
        if (visionPortal != null) {
            try {
                visionPortal.stopStreaming();
            } catch (Exception ignored) {}
        }
    }

    @Override
    public void resumeStreaming() {
        if (visionPortal != null) {
            try {
                visionPortal.resumeStreaming();
            } catch (Exception ignored) {}
        }
    }
}
