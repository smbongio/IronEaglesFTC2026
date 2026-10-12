package org.firstinspires.ftc.teamcode.interfaces;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.List;
import java.util.Set;

/**
 * Interface contract for the Vision Subsystem.
 * Manages multi-AprilTag detection, alliance target filtering, and target geometry retrieval.
 */
public interface VisionSubsystemInterface {

    /**
     * Alliance color enumeration.
     */
    enum Alliance {
        RED,
        BLUE
    }

    /**
     * Sets the active alliance for filtering target AprilTag IDs.
     *
     * @param alliance RED or BLUE
     */
    void setAlliance(Alliance alliance);

    /**
     * Updates the set of AprilTag IDs corresponding to the active/raised target hives.
     *
     * @param activeTargetTagIds Set of valid AprilTag IDs for the current active hive
     */
    void setActiveTargetTagIds(Set<Integer> activeTargetTagIds);

    /**
     * Returns all raw AprilTagDetection objects currently visible in the camera frame (0 to 16+ tags).
     *
     * @return List of active AprilTagDetection objects
     */
    List<AprilTagDetection> getRawDetections();

    /**
     * Returns spatial geometry for ALL AprilTags currently visible in the camera frame (0, 8, 13, 16+ tags).
     *
     * @return List of TargetGeometryInterface objects for every visible tag
     */
    List<TargetGeometryInterface> getAllTargetGeometries();

    /**
     * Returns spatial geometries filtered for the active team alliance (Red or Blue).
     *
     * @return List of TargetGeometryInterface objects matching active alliance
     */
    List<TargetGeometryInterface> getAllianceTargetGeometries();

    /**
     * Records one camera sample for the X-of-Y detection gate.
     * Must be called once per TeleOp loop so the gate and last-known-good geometry stay current.
     */
    void update();

    /**
     * Checks if the X-of-Y detection gate is passing: at least X of the last Y samples
     * found a valid, alliance-matching target tag. Last-known-good geometry is usable while this is true.
     *
     * @return true if the detection gate is passing
     */
    boolean isTargetAcquired();

    /**
     * Number of hits in the current detection window (for telemetry).
     */
    int getDetectionHitCount();

    /**
     * Number of samples currently stored in the detection window, up to Y (for telemetry).
     */
    int getDetectionSampleCount();

    /**
     * Checks if the acquired target is within valid physical shooting range and angle alignment.
     * Encapsulates range (e.g. 18" to 36") and bearing angle evaluation inside the Vision Subsystem.
     * Uses last-known-good geometry while the detection gate is passing.
     *
     * @return true if target distance and alignment are optimal for taking a shot
     */
    boolean isInShootingRange();

    /**
     * Function call to request spatial geometry for the best/closest valid target AprilTag.
     * While the detection gate is passing, returns the last known good geometry (refreshed on a hit,
     * reused on a miss). Returns null if the gate is not passing.
     *
     * @return Best TargetGeometryInterface or null
     */
    TargetGeometryInterface getBestTargetGeometry();

    /**
     * Shuts down or pauses the vision streaming portal to save CPU resources when not in use.
     */
    void stopStreaming();

    /**
     * Resumes vision streaming portal.
     */
    void resumeStreaming();

    /**
     * Current VisionPortal camera state name (for example STREAMING or ERROR), or NONE if the camera never opened.
     */
    String getCameraStateName();

    /**
     * Latest camera frames-per-second, or 0 if the portal is not running.
     */
    double getFps();
}
