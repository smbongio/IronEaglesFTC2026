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
     * Checks if at least one valid, alliance-matching active target tag is currently detected.
     *
     * @return true if a valid target AprilTag is acquired
     */
    boolean isTargetAcquired();

    /**
     * Checks if the acquired target is within valid physical shooting range and angle alignment.
     * Encapsulates range (e.g. 18" to 36") and bearing angle evaluation inside the Vision Subsystem.
     *
     * @return true if target distance and alignment are optimal for taking a shot
     */
    boolean isInShootingRange();

    /**
     * Function call to request spatial geometry for the best/closest valid target AprilTag.
     * Evaluates all visible tags, filters for active alliance & active hive IDs, and returns the closest target.
     * Returns null if no valid target AprilTag is currently acquired.
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
}
