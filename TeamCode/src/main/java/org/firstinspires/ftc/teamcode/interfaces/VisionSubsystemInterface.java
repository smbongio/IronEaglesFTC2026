package org.firstinspires.ftc.teamcode.interfaces;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import java.util.List;
import java.util.Set;

/**
 * Interface contract for the Vision Subsystem.
 * Manages AprilTag detection, alliance target filtering, and target geometry retrieval.
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
     * Returns all currently detected AprilTags visible to the camera.
     *
     * @return List of active AprilTagDetection objects
     */
    List<AprilTagDetection> getRawDetections();

    /**
     * Checks if a valid, alliance-matching active target tag is currently detected and in range.
     *
     * @return true if a valid target AprilTag is acquired
     */
    boolean isTargetAcquired();

    /**
     * Retrieves the AprilTagDetection object for the best/closest valid target AprilTag.
     * Returns null if no valid alliance target tag is currently acquired.
     *
     * @return AprilTagDetection or null
     */
    AprilTagDetection getTargetDetection();

    /**
     * Shuts down or pauses the vision streaming portal to save CPU resources when not in use.
     */
    void stopStreaming();

    /**
     * Resumes vision streaming portal.
     */
    void resumeStreaming();
}
