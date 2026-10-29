package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.teamcode.config.TunableValues;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;

/**
 * Sliding window over frames that contain the red audience cell (tags 34-37).
 * TRUE when at least VISION_SUCCESS_RATE of the last VISION_SAMPLE_RATE samples hit.
 */
public class TargetCellGate {
    private static final String TARGET_CLUSTER_NAME = "RED AUDIENCE";

    private final boolean[] hits;
    private int hitIndex;
    private int samplesTaken;
    private boolean lastReported;
    private boolean hasReported;

    public TargetCellGate() {
        hits = new boolean[Math.max(1, TunableValues.VISION_SAMPLE_RATE)];
    }

    public void update(List<AprilTagDetection> detections) {
        boolean seen = containsTargetCell(detections);
        hits[hitIndex] = seen;
        hitIndex = (hitIndex + 1) % hits.length;
        samplesTaken++;

        boolean passing = isTargetVisible();
        if (!hasReported || passing != lastReported) {
            RobotLog.i("[VISION] Target %s  gate %d/%d  frame=%s",
                    passing ? "TRUE" : "FALSE", getHits(), getWindow(), seen ? "hit" : "miss");
            lastReported = passing;
            hasReported = true;
        }
    }

    public boolean isTargetVisible() {
        int need = neededHits();
        return getHits() >= need && samplesTaken >= need;
    }

    public int getHits() {
        int count = 0;
        int n = Math.min(samplesTaken, hits.length);
        for (int i = 0; i < n; i++) {
            if (hits[i]) {
                count++;
            }
        }
        return count;
    }

    public int getWindow() {
        return hits.length;
    }

    public String getStatus() {
        return isTargetVisible() ? "TRUE" : "FALSE";
    }

    public String getGateStatus() {
        return String.format("%d/%d (need %d)", getHits(), getWindow(), neededHits());
    }

    private int neededHits() {
        int window = hits.length;
        int need = TunableValues.VISION_SUCCESS_RATE;
        if (need < 1) {
            return 1;
        }
        return Math.min(need, window);
    }

    private static boolean containsTargetCell(List<AprilTagDetection> detections) {
        if (detections == null) {
            return false;
        }
        for (AprilTagDetection detection : detections) {
            if (detection instanceof AprilTagSingleDetection) {
                int id = ((AprilTagSingleDetection) detection).id;
                if (id >= TunableValues.VISION_TARGET_TAG_MIN && id <= TunableValues.VISION_TARGET_TAG_MAX) {
                    return true;
                }
            } else if (detection instanceof AprilTagClusterDetection) {
                AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
                if (cluster.metadata != null && TARGET_CLUSTER_NAME.equals(cluster.metadata.name)) {
                    return true;
                }
            }
        }
        return false;
    }
}
