package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.config.TunableValues;
import org.firstinspires.ftc.teamcode.geometry.CellGeometry;
import org.firstinspires.ftc.teamcode.interfaces.AllianceInterface;
import org.firstinspires.ftc.teamcode.interfaces.TargetCellTrackerInterface;
import org.firstinspires.ftc.teamcode.util.RobotLogger;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public class TargetCellTracker implements TargetCellTrackerInterface {
    private static final double UPRIGHT_ROLL_MAX_DEG = 90.0;

    private final RobotLogger log = new RobotLogger("TARGET");
    private AllianceInterface.Color alliance;
    private boolean inView;
    private final boolean[] hits;
    private int hitIndex;
    private int samplesTaken;
    private CellGeometry lastKnownGood;
    private String lastKey = "";
    private String lastRejectKey = "";

    public TargetCellTracker() {
        hits = new boolean[TunableValues.TAG_DETECTION_WINDOW];
    }

    @Override
    public void setAlliance(AllianceInterface.Color color) {
        alliance = color;
        log.i("Tracking alliance %s", color);
    }

    @Override
    public void update(List<AprilTagDetection> detections) {
        update(detections, true);
    }

    @Override
    public void update(List<AprilTagDetection> detections, boolean reuseLastKnownGoodOnMiss) {
        CellGeometry best = null;
        if (alliance != null && detections != null) {
            for (AprilTagDetection detection : detections) {
                CellGeometry geometry = toGeometry(detection);
                if (geometry == null) {
                    continue;
                }
                if (!geometry.upright) {
                    String rejectKey = geometry.clusterName + "|" + Math.round(geometry.rollDeg);
                    if (!rejectKey.equals(lastRejectKey)) {
                        lastRejectKey = rejectKey;
                        log.i("reject roll cluster=%s roll=%.1f (need |roll| < %.0f) pct=%d",
                                geometry.clusterName, geometry.rollDeg, UPRIGHT_ROLL_MAX_DEG, geometry.percentFound);
                    }
                    continue;
                }
                if (best == null || geometry.rangeIn < best.rangeIn) {
                    best = geometry;
                }
            }
        }
        inView = best != null;
        hits[hitIndex] = inView;
        hitIndex = (hitIndex + 1) % hits.length;
        samplesTaken++;
        if (inView) {
            lastKnownGood = best;
        } else if (!reuseLastKnownGoodOnMiss || !isGatePassing()) {
            lastKnownGood = null;
        }
        String key = inView + "|" + isGatePassing() + "|" + (best == null ? "none" : best.clusterName);
        if (!key.equals(lastKey)) {
            lastKey = key;
            if (best != null) {
                log.i("inView=%b gate=%d/%d cluster=%s pct=%d range=%.1f bearing=%.1f yaw=%.1f roll=%.1f upright=%b",
                        inView, getGateHits(), getGateWindow(), best.clusterName, best.percentFound, best.rangeIn,
                        best.bearingDeg, best.yawDeg, best.rollDeg, best.upright);
            } else {
                log.i("inView=false gate=%d/%d alliance=%s detections=%d",
                        getGateHits(), getGateWindow(), alliance, detections == null ? 0 : detections.size());
            }
        }
    }

    @Override
    public boolean isTargetCellInView() {
        return inView;
    }

    @Override
    public boolean isGatePassing() {
        return getGateHits() >= TunableValues.TAG_DETECTION_MIN_HITS
                && samplesTaken >= TunableValues.TAG_DETECTION_MIN_HITS;
    }

    @Override
    public int getGateHits() {
        int count = 0;
        int n = Math.min(samplesTaken, hits.length);
        for (int i = 0; i < n; i++) {
            if (hits[i]) {
                count++;
            }
        }
        return count;
    }

    @Override
    public int getGateWindow() {
        return hits.length;
    }

    @Override
    public CellGeometry getLastKnownGood() {
        return isGatePassing() ? lastKnownGood : null;
    }

    private CellGeometry toGeometry(AprilTagDetection detection) {
        if (!(detection instanceof AprilTagClusterDetection) || detection.ftcPose == null) {
            return null;
        }
        AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
        String name = cluster.metadata == null ? "" : cluster.metadata.name;
        if (!nameMatchesAlliance(name)) {
            return null;
        }
        double roll = detection.ftcPose.roll;
        boolean upright = Math.abs(roll) < UPRIGHT_ROLL_MAX_DEG;
        return new CellGeometry(name, cluster.percentClusterFound,
                detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z, detection.ftcPose.yaw,
                detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation,
                roll, upright);
    }

    private boolean nameMatchesAlliance(String name) {
        String upper = name == null ? "" : name.toUpperCase();
        if (alliance == AllianceInterface.Color.RED) {
            return upper.contains("RED");
        }
        return upper.contains("BLUE") && !upper.contains("RED");
    }
}
