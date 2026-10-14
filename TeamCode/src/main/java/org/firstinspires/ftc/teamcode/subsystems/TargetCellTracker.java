package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.config.TunableValues;
import org.firstinspires.ftc.teamcode.geometry.CellGeometry;
import org.firstinspires.ftc.teamcode.interfaces.AllianceInterface;
import org.firstinspires.ftc.teamcode.interfaces.TargetCellTrackerInterface;
import org.firstinspires.ftc.teamcode.util.RobotLogger;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;

public class TargetCellTracker implements TargetCellTrackerInterface {
    private static final int[] RED_IDS = {30, 31, 32, 33, 34, 35, 36, 37};
    private static final int[] BLUE_IDS = {38, 39, 40, 41, 42, 43, 44, 45};

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
        CellGeometry best = null;
        if (alliance != null && detections != null) {
            for (AprilTagDetection detection : detections) {
                CellGeometry geometry = toGeometry(detection);
                if (geometry == null) {
                    continue;
                }
                if (!geometry.upright) {
                    String rejectKey = geometry.source + "|" + Math.round(geometry.tagHeightIn);
                    if (!rejectKey.equals(lastRejectKey)) {
                        lastRejectKey = rejectKey;
                        log.i("reject height source=%s height=%.1f min=%.1f",
                                geometry.source, geometry.tagHeightIn, TunableValues.CELL_UP_MIN_HEIGHT_IN);
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
        }
        if (!isGatePassing()) {
            lastKnownGood = inView ? best : null;
        }
        String key = inView + "|" + isGatePassing() + "|" + (best == null ? "none" : best.source);
        if (!key.equals(lastKey)) {
            lastKey = key;
            if (best != null) {
                log.i("inView=%b gate=%d/%d source=%s id=%d range=%.1f bearing=%.1f yaw=%.1f height=%.1f upright=%b",
                        inView, getGateHits(), getGateWindow(), best.source, best.tagId, best.rangeIn,
                        best.bearingDeg, best.yawDeg, best.tagHeightIn, best.upright);
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
        if (detection == null || detection.ftcPose == null) {
            return null;
        }
        if (detection instanceof AprilTagClusterDetection) {
            AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
            String name = cluster.metadata == null ? "" : cluster.metadata.name;
            if (!nameMatchesAlliance(name)) {
                return null;
            }
            return build( -1, "cluster " + name, detection);
        }
        if (detection instanceof AprilTagSingleDetection) {
            AprilTagSingleDetection tag = (AprilTagSingleDetection) detection;
            if (!idMatchesAlliance(tag.id)) {
                return null;
            }
            return build(tag.id, "tag " + tag.id, detection);
        }
        return null;
    }

    private CellGeometry build(int id, String source, AprilTagDetection detection) {
        double range = detection.ftcPose.range;
        double elevation = detection.ftcPose.elevation;
        double height = TunableValues.CAMERA_HEIGHT_IN
                + range * Math.sin(Math.toRadians(elevation + TunableValues.CAMERA_TILT_DEG));
        boolean upright = height >= TunableValues.CELL_UP_MIN_HEIGHT_IN;
        double yaw = detection.ftcPose.yaw;
        double x = detection.ftcPose.x;
        double y = detection.ftcPose.y;
        double z = detection.ftcPose.z;
        double horiz = Math.hypot(x, y);
        if (horiz > 0.1 && TunableValues.CELL_CENTER_OFFSET_IN != 0.0) {
            double scale = (horiz + TunableValues.CELL_CENTER_OFFSET_IN) / horiz;
            x *= scale;
            y *= scale;
        }
        return new CellGeometry(id, source, x, y, z, yaw, Math.hypot(x, y),
                detection.ftcPose.bearing, elevation, height, upright);
    }

    private boolean nameMatchesAlliance(String name) {
        String upper = name == null ? "" : name.toUpperCase();
        if (alliance == AllianceInterface.Color.RED) {
            return upper.contains("RED");
        }
        return upper.contains("BLUE") && !upper.contains("RED");
    }

    private boolean idMatchesAlliance(int id) {
        int[] ids = alliance == AllianceInterface.Color.RED ? RED_IDS : BLUE_IDS;
        for (int candidate : ids) {
            if (candidate == id) {
                return true;
            }
        }
        return false;
    }
}
