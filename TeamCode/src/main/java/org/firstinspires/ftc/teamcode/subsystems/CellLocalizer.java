package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.teamcode.config.Alliance;
import org.firstinspires.ftc.teamcode.config.TunableValues;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;

import java.util.ArrayList;
import java.util.List;

/**
 * Turns a sighting of any of the four cell tag clusters into a robot field position,
 * using the cell positions and camera mount in {@link TunableValues} and the odometry heading.
 * Also runs the other way (where the cell is, given a known robot pose) for calibration.
 */
public class CellLocalizer {

    /** One sighting: which cell, and the cell's offset from the robot center in field inches. */
    public static final class Sighting {
        public final String clusterName;
        public final double cellXIn;
        public final double cellYIn;
        public final double fieldDxIn;
        public final double fieldDyIn;
        public final double rangeIn;

        Sighting(String clusterName, double cellXIn, double cellYIn, double fieldDxIn, double fieldDyIn,
                 double rangeIn) {
            this.clusterName = clusterName;
            this.cellXIn = cellXIn;
            this.cellYIn = cellYIn;
            this.fieldDxIn = fieldDxIn;
            this.fieldDyIn = fieldDyIn;
            this.rangeIn = rangeIn;
        }

        public double robotXIn() {
            return cellXIn - fieldDxIn;
        }

        public double robotYIn() {
            return cellYIn - fieldDyIn;
        }
    }

    public List<Sighting> sightings(List<AprilTagDetection> detections, double robotHeadingDeg) {
        List<Sighting> out = new ArrayList<>();
        if (detections == null) {
            return out;
        }
        for (AprilTagDetection detection : detections) {
            if (!(detection instanceof AprilTagClusterDetection) || detection.ftcPose == null) {
                continue;
            }
            AprilTagClusterDetection cluster = (AprilTagClusterDetection) detection;
            if (cluster.metadata == null) {
                continue;
            }
            for (Alliance owner : Alliance.values()) {
                double redCellX;
                if (owner.upCellClusterName().equals(cluster.metadata.name)) {
                    redCellX = TunableValues.VISION_UP_CELL_X_IN;
                } else if (owner.downCellClusterName().equals(cluster.metadata.name)) {
                    redCellX = TunableValues.VISION_DOWN_CELL_X_IN;
                } else {
                    continue;
                }
                double[] d = fieldOffset(detection, robotHeadingDeg);
                out.add(new Sighting(cluster.metadata.name, owner.x(redCellX),
                        owner.y(TunableValues.VISION_CELL_Y_IN), d[0], d[1],
                        detection.distanceUnit.toInches(detection.ftcPose.range)));
            }
        }
        return out;
    }

    /** Field-frame inches from the robot center to the cluster origin. */
    private static double[] fieldOffset(AprilTagDetection detection, double robotHeadingDeg) {
        AprilTagPoseFtc pose = detection.ftcPose;
        double right = detection.distanceUnit.toInches(pose.x);
        double ahead = detection.distanceUnit.toInches(pose.y);
        double up = detection.distanceUnit.toInches(pose.z);

        double pitch = Math.toRadians(TunableValues.CAMERA_PITCH_DEG);
        double level = ahead * Math.cos(pitch) - up * Math.sin(pitch);

        double yaw = Math.toRadians(TunableValues.CAMERA_YAW_DEG);
        double forward = TunableValues.CAMERA_FORWARD_IN + level * Math.cos(yaw) + right * Math.sin(yaw);
        double left = TunableValues.CAMERA_LEFT_IN + level * Math.sin(yaw) - right * Math.cos(yaw);

        double h = Math.toRadians(robotHeadingDeg);
        return new double[] {
                forward * Math.cos(h) - left * Math.sin(h),
                forward * Math.sin(h) + left * Math.cos(h),
        };
    }
}
